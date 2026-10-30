use assert2::check;

use super::*;
use crate::stabilizers::copter::DEFAULT_GYRO_CUTOFF_HZ;
use crate::tuning::autotune::COPTER_LOOP_SHAPING;

/// The flight loop's tick: 250 Hz.
const DT: f32 = 0.004;

/// The gyro low-pass the designs model: the copter's default.
const GYRO_HZ: f32 = DEFAULT_GYRO_CUTOFF_HZ;

/// A rate axis that integrates its moment through one lag: `accel_radps2` per unit moment,
/// `lag_s`, and `delay_ticks`, as [`RateModel`]'s weights.
fn model(accel_radps2: f32, lag_s: f32, delay_ticks: usize) -> RateModel {
    let pole = libm::expf(-DT / lag_s);
    RateModel {
        rate_weights: [1.0 + pole, -pole],
        moment_weights: [accel_radps2 * (1.0 - pole) * DT, 0.0, 0.0],
        delay_ticks,
        dt_s: DT,
    }
}

/// Gains a test starts from: the rate I that the design keeps.
const START: Gains = Gains {
    p: 0.1,
    i: 0.2,
    d: 0.002,
};

/// `gains`' rate loop's phase margin on `model` through a gyro low-pass at `gyro_hz`, in
/// degrees: 180° plus its phase where its open-loop gain falls to 1.
fn rate_margin_deg(model: &RateModel, gains: Gains, gyro_hz: f32) -> f32 {
    let open_loop = |w_radps: f32| {
        let z1 = unit_delay(w_radps, DT);
        pid_response(gains, z1, DT) * model.response(w_radps) * LowPass::response(gyro_hz, DT, z1)
    };
    let crossover = first_crossing(DT, open_loop, |point| point.gain > 1.0).expect("a crossover");
    180.0 + crossover.phase_deg
}

/// The rate design cancels the lag (D/P is the lag) and leaves the loop the margin, whatever
/// the plant: (accel rad/s² per unit moment, lag s, delay ticks). The flown loop, with the
/// start's I that the design keeps but doesn't shape, holds the margin within 1.5° (the I costs
/// 1° at seven ticks of delay).
#[test]
fn rate_design_cancels_the_lag_and_keeps_the_margin() {
    let rows = [
        (27.0, 0.03, 1),
        (44.0, 0.05, 1),
        (125.0, 0.015, 1),
        (27.0, 0.03, 3),
        (27.0, 0.03, 7),
        (10.0, 0.08, 2),
    ];
    for (accel, lag_s, delay) in rows {
        let row = format!("accel {accel}, lag {lag_s} s, delay {delay}");
        let model = model(accel, lag_s, delay);
        let gains = design_rate(&model, START, GYRO_HZ, &COPTER_LOOP_SHAPING).expect("a design");
        check!(
            (gains.d / gains.p - lag_s).abs() < 1e-4 * lag_s,
            "{row}: {gains:?}"
        );
        check!(gains.i == START.i, "{row}");
        let margin = rate_margin_deg(&model, gains, GYRO_HZ);
        check!(
            (margin - COPTER_LOOP_SHAPING.rate_margin_deg).abs() < 1.5,
            "{row}: margin {margin}°"
        );
    }
}

/// The design spends part of the margin on the gyro low-pass the loop flies through: on a
/// `quad_x`-like axis, a design that left the 40 Hz filter out flies it with under 5° of the
/// 65° asked, near instability; with the filter, P comes out lower and the flown margin is the
/// one asked, at any cutoff (#373).
#[test]
fn rate_design_spends_margin_on_the_gyro_filter() {
    let model = model(27.0, 0.03, 1);
    let unfiltered = design_rate(&model, START, 0.0, &COPTER_LOOP_SHAPING).expect("a design");
    let lost_deg = COPTER_LOOP_SHAPING.rate_margin_deg - rate_margin_deg(&model, unfiltered, 40.0);
    check!(
        lost_deg > 60.0,
        "unfiltered design loses {lost_deg}° to the filter"
    );
    for cutoff_hz in [80.0, 40.0, 20.0] {
        let gains = design_rate(&model, START, cutoff_hz, &COPTER_LOOP_SHAPING).expect("a design");
        let margin = rate_margin_deg(&model, gains, cutoff_hz);
        check!(gains.p < unfiltered.p, "{cutoff_hz} Hz: {gains:?}");
        check!(
            (margin - COPTER_LOOP_SHAPING.rate_margin_deg).abs() < 1.0,
            "{cutoff_hz} Hz: margin {margin}°"
        );
    }
}

/// More delay leaves less phase to spend, so rate P falls with each tick of it.
#[test]
fn rate_p_falls_as_the_delay_grows() {
    let gains: Vec<f32> = (1..=7)
        .map(|delay| {
            design_rate(
                &model(27.0, 0.03, delay),
                START,
                GYRO_HZ,
                &COPTER_LOOP_SHAPING,
            )
            .expect("a design")
            .p
        })
        .collect();
    check!(gains.windows(2).all(|pair| pair[1] < pair[0]), "{gains:?}");
}

/// No lag or no crossover is a typed failure: (model, error).
#[test]
fn rate_design_fails_without_a_lag_or_a_crossover() {
    let with_pole = |pole: f32| RateModel {
        rate_weights: [1.0 + pole, -pole],
        ..model(27.0, 0.03, 1)
    };
    let rows = [
        (with_pole(1.0), DesignError::NotALag),
        (with_pole(1.3), DesignError::NotALag),
        // A moment that turns the rate the wrong way.
        (model(-27.0, 0.03, 1), DesignError::NoCrossover),
        (
            RateModel {
                moment_weights: [0.0, 0.0, 0.0],
                ..model(27.0, 0.03, 1)
            },
            DesignError::NoCrossover,
        ),
    ];
    for (model, error) in rows {
        check!(
            design_rate(&model, START, GYRO_HZ, &COPTER_LOOP_SHAPING) == Err(error),
            "{model:?}"
        );
    }
}

/// A pole at or below 0, a lag under a tick, designs D 0 and a P from the model's response
/// (#563): RealFlight's fits put the pole from −0.2 to 0.2.
#[test]
fn a_pole_at_or_below_zero_designs_d_zero() {
    for pole in [0.0, -0.2] {
        let model = RateModel {
            rate_weights: [1.0 + pole, -pole],
            ..model(27.0, 0.03, 1)
        };
        let gains = design_rate(&model, START, GYRO_HZ, &COPTER_LOOP_SHAPING);
        check!(
            gains.is_ok_and(|gains| gains.p > 0.0 && gains.d == 0.0),
            "pole {pole}: {gains:?}"
        );
    }
}

/// A fitted integrator just past 1 designs as one held at 1, for rate and angle alike:
/// `quad_x`'s fit from a start ringing with 25 ms of delay (#518), whose unstable pole alone
/// meets no crossover; a damped fit (below 1) designs on its own weights.
#[test]
fn designs_hold_a_leaking_integrator_at_one() {
    let fitted = RateModel {
        rate_weights: [1.874_103_3, -0.873_977_84],
        moment_weights: [1.057_819e-5, 0.005_188_625_3, 0.008_271_968],
        delay_ticks: 6,
        dt_s: DT,
    };
    let held = RateModel {
        rate_weights: [1.0 + 0.873_977_84, -0.873_977_84],
        ..fitted
    };
    let rate = design_rate(&fitted, START, GYRO_HZ, &COPTER_LOOP_SHAPING);
    check!(rate.is_ok());
    check!(rate == design_rate(&held, START, GYRO_HZ, &COPTER_LOOP_SHAPING));
    let rate = rate.unwrap_or_default();
    let angle = Gains {
        p: 3.0,
        i: 0.0,
        d: 0.0,
    };
    check!(
        design_angle_p(&fitted, rate, angle, GYRO_HZ, &COPTER_LOOP_SHAPING)
            == design_angle_p(&held, rate, angle, GYRO_HZ, &COPTER_LOOP_SHAPING)
    );

    let damped = RateModel {
        rate_weights: [1.85, -0.86],
        ..fitted
    };
    check!(integrating(&damped) == damped);
}

/// Angle P stays in range (less the back-off), sits at the range's top on the designed rate
/// loop (as on the sim's copters), and falls from it as the rate loop it rides on slows: rate
/// P and D at 1, ¼, and ¹⁄₁₆ of the design.
#[test]
fn angle_p_stays_in_range_and_falls_with_a_slower_rate_loop() {
    let config = COPTER_LOOP_SHAPING;
    let (low, high) = (
        config.angle_p_min * config.angle_back_off,
        config.angle_p_max * config.angle_back_off,
    );
    // Angle P alone: core's default angle D (1.0) adds enough lead to keep the margin even
    // at the range's top over these rate loops.
    let angle = Gains {
        p: 3.0,
        i: 0.0,
        d: 0.0,
    };
    let model = model(27.0, 0.03, 1);
    let designed = design_rate(&model, START, GYRO_HZ, &config).expect("a rate design");
    let values = [1.0, 0.25, 0.0625].map(|share: f32| {
        let rate = Gains {
            p: designed.p * share,
            d: designed.d * share,
            ..designed
        };
        design_angle_p(&model, rate, angle, GYRO_HZ, &config).expect("an angle design")
    });
    check!(
        values.iter().all(|p| (low..=high).contains(p)),
        "{values:?}"
    );
    check!(values[0] == high, "{values:?}");
    check!(
        values.windows(2).all(|pair| pair[1] <= pair[0]),
        "{values:?}"
    );
    check!(values[2] < high, "{values:?}");
}

/// The angle loop's phase margin at `angle_p` around the rate loop as flown: rate P and D
/// through the gyro low-pass at `gyro_hz`, only P on the rate setpoint (D on the measured
/// rate), in degrees.
fn angle_margin_deg(
    model: &RateModel,
    rate: Gains,
    angle: Gains,
    angle_p: f32,
    gyro_hz: f32,
) -> f32 {
    let one = Complex::new(1.0, 0.0);
    let rate = Gains { i: 0.0, ..rate };
    let open_loop = |w_radps: f32| {
        let z1 = unit_delay(w_radps, DT);
        let plant = model.response(w_radps);
        let feedback = pid_response(rate, z1, DT) * plant * LowPass::response(gyro_hz, DT, z1);
        let closed_rate = plant * rate.p / (one + feedback);
        let angle_pid = pid_response(
            Gains {
                p: angle_p,
                ..angle
            },
            z1,
            DT,
        );
        angle_pid * closed_rate * Complex::new(DT, 0.0) / (one - z1)
    };
    let crossover = first_crossing(DT, open_loop, |point| point.gain > 1.0).expect("a crossover");
    180.0 + crossover.phase_deg
}

/// Where the angle margin limits it, angle P leaves the flown angle loop the margin asked
/// (before the back-off): the rate loop under it acts through the gyro low-pass, with D on the
/// measured rate (#373). Rows: rate P and D at shares of the design that put angle P below the
/// range's top, at 40 and 20 Hz.
#[test]
fn angle_design_models_the_flown_rate_loop() {
    let config = COPTER_LOOP_SHAPING;
    let angle = Gains {
        p: 3.0,
        i: 0.0,
        d: 0.0,
    };
    let model = model(27.0, 0.03, 1);
    for gyro_hz in [40.0, 20.0] {
        let designed = design_rate(&model, START, gyro_hz, &config).expect("a rate design");
        for share in [0.25, 0.0625] {
            let rate = Gains {
                p: designed.p * share,
                d: designed.d * share,
                ..designed
            };
            let angle_p = design_angle_p(&model, rate, angle, gyro_hz, &config).expect("angle P");
            let limited = angle_p / config.angle_back_off;
            check!(
                limited < config.angle_p_max,
                "{gyro_hz} Hz, share {share}: at the top"
            );
            let margin = angle_margin_deg(&model, rate, angle, limited, gyro_hz);
            check!(
                (margin - config.angle_margin_deg).abs() < 0.5,
                "{gyro_hz} Hz, share {share}: margin {margin}°"
            );
        }
    }
}

/// The angle design leaves the rate loop's I out: on a fit with a pole just outside the unit
/// circle (the delayed test axis, #374), rate I at 0.2·P flips the design between P 9 and
/// `AngleUnstable` on fits a few parts in ten thousand apart.
#[test]
fn angle_design_ignores_the_rate_i() {
    let fits = [
        (
            [2.012_705_6, -1.019_460_4],
            [0.000_749_887, 0.000_659_920, 0.000_345_026],
        ),
        (
            [2.013_502_6, -1.019_933_6],
            [0.000_749_364, 0.000_658_991, 0.000_344_228],
        ),
    ];
    let rate = Gains {
        p: 1.026,
        i: 0.0,
        d: 0.2009,
    };
    let angle = Gains {
        p: 3.0,
        i: 0.0,
        d: 0.1,
    };
    for (rate_weights, moment_weights) in fits {
        let model = RateModel {
            rate_weights,
            moment_weights,
            delay_ticks: 4,
            dt_s: DT,
        };
        let without_i = design_angle_p(&model, rate, angle, GYRO_HZ, &COPTER_LOOP_SHAPING);
        let with_i = Gains {
            i: 0.2 * rate.p,
            ..rate
        };
        check!(without_i.is_ok(), "{rate_weights:?}");
        check!(
            design_angle_p(&model, with_i, angle, GYRO_HZ, &COPTER_LOOP_SHAPING) == without_i,
            "{rate_weights:?}"
        );
    }
}

/// An angle range whose least P misses the margin is a typed failure.
#[test]
fn angle_design_fails_when_the_least_p_misses_the_margin() {
    let config = LoopShapingConfig {
        angle_p_min: 200.0,
        angle_p_max: 300.0,
        ..COPTER_LOOP_SHAPING
    };
    let model = model(27.0, 0.03, 1);
    let rate = design_rate(&model, START, GYRO_HZ, &config).expect("a rate design");
    let angle = Gains {
        p: 3.0,
        i: 0.001,
        d: 1.0,
    };
    check!(
        design_angle_p(&model, rate, angle, GYRO_HZ, &config) == Err(DesignError::AngleUnstable)
    );
}

/// Each field's requirement: (config, valid).
#[test]
fn validate_checks_each_field() {
    let c = COPTER_LOOP_SHAPING;
    let rows = [
        (c, true),
        (
            LoopShapingConfig {
                rate_margin_deg: 0.0,
                ..c
            },
            false,
        ),
        (
            LoopShapingConfig {
                rate_margin_deg: 90.0,
                ..c
            },
            false,
        ),
        (
            LoopShapingConfig {
                angle_margin_deg: f32::NAN,
                ..c
            },
            false,
        ),
        (
            LoopShapingConfig {
                angle_p_min: 0.0,
                ..c
            },
            false,
        ),
        (
            LoopShapingConfig {
                angle_p_max: 0.5,
                ..c
            },
            false,
        ),
        (
            LoopShapingConfig {
                angle_p_max: f32::INFINITY,
                ..c
            },
            false,
        ),
        (
            LoopShapingConfig {
                angle_back_off: 0.0,
                ..c
            },
            false,
        ),
        (
            LoopShapingConfig {
                angle_back_off: 1.0,
                ..c
            },
            true,
        ),
        (
            LoopShapingConfig {
                angle_back_off: 1.1,
                ..c
            },
            false,
        ),
    ];
    for (config, valid) in rows {
        check!(config.validate().is_ok() == valid, "{config:?}");
    }
}

/// The walk unwraps phase, so a lag past −360° reads as such: on `(ωc/jω)·e^(−jω·τ)` the gain
/// falls to 1 at `ωc`, where the phase is `−90° − ωc·τ` in closed form. (ωc rad/s, τ s) rows;
/// the last two lag past −360°, where a wrapped phase would read a healthy margin.
#[test]
fn crossings_read_the_unwrapped_phase() {
    let rows = [(10.0, 0.01), (10.0, 0.1), (10.0, 0.5), (20.0, 0.4)];
    for (crossover_radps, delay_s) in rows {
        let open_loop = |w_radps: f32| {
            let integrator = Complex::new(0.0, -crossover_radps / w_radps);
            let angle = -w_radps * delay_s;
            integrator * Complex::new(libm::cosf(angle), libm::sinf(angle))
        };
        let point = first_crossing(DT, open_loop, |point| point.gain > 1.0).expect("a crossing");
        let expected_deg = -90.0 - (crossover_radps * delay_s).to_degrees();
        check!(
            (point.w_radps - crossover_radps).abs() < 0.01 * crossover_radps,
            "{point:?}"
        );
        check!(
            (point.phase_deg - expected_deg).abs() < 1.0,
            "{point:?} against {expected_deg}"
        );
    }
}
