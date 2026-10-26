use assert2::check;

use super::*;
use crate::tuning::autotune::COPTER_LOOP_SHAPING;

/// The flight loop's tick: 250 Hz.
const DT: f32 = 0.004;

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

/// `gains`' rate loop's phase margin on `model`, in degrees: 180° plus its phase where its
/// open-loop gain falls to 1.
fn rate_margin_deg(model: &RateModel, gains: Gains) -> f32 {
    let open_loop =
        |w_radps: f32| pid_response(gains, unit_delay(w_radps, DT), DT) * model.response(w_radps);
    let crossover = first_crossing(DT, open_loop, |point| point.gain > 1.0).expect("a crossover");
    180.0 + crossover.phase_deg
}

/// The rate design cancels the lag (D/P is the lag) and leaves the loop the margin, whatever
/// the plant: (accel rad/s² per unit moment, lag s, delay ticks). The flown loop, with the
/// start's I that the design keeps but doesn't shape, holds the margin within 1° (the I costs
/// 0.6° at seven ticks of delay).
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
        let gains = design_rate(&model, START, &COPTER_LOOP_SHAPING).expect("a design");
        check!(
            (gains.d / gains.p - lag_s).abs() < 1e-4 * lag_s,
            "{row}: {gains:?}"
        );
        check!(gains.i == START.i, "{row}");
        let margin = rate_margin_deg(&model, gains);
        check!(
            (margin - COPTER_LOOP_SHAPING.rate_margin_deg).abs() < 1.0,
            "{row}: margin {margin}°"
        );
    }
}

/// More delay leaves less phase to spend, so rate P falls with each tick of it.
#[test]
fn rate_p_falls_as_the_delay_grows() {
    let gains: Vec<f32> = (1..=7)
        .map(|delay| {
            design_rate(&model(27.0, 0.03, delay), START, &COPTER_LOOP_SHAPING)
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
        (with_pole(0.0), DesignError::NotALag),
        (with_pole(-0.2), DesignError::NotALag),
        (with_pole(1.0), DesignError::NotALag),
        (with_pole(1.3), DesignError::NotALag),
        // A moment that turns the rate the wrong way: the phase starts past the margin.
        (model(-27.0, 0.03, 1), DesignError::NoCrossover),
    ];
    for (model, error) in rows {
        check!(
            design_rate(&model, START, &COPTER_LOOP_SHAPING) == Err(error),
            "{model:?}"
        );
    }
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
    let designed = design_rate(&model, START, &config).expect("a rate design");
    let values = [1.0, 0.25, 0.0625].map(|share: f32| {
        let rate = Gains {
            p: designed.p * share,
            d: designed.d * share,
            ..designed
        };
        design_angle_p(&model, rate, angle, &config).expect("an angle design")
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

/// An angle range whose least P misses the margin is a typed failure.
#[test]
fn angle_design_fails_when_the_least_p_misses_the_margin() {
    let config = LoopShapingConfig {
        angle_p_min: 200.0,
        angle_p_max: 300.0,
        ..COPTER_LOOP_SHAPING
    };
    let model = model(27.0, 0.03, 1);
    let rate = design_rate(&model, START, &config).expect("a rate design");
    let angle = Gains {
        p: 3.0,
        i: 0.001,
        d: 1.0,
    };
    check!(design_angle_p(&model, rate, angle, &config) == Err(DesignError::AngleUnstable));
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
