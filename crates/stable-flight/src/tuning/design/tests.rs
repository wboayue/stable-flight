use assert2::check;

use super::*;
use crate::tuning::autotune::{COPTER_LOOP_SHAPING, SearchPlan};

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

/// The phase of `gains`' rate loop on `model` where its open-loop gain falls to 1, in degrees.
fn rate_margin_deg(model: &RateModel, gains: Gains) -> f32 {
    let open_loop = |w_radps: f32| {
        let z1 = unit_delay(w_radps, DT);
        let one = Complex::new(1.0, 0.0);
        let pd = one * gains.p + (one - z1) * (gains.d / DT);
        pd * model.response(w_radps)
    };
    let crossover = first_crossing(DT, |w| {
        let response = open_loop(w);
        (response, magnitude(response) > 1.0)
    })
    .expect("a crossover");
    180.0 + phase_deg(open_loop(crossover))
}

/// The rate design cancels the lag (D/P is the lag) and leaves the loop the margin, whatever
/// the plant: (accel rad/s² per unit moment, lag s, delay ticks). The margin holds within 0.5°;
/// I stays the start's.
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
            (margin - COPTER_LOOP_SHAPING.rate_margin_deg).abs() < 0.5,
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

/// The copter preset's angle range and back-off are the angle search's.
#[test]
fn copter_preset_uses_the_angle_searchs_range() {
    let search = SearchPlan::copter_angle();
    check!(COPTER_LOOP_SHAPING.angle_p_min == search.min);
    check!(COPTER_LOOP_SHAPING.angle_p_max == search.max);
    check!(COPTER_LOOP_SHAPING.angle_back_off == search.margin);
}

/// Phase reads in (−360°, 0]: a lag past −180° stays below it. (re, im) → degrees.
#[test]
fn phase_reads_as_a_lag() {
    let rows = [
        ((1.0, 0.0), 0.0),
        ((0.0, -1.0), -90.0),
        ((-1.0, -1e-6), -180.0),
        ((-1.0, 1e-6), -180.0),
        ((0.0, 1.0), -270.0),
    ];
    for ((re, im), deg) in rows {
        let phase = phase_deg(Complex::new(re, im));
        check!((phase - deg).abs() < 0.01, "({re}, {im}): {phase}");
    }
}
