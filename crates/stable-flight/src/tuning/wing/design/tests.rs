use assert2::{assert, check};

use super::*;

/// The plane spec's targets on a 30° roll step: 5% overshoot, rise in 2.2τ, 42 ms of clipping.
const CONFIG: WingDesignConfig = WingDesignConfig {
    amplitude_rad: 0.52,
    step_duration_s: 4.0,
    max_rate_radps: 1.2,
    max_moment: 0.8,
    max_integral: 2.0,
    max_overshoot: 0.05,
    max_saturated_s: 0.042,
    time_constant_range_s: [0.2, 1.5],
    rise_per_time_constant: 2.2,
};

/// A roll axis like the test plant's wing.
const ROLL: WingModel = WingModel {
    stiffness_per_s2: 0.0,
    damping_per_s: 8.0,
    control_radps2: 40.0,
    lag_s: 0.0,
    delay_s: 0.0,
    path_turn_per_s: 0.0,
};

/// The sweep's airplane pitch fit (#533).
const PITCH: WingModel = WingModel {
    stiffness_per_s2: 15.3,
    damping_per_s: 7.05,
    control_radps2: 13.0,
    lag_s: 0.0,
    delay_s: 0.0,
    path_turn_per_s: 15.1,
};

/// Runs `design` to its end; the candidates it stepped and its status.
fn run(mut design: WingDesign) -> (usize, WingDesignStatus) {
    let mut candidates = 0;
    loop {
        let status = design.advance(1);
        if status != WingDesignStatus::Running {
            return (candidates, status);
        }
        candidates += 1;
    }
}

#[test]
fn designs_gains_whose_model_step_meets_the_targets() {
    // (row, model, config)
    let pitch_step = WingDesignConfig {
        amplitude_rad: 0.17,
        max_rate_radps: 1.0,
        ..CONFIG
    };
    let rows = [
        ("roll", ROLL, CONFIG),
        (
            "roll, lagged and late",
            WingModel {
                lag_s: 0.06,
                delay_s: 0.016,
                ..ROLL
            },
            CONFIG,
        ),
        ("airplane pitch", PITCH, pitch_step),
        (
            "eXplora pitch: stiff",
            WingModel {
                stiffness_per_s2: 566.0,
                damping_per_s: 7.2,
                control_radps2: 222.0,
                path_turn_per_s: 41.0,
                ..PITCH
            },
            pitch_step,
        ),
        (
            "Alia 250 pitch: slow, its surfaces lagged",
            WingModel {
                stiffness_per_s2: 4.49,
                damping_per_s: 0.32,
                control_radps2: 0.61,
                lag_s: 0.06,
                delay_s: 0.016,
                path_turn_per_s: 0.56,
            },
            WingDesignConfig {
                amplitude_rad: 0.087,
                ..pitch_step
            },
        ),
    ];
    for (row, model, config) in rows {
        let (candidates, status) = run(WingDesign::new(model, config));
        let WingDesignStatus::Done(gains) = status else {
            panic!("{row}: {status:?}");
        };
        check!(gains.rate.ff == model.feedforward(), "{row}");
        let metrics = model_step(&model, gains, &config);
        check!(
            metrics.is_some_and(|m| meets_targets(&m, gains.time_constant_s, &config)),
            "{row}: {gains:?} {metrics:?}"
        );
        let [least_s, most_s] = config.time_constant_range_s;
        check!((least_s..=most_s).contains(&gains.time_constant_s), "{row}");
        check!(candidates <= 10 * CELLS, "{row}: {candidates} candidates");
    }
}

#[test]
fn a_model_it_cant_fly_fails() {
    // (row, model, config)
    let rows = [
        (
            "no control",
            WingModel {
                control_radps2: 0.0,
                ..ROLL
            },
            CONFIG,
        ),
        (
            "reversed control",
            WingModel {
                control_radps2: -40.0,
                ..ROLL
            },
            CONFIG,
        ),
        (
            "no damping",
            WingModel {
                damping_per_s: 0.0,
                ..ROLL
            },
            CONFIG,
        ),
        (
            "too slow for any step to rise in time",
            WingModel {
                control_radps2: 0.5,
                lag_s: 0.3,
                delay_s: 0.04,
                ..ROLL
            },
            WingDesignConfig {
                max_moment: 0.05,
                ..CONFIG
            },
        ),
    ];
    for (row, model, config) in rows {
        check!(
            run(WingDesign::new(model, config)).1 == WingDesignStatus::Failed,
            "{row}"
        );
    }
}

#[test]
fn advances_by_the_candidates_asked() {
    let mut design = WingDesign::new(ROLL, CONFIG);
    check!(design.advance(1) == WingDesignStatus::Running);
    // The first candidate at the slowest time constant passes or moves on: still running.
    check!(design.advance(1) == WingDesignStatus::Running);
    let done = design.advance(usize::MAX);
    check!(matches!(done, WingDesignStatus::Done(_)));
    // A finished design keeps its result.
    check!(design.advance(1) == done);
}

#[test]
fn meets_targets_checks_each() {
    let good = StepMetrics {
        overshoot: 0.03,
        settling_time_s: Some(1.0),
        rise_time_s: Some(0.5),
        saturated_s: 0.0,
        ..StepMetrics::default()
    };
    // (row, metrics, meets at τ 0.4 s)
    let rows = [
        ("within every target", good, true),
        (
            "overshoots",
            StepMetrics {
                overshoot: 0.06,
                ..good
            },
            false,
        ),
        (
            "unsettled",
            StepMetrics {
                settling_time_s: None,
                ..good
            },
            false,
        ),
        (
            "slow: rise past 2.2τ",
            StepMetrics {
                rise_time_s: Some(0.9),
                ..good
            },
            false,
        ),
        (
            "never rose",
            StepMetrics {
                rise_time_s: None,
                ..good
            },
            false,
        ),
        (
            "clipped",
            StepMetrics {
                saturated_s: 0.05,
                ..good
            },
            false,
        ),
    ];
    for (row, metrics, meets) in rows {
        check!(meets_targets(&metrics, 0.4, &CONFIG) == meets, "{row}");
    }
}

#[test]
fn validate_checks_each_field() {
    let rows = [
        (
            "amplitude_rad",
            WingDesignConfig {
                amplitude_rad: 0.0,
                ..CONFIG
            },
        ),
        (
            "step_duration_s",
            WingDesignConfig {
                step_duration_s: f32::NAN,
                ..CONFIG
            },
        ),
        (
            "max_rate_radps",
            WingDesignConfig {
                max_rate_radps: -1.0,
                ..CONFIG
            },
        ),
        (
            "max_moment",
            WingDesignConfig {
                max_moment: 0.0,
                ..CONFIG
            },
        ),
        (
            "max_integral",
            WingDesignConfig {
                max_integral: 0.0,
                ..CONFIG
            },
        ),
        (
            "max_overshoot",
            WingDesignConfig {
                max_overshoot: -0.1,
                ..CONFIG
            },
        ),
        (
            "max_saturated_s",
            WingDesignConfig {
                max_saturated_s: f32::INFINITY,
                ..CONFIG
            },
        ),
        (
            "time_constant_range_s[0]",
            WingDesignConfig {
                time_constant_range_s: [0.0, 1.0],
                ..CONFIG
            },
        ),
        (
            "time_constant_range_s[1]",
            WingDesignConfig {
                time_constant_range_s: [0.5, 0.4],
                ..CONFIG
            },
        ),
        (
            "rise_per_time_constant",
            WingDesignConfig {
                rise_per_time_constant: 0.0,
                ..CONFIG
            },
        ),
    ];
    assert!(CONFIG.validate().is_ok());
    for (field, config) in rows {
        let error = config.validate().expect_err(field);
        check!(error.field == field);
    }
}
