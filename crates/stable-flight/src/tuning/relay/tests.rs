use assert2::{assert, check};

use super::*;
use crate::tuning::plant::{Plant, PlantParams};

/// The flight loop's tick: 250 Hz.
const DT: f32 = 0.004;

/// Relative error allowed between the measured and exact ultimate point. The describing
/// function reads the error cycle's peak, which overstates its fundamental (the cycle is
/// closer to a triangle than a sine), so `Ku` reads low and `Tu` high: 3–12% on these plants.
const RECOVERY_TOLERANCE: f32 = 0.15;

/// How far `Ku` may read high, or `Tu` low: the bias above runs the other way, so a result
/// past this would be an unconservative error, not the approximation.
const UNCONSERVATIVE_TOLERANCE: f32 = 0.02;

/// Checks a measured point against the exact one: within [`RECOVERY_TOLERANCE`], and on the
/// conservative side (lower `Ku`, longer `Tu`) within [`UNCONSERVATIVE_TOLERANCE`].
fn check_recovered(name: &str, result: &RelayResult, (ku, tu_s): (f32, f32)) {
    let point = &result.point;
    let ku_error = point.ku / ku - 1.0;
    let tu_error = point.tu_s / tu_s - 1.0;
    check!(
        (-RECOVERY_TOLERANCE..UNCONSERVATIVE_TOLERANCE).contains(&ku_error),
        "{name}: Ku {} vs exact {ku}",
        point.ku
    );
    check!(
        (-UNCONSERVATIVE_TOLERANCE..RECOVERY_TOLERANCE).contains(&tu_error),
        "{name}: Tu {} vs exact {tu_s}",
        point.tu_s
    );
}

/// A config for the rate-axis plant (relay in normalized moment, error in rad/s).
fn config() -> RelayConfig {
    RelayConfig {
        initial_relay: 0.05,
        max_relay: 0.4,
        relay_step: 1.5,
        hysteresis: 0.002,
        min_cycle_amplitude: 0.05,
        max_cycle_amplitude: 1.0,
        max_half_cycle_s: 0.3,
        max_bias: 0.3,
        cycles: 4,
        tolerance: 0.1,
        timeout_s: 30.0,
    }
}

/// Runs `tuner` against `plant` (zero setpoint) until it finishes or `max_s` passes.
fn run(tuner: &mut RelayTuner, plant: &mut Plant, max_s: f32) -> RelayStatus {
    let ticks = (max_s / DT) as usize;
    for _ in 0..ticks {
        let error = -plant.measure();
        let output = tuner.step(DT, error);
        plant.step(output);
        if !matches!(tuner.status(), RelayStatus::Running { .. }) {
            break;
        }
    }
    tuner.status()
}

fn done(status: RelayStatus) -> RelayResult {
    match status {
        RelayStatus::Done(result) => result,
        other => panic!("expected Done, got {other:?}"),
    }
}

/// Recovers the plant's exact ultimate point within the describing-function tolerance, erring
/// conservative, with dead time, a steady offset, and measurement noise below the hysteresis
/// band.
#[test]
fn recovers_the_ultimate_point() {
    let base = PlantParams::rate_axis();
    // (name, plant)
    let cases = [
        ("rate axis", base),
        (
            "slower motors",
            PlantParams {
                lag1_s: 0.04,
                ..base
            },
        ),
        ("higher authority", PlantParams { gain: 60.0, ..base }),
        (
            "20 ms link delay",
            PlantParams {
                delay_s: 0.02,
                ..base
            },
        ),
        (
            "steady offset",
            PlantParams {
                offset: 0.08,
                ..base
            },
        ),
        (
            "noise inside the band",
            PlantParams {
                noise: 0.004,
                ..base
            },
        ),
    ];
    for (name, params) in cases {
        let mut plant = Plant::new(params, DT);
        let exact = plant.ultimate_point();
        let mut tuner = RelayTuner::new(config(), 0.0);

        let result = done(run(&mut tuner, &mut plant, 30.0));

        check_recovered(name, &result, exact);
    }
}

/// From a relay too small to reach `min_cycle_amplitude`, the relay grows until the cycle
/// does, then the run converges.
#[test]
fn ramps_the_relay_until_the_cycle_is_large_enough() {
    let mut plant = Plant::new(PlantParams::rate_axis(), DT);
    let config = RelayConfig {
        initial_relay: 0.005,
        ..config()
    };
    let mut tuner = RelayTuner::new(config, 0.0);

    let result = done(run(&mut tuner, &mut plant, 30.0));

    check!(result.relay > config.initial_relay);
    check!(result.relay <= config.max_relay);
    check!(result.cycle_amplitude >= config.min_cycle_amplitude);
}

/// `Ku = 4h / (π·√(a² − ε²))`, and nothing for a cycle inside the band.
#[test]
fn ultimate_gain_follows_the_describing_function() {
    // (relay, amplitude, hysteresis, expected)
    let cases = [
        (0.1, 0.05, 0.0, Some(0.4 / (PI * 0.05))),
        (0.1, 0.05, 0.03, Some(0.4 / (PI * 0.04))),
        (0.2, 0.5, 0.3, Some(0.8 / (PI * 0.4))),
        (0.1, 0.03, 0.03, None),
        (0.1, 0.02, 0.03, None),
    ];
    for (relay, amplitude, hysteresis, expected) in cases {
        let actual = ultimate_gain(relay, amplitude, hysteresis);
        match (actual, expected) {
            (Some(actual), Some(expected)) => {
                approx::assert_relative_eq!(actual, expected, max_relative = 1e-5);
            }
            _ => assert!(actual == expected),
        }
    }
}

/// The bias converges to the trim that cancels a steady input offset (in either direction,
/// and from a wrong seed), so the half-cycles even out.
#[test]
fn bias_trims_a_steady_offset() {
    // (offset, initial bias)
    let cases = [(0.08, 0.0), (-0.08, 0.0), (0.08, -0.1), (0.0, 0.1)];
    for (offset, initial_bias) in cases {
        let params = PlantParams {
            offset,
            ..PlantParams::rate_axis()
        };
        let mut plant = Plant::new(params, DT);
        let mut tuner = RelayTuner::new(config(), initial_bias);

        let result = done(run(&mut tuner, &mut plant, 30.0));

        check!(
            (result.bias + offset).abs() < 0.01,
            "offset {offset}, seed {initial_bias}: bias {}",
            result.bias
        );
    }
}

/// The bias stays within `max_bias`, from the seed on.
#[test]
fn bias_is_clamped_to_max_bias() {
    let tuner = RelayTuner::new(config(), 5.0);
    assert!(tuner.bias == config().max_bias);
    let tuner = RelayTuner::new(config(), f32::NAN);
    assert!(tuner.bias == 0.0);
    let tuner = RelayTuner::new(config(), f32::NEG_INFINITY);
    assert!(tuner.bias == -config().max_bias);

    let params = PlantParams {
        offset: 0.5, // more than max_bias can cancel
        ..PlantParams::rate_axis()
    };
    let mut plant = Plant::new(params, DT);
    let mut tuner = RelayTuner::new(config(), 0.0);
    run(&mut tuner, &mut plant, 30.0);
    assert!(tuner.bias.abs() <= config().max_bias);
}

/// Aborts when the error never crosses the band (a plant that doesn't respond), and when the
/// forced cycle passes `max_cycle_amplitude`; then the output is the bias alone.
#[test]
fn aborts_on_timeout_and_on_a_cycle_too_large() {
    // (name, plant, config, expected)
    let cases = [
        (
            "no response",
            PlantParams {
                gain: 0.0,
                ..PlantParams::rate_axis()
            },
            RelayConfig {
                timeout_s: 2.0,
                ..config()
            },
            RelayAbort::Timeout,
        ),
        (
            "cycle too large",
            PlantParams::rate_axis(),
            RelayConfig {
                initial_relay: 0.4,
                max_cycle_amplitude: 0.06,
                ..config()
            },
            RelayAbort::CycleTooLarge,
        ),
    ];
    for (name, params, config, expected) in cases {
        let mut plant = Plant::new(params, DT);
        let mut tuner = RelayTuner::new(config, 0.02);

        let status = run(&mut tuner, &mut plant, 10.0);

        check!(status == RelayStatus::Aborted(expected), "{name}");
        check!(tuner.step(DT, 1.0) == tuner.bias, "{name}");
    }
}

/// Once done, the output is the bias alone and the result holds.
#[test]
fn outputs_the_bias_once_done() {
    let mut plant = Plant::new(PlantParams::rate_axis(), DT);
    let mut tuner = RelayTuner::new(config(), 0.0);
    let result = done(run(&mut tuner, &mut plant, 30.0));

    assert!(tuner.step(DT, 1.0) == result.bias);
    assert!(tuner.step(DT, -1.0) == result.bias);
    assert!(tuner.status() == RelayStatus::Done(result));
}

/// Non-finite errors mid-run hold the relay's side and stay out of the measurement: the
/// result matches the exact point and the output stays finite.
#[test]
fn non_finite_errors_do_not_poison_the_run() {
    let mut plant = Plant::new(PlantParams::rate_axis(), DT);
    let exact = plant.ultimate_point();
    let mut tuner = RelayTuner::new(config(), 0.0);

    for tick in 0..(30.0 / DT) as usize {
        let error = match tick % 50 {
            0 => f32::NAN,
            25 => f32::INFINITY,
            _ => -plant.measure(),
        };
        let output = tuner.step(DT, error);
        assert!(output.is_finite(), "tick {tick}");
        plant.step(output);
        if !matches!(tuner.status(), RelayStatus::Running { .. }) {
            break;
        }
    }

    check_recovered("non-finite errors", &done(tuner.status()), exact);
}

/// Before the first finite error the relay has no side: the output is the bias.
#[test]
fn outputs_the_bias_until_the_first_finite_error() {
    let mut tuner = RelayTuner::new(config(), 0.02);
    assert!(tuner.step(DT, f32::NAN) == 0.02);
    assert!(tuner.step(DT, 0.001) == 0.02 + config().initial_relay);
}

#[test]
#[should_panic(expected = "dt must be positive and finite")]
fn zero_dt_panics() {
    RelayTuner::new(config(), 0.0).step(0.0, 0.1);
}

#[test]
#[should_panic(expected = "invalid RelayConfig")]
fn invalid_config_panics() {
    RelayTuner::new(
        RelayConfig {
            cycles: 1,
            ..config()
        },
        0.0,
    );
}

/// One row per requirement in `validate`.
#[test]
fn validate_rejects_each_broken_field() {
    let base = config();
    // (field, broken config)
    let cases = [
        (
            "initial_relay",
            RelayConfig {
                initial_relay: 0.0,
                ..base
            },
        ),
        (
            "max_relay",
            RelayConfig {
                max_relay: 0.01,
                ..base
            },
        ),
        (
            "max_relay",
            RelayConfig {
                max_relay: f32::INFINITY,
                ..base
            },
        ),
        (
            "relay_step",
            RelayConfig {
                relay_step: 1.0,
                ..base
            },
        ),
        (
            "hysteresis",
            RelayConfig {
                hysteresis: -0.1,
                ..base
            },
        ),
        (
            "min_cycle_amplitude",
            RelayConfig {
                min_cycle_amplitude: 0.002,
                ..base
            },
        ),
        (
            "max_cycle_amplitude",
            RelayConfig {
                max_cycle_amplitude: 0.05,
                ..base
            },
        ),
        (
            "max_half_cycle_s",
            RelayConfig {
                max_half_cycle_s: 0.0,
                ..base
            },
        ),
        (
            "max_bias",
            RelayConfig {
                max_bias: f32::NAN,
                ..base
            },
        ),
        (
            "cycles",
            RelayConfig {
                cycles: MAX_CYCLES + 1,
                ..base
            },
        ),
        (
            "tolerance",
            RelayConfig {
                tolerance: 0.0,
                ..base
            },
        ),
        (
            "timeout_s",
            RelayConfig {
                timeout_s: f32::INFINITY,
                ..base
            },
        ),
    ];
    assert!(base.validate().is_ok());
    for (field, broken) in cases {
        let error = broken.validate().expect_err(field);
        check!(error.field == field);
    }
}
