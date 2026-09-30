use robomath::{Quaternion, vec3};

use super::*;
use std::f32::consts::PI;

// Helper function to create a BasicController with default gains
fn setup_controller() -> CopterStabilizer {
    let rates_controller = BodyRateController::with_config(&BodyRateControllerConfig {
        gains: Gains {
            p: 1.0,
            i: 0.1,
            d: 0.05,
        },
        max_integral: 100.0,
        max_angular_rates: vec3(3.0, 3.0, 3.0), // Max roll, pitch, yaw rates in rad/s
    });

    let attitude_controller = AttitudeController::with_config(&AttitudeControllerConfig {
        gains: Gains {
            p: 2.0,
            i: 0.2,
            d: 0.1,
        },
        max_integral: 100.0,
        max_angles: vec2(PI / 4.0, PI / 4.0), // 45° for roll and pitch
        max_rate: vec2(3.0, 3.0),             // Max roll, pitch, yaw rates in rad/s
    });

    let config = CopterStabilizerConfig {
        rates_controller,
        attitude_controller,
        yaw_gains: Gains {
            p: 1.2,
            i: 0.12,
            d: 0.06,
        },
        thrust_curve_exponent: 2.0,
        //        max_rates: vec3(1.0, 1.0, 1.0), // Max roll, pitch, yaw rates
    };
    CopterStabilizer::new(config)
}

#[test]
fn test_rate_controller_zero_error() {
    let mut controller = setup_controller();
    let dt = 0.01;
    let desired_rates = vec3(0.0, 0.0, 0.0);
    let current_rates = vec3(0.0, 0.0, 0.0);

    let output = controller
        .rates_controller
        .control(dt, desired_rates, current_rates);

    // With zero error (desired and current rates are the same), PID terms should be 0
    assert_eq!(
        output,
        vec3(0.0, 0.0, 0.0),
        "Output should be zero when error is zero"
    );
}

#[test]
fn test_rate_controller_positive_error() {
    let mut controller = setup_controller();
    let dt = 0.01;
    let desired_rates = vec3(1.0, 1.0, 1.0);
    let current_rates = vec3(0.0, 0.0, 0.0);

    let output = controller
        .rates_controller
        .control(dt, desired_rates, current_rates);

    // Check that the output is positive for all axes when there is a positive error
    assert!(output.x > 0.0, "Roll moment should be positive");
    assert!(output.y > 0.0, "Pitch moment should be positive");
    assert!(output.z > 0.0, "Yaw moment should be positive");
}

#[test]
fn test_yaw_controller_zero_error() {
    let mut controller = setup_controller();
    let dt = 0.01;
    let desired_yaw = 0.0;
    let current_yaw = 0.0;

    let output = controller
        .yaw_controller
        .control(dt, desired_yaw, current_yaw);
    assert_eq!(output, 0.0, "Yaw rate should be zero when error is zero");
}

#[test]
fn test_yaw_controller_angle_wrapping() {
    let mut controller = setup_controller();
    let dt = 0.01;
    let desired_yaw = PI + 0.1;
    let current_yaw = -PI + 0.1;

    let output = controller
        .yaw_controller
        .control(dt, desired_yaw, current_yaw);
    assert!(
        output < 0.0,
        "Yaw rate should be negative for shortest path"
    );
}

#[test]
fn test_manual_control() {
    let mut controller = setup_controller();
    let dt = 0.01;
    let manual_setpoint = Setpoint::Rate(RateSetpoint {
        roll_rate: 0.0,
        pitch_rate: 0.0,
        yaw_rate: 0.0,
        thrust: 0.0,
    });
    let state = VehicleState::default();

    let output = controller.stabilize(dt, &manual_setpoint, &state);
    assert_eq!(
        output,
        ControlOutput::default(),
        "Output should be default for zero inputs"
    );
}

/// Out-of-range or NaN setpoint components stabilize exactly like their saturated
/// equivalents (the input policy), so a misbehaving guidance can't panic the loop.
#[test]
fn out_of_range_or_nan_setpoints_saturate_instead_of_panicking() {
    let nan = f32::NAN;
    // (thrust, roll, pitch, equivalent in range)
    let attitude_cases = [
        (1.2, 0.0, 0.0, (1.0, 0.0, 0.0)),
        (-0.1, 0.0, 0.0, (0.0, 0.0, 0.0)),
        (0.5, 1.21, -1.3, (0.5, 1.0, -1.0)),
        (nan, nan, 0.2, (0.0, 0.0, 0.2)),
    ];
    let state = VehicleState::default();
    for (thrust, roll, pitch, (t, r, p)) in attitude_cases {
        let setpoint = |thrust, roll, pitch| {
            Setpoint::Attitude(AttitudeSetpoint {
                roll,
                pitch,
                thrust,
                ..Default::default()
            })
        };
        let mut raw = CopterStabilizer::new(CopterStabilizerConfig::default());
        let mut saturated = CopterStabilizer::new(CopterStabilizerConfig::default());
        for _ in 0..5 {
            let a = raw.stabilize(0.004, &setpoint(thrust, roll, pitch), &state);
            let b = saturated.stabilize(0.004, &setpoint(t, r, p), &state);
            assert!(
                a == b,
                "({thrust}, {roll}, {pitch}) gave {a:?}, expected {b:?}"
            );
        }
    }

    for thrust in [1.5, -0.3, nan] {
        let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
        let output = stabilizer.stabilize(
            0.004,
            &Setpoint::Rate(RateSetpoint {
                thrust,
                ..Default::default()
            }),
            &state,
        );
        assert!(
            (0.0..=1.0).contains(&output.thrust),
            "rate thrust {thrust} gave {output:?}"
        );
    }
}

/// The attitude reference picks which estimate is tracked: identical when the Euler fields
/// agree with the quaternion (frame contract), and each follows its own source when they
/// don't (RealFlight today).
#[test]
fn attitude_reference_selects_the_tracked_estimate() {
    let (roll, pitch, heading) = (0.2, -0.1, 0.3);
    let consistent = VehicleState {
        attitude: Quaternion::from_euler(heading, pitch, roll),
        roll,
        pitch,
        heading,
        ..Default::default()
    };
    // Euler fields level and north; quaternion as above.
    let quaternion_only = VehicleState {
        roll: 0.0,
        pitch: 0.0,
        heading: 0.0,
        ..consistent.clone()
    };
    let setpoint = |reference| {
        Setpoint::Attitude(AttitudeSetpoint {
            thrust: 0.5,
            reference,
            ..Default::default()
        })
    };
    let run = |reference, state: &VehicleState| {
        CopterStabilizer::new(CopterStabilizerConfig::default()).stabilize(
            0.004,
            &setpoint(reference),
            state,
        )
    };

    let euler = run(AttitudeReference::Euler, &consistent);
    let quaternion = run(AttitudeReference::Quaternion, &consistent);
    for (a, b) in [
        (euler.moments.x, quaternion.moments.x),
        (euler.moments.y, quaternion.moments.y),
        (euler.moments.z, quaternion.moments.z),
    ] {
        approx::assert_relative_eq!(a, b, epsilon = 1e-4);
    }
    assert!(
        euler.moments != vec3(0.0, 0.0, 0.0),
        "tilted state must be corrected"
    );

    // Level Euler fields: nothing to correct. Tilted quaternion: corrects as before.
    let level = run(AttitudeReference::Euler, &quaternion_only);
    assert!(level.moments == vec3(0.0, 0.0, 0.0), "gave {level:?}");
    let tilted = run(AttitudeReference::Quaternion, &quaternion_only);
    approx::assert_relative_eq!(tilted.moments.x, quaternion.moments.x, epsilon = 1e-4);
}

/// A NaN attitude-rate estimate yields zero moments instead of panicking the body-rate loop.
#[test]
fn nan_rate_estimate_yields_zero_moments() {
    let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
    let state = VehicleState {
        angular_velocity: vec3(f32::NAN, 0.0, 0.0),
        ..Default::default()
    };
    let output = stabilizer.stabilize(
        0.004,
        &Setpoint::Attitude(AttitudeSetpoint {
            thrust: 0.5,
            roll: 0.3,
            ..Default::default()
        }),
        &state,
    );
    assert!(output.moments == vec3(0.0, 0.0, 0.0), "gave {output:?}");
}

/// A NaN rate setpoint means "hold zero rate" (damping), not "no moments".
#[test]
fn nan_rate_setpoint_holds_zero_rate() {
    let state = VehicleState {
        angular_velocity: vec3(0.5, -0.2, 0.1), // spinning: zero-rate hold must push back
        ..Default::default()
    };
    let rate = |roll_rate| {
        Setpoint::Rate(RateSetpoint {
            thrust: 0.5,
            roll_rate,
            ..Default::default()
        })
    };
    let mut with_nan = CopterStabilizer::new(CopterStabilizerConfig::default());
    let mut with_zero = CopterStabilizer::new(CopterStabilizerConfig::default());
    let a = with_nan.stabilize(0.004, &rate(f32::NAN), &state);
    let b = with_zero.stabilize(0.004, &rate(0.0), &state);
    assert!(
        a == b,
        "NaN rate setpoint gave {a:?}, zero-rate hold gives {b:?}"
    );
    assert!(a.moments != vec3(0.0, 0.0, 0.0));
}

#[test]
fn thrust_curve_raises_linear_thrust_to_the_configured_exponent() {
    // (linear thrust, exponent, expected)
    let cases = [
        (0.0, 2.0, 0.0),
        (1.0, 2.0, 1.0),
        (0.5, 2.0, 0.25),
        (0.5, 1.5, 0.353_553_4),
        (0.5, 1.0, 0.5),
    ];
    for (linear_thrust, exponent, expected) in cases {
        let stabilizer = CopterStabilizer::new(CopterStabilizerConfig {
            thrust_curve_exponent: exponent,
            ..Default::default()
        });
        approx::assert_relative_eq!(
            stabilizer.fit_to_thrust_curve(linear_thrust),
            expected,
            epsilon = 1e-6
        );
    }
    // Out of range saturates; NaN means motors off (input policy).
    let stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
    for (linear_thrust, expected) in [(f32::NAN, 0.0), (-0.5, 0.0), (1.5, 1.0)] {
        assert_eq!(stabilizer.fit_to_thrust_curve(linear_thrust), expected);
    }
}
