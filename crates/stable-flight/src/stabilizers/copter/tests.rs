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
        altitude_gains: Gains {
            p: 3.0,
            i: 0.3,
            d: 0.15,
        },
        lateral_gains: Gains {
            p: 1.5,
            i: 0.15,
            d: 0.075,
        },
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

// Helper function to create a quaternion representing identity (no rotation)
fn identity_quaternion() -> Quaternion {
    //    Quaternion::from_axis_angle(&Vec3::new(0.0, 0.0, 0.0), 0.0)
    Quaternion {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }
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
fn test_altitude_controller_zero_error() {
    let mut controller = setup_controller();
    let dt = 0.01;
    let desired_altitude = 10.0;
    let current_altitude = 10.0;
    let current_velocity = 0.0;
    let attitude = identity_quaternion();

    let output = controller.altitude_controller.control(
        dt,
        desired_altitude,
        0.0,
        current_altitude,
        current_velocity,
        attitude,
    );

    // With zero error (altitude and velocity), PID terms should be 0
    // acceleration_cmd starts at 0 - GRAVITY = -9.81
    // body_z_alignment = 1 (identity quaternion), so acceleration_cmd = -9.81
    // Map to linear_thrust:
    // linear_thrust = 0.5 * (-9.81 + 9.81) / 9.81 = 0 (in old scaling)
    // In new scaling: linear_thrust = 0.5 * (-9.81 + 9.81) / 9.81 = 0
    // But we expect hover to be around 0.5 (linear), then nonlinear curve applies
    // With quadratic curve (exponent = 2.0), expect nonlinear_thrust = 0.5^2 = 0.25
    // However, since PID terms are 0, we need to ensure hover is correct via integral term or scaling

    // For simplicity, let’s assume the controller is tuned such that hover thrust is ~0.5 (linear)
    // After quadratic curve (assuming exponent = 2.0), expect 0.5^2 = 0.25
    let expected_hover_thrust = 0.25; // Adjust based on thrust_curve_exponent
    // assert!(
    //     (output - expected_hover_thrust).abs() < 1e-2,
    //     "Output should be near hover thrust when error is zero, got {}",
    //     output
    // );
}

#[test]
fn test_altitude_controller_positive_error() {
    let mut controller = setup_controller();
    let dt = 0.01;
    let desired_altitude = 11.0;
    let current_altitude = 10.0;
    let current_velocity = 0.0;
    let attitude = identity_quaternion();

    let output = controller.altitude_controller.control(
        dt,
        desired_altitude,
        0.0,
        current_altitude,
        current_velocity,
        attitude,
    );

    //   assert!(output > 0.5, "Thrust should be greater to ascend {}", output);
}

#[test]
fn test_lateral_controller_zero_error() {
    let mut stabilizer = setup_controller();
    let dt = 0.01;
    let desired_position = vec2(0.0, 0.0);
    let current_position = vec2(0.0, 0.0);
    let current_velocity = vec2(0.0, 0.0);

    let output = stabilizer.lateral_controller.control(
        dt,
        desired_position,
        current_position,
        current_velocity,
    );
    assert_eq!(
        output,
        vec2(0.0, 0.0),
        "Output should be zero when error is zero"
    );
}

#[test]
fn test_lateral_controller_positive_error() {
    let mut stabilizer = setup_controller();
    let dt = 0.01;
    let desired_position = vec2(1.0, 1.0);
    let current_position = vec2(0.0, 0.0);
    let current_velocity = vec2(0.0, 0.0);

    let output = stabilizer.lateral_controller.control(
        dt,
        desired_position,
        current_position,
        current_velocity,
    );
    assert!(output.x > 0.0, "Roll should tilt to correct y-position");
    assert!(output.y < 0.0, "Pitch should tilt to correct x-position");
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
    let manual_command = RateCommand {
        roll_rate: 0.0,
        pitch_rate: 0.0,
        yaw_rate: 0.0,
        throttle: 0.0,
        ..Default::default()
    };
    let state = VehicleState::default();

    let output = controller.process_rate_command(dt, &manual_command, &state);
    assert_eq!(
        output,
        ControlOutput::default(),
        "Output should be default for zero inputs"
    );
}

#[test]
fn test_waypoint_control() {
    let mut controller = setup_controller();
    let dt = 0.01;
    let waypoint_command = WaypointCommand {
        position: vec2(0.0, 0.0),
        ..Default::default()
    };
    let state = VehicleState::default();

    let output = controller.process_waypoint_command(dt, &waypoint_command, &state);
    assert_eq!(
        output,
        ControlOutput::default(),
        "Output should be default for zero position"
    );
}

/// Regression: a waypoint far from the vehicle made the lateral loop request pitch ≈ -4.6
/// (normalized), and the attitude controller's range `assert!` panicked the flight loop.
#[test]
fn distant_waypoint_tilts_toward_target_instead_of_panicking() {
    use robomath::vec2;

    // (waypoint x, y) relative to a vehicle at rest at the origin
    let targets = [(30.0, 0.0), (-1000.0, 500.0), (0.0, 1e6)];
    for (x, y) in targets {
        let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
        let state = VehicleState::default();
        stabilizer.prime(&state);
        let command = Command::Waypoint(WaypointCommand {
            position: vec2(x, y),
            altitude: 20.0,
            cruise_speed: 7.0,
            ..Default::default()
        });

        // First tick: tilt toward the target. In this frame a +x target gives a negative
        // pitch moment and a +y target a positive roll moment (lateral: pitch = -a_x/g,
        // roll = a_y/g).
        let first = stabilizer.stabilize(0.004, &command, &state);
        if x != 0.0 {
            assert!(
                first.moments.y.signum() == -x.signum(),
                "waypoint ({x}, {y}): pitch moment {} doesn't point toward it",
                first.moments.y
            );
        }
        if y != 0.0 {
            assert!(
                first.moments.x.signum() == y.signum(),
                "waypoint ({x}, {y}): roll moment {} doesn't point toward it",
                first.moments.x
            );
        }

        // Many ticks, so the lateral integral would wind up if it weren't clamped.
        for _ in 0..1_000 {
            let output = stabilizer.stabilize(0.004, &command, &state);
            assert!(
                output.thrust.is_finite()
                    && output.moments.x.is_finite()
                    && output.moments.y.is_finite()
                    && output.moments.z.is_finite(),
                "waypoint ({x}, {y}) produced non-finite output {output:?}"
            );
        }
    }
}

/// Regression: in attitude mode the heading-hold target grew by `yaw * 0.0001` every tick
/// without wrapping, so ~63k ticks of full yaw stick (≈4 min at 250 Hz) pushed it past 2π
/// and the yaw controller's range `assert!` panicked the flight loop.
#[test]
fn sustained_yaw_input_keeps_heading_target_wrapped() {
    // (yaw stick, ticks) — enough ticks to pass ±2π several times over if unwrapped
    let cases = [(1.0, 200_000), (-1.0, 200_000)];
    for (yaw, ticks) in cases {
        let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
        let state = VehicleState::default();
        stabilizer.prime(&state);
        let command = Command::Attitude(AttitudeCommand {
            throttle: 0.5,
            yaw,
            arm: true,
            ..Default::default()
        });

        for _ in 0..ticks {
            let output = stabilizer.stabilize(0.004, &command, &state);
            assert!(output.moments.z.is_finite());
        }

        assert!(
            (-PI..PI).contains(&stabilizer.heading),
            "yaw {yaw}: heading target {} escaped [-π, π)",
            stabilizer.heading
        );
    }
}

/// Headings from an estimator can be in [0, 2π) (RealFlight's azimuth); the wrapped target
/// must behave exactly like its [-π, π) equivalent, with no jump on the first tick.
#[test]
fn heading_primed_above_pi_behaves_like_its_wrapped_equivalent() {
    let command = Command::Attitude(AttitudeCommand {
        throttle: 0.5,
        yaw: 1.0,
        arm: true,
        ..Default::default()
    });
    // (primed heading, equivalent heading in [-π, π))
    let cases = [
        (5.0, 5.0 - 2.0 * PI),
        (2.0 * PI - 1e-3, -1e-3),
        (3.5, 3.5 - 2.0 * PI),
    ];
    for (heading, equivalent) in cases {
        let run = |primed: f32| {
            let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
            let state = VehicleState {
                heading: primed,
                ..Default::default()
            };
            stabilizer.prime(&state);
            (0..10)
                .map(|_| stabilizer.stabilize(0.004, &command, &state).moments.z)
                .collect::<Vec<f32>>()
        };
        // The same physical heading differs in the last float bit between the two
        // representations (ulp near 5.0 ≈ 4.8e-7); the yaw D term divides that by dt, so allow
        // ~1e-3 of the ±1 moment range. A real wrap bug would be off by ~2π of error.
        for (a, b) in run(heading).into_iter().zip(run(equivalent)) {
            assert!(
                (a - b).abs() < 1e-3,
                "heading {heading}: yaw moment {a} vs {b}"
            );
        }
    }
}

/// A NaN heading at priming must not disable heading hold for good.
#[test]
fn nan_primed_heading_relatches_from_a_valid_estimate() {
    let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
    stabilizer.prime(&VehicleState {
        heading: f32::NAN,
        ..Default::default()
    });
    let command = Command::Attitude(AttitudeCommand {
        throttle: 0.5,
        arm: true,
        ..Default::default()
    });
    let valid = VehicleState {
        heading: 5.0,
        ..Default::default()
    };

    stabilizer.stabilize(0.004, &command, &valid);

    assert!(
        (stabilizer.heading - (5.0 - 2.0 * PI)).abs() < 1e-5,
        "heading target {} did not re-latch",
        stabilizer.heading
    );
}

/// Regression: out-of-range stick input (e.g. the Spektrum commander's unclamped channels) hit
/// `process_attitude_command`'s range `assert!`s and the mixer's, panicking the loop. Each row
/// must now stabilize exactly like its saturated equivalent.
#[test]
fn out_of_range_or_nan_commands_saturate_instead_of_panicking() {
    let nan = f32::NAN;
    // (throttle, roll, pitch, yaw, equivalent in range)
    let attitude_cases = [
        (1.2, 0.0, 0.0, 0.0, (1.0, 0.0, 0.0, 0.0)),
        (-0.1, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0, 0.0)),
        (0.5, 1.21, -1.3, 0.0, (0.5, 1.0, -1.0, 0.0)),
        (0.5, 0.0, 0.0, 2.0, (0.5, 0.0, 0.0, 1.0)),
        (nan, nan, 0.2, nan, (0.0, 0.0, 0.2, 0.0)),
    ];
    let state = VehicleState::default();
    for (throttle, roll, pitch, yaw, (t, r, p, y)) in attitude_cases {
        let command = |throttle, roll, pitch, yaw| {
            Command::Attitude(AttitudeCommand {
                throttle,
                roll,
                pitch,
                yaw,
                arm: true,
                mode: 0,
            })
        };
        let mut raw = CopterStabilizer::new(CopterStabilizerConfig::default());
        let mut saturated = CopterStabilizer::new(CopterStabilizerConfig::default());
        raw.prime(&state);
        saturated.prime(&state);
        for _ in 0..5 {
            let a = raw.stabilize(0.004, &command(throttle, roll, pitch, yaw), &state);
            let b = saturated.stabilize(0.004, &command(t, r, p, y), &state);
            assert!(
                a == b,
                "({throttle}, {roll}, {pitch}, {yaw}) gave {a:?}, expected {b:?}"
            );
        }
    }

    // Rate mode: throttle was never clamped and reached the mixer's assert.
    for throttle in [1.5, -0.3, nan] {
        let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
        let output = stabilizer.stabilize(
            0.004,
            &Command::Rate(RateCommand {
                throttle,
                arm: true,
                ..Default::default()
            }),
            &state,
        );
        assert!(
            (0.0..=1.0).contains(&output.thrust),
            "rate throttle {throttle} gave {output:?}"
        );
    }
}

/// A NaN attitude-rate estimate yields zero moments instead of panicking the body-rate loop.
#[test]
fn nan_rate_estimate_yields_zero_moments() {
    let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
    let state = VehicleState {
        angular_velocity: vec3(f32::NAN, 0.0, 0.0),
        ..Default::default()
    };
    stabilizer.prime(&state);
    let output = stabilizer.stabilize(
        0.004,
        &Command::Attitude(AttitudeCommand {
            throttle: 0.5,
            roll: 0.3,
            arm: true,
            ..Default::default()
        }),
        &state,
    );
    assert!(output.moments == vec3(0.0, 0.0, 0.0), "gave {output:?}");
}
