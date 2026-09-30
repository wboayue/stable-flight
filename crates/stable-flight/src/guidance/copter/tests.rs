use robomath::{Quaternion, vec2};

use super::*;
use crate::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use crate::{ControlOutput, RateCommand, Stabilizer};
use std::f32::consts::PI;

/// Guidance and stabilizer run back to back, as `FlightLoop` does, for end-to-end checks of
/// the whole copter controller.
struct Stack {
    guidance: CopterGuidance,
    stabilizer: CopterStabilizer,
}

impl Stack {
    fn new() -> Self {
        Stack {
            guidance: CopterGuidance::new(CopterGuidanceConfig::default()),
            stabilizer: CopterStabilizer::new(CopterStabilizerConfig::default()),
        }
    }

    fn prime(&mut self, state: &VehicleState) {
        self.guidance.prime(state);
        self.stabilizer.prime(state);
    }

    fn stabilize(&mut self, dt: f32, command: &Command, state: &VehicleState) -> ControlOutput {
        let setpoint = self.guidance.guide(dt, command, state);
        self.stabilizer.stabilize(dt, &setpoint, state)
    }
}

fn setup_guidance() -> CopterGuidance {
    CopterGuidance::new(CopterGuidanceConfig {
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
    })
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
fn test_altitude_controller_zero_error() {
    let mut controller = setup_guidance();
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
    let mut controller = setup_guidance();
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
    let mut guidance = setup_guidance();
    let dt = 0.01;
    let desired_position = vec2(0.0, 0.0);
    let current_position = vec2(0.0, 0.0);
    let current_velocity = vec2(0.0, 0.0);

    let output = guidance.lateral_controller.control(
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
    let mut guidance = setup_guidance();
    let dt = 0.01;
    let desired_position = vec2(1.0, 1.0);
    let current_position = vec2(0.0, 0.0);
    let current_velocity = vec2(0.0, 0.0);

    let output = guidance.lateral_controller.control(
        dt,
        desired_position,
        current_position,
        current_velocity,
    );
    assert!(output.x > 0.0, "Roll should tilt to correct y-position");
    assert!(output.y < 0.0, "Pitch should tilt to correct x-position");
}

/// Each command kind maps to the setpoint the stabilizer tracks.
#[test]
fn commands_map_to_setpoints() {
    let state = VehicleState {
        heading: 0.7,
        ..Default::default()
    };
    // (command, expected setpoint), one fresh guidance primed at heading 0.7 per row
    let cases = [
        (
            Command::Rate(RateCommand {
                throttle: 0.6,
                roll_rate: 0.1,
                pitch_rate: -0.2,
                yaw_rate: 0.3,
                ..Default::default()
            }),
            Setpoint::Rate(RateSetpoint {
                roll_rate: 0.1,
                pitch_rate: -0.2,
                yaw_rate: 0.3,
                thrust: 0.6,
            }),
        ),
        (
            Command::Attitude(AttitudeCommand {
                throttle: 0.4,
                roll: 0.5,
                pitch: -0.25,
                ..Default::default()
            }),
            Setpoint::Attitude(AttitudeSetpoint {
                roll: 0.5,
                pitch: -0.25,
                heading: wrap_angle(0.7),
                thrust: 0.4,
                reference: AttitudeReference::Euler,
            }),
        ),
        (
            Command::Attitude(AttitudeCommand {
                yaw: 1.0,
                ..Default::default()
            }),
            Setpoint::Attitude(AttitudeSetpoint {
                heading: wrap_angle(0.7 + HEADING_STEP_PER_TICK),
                ..Default::default()
            }),
        ),
        (
            // At the waypoint, at rest, level: no tilt, north. The altitude loop has no
            // gravity feed-forward, so zero error means zero linear thrust.
            Command::Waypoint(WaypointCommand::default()),
            Setpoint::Attitude(AttitudeSetpoint {
                roll: 0.0,
                pitch: 0.0,
                heading: 0.0,
                thrust: 0.0,
                reference: AttitudeReference::Quaternion,
            }),
        ),
    ];
    for (command, expected) in cases {
        let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
        guidance.prime(&state);
        let setpoint = guidance.guide(0.004, &command, &state);
        assert!(setpoint == expected, "{command:?} gave {setpoint:?}");
    }
}

#[test]
fn test_waypoint_control() {
    let mut controller = Stack::new();
    let dt = 0.01;
    let waypoint_command = WaypointCommand {
        position: vec2(0.0, 0.0),
        ..Default::default()
    };
    let state = VehicleState::default();

    let output = controller.stabilize(dt, &Command::Waypoint(waypoint_command), &state);
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
    // (waypoint x, y) relative to a vehicle at rest at the origin
    let targets = [(30.0, 0.0), (-1000.0, 500.0), (0.0, 1e6)];
    for (x, y) in targets {
        let mut stabilizer = Stack::new();
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
        let mut stabilizer = Stack::new();
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
            (-PI..PI).contains(&stabilizer.guidance.heading),
            "yaw {yaw}: heading target {} escaped [-π, π)",
            stabilizer.guidance.heading
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
            let mut stabilizer = Stack::new();
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
    let mut stabilizer = Stack::new();
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
        (stabilizer.guidance.heading - (5.0 - 2.0 * PI)).abs() < 1e-5,
        "heading target {} did not re-latch",
        stabilizer.guidance.heading
    );
}

/// Regression: out-of-range stick input (e.g. the Spektrum commander's unclamped channels) hit
/// the attitude path's range `assert!`s and the mixer's, panicking the loop. Each row
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
        let mut raw = Stack::new();
        let mut saturated = Stack::new();
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
        let mut stabilizer = Stack::new();
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

/// Regression (#57 review): a NaN altitude estimate stuck in the altitude integral, so thrust
/// stayed NaN → 0 (motors off) in waypoint mode even after the estimate recovered.
#[test]
fn nan_altitude_estimate_holds_hover_then_recovers() {
    use uom::si::f32::Length;
    use uom::si::length::meter;

    let at_altitude = |z: f32| VehicleState {
        position: robomath::Vec3 {
            x: Length::new::<meter>(0.0),
            y: Length::new::<meter>(0.0),
            z: Length::new::<meter>(z),
        },
        ..Default::default()
    };
    let command = Command::Waypoint(WaypointCommand {
        position: vec2(0.0, 0.0),
        altitude: 20.0,
        cruise_speed: 7.0,
        ..Default::default()
    });

    let mut stabilizer = Stack::new();
    stabilizer.prime(&at_altitude(10.0));
    let during = stabilizer.stabilize(0.004, &command, &at_altitude(f32::NAN));
    let after = stabilizer.stabilize(0.004, &command, &at_altitude(10.0));

    let mut fresh = Stack::new();
    fresh.prime(&at_altitude(10.0));
    let expected = fresh.stabilize(0.004, &command, &at_altitude(10.0));

    assert!(during.thrust > 0.0, "NaN altitude cut thrust: {during:?}");
    assert!(
        after.thrust == expected.thrust,
        "after NaN: {after:?} vs fresh {expected:?}"
    );
}
