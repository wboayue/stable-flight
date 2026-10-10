use crate::{Vec2, Vec3};

use super::*;
use crate::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use crate::{ControlOutput, RateCommand, Stabilizer};
use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};

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

    fn step(&mut self, dt: f32, command: &ControlInput, state: &VehicleState) -> ControlOutput {
        let setpoint = self.guidance.guide(dt, command, state);
        self.stabilizer.stabilize(dt, &setpoint, state)
    }
}

/// A stop at `position_m` with no speed cap, seen from the origin at rest.
fn lateral_toward(position_m: Vec2) -> (LateralTarget, LateralEstimate) {
    let target = LateralTarget {
        position_m,
        velocity_mps: Vec2::zeros(),
        max_speed_mps: 0.0,
    };
    let estimate = LateralEstimate {
        position_m: Vec2::zeros(),
        velocity_mps: Vec2::zeros(),
    };
    (target, estimate)
}

#[test]
fn lateral_loop_holds_level_at_the_target() {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let (target, estimate) = lateral_toward(Vec2::zeros());
    let config = guidance.config.lateral;
    let output = guidance.lateral.control(&config, 0.01, target, estimate);
    assert_eq!(
        output,
        Vec2::zeros(),
        "Output should be zero when error is zero"
    );
}

#[test]
fn lateral_loop_tilts_toward_the_target() {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let (target, estimate) = lateral_toward(Vec2::new(1.0, 1.0));
    let config = guidance.config.lateral;
    let output = tilt_for_acceleration(
        guidance.lateral.control(&config, 0.01, target, estimate),
        0.0,
    );
    assert!(output.x > 0.0, "Roll should tilt to correct y-position");
    assert!(output.y < 0.0, "Pitch should tilt to correct x-position");
}

/// The tilt toward an acceleration is taken relative to the heading: ahead is nose down,
/// right is right roll.
#[test]
fn tilt_is_relative_to_the_heading() {
    let tilt = libm::atanf(1.0 / crate::GRAVITY);
    // (acceleration north/east, heading, expected (roll, pitch))
    let cases = [
        (Vec2::new(1.0, 0.0), 0.0, Vec2::new(0.0, -tilt)), // north, facing north: nose down
        (Vec2::new(0.0, 1.0), 0.0, Vec2::new(tilt, 0.0)),  // east, facing north: right roll
        (Vec2::new(1.0, 0.0), FRAC_PI_2, Vec2::new(-tilt, 0.0)), // north, facing east: left
        (Vec2::new(0.0, 1.0), FRAC_PI_2, Vec2::new(0.0, -tilt)), // east, facing east: ahead
        (Vec2::new(1.0, 0.0), PI, Vec2::new(0.0, tilt)),   // north, facing south: nose up
        (Vec2::new(0.0, -1.0), -FRAC_PI_2, Vec2::new(0.0, -tilt)), // west, facing west
        (Vec2::new(1.0, 0.0), f32::NAN, Vec2::zeros()),
    ];
    for (accel, heading, expected) in cases {
        let output = tilt_for_acceleration(accel, heading);
        assert!(
            (output - expected).amax() < 1e-5,
            "accel {accel:?} at heading {heading} gave {output:?}, expected {expected:?}"
        );
    }
}

/// Each command kind maps to the setpoint the stabilizer tracks.
#[test]
fn commands_map_to_setpoints() {
    let state = VehicleState {
        heading_rad: 0.7,
        ..Default::default()
    };
    // (command, expected setpoint), one fresh guidance primed at heading 0.7 per row
    let cases = [
        (
            ControlInput::Rate(RateCommand {
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
            ControlInput::Attitude(AttitudeCommand {
                throttle: 0.4,
                roll: 0.5,
                pitch: -0.25,
                ..Default::default()
            }),
            // Sticks scale to the default 45° at full deflection.
            Setpoint::Attitude(AttitudeSetpoint {
                roll_rad: 0.5 * FRAC_PI_4,
                pitch_rad: -0.25 * FRAC_PI_4,
                heading_rad: wrap_angle(0.7),
                thrust: 0.4,
            }),
        ),
        (
            // Out-of-range sticks saturate to full deflection, and throttle to full thrust.
            ControlInput::Attitude(AttitudeCommand {
                throttle: 1.3,
                roll: 1.5,
                pitch: -2.0,
                ..Default::default()
            }),
            Setpoint::Attitude(AttitudeSetpoint {
                roll_rad: FRAC_PI_4,
                pitch_rad: -FRAC_PI_4,
                heading_rad: wrap_angle(0.7),
                thrust: 1.0,
            }),
        ),
        (
            ControlInput::Attitude(AttitudeCommand {
                yaw: 1.0,
                ..Default::default()
            }),
            Setpoint::Attitude(AttitudeSetpoint {
                heading_rad: wrap_angle(0.7 + HEADING_SLEW_RATE * 0.004),
                ..Default::default()
            }),
        ),
        (
            // At the waypoint, at rest, level: no tilt, the primed heading, hover thrust (the
            // altitude loop feeds gravity forward).
            ControlInput::Waypoint(WaypointCommand::default()),
            Setpoint::Attitude(AttitudeSetpoint {
                roll_rad: 0.0,
                pitch_rad: 0.0,
                heading_rad: wrap_angle(0.7),
                thrust: crate::controllers::DEFAULT_HOVER_THRUST,
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

fn heading_of(setpoint: Setpoint) -> f32 {
    match setpoint {
        Setpoint::Attitude(attitude) => attitude.heading_rad,
        Setpoint::Rate(_) => {
            unreachable!("attitude and waypoint commands yield attitude setpoints")
        }
    }
}

/// A waypoint leg holds the heading-hold target instead of turning north, so attitude flight
/// resumes the same heading after it (#70).
#[test]
fn waypoint_leg_holds_the_heading_hold_target() {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let state = VehicleState {
        heading_rad: 0.7,
        ..Default::default()
    };
    guidance.prime(&state);
    let yaw_right = ControlInput::Attitude(AttitudeCommand {
        yaw: 1.0,
        ..Default::default()
    });
    let centred = ControlInput::Attitude(AttitudeCommand::default());
    let waypoint = ControlInput::Waypoint(WaypointCommand::default());

    let mut before = 0.0;
    for _ in 0..10 {
        before = heading_of(guidance.guide(0.004, &yaw_right, &state));
    }
    for _ in 0..10 {
        let during = heading_of(guidance.guide(0.004, &waypoint, &state));
        assert!(during == before);
    }
    let after = heading_of(guidance.guide(0.004, &centred, &state));

    assert!(after == before);
    approx::assert_relative_eq!(
        before,
        0.7 + 10.0 * HEADING_SLEW_RATE * 0.004,
        epsilon = 1e-6
    );
}

/// After rate flight, attitude and waypoint flight hold the heading the vehicle has, not the
/// one it had when rate flight began (#70). A NaN heading during rate flight keeps the last
/// valid one.
#[test]
fn rate_flight_hands_over_the_current_heading() {
    let rate = ControlInput::Rate(RateCommand {
        yaw_rate: 1.0,
        ..Default::default()
    });
    // (next command, heading at the end of rate flight, expected heading target)
    let cases = [
        (ControlInput::Attitude(AttitudeCommand::default()), 2.0, 2.0),
        (ControlInput::Waypoint(WaypointCommand::default()), 2.0, 2.0),
        (
            ControlInput::Attitude(AttitudeCommand::default()),
            4.0,
            4.0 - 2.0 * PI,
        ),
        (
            ControlInput::Attitude(AttitudeCommand::default()),
            f32::NAN,
            1.5,
        ),
    ];
    for (next, last_heading, expected) in cases {
        let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
        let heading = |heading_rad| VehicleState {
            heading_rad,
            ..Default::default()
        };
        guidance.prime(&heading(0.7));
        guidance.guide(0.004, &rate, &heading(1.5));
        guidance.guide(0.004, &rate, &heading(last_heading));

        let target = heading_of(guidance.guide(0.004, &next, &heading(last_heading)));
        approx::assert_relative_eq!(target, expected, epsilon = 1e-6);
    }
}

/// Leaving waypoint flight clears the altitude and lateral loops, so the next leg carries no
/// integral from the last one.
#[test]
fn leaving_waypoints_clears_the_position_loops() {
    let config = CopterGuidanceConfig {
        lateral: LateralControllerConfig {
            velocity: Gains {
                p: 1.0,
                i: 0.5,
                d: 0.0,
            },
            ..Default::default()
        },
        ..Default::default()
    };
    let others = [
        ControlInput::Attitude(AttitudeCommand::default()),
        ControlInput::Rate(RateCommand::default()),
    ];
    for other in others {
        let mut guidance = CopterGuidance::new(config);
        let state = drifting_state(0);
        guidance.prime(&state);
        for _ in 0..50 {
            guidance.guide(0.004, &ControlInput::Waypoint(near_waypoint()), &state);
        }
        assert!(guidance.altitude != AltitudeController::default());
        assert!(guidance.lateral != LateralController::default());

        guidance.guide(0.004, &other, &state);

        assert!(
            guidance.altitude == AltitudeController::default(),
            "{other:?}"
        );
        assert!(
            guidance.lateral == LateralController::default(),
            "{other:?}"
        );
    }
}

/// The waypoint's cruise speed caps the approach and its velocity is fed forward: flying at
/// the demanded velocity needs no tilt (#109).
#[test]
fn waypoint_speed_follows_cruise_speed_and_feedforward() {
    let moving_north = |speed_mps: f32| VehicleState {
        velocity_mps: Vec3::new(speed_mps, 0.0, 0.0),
        ..Default::default()
    };
    let far_north = |cruise_speed_mps| WaypointCommand {
        position_m: Vec2::new(100.0, 0.0),
        cruise_speed_mps,
        ..Default::default()
    };
    let through_origin = WaypointCommand {
        velocity_mps: Some(Vec2::new(4.0, 0.0)),
        cruise_speed_mps: 7.0,
        ..Default::default()
    };
    // (waypoint, vehicle speed north, expected pitch: 0 level, -1 nose down, 1 nose up)
    let cases = [
        (far_north(7.0), 7.0, 0.0),  // at cruise: hold it
        (far_north(7.0), 3.0, -1.0), // below: speed up
        (far_north(7.0), 9.0, 1.0),  // above: slow down
        (far_north(0.0), 7.0, -1.0), // no cap: the position loop wants 50 m/s
        (through_origin, 4.0, 0.0),  // on the waypoint at the fed-forward velocity
        (through_origin, 0.0, -1.0),
    ];
    for (waypoint, speed_mps, expected) in cases {
        let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
        let state = moving_north(speed_mps);
        guidance.prime(&state);
        let setpoint = attitude(guidance.guide(0.004, &ControlInput::Waypoint(waypoint), &state));
        let pitch = setpoint.pitch_rad;
        let sign = if pitch.abs() < 1e-6 {
            0.0
        } else {
            pitch.signum()
        };
        assert!(
            sign == expected && setpoint.roll_rad.abs() < 1e-6,
            "{waypoint:?} at {speed_mps} m/s gave {setpoint:?}"
        );
    }
}

#[test]
fn waypoint_at_rest_on_target_yields_hover_thrust_and_no_moments() {
    let mut stack = Stack::new();
    let dt = 0.01;
    let waypoint_command = WaypointCommand {
        position_m: Vec2::zeros(),
        ..Default::default()
    };
    let state = VehicleState::default();

    let output = stack.step(dt, &ControlInput::Waypoint(waypoint_command), &state);
    // Hover linear thrust, through the stabilizer's default thrust curve.
    let hover = crate::controllers::DEFAULT_HOVER_THRUST;
    let exponent = CopterStabilizerConfig::default().thrust_curve_exponent;
    assert_eq!(
        output,
        ControlOutput {
            thrust: hover.powf(exponent),
            moments: Vec3::zeros(),
        },
        "on target at rest: hover, no moments"
    );
}

/// Regression: a waypoint far from the vehicle made the lateral loop request pitch ≈ -4.6
/// (normalized), and the attitude controller's range `assert!` panicked the flight loop.
#[test]
fn distant_waypoint_tilts_toward_target_instead_of_panicking() {
    // (waypoint x, y) relative to a vehicle at rest at the origin
    let targets = [(30.0, 0.0), (-1000.0, 500.0), (0.0, 1e6)];
    for (x, y) in targets {
        let mut stack = Stack::new();
        let state = VehicleState::default();
        stack.prime(&state);
        let command = ControlInput::Waypoint(WaypointCommand {
            position_m: Vec2::new(x, y),
            altitude_m: 20.0,
            cruise_speed_mps: 7.0,
            ..Default::default()
        });

        // First tick: tilt toward the target. In this frame a +x target gives a negative
        // pitch moment and a +y target a positive roll moment (lateral: pitch = -a_x/g,
        // roll = a_y/g).
        let first = stack.step(0.004, &command, &state);
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
            let output = stack.step(0.004, &command, &state);
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
        let mut stack = Stack::new();
        let state = VehicleState::default();
        stack.prime(&state);
        let command = ControlInput::Attitude(AttitudeCommand {
            throttle: 0.5,
            yaw,
            ..Default::default()
        });

        for _ in 0..ticks {
            let output = stack.step(0.004, &command, &state);
            assert!(output.moments.z.is_finite());
        }

        assert!(
            (-PI..PI).contains(&stack.guidance.heading),
            "yaw {yaw}: heading target {} escaped [-π, π)",
            stack.guidance.heading
        );
    }
}

/// Headings from an estimator can be in [0, 2π) (RealFlight's azimuth); the wrapped target
/// must behave exactly like its [-π, π) equivalent, with no jump on the first tick.
#[test]
fn heading_primed_above_pi_behaves_like_its_wrapped_equivalent() {
    let command = ControlInput::Attitude(AttitudeCommand {
        throttle: 0.5,
        yaw: 1.0,
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
            let mut stack = Stack::new();
            let state = VehicleState {
                heading_rad: primed,
                ..Default::default()
            };
            stack.prime(&state);
            (0..10)
                .map(|_| stack.step(0.004, &command, &state).moments.z)
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
    let mut stack = Stack::new();
    stack.prime(&VehicleState {
        heading_rad: f32::NAN,
        ..Default::default()
    });
    let command = ControlInput::Attitude(AttitudeCommand {
        throttle: 0.5,
        ..Default::default()
    });
    let valid = VehicleState {
        heading_rad: 5.0,
        ..Default::default()
    };

    stack.step(0.004, &command, &valid);

    assert!(
        (stack.guidance.heading - (5.0 - 2.0 * PI)).abs() < 1e-5,
        "heading target {} did not re-latch",
        stack.guidance.heading
    );
}

/// Regression: out-of-range stick input (e.g. the Spektrum commander's once-unclamped channels) hit
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
            ControlInput::Attitude(AttitudeCommand {
                throttle,
                roll,
                pitch,
                yaw,
                mode: 0,
            })
        };
        let mut raw = Stack::new();
        let mut saturated = Stack::new();
        raw.prime(&state);
        saturated.prime(&state);
        for _ in 0..5 {
            let a = raw.step(0.004, &command(throttle, roll, pitch, yaw), &state);
            let b = saturated.step(0.004, &command(t, r, p, y), &state);
            assert!(
                a == b,
                "({throttle}, {roll}, {pitch}, {yaw}) gave {a:?}, expected {b:?}"
            );
        }
    }

    // Rate mode: throttle was never clamped and reached the mixer's assert.
    for throttle in [1.5, -0.3, nan] {
        let mut stack = Stack::new();
        let output = stack.step(
            0.004,
            &ControlInput::Rate(RateCommand {
                throttle,
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
    let at_altitude = |z: f32| state_at(0.0, 0.0, z);
    let command = ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::zeros(),
        altitude_m: 20.0,
        cruise_speed_mps: 7.0,
        ..Default::default()
    });

    let mut stack = Stack::new();
    stack.prime(&at_altitude(10.0));
    let during = stack.step(0.004, &command, &at_altitude(f32::NAN));
    let after = stack.step(0.004, &command, &at_altitude(10.0));

    let mut fresh = Stack::new();
    fresh.prime(&at_altitude(10.0));
    let expected = fresh.step(0.004, &command, &at_altitude(10.0));

    assert!(during.thrust > 0.0, "NaN altitude cut thrust: {during:?}");
    assert!(
        after.thrust == expected.thrust,
        "after NaN: {after:?} vs fresh {expected:?}"
    );
}

/// Full yaw stick slews the heading-hold target by the same angle per second at any loop rate.
#[test]
fn heading_slew_depends_on_time_not_tick_count() {
    // (dt, ticks): one second each
    let cases = [(0.004, 250), (0.01, 100), (0.002, 500)];
    let yaw_right = ControlInput::Attitude(AttitudeCommand {
        yaw: 1.0,
        ..Default::default()
    });
    for (dt, ticks) in cases {
        let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
        let state = VehicleState::default();
        guidance.prime(&state);
        for _ in 0..ticks {
            guidance.guide(dt, &yaw_right, &state);
        }
        // f32 accumulation of ~1e-4 steps drifts ~3e-5 over 500 ticks.
        approx::assert_relative_eq!(guidance.heading, HEADING_SLEW_RATE, epsilon = 1e-4);
    }
}

/// A vehicle at rest at `(x, y, z)` (north, east, up; m).
fn state_at(x: f32, y: f32, z: f32) -> VehicleState {
    VehicleState {
        position_m: Vec3::new(x, y, z),
        ..Default::default()
    }
}

/// A vehicle drifting a little each tick, near [`near_waypoint`], so every run of either
/// waypoint loop produces a new (unsaturated) output.
fn drifting_state(tick: usize) -> VehicleState {
    let drift = tick as f32 * 0.01;
    state_at(drift, -drift, 10.0 + drift)
}

fn near_waypoint() -> WaypointCommand {
    WaypointCommand {
        position_m: Vec2::new(1.0, 1.0),
        altitude_m: 10.5,
        cruise_speed_mps: 7.0,
        ..Default::default()
    }
}

/// The lateral and altitude loop configs `CopterGuidance` flies, stepped by hand with fresh
/// controllers: an independent reference for the divider.
fn reference_configs() -> (LateralControllerConfig, AltitudeControllerConfig) {
    let config = CopterGuidanceConfig::default();
    (config.lateral, config.altitude)
}

fn reference_tilt(lateral: &mut LateralController, step: f32, state: &VehicleState) -> Vec2 {
    let waypoint = near_waypoint();
    let target = LateralTarget {
        position_m: waypoint.position_m,
        velocity_mps: Vec2::zeros(),
        max_speed_mps: waypoint.cruise_speed_mps,
    };
    let estimate = LateralEstimate {
        position_m: state.position_m.xy(),
        velocity_mps: state.velocity_mps.xy(),
    };
    tilt_for_acceleration(
        lateral.control(&reference_configs().0, step, target, estimate),
        state.heading_rad,
    )
}

fn reference_thrust(altitude: &mut AltitudeController, dt: f32, state: &VehicleState) -> f32 {
    altitude.control(
        &reference_configs().1,
        dt,
        AltitudeTarget {
            altitude_m: near_waypoint().altitude_m,
            velocity_mps: 0.0,
        },
        AltitudeEstimate {
            altitude_m: state.position_m.z,
            velocity_mps: state.velocity_mps.z,
            attitude: state.attitude,
        },
    )
}

fn attitude(setpoint: Setpoint) -> AttitudeSetpoint {
    match setpoint {
        Setpoint::Attitude(attitude) => attitude,
        Setpoint::Rate(_) => unreachable!("waypoints yield attitude setpoints"),
    }
}

/// At 3 ms ticks the 20 ms lateral loop runs on ticks 0 and 7, stepping over 3 ms and then the
/// 21 ms actually elapsed, and holds its tilt in between; the altitude loop runs every tick.
#[test]
fn lateral_loop_runs_per_navigation_period_and_altitude_loop_every_tick() {
    const DT: f32 = 0.003;
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let (mut lateral, mut altitude) = (LateralController::default(), AltitudeController::default());
    let mut tilt = Vec2::zeros();

    for tick in 0..10 {
        let state = drifting_state(tick);
        let setpoint =
            attitude(guidance.guide(DT, &ControlInput::Waypoint(near_waypoint()), &state));

        match tick {
            0 => tilt = reference_tilt(&mut lateral, DT, &state),
            7 => tilt = reference_tilt(&mut lateral, 7.0 * DT, &state),
            _ => {}
        }
        let thrust = reference_thrust(&mut altitude, DT, &state);

        approx::assert_relative_eq!(setpoint.roll_rad, tilt.x, epsilon = 1e-6);
        approx::assert_relative_eq!(setpoint.pitch_rad, tilt.y, epsilon = 1e-6);
        approx::assert_relative_eq!(setpoint.thrust, thrust, epsilon = 1e-6);
    }
}

/// A non-finite estimate on a run tick gives level for that tick only, and the next tick runs
/// the loop over the combined time instead of holding level for a period.
#[test]
fn nan_position_on_a_run_tick_retries_next_tick() {
    const DT: f32 = 0.004;
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let mut lateral = LateralController::default();
    let waypoint = ControlInput::Waypoint(near_waypoint());

    guidance.guide(DT, &waypoint, &drifting_state(0));
    reference_tilt(&mut lateral, DT, &drifting_state(0));
    for tick in 1..5 {
        guidance.guide(DT, &waypoint, &drifting_state(tick));
    }
    let lost = attitude(guidance.guide(DT, &waypoint, &state_at(f32::NAN, 0.0, 10.0)));
    let retried = attitude(guidance.guide(DT, &waypoint, &drifting_state(6)));

    assert!(
        (lost.roll_rad, lost.pitch_rad) == (0.0, 0.0),
        "gave {lost:?}"
    );
    let expected = reference_tilt(&mut lateral, 6.0 * DT, &drifting_state(6));
    approx::assert_relative_eq!(retried.roll_rad, expected.x, epsilon = 1e-6);
    approx::assert_relative_eq!(retried.pitch_rad, expected.y, epsilon = 1e-6);
}

/// Returning to waypoint mode, or re-priming, runs the lateral loop at once instead of
/// replaying a tilt held from before.
#[test]
fn leaving_waypoints_or_priming_drops_the_held_tilt() {
    let waypoint = ControlInput::Waypoint(near_waypoint());
    // (what happens between the two waypoint ticks)
    let interruptions: [fn(&mut CopterGuidance); 2] = [
        |guidance| {
            guidance.guide(
                0.004,
                &ControlInput::Attitude(AttitudeCommand::default()),
                &drifting_state(1),
            );
        },
        |guidance| guidance.prime(&drifting_state(1)),
    ];
    for (row, interrupt) in interruptions.into_iter().enumerate() {
        let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
        let held = attitude(guidance.guide(0.004, &waypoint, &drifting_state(0)));
        interrupt(&mut guidance);

        let resumed = attitude(guidance.guide(0.004, &waypoint, &drifting_state(2)));

        assert!(
            (resumed.roll_rad, resumed.pitch_rad) != (held.roll_rad, held.pitch_rad),
            "row {row}: replayed the held tilt {held:?}"
        );
    }
}

/// After priming (as `FlightLoop` does on arming), waypoint guidance flies exactly like a
/// freshly primed one: altitude and lateral integrals and the held tilt are gone.
#[test]
fn prime_clears_altitude_and_lateral_state() {
    let waypoint = ControlInput::Waypoint(near_waypoint());
    let mut wound = CopterGuidance::new(CopterGuidanceConfig::default());
    for tick in 0..250 {
        wound.guide(0.004, &waypoint, &drifting_state(tick));
    }
    let state = drifting_state(250);
    wound.prime(&state);

    let mut fresh = CopterGuidance::new(CopterGuidanceConfig::default());
    fresh.prime(&state);

    assert_eq!(
        wound.guide(0.004, &waypoint, &state),
        fresh.guide(0.004, &waypoint, &state)
    );
}

/// Asserts that `CopterGuidance::new(config)` panics with a message naming `field`.
fn assert_new_panics(config: CopterGuidanceConfig, field: &str) {
    let panic = std::panic::catch_unwind(|| CopterGuidance::new(config))
        .map(drop)
        .expect_err("accepted an invalid config");
    let message = panic
        .downcast_ref::<String>()
        .map(String::as_str)
        .unwrap_or_default();
    assert!(
        message.contains(field),
        "{config:?}: panicked with {message:?}"
    );
}

#[test]
fn rejects_negative_or_non_finite_navigation_periods() {
    for navigation_period in [-0.01, f32::NAN, f32::INFINITY] {
        let config = CopterGuidanceConfig {
            navigation_period_s: navigation_period,
            ..Default::default()
        };
        assert_new_panics(config, "navigation_period");
    }
}

/// An infinite `dt` is a caller bug (0 × ∞ is NaN), rejected like a zero one.
#[test]
#[should_panic(expected = "dt must be positive and finite")]
fn infinite_dt_panics() {
    CopterGuidance::new(CopterGuidanceConfig::default()).guide(
        f32::INFINITY,
        &ControlInput::Attitude(AttitudeCommand::default()),
        &VehicleState::default(),
    );
}

/// A far waypoint's tilt is capped at `max_tilt_rad` in magnitude, keeping its direction; a
/// near one is `atan(a / g)`, uncapped.
#[test]
fn waypoint_tilt_is_capped_in_magnitude_and_keeps_its_direction() {
    let state = VehicleState::default();
    let lateral = CopterGuidanceConfig::default().lateral;
    let near = libm::atanf(lateral.position_p * lateral.velocity.p * 0.1 / crate::GRAVITY);
    // (waypoint x, y, max_tilt_rad, expected |tilt| in rad)
    let cases = [
        (1000.0, 0.0, 0.47, 0.47),
        (1000.0, 1000.0, 0.47, 0.47),
        (-30.0, 40.0, 0.3, 0.3),
        (1000.0, -1000.0, 1.2, 1.2), // a steep cap: atan(|a| / g) ≈ 1.56 rad is cut back
        (0.1, 0.0, 0.47, near),
    ];
    for (x, y, max_tilt_rad, expected) in cases {
        let mut guidance = CopterGuidance::new(CopterGuidanceConfig {
            max_tilt_rad,
            ..Default::default()
        });
        guidance.prime(&state);
        let command = ControlInput::Waypoint(WaypointCommand {
            position_m: Vec2::new(x, y),
            ..Default::default()
        });
        let setpoint = attitude(guidance.guide(0.004, &command, &state));
        let tilt = Vec2::new(setpoint.roll_rad, setpoint.pitch_rad);

        approx::assert_relative_eq!(tilt.norm(), expected, epsilon = 1e-5);
        // Toward the target: roll follows +y (east), pitch follows -x (north is nose down).
        let toward = Vec2::new(y, -x).normalize();
        approx::assert_relative_eq!(tilt.normalize().dot(&toward), 1.0, epsilon = 1e-5);
    }
}

#[test]
fn rejects_max_tilt_outside_level_to_vertical() {
    for max_tilt_rad in [0.0, -0.5, FRAC_PI_2, 2.0, f32::NAN] {
        let config = CopterGuidanceConfig {
            max_tilt_rad,
            ..Default::default()
        };
        assert_new_panics(config, "max_tilt_rad");
    }
}

#[test]
fn rejects_negative_or_non_finite_stick_angles() {
    // (max_stick_angles_rad, offending field)
    let cases = [
        (Vec2::new(-0.1, 0.5), "max_stick_angles_rad.x"),
        (Vec2::new(0.5, f32::NAN), "max_stick_angles_rad.y"),
        (Vec2::new(f32::INFINITY, 0.5), "max_stick_angles_rad.x"),
    ];
    for (max_stick_angles_rad, field) in cases {
        let config = CopterGuidanceConfig {
            max_stick_angles_rad,
            ..Default::default()
        };
        assert_new_panics(config, field);
    }
}

/// Full stick commands `max_stick_angles_rad` on each axis, scaled linearly.
#[test]
fn sticks_scale_to_the_configured_angles() {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig {
        max_stick_angles_rad: Vec2::new(0.6, 0.3),
        ..Default::default()
    });
    let command = ControlInput::Attitude(AttitudeCommand {
        roll: -1.0,
        pitch: 0.5,
        ..Default::default()
    });
    let setpoint = attitude(guidance.guide(0.004, &command, &VehicleState::default()));
    assert!((setpoint.roll_rad, setpoint.pitch_rad) == (-0.6, 0.15));
}

/// Tilt is `atan(|a| / g)` toward the acceleration: north is nose down, east is right roll.
#[test]
fn tilt_for_acceleration_points_thrust_along_the_demand() {
    use crate::GRAVITY;
    // (acceleration north, east in m/s², expected (roll, pitch) in rad)
    let cases = [
        (Vec2::zeros(), Vec2::zeros()),
        (Vec2::new(GRAVITY, 0.0), Vec2::new(0.0, -FRAC_PI_4)),
        (Vec2::new(0.0, -GRAVITY), Vec2::new(-FRAC_PI_4, 0.0)),
        (
            Vec2::new(0.6 * GRAVITY, 0.8 * GRAVITY), // |a| = g, diagonal
            Vec2::new(0.8 * FRAC_PI_4, -0.6 * FRAC_PI_4),
        ),
        (
            Vec2::new(0.1, 0.0),
            Vec2::new(0.0, -libm::atanf(0.1 / GRAVITY)),
        ),
        (Vec2::new(f32::NAN, 1.0), Vec2::zeros()),
        (Vec2::new(f32::INFINITY, 0.0), Vec2::zeros()),
    ];
    for (accel, expected) in cases {
        let tilt = tilt_for_acceleration(accel, 0.0);
        assert!(
            (tilt - expected).amax() < 1e-6,
            "{accel:?} gave {tilt:?}, expected {expected:?}"
        );
    }
}

#[test]
fn rejects_hover_thrust_outside_zero_to_one() {
    for hover_thrust in [0.0, -0.1, 1.2, f32::NAN] {
        let mut config = CopterGuidanceConfig::default();
        config.altitude.hover_thrust = hover_thrust;
        assert_new_panics(config, "altitude.hover_thrust"); // names the loop
    }
}

#[test]
fn rejects_an_invalid_lateral_loop_naming_it() {
    let mut config = CopterGuidanceConfig::default();
    config.lateral.max_integral = -1.0;
    assert_new_panics(config, "lateral.max_integral");
}

/// An invalid config is rejected and the current one kept.
#[test]
fn set_config_rejects_an_invalid_config_and_keeps_the_current_one() {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let current = *guidance.config();
    let mut bad = [current; 4];
    bad[0].altitude.hover_thrust = 1.5;
    bad[1].lateral.max_integral = f32::NAN;
    bad[2].navigation_period_s = -0.01;
    bad[3].max_tilt_rad = 0.0;
    for config in bad {
        assert!(guidance.set_config(config).is_err(), "{config:?}");
        assert_eq!(*guidance.config(), current);
    }
}

/// Retuning the altitude and lateral integral gains rescales their integrals so each I term
/// carries over.
#[test]
fn retuning_integral_gains_rescales_the_integrals() {
    let command = ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(1.0, 0.0),
        altitude_m: 0.1, // within the altitude loop's integration band
        cruise_speed_mps: 5.0,
        ..Default::default()
    });
    let state = VehicleState::default();
    let mut config = CopterGuidanceConfig::default();
    config.lateral.velocity.i = 0.1; // default 0
    let mut guidance = CopterGuidance::new(config);
    guidance.prime(&state);
    for _ in 0..20 {
        guidance.guide(0.004, &command, &state);
    }
    let (altitude, lateral) = (
        guidance.altitude.integrated_error(),
        guidance.lateral.integrated_error(),
    );
    assert!(altitude != 0.0 && lateral.x != 0.0);

    let mut doubled = *guidance.config();
    doubled.altitude.gains.i *= 2.0;
    doubled.lateral.velocity.i *= 2.0;
    guidance.set_config(doubled).expect("valid config");

    approx::assert_relative_eq!(guidance.altitude.integrated_error(), 0.5 * altitude);
    approx::assert_relative_eq!(guidance.lateral.integrated_error(), 0.5 * lateral);
}

/// A new `navigation_period_s` applies on the next tick: the lateral loop, holding its tilt
/// under the 20 ms default, reruns at once when the period drops to every tick.
#[test]
fn set_config_applies_a_new_navigation_period_on_the_next_tick() {
    let command = ControlInput::Waypoint(near_waypoint());
    let moving = |vx: f32| VehicleState {
        velocity_mps: Vec3::new(vx, 0.0, 0.0),
        ..Default::default()
    };
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    guidance.prime(&moving(0.0));

    let first = attitude(guidance.guide(0.004, &command, &moving(0.0)));
    let held = attitude(guidance.guide(0.004, &command, &moving(1.0)));
    assert_eq!(
        (held.roll_rad, held.pitch_rad),
        (first.roll_rad, first.pitch_rad)
    );

    let mut every_tick = *guidance.config();
    every_tick.navigation_period_s = 0.0;
    guidance.set_config(every_tick).expect("valid config");
    let rerun = attitude(guidance.guide(0.004, &command, &moving(2.0)));
    assert_ne!(rerun.pitch_rad, first.pitch_rad);
}

/// Lowering `max_tilt_rad` re-caps the held waypoint tilt at once, not at the lateral loop's next
/// run.
#[test]
fn lowering_max_tilt_rad_recaps_the_held_tilt_on_the_next_tick() {
    let far = ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(1000.0, 0.0),
        altitude_m: 10.0,
        ..Default::default()
    });
    let state = VehicleState::default();
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default()); // 20 ms period
    guidance.prime(&state);
    let full = attitude(guidance.guide(0.004, &far, &state));
    approx::assert_relative_eq!(full.pitch_rad.abs(), DEFAULT_MAX_TILT_RAD, epsilon = 1e-6);

    let mut lower = *guidance.config();
    lower.max_tilt_rad = 0.3;
    guidance.set_config(lower).expect("valid config");
    let held = attitude(guidance.guide(0.004, &far, &state)); // lateral loop not due yet

    approx::assert_relative_eq!(
        Vec2::new(held.roll_rad, held.pitch_rad).norm(),
        0.3,
        epsilon = 1e-6
    );
}

/// Lowering `max_tilt_rad` below the held tilt flags `tilt` on the next tick, before the
/// lateral loop runs again.
#[test]
fn lowering_max_tilt_rad_below_the_held_tilt_flags_it_at_once() {
    let near = ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(1.0, 0.0),
        ..Default::default()
    });
    let state = VehicleState::default();
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default()); // 20 ms period
    guidance.prime(&state);
    let held = attitude(guidance.guide(0.004, &near, &state));
    assert!(!guidance.saturation().tilt);

    let mut lower = *guidance.config();
    lower.max_tilt_rad = 0.5 * held.pitch_rad.abs();
    guidance.set_config(lower).expect("valid config");
    guidance.guide(0.004, &near, &state); // lateral loop not due yet

    assert!(guidance.saturation().tilt);
}

/// Closed loop: a `WaypointFollower` (`Hold`) feeding `CopterGuidance`, flying a point mass
/// that tilts instantly (horizontal acceleration `g · tan(tilt)` toward the tilt, altitude
/// held). With a tight acceptance radius the corners are missed wide, so this also guards
/// against the leg velocity parking the vehicle past a missed waypoint.
#[test]
fn point_mass_flies_a_route_at_cruise_speed_and_holds_the_end() {
    use crate::Commander;
    use crate::commanders::{RouteEnd, WaypointFollower};

    const DT: f32 = 0.004;
    const CRUISE_MPS: f32 = 7.0;
    const ALTITUDE_M: f32 = 10.0;
    let waypoint = |x, y| WaypointCommand {
        position_m: Vec2::new(x, y),
        altitude_m: ALTITUDE_M,
        cruise_speed_mps: CRUISE_MPS,
        ..Default::default()
    };
    let route = [
        waypoint(0.0, 0.0),
        waypoint(30.0, 0.0),
        waypoint(30.0, 30.0),
        waypoint(0.0, 30.0),
    ];
    let mut follower = WaypointFollower::new(route)
        .expect("non-empty")
        .with_acceptance_radius(0.5)
        .with_route_end(RouteEnd::Hold);
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());

    let (mut position, mut velocity) = (Vec2::zeros(), Vec2::zeros());
    let state_of = |position: Vec2, velocity: Vec2| VehicleState {
        position_m: Vec3::new(position.x, position.y, ALTITUDE_M),
        velocity_mps: Vec3::new(velocity.x, velocity.y, 0.0),
        ..Default::default()
    };
    guidance.prime(&state_of(position, velocity));

    let mut max_speed_mps: f32 = 0.0;
    for _ in 0..(60.0 / DT) as usize {
        let state = state_of(position, velocity);
        let command = follower.next_command(DT, &state);
        let setpoint = attitude(guidance.guide(DT, &command.input, &state));

        // Heading north: roll tilts east, nose down (negative pitch) tilts north.
        let tilt = Vec2::new(-setpoint.pitch_rad, setpoint.roll_rad);
        let tilt_rad = tilt.norm();
        let accel_mps2 = if tilt_rad > 0.0 {
            tilt * (crate::GRAVITY * libm::tanf(tilt_rad) / tilt_rad)
        } else {
            Vec2::zeros()
        };
        velocity += accel_mps2 * DT;
        position += velocity * DT;
        max_speed_mps = max_speed_mps.max(velocity.norm());
    }

    assert!(
        follower.is_complete(),
        "stuck at waypoint {}",
        follower.current_index()
    );
    assert!(
        (position - Vec2::new(0.0, 30.0)).norm() < 0.1,
        "ended at {position:?}"
    );
    assert!(velocity.norm() < 0.05, "still moving at {velocity:?}");
    assert!(
        (CRUISE_MPS - 0.5..=CRUISE_MPS + 0.5).contains(&max_speed_mps),
        "peak speed {max_speed_mps} m/s"
    );
}

/// Attitude-mode throttle is divided by the cosine of the measured tilt, so the vertical thrust
/// the stick asks for holds while tilted (#111). The boost stops past 60°, cuts out when
/// inverted (no thrust toward the ground), a non-finite attitude gets none, and the result
/// saturates to [0, 1] with NaN as no thrust.
#[test]
fn attitude_throttle_is_boosted_for_tilt() {
    let cos = |deg: f32| deg.to_radians().cos();
    let tilted = |roll_deg: f32, pitch_deg: f32| {
        nalgebra::UnitQuaternion::from_euler_angles(
            roll_deg.to_radians(),
            pitch_deg.to_radians(),
            0.3,
        )
    };
    let nan_attitude =
        nalgebra::UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(f32::NAN, 0.0, 0.0, 0.0));
    // (throttle, measured attitude, expected thrust)
    let cases = [
        (0.5, tilted(0.0, 0.0), 0.5),
        (0.5, tilted(30.0, 0.0), 0.5 / cos(30.0)),
        (0.5, tilted(0.0, -30.0), 0.5 / cos(30.0)),
        (0.4, tilted(20.0, 20.0), 0.4 / (cos(20.0) * cos(20.0))),
        (0.4, tilted(70.0, 0.0), 0.8),  // floored at cos 60° = 0.5
        (0.4, tilted(90.0, 0.0), 0.0),  // on its side: no thrust
        (0.4, tilted(150.0, 0.0), 0.0), // inverted: no thrust
        (0.5, nan_attitude, 0.5),       // no boost
        (0.0, tilted(30.0, 0.0), 0.0),
        (0.9, tilted(45.0, 0.0), 1.0), // saturated
        (f32::NAN, tilted(30.0, 0.0), 0.0),
    ];
    for (throttle, measured, expected) in cases {
        let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
        let state = VehicleState {
            attitude: measured,
            ..Default::default()
        };
        guidance.prime(&state);
        let command = ControlInput::Attitude(AttitudeCommand {
            throttle,
            ..Default::default()
        });
        let thrust = attitude(guidance.guide(0.004, &command, &state)).thrust;
        approx::assert_relative_eq!(thrust, expected, epsilon = 1e-5);
    }
}

/// Guidance reports what its last call clipped: a waypoint tilt capped at `max_tilt_rad`
/// (held with the tilt between lateral-loop runs), and thrust clipped to [0, 1] by the
/// altitude loop or attitude mode's boost.
#[test]
fn reports_what_the_last_guide_clipped() {
    let level = VehicleState::default();
    let banked = VehicleState {
        attitude: nalgebra::UnitQuaternion::from_euler_angles(FRAC_PI_4, 0.0, 0.0),
        ..Default::default()
    };
    let waypoint = |x: f32, altitude_m| {
        ControlInput::Waypoint(WaypointCommand {
            position_m: Vec2::new(x, 0.0),
            altitude_m,
            ..Default::default()
        })
    };
    let throttle = |throttle| {
        ControlInput::Attitude(AttitudeCommand {
            throttle,
            ..Default::default()
        })
    };
    // (input, state, expected (tilt, thrust))
    let cases = [
        (waypoint(0.5, 0.0), &level, (false, false)),
        (waypoint(1000.0, 0.0), &level, (true, false)), // far: tilt capped
        (waypoint(0.0, 100.0), &level, (false, true)),  // far below: thrust at 1
        (waypoint(0.0, -100.0), &level, (false, true)), // far above: thrust at 0
        (throttle(0.5), &banked, (false, false)),       // 0.5 · √2 fits
        (throttle(0.9), &banked, (false, true)),        // 0.9 · √2 clipped
        (
            ControlInput::Rate(RateCommand::default()),
            &level,
            (false, false),
        ),
    ];
    for (input, state, (tilt, thrust)) in cases {
        let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
        guidance.prime(state);
        guidance.guide(0.004, &input, state);
        let saturation = guidance.saturation();
        assert!(
            (saturation.tilt, saturation.thrust) == (tilt, thrust),
            "{input:?}: {saturation:?}"
        );
    }

    // The capped tilt is held between lateral-loop runs, and priming clears it.
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    guidance.prime(&level);
    guidance.guide(0.004, &waypoint(1000.0, 0.0), &level);
    guidance.guide(0.004, &waypoint(1000.0, 0.0), &level); // loop not due: held
    assert!(guidance.saturation().tilt);
    guidance.prime(&level);
    assert!(guidance.saturation() == Saturation::default());
}
