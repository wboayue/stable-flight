use crate::{Vec2, Vec3};

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

    fn step(&mut self, dt: f32, command: &Command, state: &VehicleState) -> ControlOutput {
        let setpoint = self.guidance.guide(dt, command, state);
        self.stabilizer.stabilize(dt, &setpoint, state)
    }
}

#[test]
fn lateral_loop_holds_level_at_the_target() {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let dt = 0.01;
    let desired_position = Vec2::new(0.0, 0.0);
    let current_position = Vec2::new(0.0, 0.0);
    let current_velocity = Vec2::new(0.0, 0.0);

    let output = guidance.lateral_controller.control(
        dt,
        desired_position,
        current_position,
        current_velocity,
    );
    assert_eq!(
        output,
        Vec2::new(0.0, 0.0),
        "Output should be zero when error is zero"
    );
}

#[test]
fn lateral_loop_tilts_toward_the_target() {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let dt = 0.01;
    let desired_position = Vec2::new(1.0, 1.0);
    let current_position = Vec2::new(0.0, 0.0);
    let current_velocity = Vec2::new(0.0, 0.0);

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
            // Out-of-range sticks pass through; the stabilizer saturates them.
            Command::Attitude(AttitudeCommand {
                throttle: 1.3,
                roll: 1.5,
                pitch: -2.0,
                ..Default::default()
            }),
            Setpoint::Attitude(AttitudeSetpoint {
                roll: 1.5,
                pitch: -2.0,
                heading: wrap_angle(0.7),
                thrust: 1.3,
                reference: AttitudeReference::Euler,
            }),
        ),
        (
            Command::Attitude(AttitudeCommand {
                yaw: 1.0,
                ..Default::default()
            }),
            Setpoint::Attitude(AttitudeSetpoint {
                heading: wrap_angle(0.7 + HEADING_SLEW_RATE * 0.004),
                ..Default::default()
            }),
        ),
        (
            // At the waypoint, at rest, level: no tilt, north. The altitude loop maps zero
            // error to zero linear thrust (it adds no hover thrust).
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

/// Pins today's mode-switch behaviour (#70). Waypoints hold
/// north without touching the heading-hold target, so attitude mode resumes the target it
/// had before the waypoint leg, not the vehicle's current heading.
#[test]
fn heading_hold_target_survives_a_waypoint_leg() {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let state = VehicleState {
        heading: 0.7,
        ..Default::default()
    };
    guidance.prime(&state);
    let yaw_right = Command::Attitude(AttitudeCommand {
        yaw: 1.0,
        ..Default::default()
    });
    let centred = Command::Attitude(AttitudeCommand::default());
    let waypoint = Command::Waypoint(WaypointCommand::default());
    let heading_of = |setpoint: Setpoint| match setpoint {
        Setpoint::Attitude(attitude) => (attitude.heading, attitude.reference),
        Setpoint::Rate(_) => {
            unreachable!("attitude and waypoint commands yield attitude setpoints")
        }
    };

    let mut before = 0.0;
    for _ in 0..10 {
        before = heading_of(guidance.guide(0.004, &yaw_right, &state)).0;
    }
    for _ in 0..10 {
        let during = heading_of(guidance.guide(0.004, &waypoint, &state));
        assert!(during == (0.0, AttitudeReference::Quaternion));
    }
    let after = heading_of(guidance.guide(0.004, &centred, &state));

    assert!(after == (before, AttitudeReference::Euler));
    approx::assert_relative_eq!(
        before,
        0.7 + 10.0 * HEADING_SLEW_RATE * 0.004,
        epsilon = 1e-6
    );
}

#[test]
fn waypoint_at_rest_on_target_yields_no_output() {
    let mut stack = Stack::new();
    let dt = 0.01;
    let waypoint_command = WaypointCommand {
        position: Vec2::new(0.0, 0.0),
        ..Default::default()
    };
    let state = VehicleState::default();

    let output = stack.step(dt, &Command::Waypoint(waypoint_command), &state);
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
        let mut stack = Stack::new();
        let state = VehicleState::default();
        stack.prime(&state);
        let command = Command::Waypoint(WaypointCommand {
            position: Vec2::new(x, y),
            altitude: 20.0,
            cruise_speed: 7.0,
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
        let command = Command::Attitude(AttitudeCommand {
            throttle: 0.5,
            yaw,
            arm: true,
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
            let mut stack = Stack::new();
            let state = VehicleState {
                heading: primed,
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

    stack.step(0.004, &command, &valid);

    assert!(
        (stack.guidance.heading - (5.0 - 2.0 * PI)).abs() < 1e-5,
        "heading target {} did not re-latch",
        stack.guidance.heading
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
    let at_altitude = |z: f32| state_at(0.0, 0.0, z);
    let command = Command::Waypoint(WaypointCommand {
        position: Vec2::new(0.0, 0.0),
        altitude: 20.0,
        cruise_speed: 7.0,
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
    let yaw_right = Command::Attitude(AttitudeCommand {
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
    use uom::si::f32::Length;
    use uom::si::length::meter;

    VehicleState {
        position: Vec3::new(
            Length::new::<meter>(x),
            Length::new::<meter>(y),
            Length::new::<meter>(z),
        ),
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
        position: Vec2::new(1.0, 1.0),
        altitude: 10.5,
        cruise_speed: 7.0,
        ..Default::default()
    }
}

/// The lateral and altitude loops as `CopterGuidance` configures them, stepped by hand: an
/// independent reference for the divider.
fn reference_loops() -> (LateralController, AltitudeController) {
    let config = CopterGuidanceConfig::default();
    (
        LateralController::with_config(&LateralControllerConfig {
            gains: config.lateral_gains,
            max_integral: 100.0,
            max_rate: Vec2::new(2.0, 2.0),
        }),
        AltitudeController::with_config(&AltitudeControllerConfig {
            gains: config.altitude_gains,
            ..Default::default()
        }),
    )
}

fn reference_tilt(lateral: &mut LateralController, step: f32, state: &VehicleState) -> Vec2<f32> {
    let target = near_waypoint().position;
    lateral.control(
        step,
        target,
        Vec2::new(state.position.x.value, state.position.y.value),
        Vec2::new(state.velocity.x.value, state.velocity.y.value),
    )
}

fn reference_thrust(altitude: &mut AltitudeController, dt: f32, state: &VehicleState) -> f32 {
    altitude.control(
        dt,
        near_waypoint().altitude,
        0.0,
        state.position.z.value,
        state.velocity.z.value,
        state.attitude,
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
    let (mut lateral, mut altitude) = reference_loops();
    let mut tilt = Vec2::default();

    for tick in 0..10 {
        let state = drifting_state(tick);
        let setpoint = attitude(guidance.guide(DT, &Command::Waypoint(near_waypoint()), &state));

        match tick {
            0 => tilt = reference_tilt(&mut lateral, DT, &state),
            7 => tilt = reference_tilt(&mut lateral, 7.0 * DT, &state),
            _ => {}
        }
        let thrust = reference_thrust(&mut altitude, DT, &state);

        approx::assert_relative_eq!(setpoint.roll, tilt.x, epsilon = 1e-6);
        approx::assert_relative_eq!(setpoint.pitch, tilt.y, epsilon = 1e-6);
        approx::assert_relative_eq!(setpoint.thrust, thrust, epsilon = 1e-6);
    }
}

/// A non-finite estimate on a run tick gives level for that tick only, and the next tick runs
/// the loop over the combined time instead of holding level for a period.
#[test]
fn nan_position_on_a_run_tick_retries_next_tick() {
    const DT: f32 = 0.004;
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let (mut lateral, _) = reference_loops();
    let waypoint = Command::Waypoint(near_waypoint());

    guidance.guide(DT, &waypoint, &drifting_state(0));
    reference_tilt(&mut lateral, DT, &drifting_state(0));
    for tick in 1..5 {
        guidance.guide(DT, &waypoint, &drifting_state(tick));
    }
    let lost = attitude(guidance.guide(DT, &waypoint, &state_at(f32::NAN, 0.0, 10.0)));
    let retried = attitude(guidance.guide(DT, &waypoint, &drifting_state(6)));

    assert!((lost.roll, lost.pitch) == (0.0, 0.0), "gave {lost:?}");
    let expected = reference_tilt(&mut lateral, 6.0 * DT, &drifting_state(6));
    approx::assert_relative_eq!(retried.roll, expected.x, epsilon = 1e-6);
    approx::assert_relative_eq!(retried.pitch, expected.y, epsilon = 1e-6);
}

/// Returning to waypoint mode, or re-priming, runs the lateral loop at once instead of
/// replaying a tilt held from before.
#[test]
fn leaving_waypoints_or_priming_drops_the_held_tilt() {
    let waypoint = Command::Waypoint(near_waypoint());
    // (what happens between the two waypoint ticks)
    let interruptions: [fn(&mut CopterGuidance); 2] = [
        |guidance| {
            guidance.guide(
                0.004,
                &Command::Attitude(AttitudeCommand::default()),
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
            (resumed.roll, resumed.pitch) != (held.roll, held.pitch),
            "row {row}: replayed the held tilt {held:?}"
        );
    }
}

#[test]
fn rejects_negative_or_non_finite_navigation_periods() {
    for period in [-0.01, f32::NAN, f32::INFINITY] {
        let panic = std::panic::catch_unwind(|| {
            CopterGuidance::new(CopterGuidanceConfig {
                navigation_period: period,
                ..Default::default()
            })
        })
        .map(drop)
        .expect_err("accepted an invalid navigation_period");
        let message = panic
            .downcast_ref::<String>()
            .map(String::as_str)
            .unwrap_or_default();
        assert!(
            message.contains("navigation_period"),
            "period {period}: panicked with {message:?}"
        );
    }
}

/// A centred yaw stick holds the heading target even for a pathological `dt` (0 × ∞ is NaN,
/// which would re-latch the target to the estimate).
#[test]
fn centred_yaw_stick_holds_heading_for_any_dt() {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    guidance.prime(&VehicleState {
        heading: 1.0,
        ..Default::default()
    });
    let elsewhere = VehicleState {
        heading: 2.0,
        ..Default::default()
    };

    let setpoint = attitude(guidance.guide(
        f32::INFINITY,
        &Command::Attitude(AttitudeCommand::default()),
        &elsewhere,
    ));

    approx::assert_relative_eq!(setpoint.heading, 1.0, epsilon = 1e-6);
}
