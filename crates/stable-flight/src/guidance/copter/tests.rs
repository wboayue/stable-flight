use crate::{Vec2, Vec3};

use super::*;
use crate::estimation::wind::WindConfig;
use crate::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use crate::{ControlOutput, RateCommand, RateSetpoint, Stabilizer};
use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

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
        let setpoint = self
            .guidance
            .guide(dt, FlightRegime::Copter, command, state);
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
    let output = guidance
        .lateral
        .control(&config, 0.01, target, estimate, Vec2::zeros());
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
        guidance
            .lateral
            .control(&config, 0.01, target, estimate, Vec2::zeros()),
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
            }),
            Setpoint::Rate(RateSetpoint {
                roll_rate: 0.1,
                pitch_rate: -0.2,
                yaw_rate: 0.3,
                thrust: 0.6,
                idle: false,
            }),
        ),
        (
            // Absolute rates pass through; thrust saturates, not boosted.
            ControlInput::AbsoluteRate(crate::AbsoluteRateCommand {
                roll_radps: 2.0,
                pitch_radps: -1.0,
                yaw_radps: 0.5,
                thrust: 1.3,
            }),
            Setpoint::BodyRate(crate::BodyRateSetpoint {
                roll_radps: 2.0,
                pitch_radps: -1.0,
                yaw_radps: 0.5,
                thrust: 1.0,
                idle: false,
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
                transition: false,
                idle: false,
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
                transition: false,
                idle: false,
            }),
        ),
        (
            ControlInput::Attitude(AttitudeCommand {
                yaw: 1.0,
                ..Default::default()
            }),
            // Throttle at 0 is idle.
            Setpoint::Attitude(AttitudeSetpoint {
                heading_rad: wrap_angle(0.7 + HEADING_SLEW_RATE * 0.004),
                idle: true,
                ..Default::default()
            }),
        ),
        (
            // Absolute angles pass through (the stabilizer saturates them), and the heading
            // replaces the primed one. Thrust isn't boosted.
            ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
                roll_rad: 0.2,
                pitch_rad: -1.2,
                heading_rad: -2.0,
                thrust: 0.45,
            }),
            Setpoint::Attitude(AttitudeSetpoint {
                roll_rad: 0.2,
                pitch_rad: -1.2,
                heading_rad: -2.0,
                thrust: 0.45,
                transition: false,
                idle: false,
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
                transition: false,
                idle: false,
            }),
        ),
    ];
    for (command, expected) in cases {
        let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
        guidance.prime(&state);
        let setpoint = guidance.guide(0.004, FlightRegime::Copter, &command, &state);
        assert!(setpoint == expected, "{command:?} gave {setpoint:?}");
    }
}

fn heading_of(setpoint: Setpoint) -> f32 {
    match setpoint {
        Setpoint::Attitude(attitude) => attitude.heading_rad,
        _ => unreachable!("attitude and waypoint commands yield attitude setpoints"),
    }
}

/// On the waypoint, a waypoint leg holds the heading-hold target instead of turning north,
/// so attitude flight resumes the same heading after it.
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
        before = heading_of(guidance.guide(0.004, FlightRegime::Copter, &yaw_right, &state));
    }
    for _ in 0..10 {
        let during = heading_of(guidance.guide(0.004, FlightRegime::Copter, &waypoint, &state));
        assert!(during == before);
    }
    let after = heading_of(guidance.guide(0.004, FlightRegime::Copter, &centred, &state));

    assert!(after == before);
    approx::assert_relative_eq!(
        before,
        0.7 + 10.0 * HEADING_SLEW_RATE * 0.004,
        epsilon = 1e-6
    );
}

/// The default config, never yielding the heading: for checks whose vehicle doesn't turn.
fn never_yielding() -> CopterGuidanceConfig {
    CopterGuidanceConfig {
        heading_yield_s: 0.0,
        ..Default::default()
    }
}

/// A copter waypoint 100 m from the origin at `(north, east)`.
fn waypoint_toward(north_m: f32, east_m: f32) -> ControlInput {
    ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(north_m, east_m),
        ..Default::default()
    })
}

/// One tick of a leg from the origin turns the heading-hold target toward the leg's direction,
/// by at most the slew rate the shorter way round, and holds it on a leg shorter than
/// `waypoint_yaw_min_leg_m`.
#[test]
fn waypoint_flight_turns_along_the_leg() {
    let dt = 0.004;
    let step = DEFAULT_WAYPOINT_YAW_RADPS * dt;
    let near = 0.5 * step;
    // (case, heading, waypoint (north, east), expected heading) from the origin
    let cases = [
        ("ahead", 0.0, (100.0, 0.0), 0.0),
        ("right", 0.0, (0.0, 100.0), step),
        ("left", 0.0, (0.0, -100.0), -step),
        ("behind, right of the tail", 0.0, (-100.0, 1.0), step),
        ("behind, left of the tail", 0.0, (-100.0, -1.0), -step),
        ("across ±π", 3.0, (-100.0, -15.0), 3.0 + step),
        (
            "within a step",
            0.0,
            (100.0, 100.0 * libm::tanf(near)),
            near,
        ),
        ("shorter than the least leg", 1.0, (2.0, -2.0), 1.0),
        (
            "the least leg",
            1.0,
            (0.0, -DEFAULT_WAYPOINT_YAW_MIN_LEG_M),
            1.0,
        ),
        ("NaN waypoint", 1.0, (f32::NAN, 100.0), 1.0),
    ];
    for (case, heading, (north_m, east_m), expected) in cases {
        let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
        let state = VehicleState {
            heading_rad: heading,
            ..Default::default()
        };
        guidance.prime(&state);
        let setpoint = guidance.guide(
            dt,
            FlightRegime::Copter,
            &waypoint_toward(north_m, east_m),
            &state,
        );
        let heading = heading_of(setpoint);
        assert!(
            (heading - wrap_angle(expected)).abs() < 1e-5,
            "{case}: {heading}, expected {expected}"
        );
    }
}

/// A leg keeps its direction from where it began: a vehicle that overshoots the waypoint or
/// drifts off the line doesn't turn toward it. A new waypoint starts a leg from the vehicle,
/// and so does re-entering waypoint flight; a NaN position waits to start one.
#[test]
fn a_leg_keeps_the_direction_it_began_with() {
    let east = waypoint_toward(0.0, 100.0);
    let north = waypoint_toward(100.0, 0.0);
    let hover = ControlInput::Attitude(AttitudeCommand::default());
    let nan = f32::NAN;
    // (case, phases of (command, vehicle north, vehicle east), expected heading after them),
    // heading north at first, 10 s per phase: long enough to finish any turn
    let cases = [
        (
            "past the waypoint",
            [(&east, 0.0, 0.0), (&east, 0.0, 110.0), (&east, 0.0, 110.0)],
            FRAC_PI_2,
        ),
        (
            "off the line",
            [(&east, 0.0, 0.0), (&east, 50.0, 50.0), (&east, 50.0, 50.0)],
            FRAC_PI_2,
        ),
        (
            "a new waypoint",
            [(&east, 0.0, 0.0), (&north, 0.0, 50.0), (&north, 0.0, 50.0)],
            libm::atan2f(-50.0, 100.0),
        ),
        (
            "waypoint flight re-entered",
            [(&east, 0.0, 0.0), (&hover, 0.0, 0.0), (&east, 99.0, 100.0)],
            PI,
        ),
        (
            "a NaN position first",
            [(&east, nan, 0.0), (&east, nan, 0.0), (&east, 0.0, -50.0)],
            FRAC_PI_2,
        ),
    ];
    for (case, phases, expected) in cases {
        let mut guidance = CopterGuidance::new(never_yielding());
        guidance.prime(&VehicleState::default());
        let mut heading = f32::NAN;
        for (command, north_m, east_m) in phases {
            let state = state_at(north_m, east_m, 0.0);
            for _ in 0..2500 {
                heading = heading_of(guidance.guide(0.004, FlightRegime::Copter, command, &state));
            }
        }
        let error = wrap_angle(heading - expected).abs();
        assert!(error < 1e-5, "{case}: {heading}, expected {expected}");
    }
}

/// A shortest-leg change applies from the next leg, since a leg's direction is set when it
/// begins; a slew change applies at once.
#[test]
fn a_config_change_mid_leg_applies_as_documented() {
    let east = waypoint_toward(0.0, 100.0);
    let state = state_at(0.0, 0.0, 0.0);
    // (case, config set after the first tick, expected heading after 1 s)
    let cases = [
        (
            "longer shortest leg",
            CopterGuidanceConfig {
                waypoint_yaw_min_leg_m: 200.0,
                ..never_yielding()
            },
            turned_by(1.0),
        ),
        (
            "slew off",
            CopterGuidanceConfig {
                waypoint_yaw_radps: 0.0,
                ..never_yielding()
            },
            turned_by(0.004),
        ),
    ];
    for (case, config, expected) in cases {
        let mut guidance = CopterGuidance::new(never_yielding());
        guidance.prime(&VehicleState::default());
        guidance.guide(0.004, FlightRegime::Copter, &east, &state);
        guidance.set_config(config).expect("valid config");
        let mut heading = f32::NAN;
        for _ in 0..ticks_for(1.0) - 1 {
            heading = heading_of(guidance.guide(0.004, FlightRegime::Copter, &east, &state));
        }
        let error = wrap_angle(heading - expected).abs();
        assert!(error < 1e-4, "{case}: {heading}, expected {expected}");
    }
}

/// Waypoint flight turns at the slew rate per second at any loop rate, and arrives on the
/// bearing without overshooting it.
#[test]
fn waypoint_turn_depends_on_time_not_tick_count() {
    // (dt, ticks): one second each, then another second to settle on the bearing
    let cases = [(0.004, 250), (0.01, 100), (0.002, 500)];
    let east = waypoint_toward(0.0, 100.0);
    for (dt, ticks) in cases {
        let mut guidance = CopterGuidance::new(never_yielding());
        let state = VehicleState::default();
        guidance.prime(&state);
        for _ in 0..ticks {
            guidance.guide(dt, FlightRegime::Copter, &east, &state);
        }
        approx::assert_relative_eq!(guidance.heading, DEFAULT_WAYPOINT_YAW_RADPS, epsilon = 1e-4);
        for _ in 0..ticks {
            guidance.guide(dt, FlightRegime::Copter, &east, &state);
        }
        approx::assert_relative_eq!(guidance.heading, FRAC_PI_2, epsilon = 1e-6);
    }
}

/// A heading that stays over `heading_yield_rad` off the target, the vehicle yawing toward it
/// under 0.1 rad/s, for `heading_yield_s` is yielded: the target returns to the vehicle's
/// heading. A leg east from a vehicle held north: the target passes 30° at 0.5 s.
#[test]
fn a_stuck_heading_is_yielded() {
    let east = waypoint_toward(0.0, 100.0);
    let turned = |seconds: f32| (seconds * DEFAULT_WAYPOINT_YAW_RADPS).min(FRAC_PI_2);
    let yielding = CopterGuidanceConfig::default();
    // (case, config, body yaw rate (rad/s), seconds, expected heading target)
    let cases = [
        ("stuck", yielding, 0.0, 2.0, 0.0),
        ("yawing away", yielding, -0.5, 2.0, 0.0),
        ("yawing toward it too slowly", yielding, 0.09, 2.0, 0.0),
        ("yawing toward it", yielding, 0.11, 2.0, turned(2.0)),
        ("not stuck for long enough", yielding, 0.0, 1.4, turned(1.4)),
        ("never yielding", never_yielding(), 0.0, 2.0, turned(2.0)),
    ];
    for (case, config, yaw_radps, seconds, expected) in cases {
        let mut guidance = CopterGuidance::new(config);
        let state = VehicleState {
            angular_velocity_radps: Vec3::new(0.0, 0.0, yaw_radps),
            ..Default::default()
        };
        guidance.prime(&state);
        let mut heading = f32::NAN;
        for _ in 0..ticks_for(seconds) {
            heading = heading_of(guidance.guide(0.004, FlightRegime::Copter, &east, &state));
        }
        let error = wrap_angle(heading - expected).abs();
        assert!(error < 1e-4, "{case}: {heading}, expected {expected}");
    }
}

/// Once yielded, the heading-hold target follows the vehicle's heading for the rest of the
/// leg; a new waypoint, or leaving waypoint flight, turns toward the leg again. A NaN heading
/// never yields.
#[test]
fn a_yielded_heading_follows_the_vehicle_until_the_leg_changes() {
    let east = waypoint_toward(0.0, 100.0);
    let north = waypoint_toward(100.0, 0.0);
    let hover = ControlInput::Attitude(AttitudeCommand::default());
    let at = |heading_rad| VehicleState {
        heading_rad,
        ..Default::default()
    };
    let nan = f32::NAN;
    // (case, phases of (command, vehicle heading, seconds), expected heading target) from a
    // vehicle at the origin, primed north; 2 s on the leg east yields
    let cases: [(&str, &[Phase], f32); 4] = [
        ("follows", &[(&east, 0.0, 2.0), (&east, -1.0, 0.004)], -1.0),
        (
            "a new waypoint",
            &[(&east, 0.0, 2.0), (&north, 0.3, 2.0)],
            0.0,
        ),
        (
            "waypoint flight re-entered",
            &[(&east, 0.0, 2.0), (&hover, 0.0, 0.004), (&east, 0.0, 0.4)],
            turned_by(0.4),
        ),
        (
            "a NaN heading",
            &[(&east, nan, 2.0), (&east, nan, 2.0)],
            FRAC_PI_2,
        ),
    ];
    for (case, phases, expected) in cases {
        let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
        guidance.prime(&at(0.0));
        let mut heading = f32::NAN;
        for &(command, vehicle_rad, seconds) in phases {
            for _ in 0..ticks_for(seconds) {
                heading = heading_of(guidance.guide(
                    0.004,
                    FlightRegime::Copter,
                    command,
                    &at(vehicle_rad),
                ));
            }
        }
        let error = wrap_angle(heading - expected).abs();
        assert!(error < 1e-4, "{case}: {heading}, expected {expected}");
    }
}

/// A waypoint's heading turns the heading-hold target to it at `heading_rate_radps`, the
/// shorter way round and without overshooting, instead of along the leg, with the leg slew off
/// too, and is never yielded; a NaN heading leaves the leg's turn.
#[test]
fn a_waypoint_heading_is_turned_to_at_the_heading_rate() {
    let rate_radps = 30.0_f32.to_radians();
    let config = CopterGuidanceConfig {
        heading_rate_radps: rate_radps,
        ..Default::default()
    };
    let slew_off = CopterGuidanceConfig {
        waypoint_yaw_radps: 0.0,
        ..config
    };
    let step = rate_radps * 0.004;
    // (case, config, vehicle heading, waypoint (north, east), waypoint heading, seconds,
    // expected heading target); the vehicle sits at the origin, not turning
    let cases = [
        ("one tick", config, 0.0, (0.0, 0.0), 1.0, 0.004, step),
        (
            "across ±π",
            config,
            3.0,
            (0.0, 0.0),
            -3.0,
            0.004,
            3.0 + step,
        ),
        ("arrives", config, 0.0, (0.0, 0.0), 1.0, 3.0, 1.0),
        ("wrapped", config, 0.0, (0.0, 0.0), 1.0 + TAU, 3.0, 1.0),
        (
            "over the leg",
            config,
            0.0,
            (0.0, 100.0),
            -1.0,
            1.0,
            -rate_radps,
        ),
        (
            "leg slew off",
            slew_off,
            0.0,
            (0.0, 100.0),
            1.0,
            1.0,
            rate_radps,
        ),
        (
            "never yielded",
            config,
            0.0,
            (0.0, 0.0),
            FRAC_PI_2,
            4.0,
            FRAC_PI_2,
        ),
        (
            "NaN",
            config,
            0.0,
            (0.0, 100.0),
            f32::NAN,
            0.4,
            turned_by(0.4),
        ),
    ];
    for (case, config, vehicle_rad, (north_m, east_m), heading_rad, seconds, expected) in cases {
        let mut guidance = CopterGuidance::new(config);
        let state = VehicleState {
            heading_rad: vehicle_rad,
            ..Default::default()
        };
        guidance.prime(&state);
        let waypoint = ControlInput::Waypoint(WaypointCommand {
            position_m: Vec2::new(north_m, east_m),
            heading_rad: Some(heading_rad),
            ..Default::default()
        });
        let mut heading = f32::NAN;
        for _ in 0..ticks_for(seconds) {
            heading = heading_of(guidance.guide(0.004, FlightRegime::Copter, &waypoint, &state));
        }
        let error = wrap_angle(heading - expected).abs();
        assert!(error < 1e-4, "{case}: {heading}, expected {expected}");
    }
}

/// A commanded heading forgets the leg: once it is dropped, the same waypoint starts a fresh
/// leg from the vehicle, turning along it, rather than resuming a yield from before. A vehicle
/// at the origin held north; 2 s on the leg east yields.
#[test]
fn a_dropped_heading_starts_a_fresh_leg() {
    let east = waypoint_toward(0.0, 100.0);
    let ControlInput::Waypoint(east_command) = east else {
        unreachable!("waypoint_toward gives a waypoint")
    };
    let facing = ControlInput::Waypoint(WaypointCommand {
        heading_rad: Some(0.5),
        ..east_command
    });
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let state = VehicleState::default();
    guidance.prime(&state);
    let mut heading = f32::NAN;
    for (command, seconds) in [(&east, 2.0), (&facing, 1.0), (&east, 0.4)] {
        for _ in 0..ticks_for(seconds) {
            heading = heading_of(guidance.guide(0.004, FlightRegime::Copter, command, &state));
        }
    }
    let expected = 0.5 + turned_by(0.4);
    assert!(
        (heading - expected).abs() < 1e-4,
        "{heading}, expected {expected}"
    );
}

/// A command flown for a time from a vehicle at a heading: (command, heading, seconds).
type Phase<'a> = (&'a ControlInput, f32, f32);

/// How far waypoint flight turns the heading-hold target in `seconds`, in radians.
fn turned_by(seconds: f32) -> f32 {
    seconds * DEFAULT_WAYPOINT_YAW_RADPS
}

/// 4 ms ticks in `seconds`.
fn ticks_for(seconds: f32) -> usize {
    (seconds / 0.004).round() as usize
}

/// With the waypoint slew off, and in land flight, a waypoint behind holds the heading.
#[test]
fn heading_holds_with_the_slew_off_and_in_land_flight() {
    let behind = Vec2::new(-100.0, 10.0);
    let land = ControlInput::Land(LandCommand { position_m: behind });
    let slew_off = CopterGuidanceConfig {
        waypoint_yaw_radps: 0.0,
        ..Default::default()
    };
    // (case, config, command)
    let cases = [
        ("slew off", slew_off, waypoint_toward(behind.x, behind.y)),
        ("land", CopterGuidanceConfig::default(), land),
    ];
    let state = VehicleState {
        heading_rad: 0.7,
        ..Default::default()
    };
    for (case, config, command) in cases {
        let mut guidance = CopterGuidance::new(config);
        guidance.prime(&state);
        for _ in 0..100 {
            let heading = heading_of(guidance.guide(0.004, FlightRegime::Copter, &command, &state));
            assert!(heading == wrap_angle(0.7), "{case}: {heading}");
        }
    }
}

/// Stick flight after a waypoint leg resumes the heading the leg turned to.
#[test]
fn stick_flight_keeps_the_heading_a_leg_turned_to() {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let state = VehicleState::default();
    guidance.prime(&state);
    let east = waypoint_toward(0.0, 100.0);
    let mut turned = 0.0;
    for _ in 0..10 {
        turned = heading_of(guidance.guide(0.004, FlightRegime::Copter, &east, &state));
    }
    let centred = ControlInput::Attitude(AttitudeCommand::default());
    let after = heading_of(guidance.guide(0.004, FlightRegime::Copter, &centred, &state));
    approx::assert_relative_eq!(
        turned,
        10.0 * DEFAULT_WAYPOINT_YAW_RADPS * 0.004,
        epsilon = 1e-6
    );
    assert!(after == turned);
}

/// After rate flight (sticks or absolute rates), attitude and waypoint flight hold the heading
/// the vehicle has, not the one it had when rate flight began. A NaN heading during rate flight
/// keeps the last valid one.
#[test]
fn rate_flight_hands_over_the_current_heading() {
    for rate in [
        ControlInput::Rate(RateCommand {
            yaw_rate: 1.0,
            ..Default::default()
        }),
        ControlInput::AbsoluteRate(crate::AbsoluteRateCommand {
            yaw_radps: 1.0,
            ..Default::default()
        }),
    ] {
        rate_flight_hands_over_the_current_heading_from(rate);
    }
}

fn rate_flight_hands_over_the_current_heading_from(rate: ControlInput) {
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
        guidance.guide(0.004, FlightRegime::Copter, &rate, &heading(1.5));
        guidance.guide(0.004, FlightRegime::Copter, &rate, &heading(last_heading));

        let target =
            heading_of(guidance.guide(0.004, FlightRegime::Copter, &next, &heading(last_heading)));
        approx::assert_relative_eq!(target, expected, epsilon = 1e-6);
    }
}

/// An absolute attitude's heading becomes the heading-hold target, wrapped, so stick flight
/// afterward keeps it; a NaN heading holds the target it had.
#[test]
fn absolute_attitude_sets_the_heading_hold_target() {
    let absolute = |heading_rad| {
        ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
            heading_rad,
            ..Default::default()
        })
    };
    let state = VehicleState {
        heading_rad: 0.7,
        ..Default::default()
    };
    // (commanded heading, expected heading target)
    let cases = [
        (1.5, 1.5),
        (7.0, 7.0 - 2.0 * PI),
        (-4.0, -4.0 + 2.0 * PI),
        (f32::NAN, 0.7),
        (f32::INFINITY, 0.7),
    ];
    for (heading_rad, expected) in cases {
        let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
        guidance.prime(&state);
        let during =
            heading_of(guidance.guide(0.004, FlightRegime::Copter, &absolute(heading_rad), &state));
        let after = heading_of(guidance.guide(
            0.004,
            FlightRegime::Copter,
            &ControlInput::Attitude(AttitudeCommand::default()),
            &state,
        ));
        approx::assert_relative_eq!(during, expected, epsilon = 1e-5);
        approx::assert_relative_eq!(after, expected, epsilon = 1e-5);
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
        ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand::default()),
    ];
    for other in others {
        let mut guidance = CopterGuidance::new(config);
        let state = drifting_state(0);
        guidance.prime(&state);
        for _ in 0..50 {
            guidance.guide(
                0.004,
                FlightRegime::Copter,
                &ControlInput::Waypoint(near_waypoint()),
                &state,
            );
        }
        assert!(guidance.altitude != AltitudeController::default());
        assert!(guidance.lateral != LateralController::default());

        guidance.guide(0.004, FlightRegime::Copter, &other, &state);

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
/// the demanded velocity needs no tilt.
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
        let setpoint = attitude(guidance.guide(
            0.004,
            FlightRegime::Copter,
            &ControlInput::Waypoint(waypoint),
            &state,
        ));
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
        ControlOutput::new(hover.powf(exponent), Vec3::zeros()),
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

    // Absolute attitude: tilt beyond the stabilizer's 45° flies as 45° and is flagged there;
    // NaN flies level.
    let absolute = |roll_rad, pitch_rad| {
        ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
            roll_rad,
            pitch_rad,
            thrust: 0.5,
            ..Default::default()
        })
    };
    // ((roll, pitch), equivalent in range, stabilizer tilt flag)
    let absolute_cases = [
        ((1.0, -2.0), (FRAC_PI_4, -FRAC_PI_4), true),
        ((nan, 0.3), (0.0, 0.3), false),
    ];
    for ((roll, pitch), (r, p), tilt) in absolute_cases {
        let mut raw = Stack::new();
        let mut saturated = Stack::new();
        raw.prime(&state);
        saturated.prime(&state);
        for _ in 0..5 {
            let a = raw.step(0.004, &absolute(roll, pitch), &state);
            let b = saturated.step(0.004, &absolute(r, p), &state);
            assert!(a == b, "({roll}, {pitch}) gave {a:?}, expected {b:?}");
        }
        assert!(
            raw.stabilizer.saturation().tilt == tilt,
            "({roll}, {pitch})"
        );
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

/// Regression (#57): a NaN altitude estimate stays out of the altitude integral. Waypoint
/// flight holds hover thrust meanwhile, not NaN → 0 (motors off), and recovers with the
/// estimate.
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
            guidance.guide(dt, FlightRegime::Copter, &yaw_right, &state);
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
        lateral.control(
            &reference_configs().0,
            step,
            target,
            estimate,
            Vec2::zeros(),
        ),
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
        _ => unreachable!("waypoints yield attitude setpoints"),
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
        let setpoint = attitude(guidance.guide(
            DT,
            FlightRegime::Copter,
            &ControlInput::Waypoint(near_waypoint()),
            &state,
        ));

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

    guidance.guide(DT, FlightRegime::Copter, &waypoint, &drifting_state(0));
    reference_tilt(&mut lateral, DT, &drifting_state(0));
    for tick in 1..5 {
        guidance.guide(DT, FlightRegime::Copter, &waypoint, &drifting_state(tick));
    }
    let lost = attitude(guidance.guide(
        DT,
        FlightRegime::Copter,
        &waypoint,
        &state_at(f32::NAN, 0.0, 10.0),
    ));
    let retried = attitude(guidance.guide(DT, FlightRegime::Copter, &waypoint, &drifting_state(6)));

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
                FlightRegime::Copter,
                &ControlInput::Attitude(AttitudeCommand::default()),
                &drifting_state(1),
            );
        },
        |guidance| guidance.prime(&drifting_state(1)),
    ];
    for (row, interrupt) in interruptions.into_iter().enumerate() {
        let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
        let held =
            attitude(guidance.guide(0.004, FlightRegime::Copter, &waypoint, &drifting_state(0)));
        interrupt(&mut guidance);

        let resumed =
            attitude(guidance.guide(0.004, FlightRegime::Copter, &waypoint, &drifting_state(2)));

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
        wound.guide(
            0.004,
            FlightRegime::Copter,
            &waypoint,
            &drifting_state(tick),
        );
    }
    let state = drifting_state(250);
    wound.prime(&state);

    let mut fresh = CopterGuidance::new(CopterGuidanceConfig::default());
    fresh.prime(&state);

    assert_eq!(
        wound.guide(0.004, FlightRegime::Copter, &waypoint, &state),
        fresh.guide(0.004, FlightRegime::Copter, &waypoint, &state)
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
        FlightRegime::Copter,
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
        let setpoint = attitude(guidance.guide(0.004, FlightRegime::Copter, &command, &state));
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
    let setpoint = attitude(guidance.guide(
        0.004,
        FlightRegime::Copter,
        &command,
        &VehicleState::default(),
    ));
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
        guidance.guide(0.004, FlightRegime::Copter, &command, &state);
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

    let first = attitude(guidance.guide(0.004, FlightRegime::Copter, &command, &moving(0.0)));
    let held = attitude(guidance.guide(0.004, FlightRegime::Copter, &command, &moving(1.0)));
    assert_eq!(
        (held.roll_rad, held.pitch_rad),
        (first.roll_rad, first.pitch_rad)
    );

    let mut every_tick = *guidance.config();
    every_tick.navigation_period_s = 0.0;
    guidance.set_config(every_tick).expect("valid config");
    let rerun = attitude(guidance.guide(0.004, FlightRegime::Copter, &command, &moving(2.0)));
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
    let full = attitude(guidance.guide(0.004, FlightRegime::Copter, &far, &state));
    approx::assert_relative_eq!(full.pitch_rad.abs(), DEFAULT_MAX_TILT_RAD, epsilon = 1e-6);

    let mut lower = *guidance.config();
    lower.max_tilt_rad = 0.3;
    guidance.set_config(lower).expect("valid config");
    let held = attitude(guidance.guide(0.004, FlightRegime::Copter, &far, &state)); // lateral loop not due yet

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
    let held = attitude(guidance.guide(0.004, FlightRegime::Copter, &near, &state));
    assert!(!guidance.saturation().tilt);

    let mut lower = *guidance.config();
    lower.max_tilt_rad = 0.5 * held.pitch_rad.abs();
    guidance.set_config(lower).expect("valid config");
    guidance.guide(0.004, FlightRegime::Copter, &near, &state); // lateral loop not due yet

    assert!(guidance.saturation().tilt);
}

/// While the waypoint tilt is capped, the lateral integral doesn't grow along the capped
/// request: a long leg at the tilt limit leaves it about one lateral run's worth, not wound
/// to `max_integral` (#357).
#[test]
fn capped_tilt_holds_the_lateral_integral() {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let state = VehicleState::default();
    guidance.prime(&state);
    // 20 m/s asked of a vehicle at rest: 25 m/s², far past the 27° tilt cap.
    let far = ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(100.0, 0.0),
        altitude_m: 0.0,
        cruise_speed_mps: 20.0,
        ..Default::default()
    });
    for _ in 0..1_000 {
        guidance.guide(0.004, FlightRegime::Copter, &far, &state);
    }

    assert!(guidance.saturation().tilt);
    // The first run integrates freely: 20 m/s over one 0.02 s navigation period.
    let integral_m = guidance.lateral.integrated_error();
    assert!(integral_m.norm() < 0.5, "integral wound to {integral_m:?}");
}

/// Lowering `max_tilt_rad` under a held tilt counts as capped for the integral too: the next
/// lateral run doesn't grow it along the request.
#[test]
fn lowered_tilt_cap_holds_the_lateral_integral() {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let state = VehicleState::default();
    guidance.prime(&state);
    // 2 m north at rest: about 1.3 m/s², well under the default cap.
    let near = ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(2.0, 0.0),
        altitude_m: 0.0,
        cruise_speed_mps: 7.0,
        ..Default::default()
    });
    guidance.guide(0.02, FlightRegime::Copter, &near, &state);
    let integral_m = guidance.lateral.integrated_error();
    assert!(integral_m.x > 0.0);

    let mut lower = *guidance.config();
    lower.max_tilt_rad = 0.01;
    guidance.set_config(lower).expect("valid config");
    guidance.guide(0.02, FlightRegime::Copter, &near, &state);

    assert!(guidance.lateral.integrated_error() == integral_m);
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
        let setpoint = attitude(guidance.guide(DT, FlightRegime::Copter, &command.input, &state));

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
        // 7.52 m/s measured: the lateral I overshoots cruise briefly at the leg start
        // (#357; TODO(#367)).
        (CRUISE_MPS - 0.5..=CRUISE_MPS + 0.75).contains(&max_speed_mps),
        "peak speed {max_speed_mps} m/s"
    );
}

/// Attitude-mode throttle is divided by the cosine of the measured tilt, so the vertical thrust
/// the stick asks for holds while tilted. The boost stops past 60°, cuts out when
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
        let thrust = attitude(guidance.guide(0.004, FlightRegime::Copter, &command, &state)).thrust;
        approx::assert_relative_eq!(thrust, expected, epsilon = 1e-5);
    }
}

/// Guidance reports what its last call clipped: a waypoint tilt capped at `max_tilt_rad`
/// (held with the tilt between lateral-loop runs), and thrust clipped to [0, 1] by the
/// altitude loop, attitude mode's boost, or an absolute attitude's thrust.
#[test]
fn reports_what_the_last_guide_clipped() {
    let level = VehicleState::default();
    let banked = VehicleState {
        attitude: nalgebra::UnitQuaternion::from_euler_angles(FRAC_PI_4, 0.0, 0.0),
        ..Default::default()
    };
    let shooting_up = VehicleState {
        velocity_mps: Vec3::new(0.0, 0.0, 30.0),
        ..Default::default()
    };
    let waypoint = |x: f32, altitude_m| {
        ControlInput::Waypoint(WaypointCommand {
            position_m: Vec2::new(x, 0.0),
            altitude_m,
            ..Default::default()
        })
    };
    let absolute = |thrust, roll_rad| {
        ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
            roll_rad,
            thrust,
            ..Default::default()
        })
    };
    // (input, state, expected (tilt, thrust))
    let cases = [
        (waypoint(0.5, 0.0), &level, (false, false)),
        (waypoint(1000.0, 0.0), &level, (true, false)), // far: tilt capped
        (waypoint(0.0, 100.0), &level, (false, true)),  // far below: thrust at 1
        (waypoint(0.0, -100.0), &level, (false, false)), // far above: descends at the limit
        (waypoint(0.0, -100.0), &shooting_up, (false, true)), // damped to thrust 0
        (stick(0.5), &banked, (false, false)),          // 0.5 · √2 fits
        (stick(0.9), &banked, (false, true)),           // 0.9 · √2 clipped
        (absolute(0.9, 1.0), &banked, (false, false)),  // not boosted; tilt is the stabilizer's
        (absolute(1.2, 0.0), &level, (false, true)),
        (absolute(f32::NAN, 0.0), &level, (false, false)),
        (
            ControlInput::Rate(RateCommand::default()),
            &level,
            (false, false),
        ),
    ];
    for (input, state, (tilt, thrust)) in cases {
        let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
        guidance.prime(state);
        guidance.guide(0.004, FlightRegime::Copter, &input, state);
        let saturation = guidance.saturation();
        assert!(
            (saturation.tilt, saturation.thrust) == (tilt, thrust),
            "{input:?}: {saturation:?}"
        );
    }

    // The capped tilt is held between lateral-loop runs, and priming clears it.
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    guidance.prime(&level);
    guidance.guide(0.004, FlightRegime::Copter, &waypoint(1000.0, 0.0), &level);
    guidance.guide(0.004, FlightRegime::Copter, &waypoint(1000.0, 0.0), &level); // loop not due: held
    assert!(guidance.saturation().tilt);
    guidance.prime(&level);
    assert!(guidance.saturation() == Saturation::default());
}

/// Fresh guidance primed at `altitude_m`, at rest and level.
fn guidance_at(altitude_m: f32) -> (CopterGuidance, VehicleState) {
    let state = state_at(0.0, 0.0, altitude_m);
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    guidance.prime(&state);
    (guidance, state)
}

/// The altitude loop's target after one tick toward a waypoint at `altitude_m`.
fn waypoint_at(altitude_m: f32) -> ControlInput {
    ControlInput::Waypoint(WaypointCommand {
        altitude_m,
        ..Default::default()
    })
}

/// How far a descent's target has dropped after `ticks` ticks of `dt` from hover, far from
/// its waypoint and off the leash: its rate rises by `DEFAULT_DESCENT_ACCEL_MPS2 · dt` a tick,
/// up to `limit_mps`.
fn ramped_drop_m(ticks: usize, dt: f32, limit_mps: f32) -> f32 {
    (1..=ticks)
        .map(|tick| (DEFAULT_DESCENT_ACCEL_MPS2 * tick as f32 * dt).min(limit_mps) * dt)
        .sum()
}

/// A waypoint below the vehicle is approached by an altitude target lowered at a rate that
/// rises at `descent_accel_mps2` to `max_descent_mps` far from it (until the leash holds it),
/// and at the altitude loop's P times the gap near it; one above is taken at once. The target
/// starts from the vehicle's altitude, and a non-finite waypoint passes through without
/// latching.
#[test]
fn altitude_target_descends_at_the_limit_and_climbs_at_once() {
    const DT: f32 = 0.01;
    // (waypoint altitude, ticks, expected latched target) from 10 m
    let cases = [
        (
            -100.0,
            1,
            Some(10.0 - ramped_drop_m(1, DT, DEFAULT_MAX_DESCENT_MPS)),
        ),
        (
            -100.0,
            30,
            Some(10.0 - ramped_drop_m(30, DT, DEFAULT_MAX_DESCENT_MPS)),
        ),
        (-100.0, 400, Some(10.0 - ALTITUDE_LEASH_M)), // the vehicle holds still: leashed
        (9.5, 1_000, Some(9.5)),                      // reached: holds the waypoint
        (9.5, 500, Some(9.5)),                        // eased within 1 mm in 5 s
        // Near the waypoint, the first tick: lowered at the ramp's 0.01 m/s, below P (1.5)
        // times the 0.015 m gap.
        (
            10.0 - 0.015,
            1,
            Some(10.0 - ramped_drop_m(1, DT, DEFAULT_MAX_DESCENT_MPS)),
        ),
        (25.0, 1, Some(25.0)), // climbs aren't limited
        (f32::NAN, 1, None),
    ];
    for (waypoint_m, ticks, expected) in cases {
        let (mut guidance, state) = guidance_at(10.0);
        for _ in 0..ticks {
            guidance.guide(DT, FlightRegime::Copter, &waypoint_at(waypoint_m), &state);
        }
        match (guidance.altitude_target_m, expected) {
            (Some(target), Some(expected)) => {
                approx::assert_relative_eq!(target, expected, epsilon = 1e-3)
            }
            (target, expected) => assert!(target == expected, "{waypoint_m}: {target:?}"),
        }
    }
}

/// Where the leash puts the target on the waypoint, the feedforward is the same on both sides:
/// a target latched on the waypoint with the vehicle pushed just past the leash above it gets
/// no descent rate stepped in (#445: the lift rotors chattered through a transition).
#[test]
fn descent_feedforward_is_continuous_at_the_leash() {
    const DT: f32 = 0.004;
    let target = |above_m: f32| {
        let (mut guidance, _) = guidance_at(10.0 + above_m);
        guidance.altitude_target_m = Some(10.0);
        guidance.descend_toward(DT, 10.0, 10.0 + above_m)
    };
    let below = target(ALTITUDE_LEASH_M - 1e-3);
    let above = target(ALTITUDE_LEASH_M + 1e-3);
    assert!(
        (above.velocity_mps - below.velocity_mps).abs() < 0.01,
        "{below:?} → {above:?}"
    );
    assert!(
        (above.altitude_m - below.altitude_m).abs() < 0.01,
        "{below:?} → {above:?}"
    );
}

/// The descent rate fed forward shrinks toward the waypoint, so `descent_accel_mps2` (1 m/s²)
/// stops it there: linear at P 1.5 within 0.44 m, a square root beyond, `max_descent_mps` far
/// away (descending at the limit already).
#[test]
fn descent_feedforward_brakes_to_the_waypoint() {
    // (latched target's height above the waypoint, expected feedforward m/s)
    let cases = [
        (0.0, 0.0),
        (0.2, -0.3),
        (2.0, -libm::sqrtf(2.0 * (2.0 - 1.0 / 4.5))),
        (10.0, -DEFAULT_MAX_DESCENT_MPS),
    ];
    for (height_m, expected) in cases {
        let (mut guidance, _) = guidance_at(10.0 + height_m);
        guidance.altitude_target_m = Some(10.0 + height_m);
        guidance.descent_speed_mps = DEFAULT_MAX_DESCENT_MPS;
        let target = guidance.descend_toward(1e-6, 10.0, 10.0 + height_m);
        approx::assert_relative_eq!(target.velocity_mps, expected, epsilon = 1e-3);
    }
}

/// A descent from hover speeds up at `descent_accel_mps2` (1 m/s²) to `max_descent_mps`
/// rather than stepping to it, so its thrust doesn't drop at once (#417); one already under
/// way keeps its rate.
#[test]
fn descent_speeds_up_at_descent_accel() {
    const DT: f32 = 0.01;
    // (descent speed on the last tick in m/s, ticks, expected feedforward m/s), waypoint 50 m down
    let cases = [
        (0.0, 1, -0.01),
        (0.0, 150, -1.5),
        (0.0, 400, -DEFAULT_MAX_DESCENT_MPS),
        (1.0, 1, -1.01),
        (DEFAULT_MAX_DESCENT_MPS, 1, -DEFAULT_MAX_DESCENT_MPS),
    ];
    for (speed_mps, ticks, expected) in cases {
        let (mut guidance, _) = guidance_at(60.0);
        guidance.descent_speed_mps = speed_mps;
        let mut target = AltitudeTarget {
            altitude_m: f32::NAN,
            velocity_mps: f32::NAN,
        };
        for _ in 0..ticks {
            // The vehicle keeps up with the target, so the leash never holds it.
            let altitude_m = guidance.altitude_target_m.unwrap_or(60.0);
            target = guidance.descend_toward(DT, 10.0, altitude_m);
        }
        approx::assert_relative_eq!(target.velocity_mps, expected, epsilon = 1e-3);
    }
}

/// A waypoint descent right after a land picks up the land's speed and keeps speeding up from
/// it; after a land that latched nothing (a NaN altitude), it starts from rest.
#[test]
fn a_descent_after_a_land_starts_from_the_land_speed() {
    const DT: f32 = 0.01;
    let land = ControlInput::Land(LandCommand::default());
    // (altitude the land saw, expected first descent feedforward m/s)
    let cases = [
        (
            60.0,
            -(DEFAULT_LAND_SPEED_MPS + DEFAULT_DESCENT_ACCEL_MPS2 * DT),
        ),
        (f32::NAN, -DEFAULT_DESCENT_ACCEL_MPS2 * DT),
    ];
    for (land_altitude_m, expected) in cases {
        let (mut guidance, mut state) = guidance_at(60.0);
        state.position_m.z = land_altitude_m;
        // Long enough for the land to reach its speed.
        for _ in 0..100 {
            guidance.guide(DT, FlightRegime::Copter, &land, &state);
        }
        guidance.altitude_target_m = None;
        let target = guidance.descend_toward(DT, 10.0, 60.0);
        approx::assert_relative_eq!(target.velocity_mps, expected, epsilon = 1e-4);
    }
}

/// The vehicle's altitude on a tick, in m.
type AltitudeAt = fn(usize) -> f32;

/// A descent's target runs at most `ALTITUDE_LEASH_M` below the vehicle, still or climbing,
/// so a vehicle climbing as the descent begins isn't pulled down past `max_descent_mps` (#400);
/// a NaN altitude keeps lowering the latched target.
#[test]
fn descent_target_stays_on_a_leash_below_the_vehicle() {
    const DT: f32 = 0.01;
    const TICKS: usize = 400;
    let drop_m = ramped_drop_m(TICKS, DT, DEFAULT_MAX_DESCENT_MPS);
    // (case, vehicle altitude on each tick, expected latched target), waypoint far below
    let cases: [(&str, AltitudeAt, f32); 4] = [
        ("still", |_| 10.0, 10.0 - ALTITUDE_LEASH_M),
        (
            "climbing",
            |tick| 10.0 + 0.02 * tick as f32,
            10.0 + 0.02 * TICKS as f32 - ALTITUDE_LEASH_M,
        ),
        (
            "sinking past the target",
            |tick| 10.0 - 0.05 * (tick - 1) as f32,
            10.0 - drop_m,
        ),
        (
            "NaN after 10 ticks",
            |tick| if tick <= 10 { 10.0 } else { f32::NAN },
            10.0 - drop_m,
        ),
    ];
    for (case, altitude_m, expected) in cases {
        let (mut guidance, mut state) = guidance_at(10.0);
        for tick in 1..=TICKS {
            state.position_m.z = altitude_m(tick);
            guidance.guide(DT, FlightRegime::Copter, &waypoint_at(-100.0), &state);
        }
        let target = guidance.altitude_target_m;
        assert!(target.is_some(), "{case}: unlatched");
        approx::assert_relative_eq!(target.unwrap_or(f32::NAN), expected, epsilon = 1e-3);
    }
}

/// While the target slews down, the altitude loop sees it and the descent rate: the thrust is
/// what a fresh controller gives for that target (on the first tick, the ramp's first step),
/// not the cut-off a far-below waypoint gave.
#[test]
fn a_low_waypoint_descends_instead_of_cutting_thrust() {
    const DT: f32 = 0.01;
    let (mut guidance, state) = guidance_at(10.0);
    let setpoint = attitude(guidance.guide(DT, FlightRegime::Copter, &waypoint_at(0.0), &state));
    let rate_mps = DEFAULT_DESCENT_ACCEL_MPS2 * DT;
    let expected = AltitudeController::default().control(
        &CopterGuidanceConfig::default().altitude,
        DT,
        AltitudeTarget {
            altitude_m: 10.0 - rate_mps * DT,
            velocity_mps: -rate_mps,
        },
        AltitudeEstimate {
            altitude_m: 10.0,
            velocity_mps: 0.0,
            attitude: state.attitude,
        },
    );
    approx::assert_relative_eq!(setpoint.thrust, expected, epsilon = 1e-6);
    assert!(setpoint.thrust > 0.0);
    assert!(!guidance.saturation().thrust);
}

/// A level point mass moving only vertically: thrust over hover accelerates it up, and a
/// ground at 0 m, when there is one, stops it.
struct VerticalPointMass {
    altitude_m: f32,
    climb_mps: f32,
    ground: bool,
    /// Sink rate when it first hit the ground.
    touchdown_mps: Option<f32>,
}

impl VerticalPointMass {
    fn at(altitude_m: f32, ground: bool) -> Self {
        VerticalPointMass {
            altitude_m,
            climb_mps: 0.0,
            ground,
            touchdown_mps: None,
        }
    }

    fn state(&self) -> VehicleState {
        VehicleState {
            position_m: Vec3::new(0.0, 0.0, self.altitude_m),
            velocity_mps: Vec3::new(0.0, 0.0, self.climb_mps),
            ..Default::default()
        }
    }

    fn step(&mut self, dt: f32, thrust: f32) {
        let hover = crate::controllers::DEFAULT_HOVER_THRUST;
        self.climb_mps += crate::GRAVITY * (thrust / hover - 1.0) * dt;
        self.altitude_m += self.climb_mps * dt;
        if self.ground && self.altitude_m <= 0.0 {
            self.touchdown_mps.get_or_insert(-self.climb_mps);
            self.altitude_m = 0.0;
            self.climb_mps = self.climb_mps.max(0.0);
        }
    }
}

/// Closed loop on a [`VerticalPointMass`]: a waypoint 20 m down is flown at about the descent limit. The limit is on the target, so the
/// vehicle overshoots it a little while catching up (about 20% on the default gains), not the
/// free fall a far-below target gave.
#[test]
fn point_mass_descends_at_the_limit() {
    const DT: f32 = 0.004;
    let (mut guidance, _) = guidance_at(30.0);
    let mut vehicle = VerticalPointMass::at(30.0, false);
    let mut fastest_descent_mps: f32 = 0.0;
    for _ in 0..(20.0 / DT) as usize {
        let setpoint = attitude(guidance.guide(
            DT,
            FlightRegime::Copter,
            &waypoint_at(10.0),
            &vehicle.state(),
        ));
        vehicle.step(DT, setpoint.thrust);
        fastest_descent_mps = fastest_descent_mps.max(-vehicle.climb_mps);
    }
    let altitude_m = vehicle.altitude_m;
    assert!((altitude_m - 10.0).abs() < 0.1, "ended at {altitude_m} m");
    assert!(
        fastest_descent_mps < 1.25 * DEFAULT_MAX_DESCENT_MPS,
        "descended at {fastest_descent_mps} m/s"
    );
}

/// A land holds its position and lowers the altitude target at `land_speed_mps`, speeding up
/// to it at `descent_accel_mps2` from hover, never more than `ALTITUDE_LEASH_M` below the
/// vehicle; leaving it (or priming) unlatches the target.
#[test]
fn land_descends_at_land_speed_on_a_leash() {
    const DT: f32 = 0.1;
    let land = ControlInput::Land(LandCommand {
        position_m: Vec2::new(3.0, 0.0),
    });
    let ramped_m = |ticks| ramped_drop_m(ticks, DT, DEFAULT_LAND_SPEED_MPS);
    // (ticks, expected latched target) with the vehicle held at 10 m
    let cases = [
        (1, 10.0 - ramped_m(1)),
        (9, 10.0 - ramped_m(9)),
        (20, 10.0 - ALTITUDE_LEASH_M),
    ];
    for (ticks, expected) in cases {
        let (mut guidance, state) = guidance_at(10.0);
        let mut setpoint = AttitudeSetpoint::default();
        for _ in 0..ticks {
            setpoint = attitude(guidance.guide(DT, FlightRegime::Copter, &land, &state));
        }
        let target = guidance.altitude_target_m.expect("latched");
        approx::assert_relative_eq!(target, expected, epsilon = 1e-5);
        assert!(setpoint.pitch_rad < 0.0, "noses toward the land position");
    }

    // On the ground the leashed target settles thrust below hover, not at zero.
    let (mut guidance, state) = guidance_at(0.0);
    let mut thrust = 0.0;
    for _ in 0..50 {
        thrust = attitude(guidance.guide(DT, FlightRegime::Copter, &land, &state)).thrust;
    }
    let hover = crate::controllers::DEFAULT_HOVER_THRUST;
    assert!(
        thrust > 0.2 * hover && thrust < 0.8 * hover,
        "thrust {thrust}"
    );

    guidance.guide(DT, FlightRegime::Copter, &ControlInput::default(), &state);
    assert!(guidance.altitude_target_m.is_none());
    guidance.guide(DT, FlightRegime::Copter, &land, &state);
    guidance.prime(&state);
    assert!(guidance.altitude_target_m.is_none());
}

/// A land taking over a faster waypoint descent slows to `land_speed_mps` at once: that only
/// adds thrust.
#[test]
fn a_land_after_a_faster_descent_slows_to_the_land_speed_at_once() {
    const DT: f32 = 0.1;
    let land = ControlInput::Land(LandCommand::default());
    let (mut guidance, state) = guidance_at(10.0);
    guidance.descent_speed_mps = DEFAULT_MAX_DESCENT_MPS;
    guidance.guide(DT, FlightRegime::Copter, &land, &state);
    let target = guidance.altitude_target_m.expect("latched");
    approx::assert_relative_eq!(target, 10.0 - DEFAULT_LAND_SPEED_MPS * DT, epsilon = 1e-5);
}

/// Closed loop: a `WaypointFollower` ending in `Land` drives guidance on a
/// [`VerticalPointMass`] with a ground at 0 m; it lands at about `land_speed_mps` and disarms on touchdown, not before.
#[test]
fn point_mass_route_lands_softly_and_disarms() {
    use crate::Commander;
    use crate::commanders::{RouteEnd, WaypointFollower};

    const DT: f32 = 0.004;
    let route = [WaypointCommand {
        altitude_m: 5.0,
        ..Default::default()
    }];
    let mut follower = WaypointFollower::new(route)
        .expect("non-empty")
        .with_route_end(RouteEnd::Land);
    let (mut guidance, _) = guidance_at(5.0);
    let mut vehicle = VerticalPointMass::at(5.0, true);
    let mut disarmed_at_m = None;
    for _ in 0..(30.0 / DT) as usize {
        let state = vehicle.state();
        let command = follower.next_command(DT, &state);
        if !command.arm {
            disarmed_at_m = Some(vehicle.altitude_m);
            break;
        }
        let setpoint = attitude(guidance.guide(DT, FlightRegime::Copter, &command.input, &state));
        vehicle.step(DT, setpoint.thrust);
    }
    let touchdown_mps = vehicle.touchdown_mps.expect("reached the ground");
    assert!(
        (touchdown_mps - DEFAULT_LAND_SPEED_MPS).abs() < 0.2,
        "touched down at {touchdown_mps} m/s"
    );
    assert!(disarmed_at_m == Some(0.0), "disarmed at {disarmed_at_m:?}");
    assert!(follower.is_complete());
}

/// `max_descent_mps` and `heading_rate_radps` must be positive and finite; `land_speed_mps`
/// must also exceed the touchdown detector's `DESCENDING_MPS`, or a land would never be seen
/// to descend and disarm.
#[test]
fn rejects_descent_and_heading_rates_out_of_range() {
    for value in [0.0, -1.0, f32::NAN, f32::INFINITY] {
        for (field, set) in [
            (
                "max_descent_mps",
                (|c: &mut CopterGuidanceConfig, v| c.max_descent_mps = v) as fn(&mut _, f32),
            ),
            ("land_speed_mps", |c, v| c.land_speed_mps = v),
            ("heading_rate_radps", |c, v| c.heading_rate_radps = v),
        ] {
            let mut config = CopterGuidanceConfig::default();
            set(&mut config, value);
            assert!(
                config.validate().map_err(|e| e.field) == Err(field),
                "{field} {value}"
            );
        }
    }
    // (land_speed_mps, valid)
    for (land_speed_mps, valid) in [(DESCENDING_MPS, false), (0.15, false), (0.25, true)] {
        let config = CopterGuidanceConfig {
            land_speed_mps,
            ..Default::default()
        };
        assert!(config.validate().is_ok() == valid, "{land_speed_mps}");
    }
}

/// Attitude-mode sticks centered, at `throttle`.
fn stick(throttle: f32) -> ControlInput {
    ControlInput::Attitude(AttitudeCommand {
        throttle,
        ..Default::default()
    })
}

/// Rate-mode sticks centered, at `throttle`.
fn rate(throttle: f32) -> ControlInput {
    ControlInput::Rate(RateCommand {
        throttle,
        ..Default::default()
    })
}

/// A level absolute attitude at `thrust`.
fn absolute(thrust: f32) -> ControlInput {
    ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
        thrust,
        ..Default::default()
    })
}

/// Guidance with `thrust` limits and default loops, primed on `state`.
fn limited(thrust: ThrustLimits, state: &VehicleState) -> CopterGuidance {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig {
        thrust,
        ..Default::default()
    });
    guidance.prime(state);
    guidance
}

/// Piloted inputs mark their setpoint idle while the throttle is at or below `IDLE_THROTTLE`
/// (or NaN), whatever the thrust floor; waypoint and land setpoints never are, even at zero
/// thrust (#326).
#[test]
fn idle_throttle_marks_the_setpoint_idle() {
    let level = VehicleState::default();
    let shooting_up = VehicleState {
        velocity_mps: Vec3::new(0.0, 0.0, 30.0),
        ..Default::default()
    };
    let absolute_rate = |thrust| {
        ControlInput::AbsoluteRate(crate::AbsoluteRateCommand {
            thrust,
            ..Default::default()
        })
    };
    let land = ControlInput::Land(LandCommand::default());
    // (input, state, idle)
    let cases = [
        (stick(0.0), &level, true),
        (stick(0.03), &level, true),
        (stick(IDLE_THROTTLE), &level, true),
        (stick(0.06), &level, false),
        (stick(f32::NAN), &level, true),
        (rate(0.03), &level, true),
        (rate(0.06), &level, false),
        (absolute(0.03), &level, true),
        (absolute(0.06), &level, false),
        (absolute_rate(0.03), &level, true),
        (absolute_rate(0.06), &level, false),
        (waypoint_at(0.0), &shooting_up, false), // thrust 0
        (land, &level, false),
    ];
    for floor in [0.0, 0.6] {
        for (input, state, idle) in &cases {
            let limits = ThrustLimits {
                min_thrust: floor,
                ..Default::default()
            };
            let setpoint = limited(limits, state).guide(0.004, FlightRegime::Copter, input, state);
            assert!(
                setpoint.idle() == *idle,
                "{input:?} at floor {floor}: {setpoint:?}"
            );
        }
    }
}

/// `min_thrust` raises every thrust while flying: a piloted throttle above idle (after the
/// tilt boost, so even inverted), waypoint and land flight always; an idle or NaN throttle
/// passes through, and a thrust above the floor is untouched. Raising it flags `thrust`.
#[test]
fn min_thrust_floors_the_thrust_while_flying() {
    const FLOOR: f32 = 0.6;
    let level = VehicleState::default();
    let inverted = VehicleState {
        attitude: nalgebra::UnitQuaternion::from_euler_angles(PI, 0.0, 0.0),
        ..Default::default()
    };
    // Shooting up toward a waypoint at its altitude: the altitude loop cuts thrust to 0.
    let shooting_up = VehicleState {
        velocity_mps: Vec3::new(0.0, 0.0, 30.0),
        ..Default::default()
    };
    let land = ControlInput::Land(LandCommand::default());
    // (input, state, expected thrust, thrust flagged)
    let cases = [
        (stick(0.3), &level, FLOOR, true),
        (stick(0.8), &level, 0.8, false),
        (stick(IDLE_THROTTLE), &level, IDLE_THROTTLE, false), // idle: the pilot can cut
        (stick(0.0), &level, 0.0, false),
        (stick(f32::NAN), &level, 0.0, false),
        (stick(0.3), &inverted, FLOOR, true), // the boost gives none; the floor holds
        (rate(0.3), &level, FLOOR, true),
        (rate(0.0), &level, 0.0, false),
        (absolute(0.3), &level, FLOOR, true),
        (absolute(0.0), &level, 0.0, false),
        (waypoint_at(0.0), &shooting_up, FLOOR, true),
        (land, &shooting_up, FLOOR, true),
    ];
    for (input, state, expected, flagged) in cases {
        let limits = ThrustLimits {
            min_thrust: FLOOR,
            ..Default::default()
        };
        let mut guidance = limited(limits, state);
        let thrust = guidance
            .guide(0.004, FlightRegime::Copter, &input, state)
            .thrust();
        assert!(thrust == expected, "{input:?}");
        assert!(guidance.saturation().thrust == flagged, "{input:?}");
    }
}

/// With the default limits, low thrust passes through as before.
#[test]
fn no_thrust_limits_by_default() {
    let (mut guidance, state) = guidance_at(0.0);
    let shooting_up = VehicleState {
        velocity_mps: Vec3::new(0.0, 0.0, 30.0),
        ..state
    };
    let sinking = VehicleState {
        velocity_mps: Vec3::new(0.0, 0.0, -5.0),
        ..state
    };
    assert!(
        guidance
            .guide(0.004, FlightRegime::Copter, &stick(0.3), &state)
            .thrust()
            == 0.3
    );
    assert!(
        guidance
            .guide(0.004, FlightRegime::Copter, &stick(0.3), &sinking)
            .thrust()
            == 0.3
    );
    assert!(
        guidance
            .guide(0.004, FlightRegime::Copter, &waypoint_at(0.0), &shooting_up)
            .thrust()
            == 0.0
    );
}

/// The sink guard adds `sink_gain` per m/s of sink past `sink_limit_mps` to piloted thrust
/// (on top of the floor), saturated to 1, and flags `thrust`; at or inside the limit, idle,
/// disabled, or with a NaN climb rate it adds nothing.
#[test]
fn sink_guard_adds_thrust_past_the_limit() {
    // (input, climb m/s, (sink_limit_mps, min_thrust), expected thrust, thrust flagged)
    let cases = [
        (stick(0.5), 0.0, (1.0, 0.0), 0.5, false),
        (stick(0.5), -1.0, (1.0, 0.0), 0.5, false), // at the limit
        (stick(0.5), -2.0, (1.0, 0.0), 0.7, true),
        (stick(0.5), -10.0, (1.0, 0.0), 1.0, true), // saturated
        (stick(0.0), -3.0, (1.0, 0.0), 0.0, false), // idle: on the ground or cut
        (stick(0.5), f32::NAN, (1.0, 0.0), 0.5, false),
        (stick(0.5), -5.0, (0.0, 0.0), 0.5, false), // disabled
        (stick(0.3), -2.0, (1.0, 0.6), 0.8, true),  // on top of the floor
        (rate(0.5), -2.0, (1.0, 0.0), 0.7, true),
        (absolute(0.5), -2.0, (1.0, 0.0), 0.7, true),
    ];
    for (input, climb_mps, (sink_limit_mps, min_thrust), expected, flagged) in cases {
        let limits = ThrustLimits {
            sink_limit_mps,
            sink_gain: 0.2,
            min_thrust,
            ..Default::default()
        };
        let state = VehicleState {
            velocity_mps: Vec3::new(0.0, 0.0, climb_mps),
            ..Default::default()
        };
        let mut guidance = limited(limits, &state);
        let thrust = guidance
            .guide(0.004, FlightRegime::Copter, &input, &state)
            .thrust();
        approx::assert_relative_eq!(thrust, expected, epsilon = 1e-5);
        assert!(
            guidance.saturation().thrust == flagged,
            "{input:?} at {climb_mps}"
        );
    }
}

/// The tilt boost stops at `max_boost_thrust`, flagging `thrust`; a stick above it passes
/// through unboosted.
#[test]
fn tilt_boost_stops_at_max_boost_thrust() {
    let cos = |deg: f32| deg.to_radians().cos();
    // (throttle, roll in degrees, expected thrust, thrust flagged)
    let cases = [
        (0.8, 45.0_f32, 0.9, true),
        (0.8, 20.0, 0.8 / cos(20.0), false),
        (0.95, 45.0, 0.95, true), // above the ceiling: the stick, not boosted
        (0.95, 0.0, 0.95, false),
        (f32::NAN, 45.0, 0.0, false),
    ];
    for (throttle, roll_deg, expected, flagged) in cases {
        let limits = ThrustLimits {
            max_boost_thrust: 0.9,
            ..Default::default()
        };
        let state = VehicleState {
            attitude: nalgebra::UnitQuaternion::from_euler_angles(roll_deg.to_radians(), 0.0, 0.0),
            ..Default::default()
        };
        let mut guidance = limited(limits, &state);
        let thrust = guidance
            .guide(0.004, FlightRegime::Copter, &stick(throttle), &state)
            .thrust();
        approx::assert_relative_eq!(thrust, expected, epsilon = 1e-5);
        assert!(
            guidance.saturation().thrust == flagged,
            "{throttle} at {roll_deg}°"
        );
    }
}

/// `ThrustLimits::validate`, reached through the guidance config with the `thrust` section.
#[test]
fn rejects_thrust_limits_out_of_range() {
    let base = ThrustLimits::default;
    // (limits, rejected field or None)
    let cases = [
        (
            ThrustLimits {
                min_thrust: 0.6,
                sink_limit_mps: 1.0,
                sink_gain: 0.2,
                ..base()
            },
            None,
        ),
        (
            ThrustLimits {
                min_thrust: -0.1,
                ..base()
            },
            Some("min_thrust"),
        ),
        (
            ThrustLimits {
                min_thrust: 1.0,
                ..base()
            },
            Some("min_thrust"),
        ),
        (
            ThrustLimits {
                min_thrust: f32::NAN,
                ..base()
            },
            Some("min_thrust"),
        ),
        (
            ThrustLimits {
                sink_limit_mps: -1.0,
                ..base()
            },
            Some("sink_limit_mps"),
        ),
        (
            ThrustLimits {
                sink_limit_mps: f32::INFINITY,
                ..base()
            },
            Some("sink_limit_mps"),
        ),
        (
            ThrustLimits {
                sink_gain: -0.1,
                ..base()
            },
            Some("sink_gain"),
        ),
        (
            ThrustLimits {
                sink_gain: f32::NAN,
                ..base()
            },
            Some("sink_gain"),
        ),
        (
            ThrustLimits {
                max_boost_thrust: 0.5,
                ..base()
            },
            None,
        ),
        (
            ThrustLimits {
                max_boost_thrust: 0.0,
                ..base()
            },
            Some("max_boost_thrust"),
        ),
        (
            ThrustLimits {
                max_boost_thrust: 1.1,
                ..base()
            },
            Some("max_boost_thrust"),
        ),
    ];
    for (thrust, rejected) in cases {
        let config = CopterGuidanceConfig {
            thrust,
            ..Default::default()
        };
        let result = config.validate().map_err(|e| (e.section, e.field));
        assert!(
            result == rejected.map_or(Ok(()), |field| Err((Some("thrust"), field))),
            "{thrust:?}"
        );
    }
}

/// With `max_accel_mps2` set, a primed hover starting a leg asks for the tilt of the shaped
/// acceleration, not of the leg's whole speed: the velocity target starts at the vehicle's.
#[test]
fn shaped_lateral_loop_starts_a_leg_gently() {
    let leg = ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(0.0, 100.0),
        cruise_speed_mps: 3.0,
        ..Default::default()
    });
    // The loop's first run covers the 0.004 s tick, from rest, heading north, I 0.4:
    // (max_accel_mps2, expected right roll in degrees from the acceleration east)
    let cases = [
        // Off: P 1.26 on the whole 3 m/s, plus I on 0.004 s of it.
        (
            0.0,
            libm::atanf((1.26 * 3.0 + 0.4 * 0.012) / GRAVITY).to_degrees(),
        ),
        // 1 m/s²: its feedforward, plus P and I on the 0.004 m/s step.
        (
            1.0,
            libm::atanf((1.0 + 1.26 * 0.004 + 0.4 * 0.000016) / GRAVITY).to_degrees(),
        ),
    ];
    for (max_accel_mps2, expected_deg) in cases {
        let mut config = CopterGuidanceConfig::default();
        config.lateral.max_accel_mps2 = max_accel_mps2;
        let mut guidance = CopterGuidance::new(config);
        let state = VehicleState::default();
        guidance.prime(&state);
        let Setpoint::Attitude(setpoint) =
            guidance.guide(0.004, FlightRegime::Copter, &leg, &state)
        else {
            panic!("waypoint flight gives an attitude setpoint");
        };
        let roll_deg = setpoint.roll_rad.to_degrees();
        assert!(
            (roll_deg - expected_deg).abs() < 0.05,
            "limit {max_accel_mps2}: roll {roll_deg}°, expected {expected_deg}°"
        );
    }
}

/// A level vehicle `altitude_m` up, not moving, pushed east at 0.6 m/s²: with 0.3/s momentum
/// drag, a 2 m/s wind toward the east.
fn pushed_east_at(altitude_m: f32) -> VehicleState {
    VehicleState {
        position_m: Vec3::new(0.0, 0.0, altitude_m),
        acceleration_mps2: Vec3::new(0.0, 0.6, 0.0),
        ..Default::default()
    }
}

/// Guidance with 0.3/s momentum drag on both axes, primed on the ground at 0 m.
fn wind_guidance() -> CopterGuidance {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig {
        wind: WindConfig {
            momentum_per_s: Vec2::new(0.3, 0.3),
            ..Default::default()
        },
        ..Default::default()
    });
    guidance.prime(&pushed_east_at(0.0));
    guidance
}

/// The wind estimate updates only more than `AIRBORNE_HEIGHT_M` above the altitude primed on
/// the ground, under any input; without drag coefficients it never does.
#[test]
fn estimates_the_wind_only_when_airborne() {
    let hover = ControlInput::Waypoint(WaypointCommand::default());
    let sticks = ControlInput::Attitude(AttitudeCommand::default());
    // (case, guidance, input, altitude, expected estimate)
    let cases = [
        ("on the ground", wind_guidance(), &hover, 0.0, None),
        ("below the height", wind_guidance(), &hover, 0.9, None),
        (
            "airborne",
            wind_guidance(),
            &hover,
            5.0,
            Some(Vec2::new(0.0, 2.0)),
        ),
        (
            "airborne on sticks",
            wind_guidance(),
            &sticks,
            5.0,
            Some(Vec2::new(0.0, 2.0)),
        ),
        (
            "no drag coefficients",
            CopterGuidance::new(CopterGuidanceConfig::default()),
            &hover,
            5.0,
            None,
        ),
    ];
    for (case, mut guidance, input, altitude_m, expected) in cases {
        guidance.prime(&pushed_east_at(0.0));
        for _ in 0..ticks_for(10.0) {
            guidance.guide(
                0.004,
                FlightRegime::Copter,
                input,
                &pushed_east_at(altitude_m),
            );
        }
        let wind = guidance.wind().map(|estimate| estimate.velocity_mps);
        match expected {
            None => assert!(wind.is_none(), "{case}: {wind:?}"),
            Some(expected) => {
                let error = (wind.expect("estimated") - expected).norm();
                assert!(error < 0.01, "{case}: {wind:?}, expected {expected}");
            }
        }
    }
}

/// Primed in the air (a VTOL back in hover) guidance keeps the wind and the ground altitude;
/// primed back on the ground, it forgets the wind.
#[test]
fn priming_in_the_air_keeps_the_wind() {
    let hover = ControlInput::Waypoint(WaypointCommand::default());
    let mut guidance = wind_guidance();
    for _ in 0..ticks_for(10.0) {
        guidance.guide(0.004, FlightRegime::Copter, &hover, &pushed_east_at(5.0));
    }
    let before = guidance.wind();
    guidance.prime(&pushed_east_at(5.0));
    assert!(guidance.wind() == before);
    guidance.guide(0.004, FlightRegime::Copter, &hover, &pushed_east_at(5.0));
    assert!(guidance.wind().is_some());

    guidance.prime(&pushed_east_at(0.5));
    assert!(guidance.wind().is_none());
}

/// Paused (a VTOL outside hover), the wind estimate doesn't update and keeps what it had;
/// resumed, it updates again.
#[test]
fn a_paused_wind_estimate_keeps_its_value() {
    let hover = ControlInput::Waypoint(WaypointCommand::default());
    let mut guidance = wind_guidance();
    guidance.set_wind_paused(true);
    for _ in 0..ticks_for(2.0) {
        guidance.guide(0.004, FlightRegime::Copter, &hover, &pushed_east_at(5.0));
    }
    assert!(guidance.wind().is_none());

    guidance.set_wind_paused(false);
    for _ in 0..ticks_for(10.0) {
        guidance.guide(0.004, FlightRegime::Copter, &hover, &pushed_east_at(5.0));
    }
    let settled = guidance.wind();
    guidance.set_wind_paused(true);
    let calm = VehicleState {
        acceleration_mps2: Vec3::zeros(),
        ..pushed_east_at(5.0)
    };
    for _ in 0..ticks_for(2.0) {
        guidance.guide(0.004, FlightRegime::Copter, &hover, &calm);
    }
    assert!(guidance.wind() == settled);
}

/// With `wind_heading_min_mps` set, a hold or a land turns the heading-hold target 80° off
/// where a steady 2 m/s wind blowing east (toward 90°) comes from, to the wingtip heading
/// nearer the vehicle's (−10° from 30°); a waypoint's heading wins, a long leg keeps its own heading until within
/// `waypoint_yaw_min_leg_m`, and 0 turns nothing.
#[test]
fn turns_a_wingtip_into_the_wind_while_holding_or_landing() {
    const DT: f32 = 0.01;
    // Long enough for the estimate to settle and the target to turn 30° at 60°/s.
    const TICKS: usize = 1000;
    let hold = ControlInput::Waypoint(WaypointCommand {
        altitude_m: 10.0,
        ..Default::default()
    });
    let east_leg = ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(0.0, 50.0),
        altitude_m: 10.0,
        ..Default::default()
    });
    let facing = ControlInput::Waypoint(WaypointCommand {
        altitude_m: 10.0,
        heading_rad: Some(FRAC_PI_4),
        ..Default::default()
    });
    let land = ControlInput::Land(LandCommand::default());
    // (case, min wind m/s, input, expected heading-hold target deg)
    let cases: [(&str, f32, &ControlInput, f32); 5] = [
        ("hold", 1.0, &hold, -10.0),
        ("land", 1.0, &land, -10.0),
        ("off", 0.0, &hold, 30.0),
        ("waypoint heading wins", 1.0, &facing, 45.0),
        ("long leg turns along it", 1.0, &east_leg, 90.0),
    ];
    for (case, min_speed_mps, input, expected_deg) in cases {
        let config = CopterGuidanceConfig {
            wind_heading_min_mps: min_speed_mps,
            // The vehicle here never yaws: no yield.
            heading_yield_s: 0.0,
            // 0.6 m/s² of push at 0.3/s is 2 m/s of wind.
            wind: WindConfig {
                momentum_per_s: Vec2::new(0.3, 0.3),
                ..Default::default()
            },
            ..Default::default()
        };
        let mut guidance = CopterGuidance::new(config);
        let state = VehicleState {
            position_m: Vec3::new(0.0, 0.0, 10.0),
            acceleration_mps2: Vec3::new(0.0, 0.6, 0.0),
            heading_rad: 30.0_f32.to_radians(),
            ..Default::default()
        };
        guidance.prime(&VehicleState {
            position_m: Vec3::zeros(),
            ..state
        });
        let mut setpoint = AttitudeSetpoint::default();
        for _ in 0..TICKS {
            setpoint = attitude(guidance.guide(DT, FlightRegime::Copter, input, &state));
        }
        let heading_deg = wrap_angle(setpoint.heading_rad).to_degrees();
        assert2::check!(
            (heading_deg - expected_deg).abs() < 0.5,
            "{case}: {heading_deg}°"
        );
    }
}

/// A long leg ends on station: within `waypoint_yaw_min_leg_m` of its waypoint the target
/// turns from the leg's direction to the wingtip heading.
#[test]
fn a_long_leg_turns_a_wingtip_into_the_wind_on_arrival() {
    const DT: f32 = 0.01;
    let config = CopterGuidanceConfig {
        wind_heading_min_mps: 1.0,
        heading_yield_s: 0.0,
        wind: WindConfig {
            momentum_per_s: Vec2::new(0.3, 0.3),
            ..Default::default()
        },
        ..Default::default()
    };
    let mut guidance = CopterGuidance::new(config);
    guidance.prime(&state_at(0.0, 0.0, 0.0));
    let east_leg = ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(0.0, 50.0),
        altitude_m: 10.0,
        ..Default::default()
    });
    let mut state = VehicleState {
        position_m: Vec3::new(0.0, 0.0, 10.0),
        acceleration_mps2: Vec3::new(0.0, 0.6, 0.0),
        heading_rad: 30.0_f32.to_radians(),
        ..Default::default()
    };
    let mut setpoint = AttitudeSetpoint::default();
    // (vehicle east of the start m, expected target deg after 10 s there)
    for (east_m, expected_deg) in [(0.0, 90.0), (48.0, -10.0)] {
        state.position_m.y = east_m;
        for _ in 0..1000 {
            setpoint = attitude(guidance.guide(DT, FlightRegime::Copter, &east_leg, &state));
        }
        let heading_deg = wrap_angle(setpoint.heading_rad).to_degrees();
        assert2::check!(
            (heading_deg - expected_deg).abs() < 0.5,
            "{east_m} m: {heading_deg}°"
        );
    }
}
