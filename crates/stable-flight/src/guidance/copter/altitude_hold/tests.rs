use approx::assert_relative_eq;
use assert2::{assert, check};

use super::*;
use crate::guidance::copter::CopterGuidanceConfig;
use crate::{AttitudeCommand, ControlInput, FlightRegime, Guidance, Setpoint, WaypointCommand};

const DT: f32 = 0.004;

/// The climb stick maps to a rate: 0 inside the deadzone, linear past it to the top rate at
/// either end, saturated past ±1, NaN centered.
#[test]
fn climb_stick_maps_to_a_rate() {
    let config = AltitudeHoldConfig::default();
    // (case, stick, climb rate in m/s)
    let rows = [
        ("centered", 0.0, 0.0),
        ("inside the deadzone up", 0.2, 0.0),
        ("inside the deadzone down", -0.19, 0.0),
        ("halfway past the deadzone up", 0.6, 1.25),
        ("halfway past the deadzone down", -0.6, -0.75),
        ("full up", 1.0, 2.5),
        ("full down", -1.0, -1.5),
        ("past full up", 3.0, 2.5),
        ("past full down", -3.0, -1.5),
        ("NaN", f32::NAN, 0.0),
    ];
    for (case, stick, expected_mps) in rows {
        assert_relative_eq!(config.climb_rate_mps(stick), expected_mps, epsilon = 1e-6);
        check!(config.climb_rate_mps(stick).is_finite(), "{case}");
    }
    let uneven = AltitudeHoldConfig {
        deadzone: 0.0,
        max_climb_mps: 3.0,
        max_sink_mps: 1.5,
        ..config
    };
    check!(uneven.climb_rate_mps(0.5) == 1.5);
    check!(uneven.climb_rate_mps(-0.5) == -0.75);
}

/// `validate` rejects a deadzone outside `[0, 1)` and a non-positive or non-finite rate or
/// acceleration.
#[test]
fn config_validation() {
    let good = AltitudeHoldConfig::default();
    check!(good.validate() == Ok(()));
    check!(
        AltitudeHoldConfig {
            deadzone: 0.0,
            ..good
        }
        .validate()
            == Ok(())
    );
    let rows = [
        (
            "deadzone",
            AltitudeHoldConfig {
                deadzone: 1.0,
                ..good
            },
        ),
        (
            "deadzone",
            AltitudeHoldConfig {
                deadzone: -0.1,
                ..good
            },
        ),
        (
            "deadzone",
            AltitudeHoldConfig {
                deadzone: f32::NAN,
                ..good
            },
        ),
        (
            "max_climb_mps",
            AltitudeHoldConfig {
                max_climb_mps: 0.0,
                ..good
            },
        ),
        (
            "max_sink_mps",
            AltitudeHoldConfig {
                max_sink_mps: f32::INFINITY,
                ..good
            },
        ),
        (
            "accel_mps2",
            AltitudeHoldConfig {
                accel_mps2: -1.0,
                ..good
            },
        ),
        (
            "lock_speed_mps",
            AltitudeHoldConfig {
                lock_speed_mps: 0.0,
                ..good
            },
        ),
    ];
    for (field, config) in rows {
        let error = config.validate().expect_err("invalid");
        check!(error.field == field, "{config:?}");
    }
    let mut guidance = CopterGuidanceConfig::default();
    guidance.altitude_hold.deadzone = 1.0;
    let error = guidance.validate().expect_err("invalid");
    check!(error.section == Some("altitude_hold"));
}

fn hold(climb: f32) -> ControlInput {
    ControlInput::AltitudeHold(AltitudeHoldCommand {
        climb,
        ..Default::default()
    })
}

fn at(altitude_m: f32, climb_mps: f32) -> VehicleState {
    VehicleState {
        position_m: [0.0, 0.0, altitude_m].into(),
        velocity_mps: [0.0, 0.0, climb_mps].into(),
        ..Default::default()
    }
}

fn attitude(setpoint: Setpoint) -> AttitudeSetpoint {
    let Setpoint::Attitude(attitude) = setpoint else {
        panic!("not an attitude: {setpoint:?}");
    };
    attitude
}

/// Guidance primed on the ground at 0 m, as at arming.
fn armed_on_ground() -> CopterGuidance {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    guidance.prime(&at(0.0, 0.0));
    guidance
}

fn guide(
    guidance: &mut CopterGuidance,
    input: ControlInput,
    state: &VehicleState,
) -> AttitudeSetpoint {
    attitude(guidance.guide(DT, FlightRegime::Copter, &input, state))
}

/// Armed on the ground, a stick at or below the deadzone idles the motors; above it the
/// vehicle takes off, its target rising from the ground.
#[test]
fn idles_on_the_ground_until_the_stick_climbs() {
    for stick in [0.0, 0.2, -1.0, f32::NAN] {
        let mut guidance = armed_on_ground();
        let setpoint = guide(&mut guidance, hold(stick), &at(0.0, 0.0));
        check!(setpoint.idle, "stick {stick}");
        check!(setpoint.thrust == 0.0, "stick {stick}");
    }

    let mut guidance = armed_on_ground();
    guide(&mut guidance, hold(0.0), &at(0.0, 0.0));
    let setpoint = guide(&mut guidance, hold(1.0), &at(0.0, 0.0));
    check!(!setpoint.idle);
    let hover = CopterGuidanceConfig::default().altitude.hover_thrust;
    check!(setpoint.thrust > hover, "climbs off the ground");
    let target_m = guidance.altitude_target_m.expect("latched");
    check!(
        target_m > 0.0 && target_m < 0.01,
        "one tick's climb: {target_m}"
    );
}

/// Entered in the air, from flight that wasn't idle, even below the airborne height, it flies:
/// a centered stick holds the height at hover thrust.
#[test]
fn entered_in_the_air_it_holds_the_height() {
    let hover = CopterGuidanceConfig::default().altitude.hover_thrust;
    for altitude_m in [0.5, 10.0] {
        let mut guidance = armed_on_ground();
        let flying = ControlInput::Attitude(AttitudeCommand {
            throttle: hover,
            ..Default::default()
        });
        guide(&mut guidance, flying, &at(altitude_m, 0.0));
        let setpoint = guide(&mut guidance, hold(0.0), &at(altitude_m, 0.0));
        check!(!setpoint.idle, "at {altitude_m} m");
        assert_relative_eq!(setpoint.thrust, hover, epsilon = 1e-4);
        check!(guidance.altitude_target_m == Some(altitude_m));
    }
}

/// The target's climb rate follows the stick at the configured acceleration, and the target
/// stays within the leash of a vehicle that doesn't follow it.
#[test]
fn the_target_follows_the_stick_and_stays_leashed() {
    let config = CopterGuidanceConfig::default();
    let mut guidance = armed_on_ground();
    guide(
        &mut guidance,
        ControlInput::Attitude(AttitudeCommand {
            throttle: 0.5,
            ..Default::default()
        }),
        &at(10.0, 0.0),
    );
    let ticks = (0.5 / DT).round() as usize;
    for _ in 0..ticks {
        guide(&mut guidance, hold(1.0), &at(10.0, 0.0));
    }
    // Half a second at 2.5 m/s²: climbing at 1.25 m/s.
    assert_relative_eq!(-guidance.descent_speed_mps, 1.25, epsilon = 0.01);
    for _ in 0..(5.0 / DT) as usize {
        guide(&mut guidance, hold(1.0), &at(10.0, 0.0));
    }
    check!(-guidance.descent_speed_mps == config.altitude_hold.max_climb_mps);
    check!(guidance.altitude_target_m == Some(10.0 + config.altitude_leash_m));
    for _ in 0..(5.0 / DT) as usize {
        guide(&mut guidance, hold(-1.0), &at(10.0, 0.0));
    }
    check!(guidance.descent_speed_mps == config.altitude_hold.max_sink_mps);
    check!(guidance.altitude_target_m == Some(10.0 - config.altitude_leash_m));
}

/// A descent on the stick that the ground stops idles the motors there; the next climb takes
/// off from the vehicle.
#[test]
fn a_descent_the_ground_stops_idles_there() {
    let mut guidance = armed_on_ground();
    guide(&mut guidance, hold(1.0), &at(0.0, 0.0));
    for _ in 0..(1.0 / DT) as usize {
        let setpoint = guide(&mut guidance, hold(-1.0), &at(3.0, -1.0));
        check!(!setpoint.idle, "descending");
    }
    let mut idled_s = None;
    for tick in 0..(3.0 / DT) as usize {
        if guide(&mut guidance, hold(-1.0), &at(0.0, 0.0)).idle {
            idled_s = Some(tick as f32 * DT);
            break;
        }
    }
    let idled_s = idled_s.expect("touched down");
    check!(
        idled_s > 0.9 && idled_s < 1.2,
        "after the touchdown hold: {idled_s} s"
    );
    check!(guidance.altitude_target_m == None);
    let takeoff = guide(&mut guidance, hold(1.0), &at(0.0, 0.0));
    check!(!takeoff.idle);
    check!(guidance.altitude_target_m.is_some_and(|t| t < 0.01));
}

/// Switching between waypoint and altitude-hold flight carries the altitude target over, so
/// neither steps it; leaving for attitude flight drops it.
#[test]
fn the_target_carries_between_waypoint_and_altitude_hold() {
    let mut guidance = armed_on_ground();
    let waypoint = ControlInput::Waypoint(WaypointCommand {
        altitude_m: 5.0,
        ..Default::default()
    });
    guide(&mut guidance, waypoint, &at(10.0, 0.0));
    let waypoint_target = guidance.altitude_target_m.expect("descending target");
    guide(&mut guidance, hold(0.0), &at(10.0, 0.0));
    let held = guidance.altitude_target_m.expect("carried");
    assert!(
        (held - waypoint_target).abs() < 0.02,
        "{held} vs {waypoint_target}"
    );
    guide(
        &mut guidance,
        ControlInput::Attitude(AttitudeCommand::default()),
        &at(10.0, 0.0),
    );
    check!(guidance.altitude_target_m == None);
}

/// A NaN altitude holds hover thrust and latches nothing, without panicking.
#[test]
fn a_nan_altitude_holds_hover() {
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let setpoint = guide(&mut guidance, hold(1.0), &at(f32::NAN, f32::NAN));
    let hover = CopterGuidanceConfig::default().altitude.hover_thrust;
    check!(setpoint.thrust == hover);
    check!(guidance.altitude_target_m == None);
}

/// Let go after a climb that ran ahead of its target, as the RealFlight QuadCopterX did on a
/// host running ticks slow (#441), the vehicle brakes with the target on it and holds the
/// height it slows at, not the target it ran ahead of: it doesn't sink back.
#[test]
fn a_released_climb_holds_where_the_vehicle_stops() {
    let config = CopterGuidanceConfig::default();
    let mut guidance = armed_on_ground();
    let hover = ControlInput::Attitude(AttitudeCommand {
        throttle: config.altitude.hover_thrust,
        ..Default::default()
    });
    guide(&mut guidance, hover, &at(10.0, 0.0));
    // A second's climb on the stick, the vehicle 0.4 m over the target the whole way.
    let ticks = (1.0 / DT).round() as usize;
    for tick in 0..ticks {
        let target_m = guidance.altitude_target_m.unwrap_or(10.0);
        guide(&mut guidance, hold(0.6), &at(target_m + 0.4, 0.6));
        check!(!guidance.hold.locked, "tick {tick}");
    }
    let released_m = guidance.altitude_target_m.expect("latched") + 0.4;

    // Released: braking, the target sits on the vehicle until it is slow.
    guide(&mut guidance, hold(0.0), &at(released_m, 0.8));
    check!(guidance.altitude_target_m == Some(released_m));
    check!(!guidance.hold.locked);
    for _ in 0..ticks {
        guide(&mut guidance, hold(0.0), &at(released_m + 0.05, 0.3));
    }
    check!(
        guidance.hold.locked,
        "slower than lock_speed_mps once the rate fell to 0"
    );
    let locked_m = guidance.altitude_target_m.expect("locked");
    check!(locked_m == released_m + 0.05, "the height it slowed at");

    // Locked, the target holds while the vehicle drifts; a climb unlocks it.
    guide(&mut guidance, hold(0.0), &at(released_m - 0.2, -0.1));
    check!(guidance.altitude_target_m == Some(locked_m));
    guide(&mut guidance, hold(1.0), &at(locked_m, 0.0));
    check!(!guidance.hold.locked);
}
