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

/// `validate` rejects a deadzone outside `[0, 1)`, a non-positive or non-finite rate or
/// acceleration, an invalid touchdown, and a grounded target share outside `(0, 1]`.
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
        (
            "hold_s",
            AltitudeHoldConfig {
                touchdown: TouchdownConfig {
                    hold_s: 0.0,
                    ..good.touchdown
                },
                ..good
            },
        ),
        (
            "grounded_target_share",
            AltitudeHoldConfig {
                grounded_target_share: 0.0,
                ..good
            },
        ),
        (
            "grounded_target_share",
            AltitudeHoldConfig {
                grounded_target_share: 1.5,
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
/// stays within the leash of a vehicle that doesn't follow it (held at 10 m, moving at a
/// climb or sink of its own, so it doesn't read as on the ground).
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
        guide(&mut guidance, hold(-1.0), &at(10.0, -0.5));
    }
    check!(guidance.descent_speed_mps == config.altitude_hold.max_sink_mps);
    check!(guidance.altitude_target_m == Some(10.0 - config.altitude_leash_m));
}

/// A descent on the stick that the ground stops idles the motors there, once the target has
/// run a quarter of the leash below the vehicle and the vehicle has held still for the touchdown hold;
/// the next climb takes off from the vehicle.
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
    // The target, leashed 1 m over the stopped vehicle, sinks to 0.25 m below it at 1.5 m/s
    // (0.83 s), then the vehicle holds still for the 1 s touchdown hold.
    check!(
        idled_s > 1.75 && idled_s < 1.95,
        "after the target ran below and the touchdown hold: {idled_s} s"
    );
    check!(guidance.altitude_target_m == None);
    let takeoff = guide(&mut guidance, hold(1.0), &at(0.0, 0.0));
    check!(!takeoff.idle);
    check!(guidance.altitude_target_m.is_some_and(|t| t < 0.01));
}

/// Switching between waypoint and altitude-hold flight carries the altitude target over, so
/// neither steps it back to the vehicle; leaving for attitude flight drops it. The vehicle
/// lags its target here (held at 10 m), so a target rebuilt from the vehicle would differ.
#[test]
fn the_target_carries_between_waypoint_and_altitude_hold() {
    let mut guidance = armed_on_ground();
    let vehicle = at(10.0, 0.0);
    guide(
        &mut guidance,
        ControlInput::Attitude(AttitudeCommand {
            throttle: 0.5,
            ..Default::default()
        }),
        &vehicle,
    );
    // A climb on the stick runs the target to the leash above the lagging vehicle.
    for _ in 0..(2.0 / DT) as usize {
        guide(&mut guidance, hold(1.0), &vehicle);
    }
    let climbed_m = guidance.altitude_target_m.expect("latched");
    check!(climbed_m == 11.0);

    // To a waypoint below: its descent starts from the carried target, not the vehicle.
    let waypoint = ControlInput::Waypoint(WaypointCommand {
        altitude_m: 5.0,
        ..Default::default()
    });
    guide(&mut guidance, waypoint, &vehicle);
    let waypoint_m = guidance.altitude_target_m.expect("descending target");
    check!(
        (waypoint_m - climbed_m).abs() < 0.02,
        "{waypoint_m} vs {climbed_m}"
    );

    // Back to a climb on the stick: it moves on from the waypoint's target.
    guide(&mut guidance, hold(1.0), &vehicle);
    let held_m = guidance.altitude_target_m.expect("carried");
    check!(
        (held_m - waypoint_m).abs() < 0.02,
        "{held_m} vs {waypoint_m}"
    );

    guide(
        &mut guidance,
        ControlInput::Attitude(AttitudeCommand::default()),
        &at(10.0, 0.0),
    );
    check!(guidance.altitude_target_m == None);
}

/// Held still on the ground against a stick down, the altitude loop has pulled the thrust well
/// under hover before the vehicle idles: the low-thrust check of ArduPilot's and PX4's land
/// detectors (`Copter::update_land_detector`, `MulticopterLandDetector`) that the target
/// condition stands in for. PX4's ground contact relaxes its share to 0.6 of hover with a hover
/// estimate; the default config falls to 0.27 of a 0.5 hover, so a vehicle that still holds
/// its height in the air is held up by something carrying about half its weight. The integral
/// the ground pushed against is gone after the idle.
#[test]
fn the_thrust_falls_well_under_hover_before_an_idle() {
    let config = CopterGuidanceConfig::default();
    let mut guidance = armed_on_ground();
    guide(&mut guidance, hold(1.0), &at(0.0, 0.0));
    let mut last_thrust = config.altitude.hover_thrust;
    let mut idled = false;
    for _ in 0..(4.0 / DT) as usize {
        let setpoint = guide(&mut guidance, hold(-1.0), &at(0.0, 0.0));
        if setpoint.idle {
            idled = true;
            break;
        }
        last_thrust = setpoint.thrust;
    }
    assert!(idled);
    let share = last_thrust / config.altitude.hover_thrust;
    check!(share < 0.6, "thrust {last_thrust} at idle");
    check!(guidance.altitude == AltitudeController::default());
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

/// A vehicle already resting on the ground with its height locked (a stick let go less than a
/// second after touchdown) idles once the stick is pushed down, with no descent to see
/// (#441). A slow sink in the air, which the vehicle follows, never idles.
#[test]
fn a_stick_down_on_the_ground_idles_without_a_descent() {
    let mut guidance = armed_on_ground();
    guide(&mut guidance, hold(1.0), &at(0.0, 0.0));
    // Released on the ground: brakes and locks the height there, flying, not idle.
    for _ in 0..(1.0 / DT) as usize {
        let setpoint = guide(&mut guidance, hold(0.0), &at(0.0, 0.0));
        check!(!setpoint.idle);
    }
    check!(guidance.hold.locked);
    let mut idled_s = None;
    for tick in 0..(4.0 / DT) as usize {
        if guide(&mut guidance, hold(-1.0), &at(0.0, 0.0)).idle {
            idled_s = Some(tick as f32 * DT);
            break;
        }
    }
    let idled_s = idled_s.expect("idled on the ground");
    check!(idled_s < 2.5, "{idled_s} s");

    // In the air, a stick just past the deadzone sinks at 0.03 m/s; the vehicle follows the
    // target to within a centimeter, slower than the touchdown's stopped speed, for 10 s.
    let mut guidance = armed_on_ground();
    guide(
        &mut guidance,
        ControlInput::Attitude(AttitudeCommand {
            throttle: 0.5,
            ..Default::default()
        }),
        &at(10.0, 0.0),
    );
    let sink_mps = CopterGuidanceConfig::default()
        .altitude_hold
        .climb_rate_mps(-0.216)
        .abs();
    let mut altitude_m = 10.0;
    for _ in 0..(10.0 / DT) as usize {
        let setpoint = guide(&mut guidance, hold(-0.216), &at(altitude_m, -sink_mps));
        check!(!setpoint.idle, "a slow sink in the air");
        altitude_m = guidance.altitude_target_m.expect("latched") + 0.01;
    }
}
