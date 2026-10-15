//! Copter waypoints on `airframes::quadplane()`: legs flown on the lift rotors, ahead, behind,
//! and to the side, and into wind, with the pusher taking the nose-down pitch (#427).

mod common;

use assert2::check;
use stable_flight::{Command, ControlInput, Vec2, Vec3, WaypointCommand};
use stable_flight_sim::{Wind, airframes};

use common::hover_at;
use common::plane::Interlude;
use common::quadplane::{QuadPlanePlant, quadplane_loop};
use common::transition::{FORWARD_AT_S, HOLD_S, TRANSITION_ALTITUDE_M, check_altitude_loss, trace};
use common::{CALM_ALTITUDE_M, CALM_ANGLE_DEG, CALM_HORIZONTAL_M};

/// Leg length, in m.
const LEG_M: f32 = 100.0;

/// ArduPilot's quadplane waypoint speed (`Q_WP_SPD`, `ArduPlane/quadplane.cpp:607`), in m/s.
const WP_SPEED_MPS: f32 = 5.0;

/// No cruise cap: the 27° tilt cap bounds the speed (measured up to 15 m/s).
const UNCAPPED: f32 = 0.0;

/// Headwind the quadplane holds and flies upwind into, in m/s: where copter flight without
/// forward thrust can't climb off the ground (#427).
const HEADWIND_MPS: f32 = 11.0;

/// Seconds a leg takes to arrive and settle: at 5 m/s, 21 s measured to arrive within 2 m,
/// 22 s upwind.
const LEG_S: f32 = 40.0;

/// Hovers over the origin at [`TRANSITION_ALTITUDE_M`], then from [`FORWARD_AT_S`] flies to
/// `target_m` at `cruise_mps`.
fn leg(target_m: Vec2, cruise_mps: f32) -> Interlude<common::Hover> {
    let waypoint = WaypointCommand {
        position_m: target_m,
        altitude_m: TRANSITION_ALTITUDE_M,
        cruise_speed_mps: cruise_mps,
        ..Default::default()
    };
    Interlude::new(
        hover_at(TRANSITION_ALTITUDE_M),
        FORWARD_AT_S,
        f32::INFINITY,
        Command::new(true, ControlInput::Waypoint(waypoint)),
    )
}

/// Flies a leg to `target_m` at `cruise_mps` in a steady `wind_mps`, then checks it stayed
/// within PX4's altitude loss throughout and holds the end of the leg within the calm-air
/// altitude and position bands; the tilt band only in calm air, since a hold in wind leans.
fn fly_leg(case: &str, wind_mps: Vec3, target_m: Vec2, cruise_mps: f32) {
    let wind = Wind {
        steady_mps: wind_mps,
        gust: None,
    };
    let plant = QuadPlanePlant::in_wind(airframes::quadplane(), wind);
    let mut flight_loop = quadplane_loop(&plant, leg(target_m, cruise_mps));
    let flight = trace(&mut flight_loop, FORWARD_AT_S + LEG_S + HOLD_S);

    check_altitude_loss(flight.heights(FORWARD_AT_S, f32::INFINITY), case);

    let point_m = Vec3::new(target_m.x, target_m.y, TRANSITION_ALTITUDE_M);
    let worst = flight.level_hold(FORWARD_AT_S + LEG_S, point_m);
    check!(worst.altitude_m < CALM_ALTITUDE_M, "{case}: {worst:?}");
    check!(worst.horizontal_m < CALM_HORIZONTAL_M, "{case}: {worst:?}");
    if wind_mps == Vec3::zeros() {
        check!(worst.tilt_deg < CALM_ANGLE_DEG, "{case}: {worst:?}");
    }
}

/// In calm air, legs ahead, behind, and to the side, uncapped and at ArduPilot's waypoint
/// speed, stay within PX4's altitude loss (`VT_QC_T_ALT_LOSS`) and settle on the point. Nose
/// down the pusher takes the pitch past 3°; tail first it brakes.
#[test]
fn flies_copter_legs_in_every_direction() {
    let rows = [
        ("ahead", Vec2::new(LEG_M, 0.0)),
        ("behind", Vec2::new(-LEG_M, 0.0)),
        ("side", Vec2::new(0.0, LEG_M)),
    ];
    for (direction, target_m) in rows {
        for cruise_mps in [UNCAPPED, WP_SPEED_MPS] {
            let case = format!("{direction} at {cruise_mps} m/s");
            fly_leg(&case, Vec3::zeros(), target_m, cruise_mps);
        }
    }
}

/// In a headwind from the north the quadplane takes off, holds over the origin, and flies a
/// leg upwind and one downwind at ArduPilot's waypoint speed: the pusher holds it against the
/// wind, the wing level.
#[test]
fn holds_and_flies_legs_in_a_headwind() {
    let headwind_mps = Vec3::new(-HEADWIND_MPS, 0.0, 0.0);
    let rows = [
        ("hold", Vec2::zeros(), UNCAPPED),
        ("upwind", Vec2::new(LEG_M, 0.0), WP_SPEED_MPS),
        ("downwind", Vec2::new(-LEG_M, 0.0), WP_SPEED_MPS),
    ];
    for (case, target_m, cruise_mps) in rows {
        fly_leg(case, headwind_mps, target_m, cruise_mps);
    }
}
