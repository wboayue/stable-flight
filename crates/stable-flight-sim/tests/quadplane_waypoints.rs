//! Copter waypoints on `airframes::quadplane()`: legs flown on the lift rotors, ahead, behind,
//! and to the side, and into wind, with the pusher taking the nose-down pitch (#427), each
//! turned to face its waypoint (#452), slowed while the nose is off it (#437), and stopped on
//! it with the velocity target shaped (#466) or braking along a curve unshaped (#488).

mod common;

use assert2::check;
use stable_flight::commanders::{RouteEnd, WaypointFollower};
use stable_flight::params::Params;
use stable_flight::{Command, ControlInput, Vec2, Vec3, WaypointCommand, euler_angles};
use stable_flight_sim::airframes::QuadPlaneAirframe;
use stable_flight_sim::{Wind, airframes};

use common::hover_at;
use common::plane::Interlude;
use common::quadplane::{QuadPlanePlant, hover_heading_error_deg, quadplane_loop};
use common::transition::{FORWARD_AT_S, HOLD_S, TRANSITION_ALTITUDE_M, check_altitude_loss, trace};
use common::{CALM_ALTITUDE_M, CALM_ANGLE_DEG, CALM_HORIZONTAL_M, FACING_DEG, heading_error_deg};

/// Leg length, in m.
const LEG_M: f32 = 100.0;

/// ArduPilot's quadplane waypoint speed (`Q_WP_SPD`, `ArduPlane/quadplane.cpp:607`), in m/s.
const WP_SPEED_MPS: f32 = 5.0;

/// No cruise cap: the 27° tilt cap and the braking curve bound the speed (measured up to
/// 11.8 m/s on a [`LEG_M`] leg).
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
/// altitude and position bands. In calm air it holds within the tilt band, facing the way the
/// leg went; in wind, which leans the hold and whose fin turns the nose into it, the heading
/// target has yielded to the nose (#431).
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
    if wind_mps != Vec3::zeros() {
        let (.., telemetry) = flight_loop.into_parts();
        let error_deg = hover_heading_error_deg(&telemetry);
        check!(
            error_deg < CALM_ANGLE_DEG,
            "{case}: target {error_deg}° off the nose"
        );
        return;
    }
    check!(worst.tilt_deg < CALM_ANGLE_DEG, "{case}: {worst:?}");
    if target_m != Vec2::zeros() {
        let bearing_rad = target_m.y.atan2(target_m.x);
        let facing_deg = flight
            .window(FORWARD_AT_S + LEG_S, f32::INFINITY)
            .map(|sample| heading_error_deg(sample.state.heading_rad, bearing_rad))
            .fold(0.0, f32::max);
        check!(facing_deg < FACING_DEG, "{case}: {facing_deg}°");
    }
}

/// In calm air, legs ahead, behind, and to the side, uncapped and at ArduPilot's waypoint
/// speed, stay within PX4's altitude loss (`VT_QC_T_ALT_LOSS`) and settle on the point, turned
/// to fly each nose first. Nose down the pusher takes the pitch past 3°.
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
/// wind, the wing level. Downwind, the fin holds the nose upwind, and the heading yields to it.
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

/// Most the tilt may pass the tilt cap on an uncapped leg, in degrees (bound: #437).
const TILT_OVER_CAP_DEG: f32 = 5.0;

/// The length of the RealFlight probe's legs that flipped the Titan Cobra flying the second
/// tail first (#437), in m.
const PROBE_LEG_M: f32 = 260.0;

/// Seconds of the probe flown after [`FORWARD_AT_S`]: past the turnaround (29 s measured) and
/// well into the leg back, which the slowest row, the quadplane tail first at 5 m/s, ends at
/// 81 s.
const PROBE_S: f32 = 60.0;

/// A waypoint at `north_m` over the line north, at [`TRANSITION_ALTITUDE_M`], uncapped.
fn on_the_line(north_m: f32) -> WaypointCommand {
    WaypointCommand {
        position_m: Vec2::new(north_m, 0.0),
        altitude_m: TRANSITION_ALTITUDE_M,
        ..Default::default()
    }
}

/// An uncapped leg out and back, the probe's, stays within 5° of the tilt cap and PX4's
/// altitude loss, slowed while the nose is off the waypoint (#437): with the turn on, through
/// the turnaround as the nose comes round; with it off, the whole way back tail first. Uncut,
/// the quadplane tilts to 27° and 31° (37° and 39° braking linearly), and the Alia tail first
/// to 39°, sinking 30 m; cut, all stay within 27.0°. The Alia with the turn on keeps within
/// the bound uncut (28°), so it isn't a row.
#[test]
fn slows_uncapped_legs_the_nose_is_off() {
    let route = [on_the_line(0.0), on_the_line(PROBE_LEG_M), on_the_line(0.0)];
    let quadplane: fn() -> QuadPlaneAirframe = airframes::quadplane;
    let alia: fn() -> QuadPlaneAirframe = airframes::alia250;
    // (case, airframe, waypoint yaw in deg/s)
    let rows = [
        ("quadplane, turn on", quadplane, 60.0),
        ("quadplane, turn off", quadplane, 0.0),
        ("Alia, turn off", alia, 0.0),
    ];
    for (case, airframe, yaw_degps) in rows {
        let airframe = airframe();
        let cap_deg = airframe.guidance.hover.max_tilt_rad.to_degrees();
        let follower = WaypointFollower::new(route)
            .expect("waypoints")
            .with_route_end(RouteEnd::Hold);
        let mut flight_loop = quadplane_loop(&QuadPlanePlant::clean(airframe), follower);
        flight_loop
            .set_param("GUID_YAW_SLEW", yaw_degps)
            .expect("a param");
        let flight = trace(&mut flight_loop, FORWARD_AT_S + PROBE_S);

        check_altitude_loss(flight.heights(FORWARD_AT_S, f32::INFINITY), case);
        let tilt_deg = flight
            .window(FORWARD_AT_S, f32::INFINITY)
            .map(|sample| {
                let (roll_rad, pitch_rad, _) = euler_angles(&sample.state.attitude);
                roll_rad.abs().max(pitch_rad.abs()).to_degrees()
            })
            .fold(0.0, f32::max);
        check!(
            tilt_deg < cap_deg + TILT_OVER_CAP_DEG,
            "{case}: {tilt_deg}° against a {cap_deg}° cap"
        );
    }
}

/// Most an uncapped probe leg passes its waypoint on the linear position loop (`GUID_LAT_ACC`
/// and `GUID_LAT_BRK` 0), in m: measured 13.9 m.
const UNSHAPED_OVERSHOOT_UNCAPPED_M: f32 = 14.0;

/// Most a probe leg at [`SHAPED_CRUISE_MPS`] passes its waypoint on the linear position loop,
/// in m: measured 2.0 m.
const UNSHAPED_OVERSHOOT_CRUISE_M: f32 = 2.0;

/// The capped row's leg speed, in m/s.
const SHAPED_CRUISE_MPS: f32 = 10.0;

/// Seconds of the shaped-leg flight from the start on the gear: the climb and the leg, and
/// past its stop (the slowest row, 10 m/s at 1 m/s², measured 35 s from the leg's start to
/// within 2 m of the waypoint).
const SHAPED_FLIGHT_S: f32 = 120.0;

/// With the velocity target shaped at `GUID_LAT_ACC`, the probe leg stops on its waypoint no
/// further past it than the linear position loop (shaping ignores `GUID_LAT_BRK`): the
/// position loop brakes along a curve the shaped target can follow, so braking starts in time
/// (#466). Linear braking under a rate limit overshot by
/// 193 m and 145 m uncapped, 37 m and 5.6 m at 10 m/s.
#[test]
fn stops_on_the_waypoint_with_acceleration_shaping() {
    // (cruise speed in m/s, most it may pass the waypoint in m)
    let legs = [
        (UNCAPPED, UNSHAPED_OVERSHOOT_UNCAPPED_M),
        (SHAPED_CRUISE_MPS, UNSHAPED_OVERSHOOT_CRUISE_M),
    ];
    for (cruise_mps, band_m) in legs {
        for accel_mps2 in [1.0, 2.5] {
            let route =
                [on_the_line(0.0), on_the_line(PROBE_LEG_M)].map(|waypoint| WaypointCommand {
                    cruise_speed_mps: cruise_mps,
                    ..waypoint
                });
            let follower = WaypointFollower::new(route)
                .expect("waypoints")
                .with_route_end(RouteEnd::Hold);
            let plant = QuadPlanePlant::clean(airframes::quadplane());
            let mut flight_loop = quadplane_loop(&plant, follower);
            flight_loop
                .set_param("GUID_LAT_ACC", accel_mps2)
                .expect("a param");
            let flight = trace(&mut flight_loop, SHAPED_FLIGHT_S);

            let past_m = flight
                .samples
                .iter()
                .map(|sample| sample.state.position_m.x - PROBE_LEG_M)
                .fold(f32::MIN, f32::max);
            let last = flight.samples.last().expect("a flight");
            let off_m = (last.state.position_m.xy() - Vec2::new(PROBE_LEG_M, 0.0)).norm();
            let case = format!("{cruise_mps} m/s at {accel_mps2} m/s²");
            check!(past_m <= band_m, "{case}: {past_m} m past");
            check!(off_m < CALM_HORIZONTAL_M, "{case}: ends {off_m} m off");
        }
    }
}

/// The RealFlight Titan Cobra's copter leg out and back (`flights/cobra_out_back.csv`), in m.
const COBRA_LEG_M: f32 = 300.0;

/// Most an uncapped leg may pass its waypoint, in m: the Cobra's RealFlight target (#488).
const BRAKED_OVERSHOOT_M: f32 = 5.0;

/// Seconds of the out-and-back flight from the start on the gear: the climb, both legs, and
/// past the stop home (the slowest row, the Cobra-like fit, measured within 2 m of home at
/// 57.6 s).
const OUT_BACK_S: f32 = 80.0;

/// [`airframes::quadplane`] fitted to the RealFlight Titan Cobra's copter legs (#488): drag a
/// fifth of the model's (it coasts at 0.55 m/s² at 13 m/s; the Cobra about 0.6) and a pusher
/// losing 2.8 N per m/s of airspeed, so uncapped legs top out at 16.9 m/s (the Cobra 16.7) and
/// overshoot 20 m on the linear position loop (the Cobra 15–20).
fn cobra_like() -> QuadPlaneAirframe {
    let mut airframe = airframes::quadplane();
    airframe.model.wing.aero.c_drag_0 *= 0.2;
    airframe.model.lift.drag_n_per_mps *= 0.2;
    airframe.model.wing.thrust_falloff_n_per_mps = 2.8;
    airframe
}

/// Unshaped, an uncapped leg out and back stops within 5 m of both waypoints, braking along
/// the curve at `GUID_LAT_BRK` (#488): the linear position loop starts braking `2 · v` out,
/// which at 17–20 m/s overshot 14.3 and 12.2 m on the quadplane and 19.9 and 17.6 m on the
/// Cobra-like fit. At 1.5 m/s² they stop 1.6 m short and on the point, and 1.9 and 1.8 m past.
#[test]
fn brakes_uncapped_legs_in_time_unshaped() {
    let quadplane: fn() -> QuadPlaneAirframe = airframes::quadplane;
    let cobra: fn() -> QuadPlaneAirframe = cobra_like;
    for (case, airframe) in [("quadplane", quadplane), ("Cobra-like", cobra)] {
        let route = [on_the_line(0.0), on_the_line(COBRA_LEG_M), on_the_line(0.0)];
        let follower = WaypointFollower::new(route)
            .expect("waypoints")
            .with_route_end(RouteEnd::Hold);
        let mut flight_loop = quadplane_loop(&QuadPlanePlant::clean(airframe()), follower);
        let flight = trace(&mut flight_loop, OUT_BACK_S);

        let north_m = |sample: &common::transition::Sample| sample.state.position_m.x;
        let far = flight
            .samples
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| north_m(a).total_cmp(&north_m(b)))
            .expect("a flight");
        let past_far_m = north_m(far.1) - COBRA_LEG_M;
        let past_home_m = -flight.samples[far.0..]
            .iter()
            .map(north_m)
            .fold(f32::MAX, f32::min);
        check!(
            past_far_m <= BRAKED_OVERSHOOT_M,
            "{case}: {past_far_m} m past far"
        );
        check!(
            past_home_m <= BRAKED_OVERSHOOT_M,
            "{case}: {past_home_m} m past home"
        );
        let last = flight.samples.last().expect("a flight");
        let off_m = last.state.position_m.xy().norm();
        check!(off_m < CALM_HORIZONTAL_M, "{case}: ends {off_m} m off");
    }
}
