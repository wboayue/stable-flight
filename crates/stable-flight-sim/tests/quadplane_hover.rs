//! Hover on the lift rotors of `airframes::quadplane()`: take off from its gear to a waypoint
//! on core's copter guidance and stabilizer, and hold altitude, position, and heading; in a
//! crosswind, yield the heading to the fin.

mod common;

use assert2::check;
use stable_flight::{Vec2, Vec3};
use stable_flight_sim::{Wind, airframes};

use common::quadplane::{QuadPlanePlant, hover_heading_error_deg, quadplane_loop};
use common::{
    CALM_ALTITUDE_M, CALM_ANGLE_DEG, CALM_HORIZONTAL_M, HOVER_ALTITUDE_M as ALTITUDE_M,
    HOVER_SETTLE_S as SETTLE_S, fly, hover_at, hover_follower,
};

/// Height of the windy takeoffs' hover point, in m: high enough for the fin to turn the nose
/// on the way up.
const CLIMB_TO_M: f32 = 30.0;

/// Most a takeoff to [`CLIMB_TO_M`] in a crosswind may take beyond the same takeoff in a
/// headwind, in s. Yielding the heading to the fin, measured 1.4 s at 8 m/s and 1.0 s at
/// 11 m/s; fighting it, 8.1 and 7.5 s (#431).
const CROSSWIND_DELAY_S: f32 = 2.0;

/// Largest deviations from the hover point seen while holding.
#[derive(Debug, Default)]
struct Deviation {
    altitude_m: f32,
    horizontal_m: f32,
    tilt_deg: f32,
    heading_deg: f32,
}

/// The copter's calm-air bands (`hover.rs`): on its lift rotors, the quadplane holds as
/// tightly, its wing still and the pusher stopped.
#[test]
fn holds_altitude_position_and_heading_in_calm_air() {
    let plant = QuadPlanePlant::clean(airframes::quadplane());
    let mut flight_loop = quadplane_loop(&plant, hover_follower());
    fly(&mut flight_loop, SETTLE_S, |_, _| {});
    check!(flight_loop.commander().is_complete());

    let point_m = Vec3::new(0.0, 0.0, ALTITUDE_M);
    let mut worst = Deviation::default();
    fly(&mut flight_loop, 10.0, |flight_loop, _| {
        let state = flight_loop.state();
        let off_m = state.position_m - point_m;
        worst.altitude_m = worst.altitude_m.max(off_m.z.abs());
        worst.horizontal_m = worst.horizontal_m.max(Vec2::new(off_m.x, off_m.y).norm());
        worst.tilt_deg = worst
            .tilt_deg
            .max(state.roll_rad.abs().max(state.pitch_rad.abs()).to_degrees());
        worst.heading_deg = worst.heading_deg.max(
            stable_flight::wrap_angle(state.heading_rad)
                .to_degrees()
                .abs(),
        );
    });
    check!(worst.altitude_m < CALM_ALTITUDE_M, "{worst:?}");
    check!(worst.horizontal_m < CALM_HORIZONTAL_M, "{worst:?}");
    check!(worst.tilt_deg < CALM_ANGLE_DEG, "{worst:?}");
    check!(worst.heading_deg < CALM_ANGLE_DEG, "{worst:?}");
    let pusher = usize::from(flight_loop.vehicle().pusher_slot());
    let (.., telemetry) = flight_loop.into_parts();
    check!(
        telemetry
            .iter()
            .all(|tick| tick.actuators.motors[pusher] == 0.0)
    );
}

/// Seconds a takeoff to [`CLIMB_TO_M`] in `wind_mps` takes to come within 1 m of it, and how
/// far the nose ends off its heading-hold target, in degrees.
fn climb_in(wind_mps: Vec3) -> (Option<f32>, f32) {
    let wind = Wind {
        steady_mps: wind_mps,
        gust: None,
    };
    let plant = QuadPlanePlant::in_wind(airframes::quadplane(), wind);
    let mut flight_loop = quadplane_loop(&plant, hover_at(CLIMB_TO_M));
    let mut climbed_s = None;
    fly(&mut flight_loop, 40.0, |flight_loop, elapsed_s| {
        if climbed_s.is_none() && flight_loop.state().position_m.z > CLIMB_TO_M - 1.0 {
            climbed_s = Some(elapsed_s);
        }
    });
    let (.., telemetry) = flight_loop.into_parts();
    (climbed_s, hover_heading_error_deg(&telemetry))
}

/// In a crosswind the fin turns the nose into the wind, and the heading-hold target yields to
/// it instead of fighting it, so the takeoff climbs about as fast as into a headwind (#431).
#[test]
fn takes_off_in_a_crosswind_about_as_fast_as_into_a_headwind() {
    for wind_mps in [8.0, 11.0] {
        let (headwind_s, _) = climb_in(Vec3::new(-wind_mps, 0.0, 0.0));
        let (crosswind_s, error_deg) = climb_in(Vec3::new(0.0, wind_mps, 0.0));
        let (Some(headwind_s), Some(crosswind_s)) = (headwind_s, crosswind_s) else {
            panic!("{wind_mps} m/s: never climbed ({headwind_s:?}, {crosswind_s:?})");
        };
        check!(
            crosswind_s - headwind_s < CROSSWIND_DELAY_S,
            "{wind_mps} m/s: {crosswind_s} s vs {headwind_s} s"
        );
        check!(error_deg < CALM_ANGLE_DEG, "{wind_mps} m/s");
    }
}
