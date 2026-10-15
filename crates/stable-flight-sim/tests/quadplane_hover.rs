//! Hover on the lift rotors of `airframes::quadplane()`: take off from its gear to a waypoint
//! on core's copter guidance and stabilizer, and hold altitude, position, and heading.

mod common;

use assert2::check;
use stable_flight::{Vec2, Vec3};
use stable_flight_sim::airframes;

use common::quadplane::{QuadPlanePlant, quadplane_loop};
use common::{
    CALM_ALTITUDE_M, CALM_ANGLE_DEG, CALM_HORIZONTAL_M, HOVER_ALTITUDE_M as ALTITUDE_M,
    HOVER_SETTLE_S as SETTLE_S, fly, hover_follower,
};

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
