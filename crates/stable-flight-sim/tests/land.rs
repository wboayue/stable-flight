//! Land → landed: take off to a waypoint, land on it, and disarm on the ground.

mod common;

use assert2::check;
use stable_flight::commanders::{RouteEnd, WaypointFollower};
use stable_flight_sim::Wind;

use common::{fly, sim_loop, waypoint};

const ALTITUDE_M: f32 = 5.0;

/// Where to land: far enough that the climb to `ALTITUDE_M` finishes on the way, since the
/// follower accepts a waypoint on horizontal distance alone (TODO(#359)).
const NORTH_M: f32 = 20.0;
const EAST_M: f32 = -15.0;

#[test]
fn lands_on_the_waypoint_and_disarms() {
    let follower = WaypointFollower::new([waypoint(NORTH_M, EAST_M, ALTITUDE_M)])
        .expect("one waypoint")
        .with_route_end(RouteEnd::Land);
    let mut flight_loop = sim_loop(follower, Wind::default());

    let mut peak_m = 0.0_f32;
    let mut touchdown_mps = 0.0_f32;
    let mut landed_s = None;
    let mut elapsed_s = 0.0;
    fly(&mut flight_loop, 20.0, |flight_loop| {
        elapsed_s += common::TICK.as_secs_f32();
        let state = flight_loop.state();
        peak_m = peak_m.max(state.position_m.z);
        if state.position_m.z < 0.05 && peak_m > 1.0 {
            touchdown_mps = touchdown_mps.max(-state.velocity_mps.z);
        }
        if landed_s.is_none() && flight_loop.commander().is_complete() {
            landed_s = Some(elapsed_s);
        }
    });

    check!(peak_m > ALTITUDE_M - 0.5, "peak {peak_m} m");
    check!(landed_s.is_some(), "never landed");
    check!(!flight_loop.armed());
    let state = flight_loop.state().clone();
    check!(state.position_m.z == 0.0);
    check!(
        (state.position_m.x - NORTH_M).abs() < 0.2,
        "{:?}",
        state.position_m
    );
    check!(
        (state.position_m.y - EAST_M).abs() < 0.2,
        "{:?}",
        state.position_m
    );
    // A gentle touchdown: the landing descent speed, not a drop.
    check!(touchdown_mps < 1.0, "touchdown at {touchdown_mps} m/s");

    let (_, vehicle, ..) = flight_loop.into_parts();
    check!(vehicle.on_ground());
}
