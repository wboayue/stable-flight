//! Land → landed: take off to a waypoint, land on it, and disarm on the ground.

mod common;

use assert2::check;
use stable_flight::commanders::waypoint::DEFAULT_ALTITUDE_ACCEPTANCE_M;
use stable_flight::commanders::{RouteEnd, WaypointFollower};
use stable_flight_sim::Wind;

use common::{fly, sim_loop, waypoint};

const ALTITUDE_M: f32 = 5.0;

/// Where to land: a leg away from the start, so the land follows a cruise.
const NORTH_M: f32 = 20.0;
const EAST_M: f32 = -15.0;

/// Touchdown distance from the waypoint per axis, in m: 0.21 measured on the x axis, the lateral
/// I's slow tail after the 25 m leg (#357), which shaping the velocity target doesn't shorten
/// (#367).
const MAX_TOUCHDOWN_ERROR_M: f32 = 0.3;

#[test]
fn lands_on_the_waypoint_and_disarms() {
    let follower = WaypointFollower::new([waypoint(NORTH_M, EAST_M, ALTITUDE_M)])
        .expect("one waypoint")
        .with_route_end(RouteEnd::Land);
    let mut flight_loop = sim_loop(follower, Wind::default());

    let mut peak_m = 0.0_f32;
    let mut touchdown_mps = 0.0_f32;
    let mut landed_s = None;
    fly(&mut flight_loop, 20.0, |flight_loop, elapsed_s| {
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
        (state.position_m.x - NORTH_M).abs() < MAX_TOUCHDOWN_ERROR_M,
        "{:?}",
        state.position_m
    );
    check!(
        (state.position_m.y - EAST_M).abs() < MAX_TOUCHDOWN_ERROR_M,
        "{:?}",
        state.position_m
    );
    // A gentle touchdown: the landing descent speed, not a drop.
    check!(touchdown_mps < 1.0, "touchdown at {touchdown_mps} m/s");

    let (_, vehicle, ..) = flight_loop.into_parts();
    check!(vehicle.on_ground());
}

/// One waypoint straight over the start: the follower waits for the climb before landing, so
/// the vehicle takes off, lands back on the start, and disarms (#359).
#[test]
fn lands_back_on_a_waypoint_over_the_start() {
    let follower = WaypointFollower::new([waypoint(0.0, 0.0, ALTITUDE_M)])
        .expect("one waypoint")
        .with_route_end(RouteEnd::Land);
    let mut flight_loop = sim_loop(follower, Wind::default());

    let mut peak_m = 0.0_f32;
    fly(&mut flight_loop, 20.0, |flight_loop, _| {
        peak_m = peak_m.max(flight_loop.state().position_m.z);
    });

    // Climbed into the waypoint's altitude acceptance before landing: 4.27 m peak measured.
    check!(
        peak_m >= ALTITUDE_M - DEFAULT_ALTITUDE_ACCEPTANCE_M,
        "peak {peak_m} m"
    );
    check!(flight_loop.commander().is_complete(), "never landed");
    check!(!flight_loop.armed());
    let position_m = flight_loop.state().position_m;
    check!(position_m.z == 0.0);
    check!(
        position_m.xy().norm() < MAX_TOUCHDOWN_ERROR_M,
        "{position_m:?}"
    );
}
