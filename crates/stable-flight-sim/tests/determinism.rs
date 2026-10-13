//! The same flight twice gives the same states, bit for bit.

mod common;

use assert2::check;
use stable_flight::Vec3;
use stable_flight::commanders::{RouteEnd, WaypointFollower};
use stable_flight_sim::{Gust, Wind};

use common::{fly, sim_loop, waypoint};

/// A windy route with a gust; every tick's state.
fn flight() -> Vec<stable_flight::VehicleState> {
    let route = [waypoint(0.0, 0.0, 4.0), waypoint(10.0, 5.0, 6.0)];
    let follower = WaypointFollower::new(route)
        .expect("waypoints")
        .with_route_end(RouteEnd::Land);
    let wind = Wind {
        steady_mps: Vec3::new(-2.0, 1.0, 0.0),
        gust: Some(Gust {
            start_s: 6.0,
            duration_s: 1.5,
            peak_mps: Vec3::new(0.0, 4.0, -1.0),
        }),
    };
    let mut flight_loop = sim_loop(follower, wind);
    let mut states = Vec::new();
    fly(&mut flight_loop, 20.0, |flight_loop| {
        states.push(flight_loop.state().clone());
    });
    states
}

#[test]
fn repeats_bit_for_bit() {
    let first = flight();
    check!(first.len() == 5000);
    check!(first == flight());
}
