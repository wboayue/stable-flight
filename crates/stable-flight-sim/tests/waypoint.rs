//! Waypoint following: a square route flown at cruise speed, each leg on its line and at its
//! altitude, ending in a hold on the last corner.

mod common;

use assert2::check;
use stable_flight::commanders::{RouteEnd, WaypointFollower};
use stable_flight::{Vec2, WaypointCommand};
use stable_flight_sim::Wind;

use common::{fly, sim_loop};

const ALTITUDE_M: f32 = 5.0;
const SIDE_M: f32 = 20.0;
const CRUISE_MPS: f32 = 3.0;

/// Largest distance off a leg's line on its middle half, in m. Copter guidance flies to the
/// waypoint (with the leg's velocity fed forward), not along the line, so the corner cut
/// fades over the leg rather than being held off: about 1.5 m on the takeoff leg, under 1 m
/// on the others, measured on `quad_x`.
const MAX_CROSS_TRACK_M: f32 = 2.0;

/// Largest altitude error along a leg, in m (under 0.2 measured).
const MAX_ALTITUDE_ERROR_M: f32 = 0.3;

/// A corner of the route.
fn corner(north_m: f32, east_m: f32) -> WaypointCommand {
    WaypointCommand {
        position_m: Vec2::new(north_m, east_m),
        altitude_m: ALTITUDE_M,
        cruise_speed_mps: CRUISE_MPS,
        ..Default::default()
    }
}

/// Where `point` is relative to the leg from `from` to `to`: the fraction of the leg flown
/// along it, and the distance off its line, in m.
fn leg_position(point: Vec2, from: Vec2, to: Vec2) -> (f32, f32) {
    let leg = to - from;
    let along = leg.normalize();
    let offset = point - from;
    let along_m = offset.dot(&along);
    (along_m / leg.norm(), (offset - along * along_m).norm())
}

/// The worst of one leg, from reaching its start corner to reaching its end corner.
#[derive(Debug, Default, Clone, Copy)]
struct Leg {
    /// Off the line on the middle half of the leg; the ends round the corners, which the
    /// follower switches 3 m short of (its acceptance radius).
    cross_track_m: f32,
    altitude_m: f32,
    duration_s: f32,
}

#[test]
fn flies_a_square_on_its_legs_and_holds_the_last_corner() {
    // Takes off toward the first corner (not over the start: #359), then flies the square.
    let route = [
        corner(SIDE_M, 0.0),
        corner(SIDE_M, SIDE_M),
        corner(0.0, SIDE_M),
        corner(0.0, 0.0),
    ];
    let follower = WaypointFollower::new(route)
        .expect("waypoints")
        .with_route_end(RouteEnd::Hold);
    let mut flight_loop = sim_loop(follower, Wind::default());

    let mut legs = [Leg::default(); 3];
    let mut completed_s = None;
    fly(&mut flight_loop, 60.0, |flight_loop, elapsed_s| {
        let follower = flight_loop.commander();
        if completed_s.is_none() && follower.is_complete() {
            completed_s = Some(elapsed_s);
        }
        // Legs between corners: from corner `index − 1` to `index`, while `index` is active.
        let index = follower.current_index();
        if index == 0 || follower.is_complete() {
            return;
        }
        let state = flight_loop.state();
        let position = Vec2::new(state.position_m.x, state.position_m.y);
        let leg = &mut legs[index - 1];
        let (from, to) = (route[index - 1].position_m, route[index].position_m);
        let (fraction, cross_track_m) = leg_position(position, from, to);
        if (0.25..=0.75).contains(&fraction) {
            leg.cross_track_m = leg.cross_track_m.max(cross_track_m);
        }
        leg.altitude_m = leg.altitude_m.max((state.position_m.z - ALTITUDE_M).abs());
        leg.duration_s += common::TICK.as_secs_f32();
    });

    check!(completed_s.is_some(), "never reached the last corner");
    let leg_s = SIDE_M / CRUISE_MPS;
    for (index, leg) in legs.iter().enumerate() {
        check!(
            leg.cross_track_m < MAX_CROSS_TRACK_M,
            "leg {index}: {leg:?}"
        );
        check!(
            leg.altitude_m < MAX_ALTITUDE_ERROR_M,
            "leg {index}: {leg:?}"
        );
        // At cruise speed, give or take the turn at each end.
        check!(
            (leg.duration_s - leg_s).abs() < 0.3 * leg_s,
            "leg {index}: {leg:?}"
        );
    }
    let state = flight_loop.state();
    let last = route[3].position_m;
    let held_m = (Vec2::new(state.position_m.x, state.position_m.y) - last).norm();
    check!(held_m < 0.1, "{held_m} m off the last corner");
}
