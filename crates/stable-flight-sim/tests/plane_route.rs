//! Plane routes on `airframes::plane()`: a leg climbing steeper than the airplane can climb
//! waits on its waypoint in a loiter until at its altitude, then flies on (#451).

mod common;

use assert2::check;
use stable_flight::commanders::waypoint::PLANE_ALTITUDE_ACCEPTANCE_M;
use stable_flight::commanders::{RouteEnd, WaypointFollower};
use stable_flight::{Vec2, WaypointCommand};
use stable_flight_sim::airframes;

use common::fly;
use common::plane::{PlanePlant, plane_loop};

/// The climb on the steep leg, in m: 200 m over a 200 m leg needs about 15 m/s of climb at
/// trim airspeed, three times the total energy control's 5 m/s climb limit.
const CLIMB_M: f32 = 200.0;

/// North of the start of the waypoint the steep leg climbs to, in m.
const STEEP_NORTH_M: f32 = 400.0;

/// Farthest from the steep leg's waypoint while waiting on it, in m: no external band
/// applies. Measured 52 m (the 40 m loiter plus its entry from the line); 60 m leaves about
/// 15% margin. Flying on along the line instead measures 390 m.
const LOITER_BAND_M: f32 = 60.0;

/// Seconds the route gets: measured 81 s to its last waypoint.
const ROUTE_S: f32 = 150.0;

#[test]
fn waits_on_a_waypoint_too_steep_to_climb_to_in_a_loiter() {
    let plant = PlanePlant::clean(airframes::plane());
    let start_m = plant.altitude_m();
    let cruise_mps = plant.airframe.stabilizer.trim_airspeed_mps;
    let at = |north_m: f32, altitude_m: f32| WaypointCommand {
        position_m: Vec2::new(north_m, 0.0),
        altitude_m,
        cruise_speed_mps: cruise_mps,
        ..Default::default()
    };
    let top_m = start_m + CLIMB_M;
    // The start (no leg into it: it's reached where the airplane begins), a level leg, the
    // steep leg, and a level leg on at the top.
    let route = vec![
        at(0.0, start_m),
        at(STEEP_NORTH_M - 200.0, start_m),
        at(STEEP_NORTH_M, top_m),
        at(STEEP_NORTH_M + 300.0, top_m),
    ];
    let follower = WaypointFollower::new(route)
        .expect("non-empty")
        .with_route_end(RouteEnd::Hold)
        .with_altitude_acceptance(PLANE_ALTITUDE_ACCEPTANCE_M)
        .with_wait_for_altitude(true);
    let mut flight_loop = plane_loop(&plant, follower);

    let steep = Vec2::new(STEEP_NORTH_M, 0.0);
    let (mut reached_steep, mut from_steep_m, mut advanced_off_m) = (false, 0.0_f32, f32::NAN);
    fly(&mut flight_loop, ROUTE_S, |flight_loop, _| {
        let position_m = flight_loop.state().position_m;
        match flight_loop.commander().current_index() {
            2 => {
                reached_steep |= position_m.x >= STEEP_NORTH_M;
                if reached_steep {
                    from_steep_m = from_steep_m.max((position_m.xy() - steep).norm());
                }
            }
            3 if advanced_off_m.is_nan() => advanced_off_m = position_m.z - top_m,
            _ => {}
        }
    });

    check!(reached_steep, "never reached the steep leg's waypoint");
    check!(
        from_steep_m <= LOITER_BAND_M,
        "{from_steep_m} m from the steep leg's waypoint while waiting on it"
    );
    check!(
        advanced_off_m.abs() <= PLANE_ALTITUDE_ACCEPTANCE_M,
        "advanced {advanced_off_m} m off its altitude"
    );
    check!(flight_loop.commander().is_complete(), "route not complete");
}
