//! Waypoint following: a square route flown at cruise speed, each leg on its line and at its
//! altitude, ending in a hold on the last corner; a leg behind flown nose first.

mod common;

use std::cell::RefCell;
use std::rc::Rc;

use assert2::check;
use stable_flight::commanders::{RouteEnd, WaypointFollower};
use stable_flight::{
    ControlInput, LandCommand, Setpoint, TelemetryPublisher, TelemetryRecord, Vec2, WaypointCommand,
};
use stable_flight_sim::{Wind, airframes};

use common::{
    ARRIVAL_UNDERSHOOT_M, FACING_DEG, Plant, ThenFly, fly, heading_error_deg, hover_at, pipeline,
    sim_loop, ticks,
};

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

/// A leg behind the vehicle turns the nose toward it and flies it nose first, from a quarter of
/// the way along to the hold on its end (#452), within the square's line and hold bands.
#[test]
fn turns_to_fly_a_leg_behind_nose_first() {
    let behind = corner(-2.0 * SIDE_M, 0.0);
    let follower = WaypointFollower::new([behind])
        .expect("one waypoint")
        .with_route_end(RouteEnd::Hold);
    let mut flight_loop = sim_loop(follower, Wind::default());
    let bearing_rad = core::f32::consts::PI;

    let mut worst_deg: f32 = 0.0;
    let mut cross_track_m: f32 = 0.0;
    fly(&mut flight_loop, 40.0, |flight_loop, _| {
        let state = flight_loop.state();
        let position = Vec2::new(state.position_m.x, state.position_m.y);
        let (fraction, off_m) = leg_position(position, Vec2::zeros(), behind.position_m);
        if fraction >= 0.25 {
            worst_deg = worst_deg.max(heading_error_deg(state.heading_rad, bearing_rad));
        }
        if (0.25..=0.75).contains(&fraction) {
            cross_track_m = cross_track_m.max(off_m);
        }
    });

    check!(worst_deg < FACING_DEG);
    check!(cross_track_m < MAX_CROSS_TRACK_M);
    let state = flight_loop.state();
    let held_m = (Vec2::new(state.position_m.x, state.position_m.y) - behind.position_m).norm();
    check!(held_m < 0.1, "{held_m} m off the waypoint");
}

/// A descent from 30 m to a waypoint at 10 m stops on it: the target brakes to the waypoint
/// instead of snapping onto it at full descent rate (#403).
#[test]
fn descends_onto_a_lower_waypoint_without_sinking_past_it() {
    let route = [
        WaypointCommand {
            altitude_m: 30.0,
            ..Default::default()
        },
        WaypointCommand {
            altitude_m: 10.0,
            ..Default::default()
        },
    ];
    let follower = WaypointFollower::new(route)
        .expect("two waypoints")
        .with_route_end(RouteEnd::Hold);
    let mut flight_loop = sim_loop(follower, Wind::default());
    let mut lowest_m = f32::INFINITY;
    let mut descending = false;
    fly(&mut flight_loop, 60.0, |flight_loop, _| {
        let altitude_m = flight_loop.state().position_m.z;
        descending |= altitude_m > 25.0 && flight_loop.state().velocity_mps.z < -1.0;
        if descending {
            lowest_m = lowest_m.min(altitude_m);
        }
    });
    check!(descending);
    check!(
        10.0 - lowest_m < ARRIVAL_UNDERSHOOT_M,
        "lowest {lowest_m} m"
    );
}

/// Records each tick's attitude setpoint thrust.
struct ThrustLog(Rc<RefCell<Vec<f32>>>);

impl TelemetryPublisher for ThrustLog {
    fn publish(&mut self, record: &TelemetryRecord) {
        if let Setpoint::Attitude(setpoint) = record.setpoint {
            self.0.borrow_mut().push(setpoint.thrust);
        }
    }
}

/// A descent or a land from a settled hover speeds up at `descent_accel_mps2` rather than
/// stepping to its speed, so thrust dips no more than that acceleration asks: hover · a / g,
/// 0.086 at ArduPilot's `WP_ACC_Z` of 1 m/s² (`libraries/AC_WPNav/AC_WPNav.cpp:14`). Measured
/// on `quad_x`: 0.030 for the descent, 0.028 for the land; stepping dipped them 0.52 (to
/// 3 m/s) and 0.121 (to the 0.7 m/s land speed). #417: on the RealFlight eXplora, a descent
/// step took thrust to its floor and dropped the nose 10°; #476: a land step set off a roll
/// cycle.
#[test]
fn starts_a_descent_without_a_thrust_step() {
    const DESCEND_AT_S: f32 = 15.0;
    let airframe = airframes::quad_x();
    let cases = [
        (
            "waypoint",
            ControlInput::Waypoint(WaypointCommand {
                altitude_m: 5.0,
                ..Default::default()
            }),
        ),
        ("land", ControlInput::Land(LandCommand::default())),
    ];
    for (case, input) in cases {
        let log = Rc::new(RefCell::new(Vec::new()));
        let commander = ThenFly {
            hover: hover_at(20.0),
            elapsed_s: 0.0,
            after_s: DESCEND_AT_S,
            input: Some(input),
        };
        let mut flight_loop = pipeline(&Plant::clean(airframe), commander, ThrustLog(log.clone()));
        fly(&mut flight_loop, DESCEND_AT_S + 1.0, |_, _| {});

        let thrust = log.borrow();
        let start = ticks(DESCEND_AT_S);
        let hover = thrust[start - 2];
        let lowest = thrust[start..]
            .iter()
            .copied()
            .fold(f32::INFINITY, f32::min);
        let allowed = hover * airframe.guidance.descent_accel_mps2 / stable_flight::GRAVITY;
        check!(
            hover - lowest < allowed,
            "{case}: hover {hover}, lowest {lowest}"
        );
    }
}
