use crate::{Vec2, Vec3};
use approx::assert_relative_eq;
use assert2::check;

use super::*;

/// Builds a `VehicleState` positioned at the given horizontal coordinates (meters).
fn state_at(x: f32, y: f32) -> VehicleState {
    VehicleState {
        position_m: Vec3::new(x, y, 0.0),
        ..Default::default()
    }
}

/// Builds a waypoint at the given horizontal position, with defaults elsewhere.
fn wp(x: f32, y: f32) -> WaypointCommand {
    WaypointCommand {
        position_m: Vec2::new(x, y),
        ..Default::default()
    }
}

/// Steps the follower once from position `(x, y)` and returns the resulting target's x.
fn target_x_after_step<W: AsRef<[WaypointCommand]>>(
    follower: &mut WaypointFollower<W>,
    x: f32,
    y: f32,
) -> f32 {
    let command = follower.next_command(0.1, &state_at(x, y));
    assert2::assert!(command.arm);
    assert2::assert!(let ControlInput::Waypoint(target) = command.input);
    target.position_m.x
}

#[test]
fn rejects_an_empty_waypoint_list() {
    let empty: [WaypointCommand; 0] = [];
    assert2::assert!(let Err(EmptyWaypoints) = WaypointFollower::new(empty));
    assert2::assert!(let Err(EmptyWaypoints) = WaypointFollower::new(Vec::new()));
}

#[test]
fn advances_through_waypoints_and_loops() {
    let mut follower =
        WaypointFollower::new([wp(0.0, 0.0), wp(10.0, 0.0), wp(20.0, 0.0)]).expect("non-empty");

    // (vehicle x, expected target x after the step)
    let steps = [
        (0.0, 10.0),  // at waypoint 0 -> advance to 1
        (10.0, 20.0), // at waypoint 1 -> advance to 2
        (20.0, 0.0),  // at waypoint 2 (last) -> wrap to 0
    ];
    for (x, expected) in steps {
        assert_relative_eq!(target_x_after_step(&mut follower, x, 0.0), expected);
    }
}

#[test]
fn holds_waypoint_until_within_acceptance_radius() {
    let mut follower = WaypointFollower::new([wp(0.0, 0.0), wp(50.0, 0.0)]).expect("non-empty");
    let radius = DEFAULT_ACCEPTANCE_RADIUS_M;

    // (vehicle x, expected target x after the step)
    let steps = [
        (100.0, 0.0),         // far away: hold waypoint 0
        (radius + 1.0, 0.0),  // just outside the radius: hold
        (radius - 1.0, 50.0), // inside the radius: advance to waypoint 1
    ];
    for (x, expected) in steps {
        assert_relative_eq!(target_x_after_step(&mut follower, x, 0.0), expected);
    }
}

#[test]
fn custom_acceptance_radius_changes_advance_threshold() {
    let mut follower = WaypointFollower::new([wp(0.0, 0.0), wp(50.0, 0.0)])
        .expect("non-empty")
        .with_acceptance_radius(10.0);

    // Outside the default radius but inside the custom 10 m one.
    let distance = DEFAULT_ACCEPTANCE_RADIUS_M + 5.0;
    assert!(distance < 10.0);
    assert_relative_eq!(target_x_after_step(&mut follower, distance, 0.0), 50.0);
}

#[test]
fn works_with_static_slices_and_vecs() {
    static PATH: [WaypointCommand; 2] = [
        WaypointCommand {
            position_m: Vec2::new(0.0, 0.0),
            velocity_mps: None,
            altitude_m: 5.0,
            cruise_speed_mps: 2.0,
            roll_rad: None,
        },
        WaypointCommand {
            position_m: Vec2::new(30.0, 0.0),
            velocity_mps: None,
            altitude_m: 5.0,
            cruise_speed_mps: 2.0,
            roll_rad: None,
        },
    ];

    let mut from_static = WaypointFollower::new(&PATH[..]).expect("non-empty");
    let mut from_vec = WaypointFollower::new(PATH.to_vec()).expect("non-empty");

    assert_relative_eq!(target_x_after_step(&mut from_static, 0.0, 0.0), 30.0);
    assert_relative_eq!(target_x_after_step(&mut from_vec, 0.0, 0.0), 30.0);
    check!(from_static.current_index() == 1);
    check!(from_vec.waypoints().len() == 2);
}

/// Steps the follower once from position `(x, y)` and returns the emitted waypoint.
fn step_at<W: AsRef<[WaypointCommand]>>(
    follower: &mut WaypointFollower<W>,
    x: f32,
    y: f32,
) -> WaypointCommand {
    let command = follower.next_command(0.1, &state_at(x, y));
    assert2::assert!(command.arm);
    assert2::assert!(let ControlInput::Waypoint(target) = command.input);
    target
}

/// A waypoint at `(x, y)` flown at `cruise_speed_mps`.
fn cruise(x: f32, y: f32, cruise_speed_mps: f32) -> WaypointCommand {
    WaypointCommand {
        cruise_speed_mps,
        ..wp(x, y)
    }
}

#[test]
fn hold_stops_on_the_last_waypoint_and_completes() {
    let mut follower = WaypointFollower::new([wp(0.0, 0.0), wp(10.0, 0.0), wp(20.0, 0.0)])
        .expect("non-empty")
        .with_route_end(RouteEnd::Hold);

    // (vehicle x, expected target x after the step, complete after the step)
    let steps = [
        (0.0, 10.0, false),
        (10.0, 20.0, false),
        (5.0, 20.0, false), // short of the last: keep flying to it
        (20.0, 20.0, true), // at the last: hold it, done
        (0.0, 20.0, true),  // pushed off: still hold it, still done
    ];
    for (x, expected, complete) in steps {
        assert_relative_eq!(step_at(&mut follower, x, 0.0).position_m.x, expected);
        check!(follower.is_complete() == complete, "at x {x}");
    }
}

#[test]
fn loop_never_completes() {
    let mut follower = WaypointFollower::new([wp(0.0, 0.0), wp(10.0, 0.0)]).expect("non-empty");
    for x in [0.0, 10.0, 0.0, 10.0] {
        step_at(&mut follower, x, 0.0);
        check!(!follower.is_complete());
    }
    check!(follower.current_index() == 0);
}

#[test]
fn single_waypoint_route_holds_or_loops_in_place() {
    for route_end in [RouteEnd::Loop, RouteEnd::Hold] {
        let mut follower = WaypointFollower::new([cruise(5.0, 5.0, 3.0)])
            .expect("non-empty")
            .with_route_end(route_end);
        for _ in 0..3 {
            let target = step_at(&mut follower, 5.0, 5.0);
            check!(target.position_m == Vec2::new(5.0, 5.0));
            check!(target.velocity_mps == None, "{route_end:?}");
        }
        check!(follower.is_complete() == (route_end == RouteEnd::Hold));
    }
}

/// Each waypoint after the first carries a velocity along its leg at its cruise speed,
/// except the one held at the end; a looping route's wrap leg gets one too.
#[test]
fn feeds_cruise_velocity_along_each_leg() {
    let route = [
        cruise(0.0, 0.0, 5.0),
        cruise(10.0, 0.0, 7.0),
        cruise(10.0, 10.0, 7.0),
    ];
    // (route end, vehicle position, expected (target, velocity))
    let hold = [
        (
            (0.0, 0.0),
            (Vec2::new(10.0, 0.0), Some(Vec2::new(7.0, 0.0))),
        ),
        ((10.0, 0.0), (Vec2::new(10.0, 10.0), None)), // the last: stop there
    ];
    let looping = [
        (
            (0.0, 0.0),
            (Vec2::new(10.0, 0.0), Some(Vec2::new(7.0, 0.0))),
        ),
        (
            (10.0, 0.0),
            (Vec2::new(10.0, 10.0), Some(Vec2::new(0.0, 7.0))),
        ),
        // The wrap leg back to the first, at its 5 m/s.
        (
            (10.0, 10.0),
            (
                Vec2::zeros(),
                Some(Vec2::new(-1.0, -1.0) * (5.0 / 2.0_f32.sqrt())),
            ),
        ),
    ];
    for (route_end, steps) in [(RouteEnd::Hold, &hold[..]), (RouteEnd::Loop, &looping[..])] {
        let mut follower = WaypointFollower::new(route)
            .expect("non-empty")
            .with_route_end(route_end);
        // The first waypoint, flown to from wherever the vehicle is: none.
        check!(step_at(&mut follower, 50.0, 50.0).velocity_mps == None);
        for &((x, y), (position, velocity)) in steps {
            let target = step_at(&mut follower, x, y);
            check!(target.position_m == position, "{route_end:?} at ({x}, {y})");
            match (target.velocity_mps, velocity) {
                (Some(got), Some(want)) => {
                    assert!((got - want).amax() < 1e-5, "{got:?} vs {want:?}")
                }
                (got, want) => {
                    check!(got == want, "{route_end:?} at ({x}, {y})");
                }
            }
        }
    }
}

/// A waypoint's own velocity wins; no positive cruise speed or a zero-length leg gives none.
#[test]
fn leg_velocity_defers_to_the_waypoint_and_needs_a_direction_and_speed() {
    let own = WaypointCommand {
        velocity_mps: Some(Vec2::new(1.0, 2.0)),
        ..cruise(10.0, 0.0, 7.0)
    };
    // (second waypoint, expected velocity after advancing to it)
    let cases = [
        (own, Some(Vec2::new(1.0, 2.0))),
        (cruise(10.0, 0.0, 0.0), None),
        (cruise(10.0, 0.0, -2.0), None),
        (cruise(10.0, 0.0, f32::NAN), None),
        (cruise(0.0, 0.0, 7.0), None), // same place as the first: no direction
    ];
    for (second, expected) in cases {
        let mut follower =
            WaypointFollower::new([wp(0.0, 0.0), second, wp(50.0, 50.0)]).expect("non-empty");
        let target = step_at(&mut follower, 0.0, 0.0);
        check!(follower.current_index() == 1);
        check!(target.velocity_mps == expected, "{second:?}");
    }
}

/// A waypoint missed wide still counts once the vehicle is past it along the leg into it; the
/// first waypoint, with no leg, needs the radius.
#[test]
fn advances_past_a_waypoint_missed_wide() {
    let route = [wp(0.0, 0.0), wp(20.0, 0.0), wp(20.0, 20.0), wp(40.0, 20.0)];
    let mut follower = WaypointFollower::new(route).expect("non-empty");

    // (vehicle position, expected active index after the step)
    let steps = [
        ((10.0, 5.0), 0),     // first waypoint: no leg, outside the radius
        ((0.0, 0.0), 1),      // on it: leg +x toward (20, 0)
        ((19.0, 5.0), 1),     // 5 m wide, not yet past
        ((21.0, 5.0), 2),     // 5 m wide, past along +x: advance; leg +y toward (20, 20)
        ((25.0, 19.0), 2),    // wide, not yet past along +y
        ((25.0, 21.0), 3),    // past along +y
        ((0.0, f32::NAN), 3), // NaN position: neither reached nor passed
    ];
    for ((x, y), expected) in steps {
        step_at(&mut follower, x, y);
        check!(follower.current_index() == expected, "at ({x}, {y})");
    }
}

/// Passing the held last waypoint completes the route, like reaching it.
#[test]
fn hold_completes_when_the_last_waypoint_is_passed() {
    let mut follower = WaypointFollower::new([wp(0.0, 0.0), wp(20.0, 0.0)])
        .expect("non-empty")
        .with_route_end(RouteEnd::Hold);
    step_at(&mut follower, 0.0, 0.0);
    check!(!follower.is_complete());
    let target = step_at(&mut follower, 22.0, 8.0);
    check!(follower.is_complete());
    check!(target.position_m == Vec2::new(20.0, 0.0));
}
