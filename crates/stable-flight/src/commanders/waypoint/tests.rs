use approx::assert_relative_eq;
use assert2::{check, let_assert};
use robomath::{Vec3, vec2};
use uom::si::f32::Length;
use uom::si::length::meter;

use super::*;

/// Builds a `VehicleState` positioned at the given horizontal coordinates (meters).
fn state_at(x: f32, y: f32) -> VehicleState {
    VehicleState {
        position: Vec3 {
            x: Length::new::<meter>(x),
            y: Length::new::<meter>(y),
            z: Length::new::<meter>(0.0),
        },
        ..Default::default()
    }
}

/// Builds a waypoint at the given horizontal position, with defaults elsewhere.
fn wp(x: f32, y: f32) -> WaypointCommand {
    WaypointCommand {
        position: vec2(x, y),
        ..Default::default()
    }
}

/// Steps the follower once from position `(x, y)` and returns the resulting target's x.
fn target_x_after_step<W: AsRef<[WaypointCommand]>>(
    follower: &mut WaypointFollower<W>,
    x: f32,
    y: f32,
) -> f32 {
    let_assert!(Command::Waypoint(target) = follower.next_command(0.1, &state_at(x, y)));
    target.position.x
}

#[test]
fn rejects_an_empty_waypoint_list() {
    let empty: [WaypointCommand; 0] = [];
    let_assert!(Err(EmptyWaypoints) = WaypointFollower::new(empty));
    let_assert!(Err(EmptyWaypoints) = WaypointFollower::new(Vec::new()));
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
    let radius = DEFAULT_ACCEPTANCE_RADIUS;

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
    let distance = DEFAULT_ACCEPTANCE_RADIUS + 5.0;
    assert!(distance < 10.0);
    assert_relative_eq!(target_x_after_step(&mut follower, distance, 0.0), 50.0);
}

#[test]
fn works_with_static_slices_and_vecs() {
    static PATH: [WaypointCommand; 2] = [
        WaypointCommand {
            position: robomath::Vec2 { x: 0.0, y: 0.0 },
            velocity: None,
            altitude: 5.0,
            cruise_speed: 2.0,
            roll: None,
        },
        WaypointCommand {
            position: robomath::Vec2 { x: 30.0, y: 0.0 },
            velocity: None,
            altitude: 5.0,
            cruise_speed: 2.0,
            roll: None,
        },
    ];

    let mut from_static = WaypointFollower::new(&PATH[..]).expect("non-empty");
    let mut from_vec = WaypointFollower::new(PATH.to_vec()).expect("non-empty");

    assert_relative_eq!(target_x_after_step(&mut from_static, 0.0, 0.0), 30.0);
    assert_relative_eq!(target_x_after_step(&mut from_vec, 0.0, 0.0), 30.0);
    check!(from_static.current_index() == 1);
    check!(from_vec.waypoints().len() == 2);
}
