use approx::assert_relative_eq;
use assert2::{check, let_assert};
use robomath::{Vec3, vec2};
use uom::si::f32::Length;
use uom::si::length::meter;

use super::{ScriptCommander, ScriptError, parse_waypoints};
use stable_flight::{Command, Commander, VehicleState, WaypointCommand};

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

/// Extracts the waypoint from a `Command`, failing the test otherwise.
fn waypoint(command: Command) -> WaypointCommand {
    let_assert!(Command::Waypoint(wp) = command);
    wp
}

/// Builds a waypoint at the given horizontal position, with defaults elsewhere.
fn wp(x: f32, y: f32) -> WaypointCommand {
    WaypointCommand {
        position: vec2(x, y),
        ..Default::default()
    }
}

/// Steps the commander once from position `(x, y)` and returns the resulting target's x.
fn target_x_after_step(commander: &mut ScriptCommander, x: f32, y: f32) -> f32 {
    waypoint(commander.next_command(0.1, &state_at(x, y)))
        .position
        .x
}

#[test]
fn parses_valid_waypoints() {
    let csv = "\
x,y,altitude,cruise_speed
30.0,0.0,27.5,7.0
0.0,15.0,40.0,7.0
-30.0,-15.0,15.0,7.0
";
    let_assert!(Ok(waypoints) = parse_waypoints("test.csv", csv));
    check!(waypoints.len() == 3);

    // (x, y, altitude, cruise_speed) expected per row.
    let expected = [
        (30.0, 0.0, 27.5, 7.0),
        (0.0, 15.0, 40.0, 7.0),
        (-30.0, -15.0, 15.0, 7.0),
    ];
    for (wp, (x, y, alt, speed)) in waypoints.iter().zip(expected) {
        assert_relative_eq!(wp.position.x, x);
        assert_relative_eq!(wp.position.y, y);
        assert_relative_eq!(wp.altitude, alt);
        assert_relative_eq!(wp.cruise_speed, speed);
        check!(wp.velocity.is_none());
        check!(wp.roll.is_none());
    }
}

#[test]
fn skips_header_blank_and_comment_lines() {
    let csv = "\
# a figure-eight path
x,y,altitude,cruise_speed

10.0,20.0,15.0,5.0

# trailing comment
11.0,21.0,16.0,5.0
";
    let_assert!(Ok(waypoints) = parse_waypoints("test.csv", csv));
    check!(waypoints.len() == 2);
    assert_relative_eq!(waypoints[0].position.x, 10.0);
    assert_relative_eq!(waypoints[1].altitude, 16.0);
}

#[test]
fn parses_headerless_file() {
    let csv = "5.0,6.0,7.0,8.0\n";
    let_assert!(Ok(waypoints) = parse_waypoints("test.csv", csv));
    check!(waypoints.len() == 1);
    assert_relative_eq!(waypoints[0].cruise_speed, 8.0);
}

#[test]
fn empty_or_comment_only_file_has_no_waypoints() {
    let_assert!(Ok(waypoints) = parse_waypoints("test.csv", "# just a comment\n\n"));
    check!(waypoints.is_empty());
    // The commander rejects an empty waypoint list.
    let_assert!(Err(ScriptError::Empty) = ScriptCommander::from_waypoints(waypoints));
}

#[test]
fn wrong_field_count_is_a_parse_error() {
    let csv = "\
x,y,altitude,cruise_speed
1.0,2.0,3.0
";
    let_assert!(
        Err(ScriptError::Parse {
            path,
            line,
            message
        }) = parse_waypoints("test.csv", csv)
    );
    check!(path == "test.csv");
    check!(line == 2);
    check!(message.contains("expected 4 fields"));
}

#[test]
fn non_numeric_field_is_a_parse_error() {
    let csv = "\
x,y,altitude,cruise_speed
1.0,2.0,3.0,7.0
1.0,two,3.0,7.0
";
    let_assert!(Err(ScriptError::Parse { line, message, .. }) = parse_waypoints("test.csv", csv));
    check!(line == 3);
    check!(message.contains("invalid y"));
}

#[test]
fn non_finite_field_is_a_parse_error() {
    for bad in ["nan", "inf", "-inf"] {
        let csv = format!("x,y,altitude,cruise_speed\n1.0,{bad},3.0,7.0\n");
        let_assert!(
            Err(ScriptError::Parse { line, message, .. }) = parse_waypoints("test.csv", &csv)
        );
        check!(line == 2);
        check!(message.contains("non-finite y"));
    }
}

#[test]
fn new_reports_io_error_for_missing_file() {
    let_assert!(Err(ScriptError::Io { path, .. }) = ScriptCommander::new("does/not/exist.csv"));
    check!(path == "does/not/exist.csv");
}

#[test]
fn follows_loaded_waypoints_via_core_follower() {
    // Following semantics are tested in core (`stable_flight::commanders::waypoint`); this
    // only checks that ScriptCommander delegates to it, radius override included.
    let waypoints = vec![wp(0.0, 0.0), wp(50.0, 0.0)];
    let_assert!(Ok(commander) = ScriptCommander::from_waypoints(waypoints));
    let mut commander = commander.with_acceptance_radius(10.0);

    assert_relative_eq!(target_x_after_step(&mut commander, 8.0, 0.0), 50.0);
    assert_relative_eq!(target_x_after_step(&mut commander, 49.0, 0.0), 0.0);
}
