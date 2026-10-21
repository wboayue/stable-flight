use approx::assert_relative_eq;
use assert2::check;
use stable_flight::{Vec2, Vec3};
use std::f32::consts::{FRAC_PI_2, FRAC_PI_4};

use super::{Holds, Leg, ScriptCommander, ScriptError, parse_route};
use stable_flight::commanders::RouteEnd;
use stable_flight::{
    Command, Commander, ControlInput, FlightRegime, VehicleState, WaypointCommand,
};

/// The waypoints of a parsed script, without their regimes.
fn parse_waypoints(path: &str, contents: &str) -> Result<Vec<WaypointCommand>, ScriptError> {
    Ok(parse_route(path, contents)?
        .into_iter()
        .map(|leg| leg.waypoint)
        .collect())
}

/// Builds a `VehicleState` positioned at the given horizontal coordinates (meters).
fn state_at(x: f32, y: f32) -> VehicleState {
    VehicleState {
        position_m: Vec3::new(x, y, 0.0),
        ..Default::default()
    }
}

/// Extracts the waypoint from an armed `Command`, failing the test otherwise.
fn waypoint(command: Command) -> WaypointCommand {
    assert2::assert!(command.arm);
    assert2::assert!(let ControlInput::Waypoint(wp) = command.input);
    wp
}

/// Builds a waypoint at the given horizontal position, with defaults elsewhere.
fn wp(x: f32, y: f32) -> WaypointCommand {
    WaypointCommand {
        position_m: Vec2::new(x, y),
        ..Default::default()
    }
}

/// Steps the commander once from position `(x, y)` and returns the resulting target's x.
fn target_x_after_step(commander: &mut ScriptCommander, x: f32, y: f32) -> f32 {
    waypoint(commander.next_command(0.1, &state_at(x, y)))
        .position_m
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
    assert2::assert!(let Ok(waypoints) = parse_waypoints("test.csv", csv));
    check!(waypoints.len() == 3);

    // (x, y, altitude, cruise_speed) expected per row.
    let expected = [
        (30.0, 0.0, 27.5, 7.0),
        (0.0, 15.0, 40.0, 7.0),
        (-30.0, -15.0, 15.0, 7.0),
    ];
    for (wp, (x, y, alt, speed)) in waypoints.iter().zip(expected) {
        assert_relative_eq!(wp.position_m.x, x);
        assert_relative_eq!(wp.position_m.y, y);
        assert_relative_eq!(wp.altitude_m, alt);
        assert_relative_eq!(wp.cruise_speed_mps, speed);
        check!(wp.velocity_mps.is_none());
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
    assert2::assert!(let Ok(waypoints) = parse_waypoints("test.csv", csv));
    check!(waypoints.len() == 2);
    assert_relative_eq!(waypoints[0].position_m.x, 10.0);
    assert_relative_eq!(waypoints[1].altitude_m, 16.0);
}

#[test]
fn parses_headerless_file() {
    let csv = "5.0,6.0,7.0,8.0\n";
    assert2::assert!(let Ok(waypoints) = parse_waypoints("test.csv", csv));
    check!(waypoints.len() == 1);
    assert_relative_eq!(waypoints[0].cruise_speed_mps, 8.0);
}

#[test]
fn empty_or_comment_only_file_has_no_waypoints() {
    assert2::assert!(let Ok(waypoints) = parse_waypoints("test.csv", "# just a comment\n\n"));
    check!(waypoints.is_empty());
    // The commander rejects an empty waypoint list.
    assert2::assert!(let Err(ScriptError::Empty) = ScriptCommander::from_waypoints(waypoints));
}

#[test]
fn wrong_field_count_is_a_parse_error() {
    let csv = "\
x,y,altitude,cruise_speed
1.0,2.0,3.0
";
    assert2::assert!(
        let Err(ScriptError::Parse {
            path,
            line,
            message
        }) = parse_waypoints("test.csv", csv)
    );
    check!(path == "test.csv");
    check!(line == 2);
    check!(message.contains("expected 4 to 7 fields"));
}

#[test]
fn non_numeric_field_is_a_parse_error() {
    let csv = "\
x,y,altitude,cruise_speed
1.0,2.0,3.0,7.0
1.0,two,3.0,7.0
";
    assert2::assert!(let Err(ScriptError::Parse { line, message, .. }) = parse_waypoints("test.csv", csv));
    check!(line == 3);
    check!(message.contains("invalid y"));
}

#[test]
fn non_finite_field_is_a_parse_error() {
    for bad in ["nan", "inf", "-inf"] {
        let csv = format!("x,y,altitude,cruise_speed\n1.0,{bad},3.0,7.0\n");
        assert2::assert!(
            let Err(ScriptError::Parse { line, message, .. }) = parse_waypoints("test.csv", &csv)
        );
        check!(line == 2);
        check!(message.contains("non-finite y"));
    }
}

#[test]
fn new_reports_io_error_for_missing_file() {
    assert2::assert!(let Err(ScriptError::Io { path, .. }) = ScriptCommander::new("does/not/exist.csv"));
    check!(path == "does/not/exist.csv");
}

#[test]
fn follows_loaded_waypoints_via_core_follower() {
    // Following semantics are tested in core (`stable_flight::commanders::waypoint`); this
    // only checks that ScriptCommander delegates to it, radius override included.
    let waypoints = vec![wp(0.0, 0.0), wp(50.0, 0.0)];
    assert2::assert!(let Ok(commander) = ScriptCommander::from_waypoints(waypoints));
    let mut commander = commander.with_acceptance_radius(10.0);

    assert_relative_eq!(target_x_after_step(&mut commander, 8.0, 0.0), 50.0);
    check!(commander.follower().current_index() == 1);
    assert_relative_eq!(target_x_after_step(&mut commander, 49.0, 0.0), 0.0);
    check!(commander.follower().waypoints().len() == 2);
}

/// The optional fifth field is the leg's regime, any case; absent or empty is copter; anything
/// else, or an eighth field, is a parse error.
#[test]
fn fifth_field_is_the_regime() {
    let csv = "\
x,y,altitude,cruise_speed,regime
0,0,20,3
200,0,30,14,plane
200,100,30,14, Plane
0,0,20,3,copter
0,0,20,3,,90
";
    assert2::assert!(let Ok(route) = parse_route("test.csv", csv));
    let regimes: Vec<_> = route.iter().map(|leg| leg.regime).collect();
    check!(
        regimes
            == [
                FlightRegime::Copter,
                FlightRegime::Plane,
                FlightRegime::Plane,
                FlightRegime::Copter,
                FlightRegime::Copter,
            ]
    );

    for bad in ["0,0,20,3,boat", "0,0,20,3,plane,0,0,extra"] {
        assert2::assert!(let Err(ScriptError::Parse { line, .. }) = parse_route("test.csv", bad));
        check!(line == 1, "{bad}");
    }
}

/// The commander asks for the regime of the leg to the current waypoint, changing as legs
/// advance.
#[test]
fn commander_requests_each_legs_regime() {
    // Through both builders, which rebuild the commander: the regimes carry over.
    let mut commander = ScriptCommander::from_route(vec![
        (wp(0.0, 0.0), FlightRegime::Copter),
        (wp(100.0, 0.0), FlightRegime::Plane),
        (wp(100.0, 100.0), FlightRegime::Copter),
    ])
    .expect("non-empty")
    .with_acceptance_radius(1.0)
    .with_route_end(stable_flight::commanders::RouteEnd::Hold);
    // At the first waypoint: advances to the plane leg.
    check!(commander.next_command(0.01, &state_at(0.0, 0.0)).regime == FlightRegime::Plane);
    check!(commander.next_command(0.01, &state_at(50.0, 0.0)).regime == FlightRegime::Plane);
    // At the second: the copter leg.
    check!(commander.next_command(0.01, &state_at(100.0, 0.0)).regime == FlightRegime::Copter);

    let all_copter = ScriptCommander::from_waypoints(vec![wp(0.0, 0.0), wp(10.0, 0.0)])
        .expect("non-empty")
        .next_command(0.01, &state_at(0.0, 0.0));
    check!(all_copter.regime == FlightRegime::Copter);
}

/// Each leg's waypoint is reached within its regime's altitude acceptance: a plane leg's
/// loose band takes it 5 m off, a copter leg's tight band doesn't, and either band can be
/// overridden.
#[test]
fn altitude_acceptance_follows_each_legs_regime() {
    let at = |x: f32, altitude_m: f32| WaypointCommand {
        altitude_m,
        ..wp(x, 0.0)
    };
    let route = || {
        vec![
            (at(0.0, 30.0), FlightRegime::Copter),
            (at(100.0, 30.0), FlightRegime::Plane),
            (at(200.0, 30.0), FlightRegime::Copter),
        ]
    };
    let state = |x: f32, z: f32| VehicleState {
        position_m: Vec3::new(x, 0.0, z),
        ..Default::default()
    };
    // (name, overrides (copter, plane), vehicle (x, z) on each waypoint, expected index after)
    let cases = [
        ("copter leg, 5 m low", (None, None), &[(0.0, 25.0)][..], 0),
        ("copter leg, at altitude", (None, None), &[(0.0, 30.0)], 1),
        (
            "plane leg, 5 m low",
            (None, None),
            &[(0.0, 30.0), (100.0, 25.0)],
            2,
        ),
        ("copter band widened", (Some(6.0), None), &[(0.0, 25.0)], 1),
        (
            "plane band tightened",
            (None, Some(1.0)),
            &[(0.0, 30.0), (100.0, 25.0)],
            1,
        ),
    ];
    for (name, (copter, plane), steps, expected) in cases {
        let mut commander = ScriptCommander::from_route(route()).expect("non-empty");
        if let Some(band) = copter {
            commander = commander.with_altitude_acceptance(FlightRegime::Copter, band);
        }
        if let Some(band) = plane {
            commander = commander.with_altitude_acceptance(FlightRegime::Plane, band);
        }
        for &(x, z) in steps {
            commander.next_command(0.01, &state(x, z));
        }
        check!(commander.follower().current_index() == expected, "{name}");
    }
}

/// The optional sixth field is the heading in degrees and the seventh the hold in seconds;
/// absent or empty is no heading and no hold. A non-numeric or non-finite heading or hold, or
/// a negative hold, is a parse error.
#[test]
fn sixth_and_seventh_fields_are_heading_and_hold() {
    // (row, expected (heading, hold_s))
    let cases = [
        ("0,0,20,3", (None, 0.0)),
        ("0,0,20,3,,90", (Some(FRAC_PI_2), 0.0)),
        ("0,0,20,3,plane,-45,20", (Some(-FRAC_PI_4), 20.0)),
        ("0,0,20,3,,,5", (None, 5.0)),
        ("0,0,20,3,copter,,", (None, 0.0)),
    ];
    for (row, (heading_rad, hold_s)) in cases {
        assert2::assert!(let Ok(route) = parse_route("test.csv", row));
        let [leg] = route[..] else {
            panic!("{row}: expected one row, got {route:?}");
        };
        check!(leg.waypoint.heading_rad == heading_rad, "{row}");
        check!(leg.hold_s == hold_s, "{row}");
    }

    // (row, expected message)
    let errors = [
        ("0,0,20,3,,east", "invalid heading"),
        ("0,0,20,3,,nan", "non-finite heading"),
        ("0,0,20,3,,0,soon", "invalid hold_s"),
        ("0,0,20,3,,0,inf", "non-finite hold_s"),
        ("0,0,20,3,,0,-1", "negative hold_s"),
    ];
    for (row, expected) in errors {
        assert2::assert!(let Err(ScriptError::Parse { message, .. }) = parse_route("test.csv", row));
        check!(message.contains(expected), "{row}: {message}");
    }
}

/// A route of waypoints along x, each with the given hold and heading, and a 5 m/s cruise.
fn holding_route(holds: &[(f32, f32)]) -> Vec<Leg> {
    holds
        .iter()
        .enumerate()
        .map(|(index, &(hold_s, heading_rad))| Leg {
            waypoint: WaypointCommand {
                cruise_speed_mps: 5.0,
                heading_rad: Some(heading_rad),
                ..wp(50.0 * index as f32, 0.0)
            },
            regime: FlightRegime::Copter,
            hold_s,
        })
        .collect()
}

/// A reached waypoint with a hold keeps being commanded for its hold, without the leg's
/// fly-through velocity, while the follower waits on the next waypoint; then the next is
/// flown. 0.35 s of 0.1 s ticks: the reaching tick and three more.
#[test]
fn holds_a_reached_waypoint_for_its_hold() {
    let mut commander =
        ScriptCommander::from_legs(holding_route(&[(0.0, 0.0), (0.35, 1.0), (0.0, 2.0)]))
            .expect("non-empty");
    // Reaches the first, then flies the leg to the second with its velocity.
    commander.next_command(0.1, &state_at(0.0, 0.0));
    let on_leg = waypoint(commander.next_command(0.1, &state_at(25.0, 0.0)));
    check!(on_leg.velocity_mps == Some(Vec2::new(5.0, 0.0)));

    let mut targets = Vec::new();
    for _ in 0..5 {
        let target = waypoint(commander.next_command(0.1, &state_at(50.0, 0.0)));
        targets.push((target.position_m.x, target.velocity_mps, target.heading_rad));
    }
    let held = (50.0, None, Some(1.0));
    let next = (100.0, Some(Vec2::new(5.0, 0.0)), Some(2.0));
    check!(targets == [held, held, held, held, next]);
    check!(commander.follower().current_index() == 2);
}

/// A hold on the last waypoint of a route that lands is flown before the land.
#[test]
fn holds_the_last_waypoint_before_the_land() {
    let mut commander = ScriptCommander::from_legs(holding_route(&[(0.35, 1.0)]))
        .expect("non-empty")
        .with_route_end(RouteEnd::Land);
    let inputs: Vec<_> = (0..5)
        .map(|_| commander.next_command(0.1, &state_at(0.0, 0.0)).input)
        .collect();
    let held = ControlInput::Waypoint(holding_route(&[(0.35, 1.0)])[0].waypoint);
    check!(inputs[..4] == [held; 4]);
    assert2::assert!(let ControlInput::Land(_) = inputs[4]);
}

/// A follower tick for [`Holds::start`]: (waypoint it began on, waypoint it ended on, lands).
type FollowerTick = (usize, usize, bool);

/// A hold starts when the follower moves on from a waypoint with one, to another waypoint or
/// into the land, and only on that tick; it lasts its seconds, and a NaN `dt` ends it.
#[test]
fn a_hold_starts_on_moving_on_and_lasts_its_seconds() {
    // (case, follower ticks (reached, current, lands), expected start of the last)
    let cases: [(&str, &[FollowerTick], Option<usize>); 6] = [
        ("moved on", &[(1, 2, false)], Some(1)),
        ("still on it", &[(1, 1, false)], None),
        ("no hold", &[(0, 1, false)], None),
        ("looped back", &[(2, 0, false)], Some(2)),
        ("into the land", &[(1, 1, false), (1, 1, true)], Some(1)),
        ("landing on", &[(1, 1, true), (1, 1, true)], None),
    ];
    for (case, ticks, expected) in cases {
        let mut holds = Holds::new(vec![0.0, 0.35, 0.2]);
        let mut started = None;
        for &(reached, current, lands) in ticks {
            started = holds.start(reached, current, lands);
        }
        check!(started == expected, "{case}");
    }

    let mut holds = Holds::new(vec![0.35]);
    holds.start(0, 0, true);
    let held: Vec<_> = (0..4).map(|_| holds.held(0.1)).collect();
    check!(held == [Some(0), Some(0), Some(0), None]);
    holds.start(0, 1, false);
    check!(holds.held(f32::NAN) == None);
}
