use assert2::check;
use stable_flight::commanders::maneuver::{Axis, Maneuver, ManeuverConfig};
use stable_flight::{ControlInput, VehicleState};

use super::{Actions, RowActions, Step};

/// Rows holding 0, 0.35, and 0.2 s.
fn holds() -> Actions {
    let hold = |hold_s| RowActions {
        maneuver: None,
        hold_s,
    };
    Actions::new(
        vec![hold(0.0), hold(0.35), hold(0.2)],
        ManeuverConfig::default(),
    )
}

/// A route tick for [`Actions::start`]: (waypoint it began on, waypoint it ended on, lands).
type RouteTick = (usize, usize, bool);

/// A row's actions start when the route moves on from its waypoint, to another waypoint or
/// into the land, and only on that tick: (case, route ticks, the last tick's start).
#[test]
fn actions_start_on_moving_on() {
    let cases: [(&str, &[RouteTick], Option<usize>); 6] = [
        ("moved on", &[(1, 2, false)], Some(1)),
        ("still on it", &[(1, 1, false)], None),
        ("no hold", &[(0, 1, false)], None),
        ("looped back", &[(2, 0, false)], Some(2)),
        ("into the land", &[(1, 1, false), (1, 1, true)], Some(1)),
        ("landing on", &[(1, 1, true), (1, 1, true)], None),
    ];
    let state = VehicleState::default();
    for (case, ticks, expected) in cases {
        let mut actions = holds();
        let mut started = None;
        for &(reached, current, lands) in ticks {
            started = actions.start(reached, current, lands, 0.1, &state);
        }
        check!(
            started == expected.map(|index| Step::Hold { index }),
            "{case}"
        );
    }
}

/// A hold lasts its seconds from the tick it starts, and a NaN `dt` ends it.
#[test]
fn a_hold_lasts_its_seconds() {
    let state = VehicleState::default();
    let mut actions = holds();
    check!(actions.step(0.1, &state).is_none());
    actions.start(1, 2, false, 0.1, &state);
    let held: Vec<_> = (0..4).map(|_| actions.step(0.1, &state)).collect();
    let hold = Some(Step::Hold { index: 1 });
    check!(held == [hold, hold, hold, None]);
    actions.start(2, 0, false, 0.1, &state);
    check!(actions.step(f32::NAN, &state).is_none());
}

/// A row's maneuver flies from the tick its actions start, then its hold: a doublet of
/// quarter-second halves, a level, still vehicle's one-tick recovery, then a 0.25 s hold.
#[test]
fn a_maneuver_flies_before_the_hold() {
    let row = RowActions {
        maneuver: Some(Maneuver::doublet(Axis::Yaw, 0.5, 0.25).expect("valid")),
        hold_s: 0.25,
    };
    let mut actions = Actions::new(vec![RowActions::default(), row], ManeuverConfig::default());
    let state = VehicleState::default();
    let mut steps = vec![actions.start(1, 2, false, 0.25, &state)];
    steps.extend((0..8).map(|_| actions.step(0.25, &state)));
    let kinds: Vec<_> = steps
        .iter()
        .map(|step| match step {
            Some(Step::Maneuver {
                index: 1,
                input: ControlInput::AbsoluteRate(rates),
            }) => Some(rates.yaw_radps),
            Some(Step::Maneuver {
                index: 1,
                input: ControlInput::AbsoluteAttitude(_),
            }) => Some(f32::INFINITY),
            Some(Step::Hold { index: 1 }) => Some(f32::NEG_INFINITY),
            _ => None,
        })
        .collect();
    // Rates +0.5, -0.5, four ticks of pause, one of recovery, the hold, then the route.
    let pause = Some(0.0);
    check!(
        kinds
            == [
                Some(0.5),
                Some(-0.5),
                pause,
                pause,
                pause,
                pause,
                Some(f32::INFINITY),
                Some(f32::NEG_INFINITY),
                None,
            ]
    );
}
