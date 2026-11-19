use std::cell::Cell;
use std::rc::Rc;

use assert2::{assert, check};

use super::*;
use crate::{AttitudeCommand, FlightRegime, LandCommand, RateCommand, Vec2, WaypointCommand};

const DT: f32 = 0.004;

/// Flies whatever its cell holds, counting its calls.
struct Scripted {
    command: Rc<Cell<Command>>,
    calls: Rc<Cell<u32>>,
}

impl Commander for Scripted {
    fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
        self.calls.set(self.calls.get() + 1);
        self.command.get()
    }
}

fn sticks(roll: f32, pitch: f32) -> Command {
    Command::armed(ControlInput::Attitude(AttitudeCommand {
        throttle: 0.4,
        roll,
        pitch,
        yaw: 0.0,
    }))
}

fn state_at(x_m: f32, altitude_m: f32) -> VehicleState {
    VehicleState {
        position_m: [x_m, 2.0, altitude_m].into(),
        ..Default::default()
    }
}

/// A hold over a scripted inner commander flying `first`, engaged at `state_at(1, 10)`.
fn engaged(first: Command) -> (AutotuneHold<Scripted>, Rc<Cell<Command>>, Rc<Cell<u32>>) {
    let command = Rc::new(Cell::new(first));
    let calls = Rc::new(Cell::new(0));
    let inner = Scripted {
        command: Rc::clone(&command),
        calls: Rc::clone(&calls),
    };
    let mut hold = AutotuneHold::new(inner, AutotuneHoldConfig::default()).expect("valid");
    let state = state_at(1.0, 10.0);
    hold.next_command(DT, &state);
    hold.engage(&state);
    (hold, command, calls)
}

fn waypoint(command: &Command) -> WaypointCommand {
    let ControlInput::Waypoint(waypoint) = command.input else {
        panic!("not a hold: {command:?}");
    };
    waypoint
}

/// Engaged, it holds the position it was engaged at, armed, whatever the inner sticks' throttle,
/// and still runs the inner commander each tick.
#[test]
fn holds_where_it_was_engaged() {
    let (mut hold, _command, calls) = engaged(sticks(0.0, 0.03));
    let command = hold.next_command(DT, &state_at(4.0, 8.0));

    check!(command.arm);
    let waypoint = waypoint(&command);
    check!(waypoint.position_m == Vec2::new(1.0, 2.0));
    check!(waypoint.altitude_m == 10.0);
    check!(waypoint.cruise_speed_mps == DEFAULT_HOLD_SPEED_MPS);
    check!(calls.get() == 2);
    check!(!hold.take_override());
    check!(hold.is_engaged());
}

/// Before engaging and after releasing, the inner command flies untouched.
#[test]
fn flies_the_inner_commander_unless_engaged() {
    let command = sticks(0.2, 0.0);
    let inner = Scripted {
        command: Rc::new(Cell::new(command)),
        calls: Rc::new(Cell::new(0)),
    };
    let mut hold = AutotuneHold::new(inner, AutotuneHoldConfig::default()).expect("valid");
    check!(hold.next_command(DT, &state_at(0.0, 5.0)) == command);

    let (mut hold, inner_command, _) = engaged(sticks(0.0, 0.0));
    hold.release();
    inner_command.set(command);
    check!(hold.next_command(DT, &state_at(0.0, 5.0)) == command);
    check!(!hold.take_override());
}

/// The inner command takes over, once, on a disarm, a land, or a roll or pitch stick past the
/// threshold; yaw, throttle, a stick within the threshold, a NaN stick, and a waypoint don't.
#[test]
fn the_inner_commander_takes_over() {
    let land = Command::armed(ControlInput::Land(LandCommand {
        position_m: Vec2::new(5.0, 5.0),
    }));
    let rate = |roll_rate, pitch_rate| {
        Command::armed(ControlInput::Rate(RateCommand {
            throttle: 0.4,
            roll_rate,
            pitch_rate,
            yaw_rate: 0.9,
        }))
    };
    let yaw_and_throttle = Command::armed(ControlInput::Attitude(AttitudeCommand {
        throttle: 0.9,
        yaw: 1.0,
        ..Default::default()
    }));
    let rows = [
        ("disarm", Command::default(), true),
        ("land", land, true),
        ("roll stick", sticks(-0.06, 0.0), true),
        ("pitch stick", sticks(0.0, 0.06), true),
        ("roll rate stick", rate(0.06, 0.0), true),
        ("pitch rate stick", rate(0.0, -0.06), true),
        ("within the threshold", sticks(0.04, -0.04), false),
        ("NaN sticks", sticks(f32::NAN, f32::NAN), false),
        ("yaw and throttle", yaw_and_throttle, false),
        ("rate yaw", rate(0.0, 0.0), false),
        (
            "waypoint",
            Command::armed(ControlInput::Waypoint(Default::default())),
            false,
        ),
    ];
    for (case, command, takes_over) in rows {
        let (mut hold, inner, _) = engaged(sticks(0.0, 0.0));
        inner.set(command);
        let flown = hold.next_command(DT, &state_at(0.0, 5.0));
        check!((flown == command) == takes_over, "{case}: {flown:?}");
        check!(hold.take_override() == takes_over, "{case}");
        check!(!hold.take_override(), "{case}: cleared");
        check!(hold.is_engaged() == !takes_over, "{case}");
    }
}

/// The hold keeps the inner commander's regime request, so a ground station's transition isn't
/// dropped by the hold.
#[test]
fn keeps_the_inner_regime() {
    let mut command = sticks(0.0, 0.0);
    command.regime = FlightRegime::Plane;
    let (mut hold, _inner, _) = engaged(command);
    check!(hold.next_command(DT, &state_at(0.0, 5.0)).regime == FlightRegime::Plane);
}

/// A hold engaged while the inner commander was disarmed stays disarmed.
#[test]
fn never_arms_a_disarmed_vehicle() {
    let (mut hold, inner, _) = engaged(Command::default());
    inner.set(Command::armed(ControlInput::Waypoint(Default::default())));
    check!(!hold.next_command(DT, &state_at(0.0, 5.0)).arm);
}

/// `validate` rejects a non-positive or non-finite hold speed and an override stick outside
/// (0, 1].
#[test]
fn config_validation() {
    check!(AutotuneHoldConfig::default().validate() == Ok(()));
    let rows = [
        ("hold_speed_mps", 0.0, DEFAULT_OVERRIDE_STICK),
        ("hold_speed_mps", f32::NAN, DEFAULT_OVERRIDE_STICK),
        ("override_stick", 2.0, 0.0),
        ("override_stick", 2.0, 1.5),
        ("override_stick", 2.0, f32::NAN),
    ];
    for (field, hold_speed_mps, override_stick) in rows {
        let config = AutotuneHoldConfig {
            hold_speed_mps,
            override_stick,
        };
        let error = config.validate().expect_err("invalid");
        assert!(error.field == field, "{config:?}: {error:?}");
    }
    let edge = AutotuneHoldConfig {
        override_stick: 1.0,
        ..Default::default()
    };
    check!(edge.validate() == Ok(()));
}

/// The inner commander's failsafe state is the hold's, engaged or not, so the link still reports
/// who flies.
#[test]
fn reports_the_inner_failsafe_state() {
    struct Failsafe;
    impl Commander for Failsafe {
        fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
            Command::default()
        }
        fn failsafe(&self) -> Option<FailsafeState> {
            Some(FailsafeState::Fallback)
        }
    }
    let mut hold = AutotuneHold::new(Failsafe, AutotuneHoldConfig::default()).expect("valid");
    check!(hold.failsafe() == Some(FailsafeState::Fallback));
    hold.engage(&state_at(0.0, 5.0));
    check!(hold.failsafe() == Some(FailsafeState::Fallback));
}
