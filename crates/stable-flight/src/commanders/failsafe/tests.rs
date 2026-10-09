use std::cell::Cell;
use std::rc::Rc;

use assert2::{assert, check};

use super::*;
use crate::{AttitudeCommand, RateCommand};

const DT: f32 = 0.1;

/// What the scripted link reports each tick, shared with the test.
#[derive(Debug, Default)]
struct LinkScript {
    since_s: Cell<Option<f32>>,
    arm: Cell<bool>,
    ticks: Cell<u32>,
}

/// A [`LinkedCommander`] whose freshness and arm state the test sets; flies a rate command.
struct Link(Rc<LinkScript>);

impl Commander for Link {
    fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
        self.0.ticks.set(self.0.ticks.get() + 1);
        Command::new(self.0.arm.get(), ControlInput::Rate(RateCommand::default()))
    }
}

impl LinkedCommander for Link {
    fn since_update_s(&self) -> Option<f32> {
        self.0.since_s.get()
    }
}

/// What the recording fallback saw, and whether it releases, shared with the test.
#[derive(Debug)]
struct FallbackScript {
    engaged: Cell<u32>,
    engaged_armed: Cell<Option<bool>>,
    can_release: Cell<bool>,
}

/// A [`Fallback`] that keeps the arm state it was engaged with; flies an attitude command.
struct Recorder {
    script: Rc<FallbackScript>,
    armed: bool,
}

impl Commander for Recorder {
    fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
        Command::new(
            self.armed,
            ControlInput::Attitude(AttitudeCommand::default()),
        )
    }
}

impl Fallback for Recorder {
    fn engage(&mut self, _state: &VehicleState, armed: bool) {
        self.armed = armed;
        self.script.engaged.set(self.script.engaged.get() + 1);
        self.script.engaged_armed.set(Some(armed));
    }

    fn can_release(&self) -> bool {
        self.script.can_release.get()
    }
}

/// A failsafe over the scripted link and recorder, with a 0.5 s timeout and 0.3 s recovery.
fn failsafe() -> (
    FailsafeCommander<Link, Recorder>,
    Rc<LinkScript>,
    Rc<FallbackScript>,
) {
    let link = Rc::new(LinkScript::default());
    let fallback = Rc::new(FallbackScript {
        engaged: Cell::new(0),
        engaged_armed: Cell::new(None),
        can_release: Cell::new(true),
    });
    let commander = FailsafeCommander::new(
        Link(link.clone()),
        Recorder {
            script: fallback.clone(),
            armed: false,
        },
        FailsafeConfig {
            timeout_s: 0.5,
            recover_after_s: 0.3,
        },
    )
    .expect("valid config");
    (commander, link, fallback)
}

/// One tick of a scenario: what the link reports, and what the failsafe should do.
struct Tick {
    since_s: Option<f32>,
    link_arm: bool,
    can_release: bool,
    state: FailsafeState,
    arm: bool,
    /// Fallback engagements so far.
    engaged: u32,
}

const fn tick(since_s: Option<f32>, link_arm: bool, state: FailsafeState, arm: bool) -> Tick {
    Tick {
        since_s,
        link_arm,
        can_release: true,
        state,
        arm,
        engaged: 1,
    }
}

/// Runs `ticks` through a fresh failsafe, checking each, `DT` apart. Returns what the fallback
/// saw, for checks beyond the table.
fn run(scenario: &str, ticks: &[Tick]) -> Rc<FallbackScript> {
    use FailsafeState::{Fallback as F, Primary as P};
    let (mut commander, link, fallback) = failsafe();
    for (i, t) in ticks.iter().enumerate() {
        link.since_s.set(t.since_s);
        link.arm.set(t.link_arm);
        fallback.can_release.set(t.can_release);
        let command = commander.next_command(DT, &VehicleState::default());

        check!(
            commander.failsafe() == Some(t.state),
            "{scenario}: tick {i}"
        );
        check!(command.arm == t.arm, "{scenario}: tick {i} arm");
        check!(
            fallback.engaged.get() == t.engaged,
            "{scenario}: tick {i} engaged"
        );
        let flew = match command.input {
            ControlInput::Rate(_) => P,
            _ => F,
        };
        check!(flew == t.state, "{scenario}: tick {i} input from");
    }
    check!(
        link.ticks.get() == ticks.len() as u32,
        "{scenario}: primary runs every tick"
    );
    fallback
}

#[test]
fn starts_in_the_fallback_and_hands_over_after_recovery() {
    use FailsafeState::{Fallback as F, Primary as P};
    let fresh = Some(0.0);
    run(
        "startup",
        &[
            tick(None, false, F, false), // never heard
            tick(fresh, true, F, false), // fresh 0.1 s; the fallback can't arm
            tick(fresh, true, F, false), // 0.2 s
            tick(fresh, true, P, true),  // 0.3 s: the primary flies, and arms
            tick(fresh, true, P, true),
        ],
    );
}

#[test]
fn falls_back_on_timeout_keeping_the_arm_state() {
    use FailsafeState::{Fallback as F, Primary as P};
    let fresh = Some(0.0);
    let ticks = [
        tick(fresh, true, F, false),
        tick(fresh, true, F, false),
        tick(fresh, true, P, true),
        tick(Some(0.5), true, P, true), // at the timeout: still fresh
        Tick {
            engaged: 2,
            ..tick(Some(0.6), true, F, true) // stale: falls back armed
        },
        Tick {
            engaged: 2,
            ..tick(None, false, F, true) // a stale primary's disarm is no one's
        },
    ];
    let fallback = run("timeout", &ticks);
    check!(
        fallback.engaged_armed.get() == Some(true),
        "engaged with the arm state"
    );
}

#[test]
fn a_dropout_restarts_the_recovery_wait() {
    use FailsafeState::{Fallback as F, Primary as P};
    let fresh = Some(0.0);
    run(
        "flapping link",
        &[
            tick(fresh, false, F, false),     // 0.1 s fresh
            tick(fresh, false, F, false),     // 0.2 s
            tick(Some(0.6), false, F, false), // stale: the wait restarts
            tick(fresh, false, F, false),     // 0.1 s
            tick(fresh, false, F, false),     // 0.2 s
            tick(fresh, false, P, false),     // 0.3 s
        ],
    );
}

#[test]
fn a_fallback_that_cant_release_keeps_flying() {
    use FailsafeState::{Fallback as F, Primary as P};
    let fresh = Some(0.0);
    let holding = |t: Tick| Tick {
        can_release: false,
        ..t
    };
    run(
        "landing",
        &[
            holding(tick(fresh, false, F, false)),
            holding(tick(fresh, false, F, false)),
            holding(tick(fresh, false, F, false)), // recovered, but mid-landing
            holding(tick(fresh, false, F, false)),
            tick(fresh, false, P, false), // released
        ],
    );
}

#[test]
fn honors_a_fresh_disarm_while_the_fallback_flies() {
    use FailsafeState::{Fallback as F, Primary as P};
    let fresh = Some(0.0);
    run(
        "disarm during recovery",
        &[
            tick(fresh, true, F, false),
            tick(fresh, true, F, false),
            tick(fresh, true, P, true), // armed by the link
            Tick {
                engaged: 2,
                ..tick(Some(0.6), true, F, true) // lost: holds armed
            },
            Tick {
                engaged: 2,
                ..tick(fresh, false, F, false) // back and disarming: honored at once
            },
        ],
    );
}

#[test]
fn rejects_invalid_config() {
    // (config, valid)
    let cases = [
        (FailsafeConfig::default(), true),
        (
            FailsafeConfig {
                recover_after_s: 0.0,
                ..FailsafeConfig::default()
            },
            true,
        ),
        (
            FailsafeConfig {
                timeout_s: 0.0,
                ..FailsafeConfig::default()
            },
            false,
        ),
        (
            FailsafeConfig {
                timeout_s: f32::NAN,
                ..FailsafeConfig::default()
            },
            false,
        ),
        (
            FailsafeConfig {
                recover_after_s: -1.0,
                ..FailsafeConfig::default()
            },
            false,
        ),
    ];
    for (config, valid) in cases {
        check!(config.validate().is_ok() == valid, "{config:?}");
    }
    let (_, link, fallback) = failsafe();
    let result = FailsafeCommander::new(
        Link(link),
        Recorder {
            script: fallback,
            armed: false,
        },
        FailsafeConfig {
            timeout_s: -1.0,
            ..FailsafeConfig::default()
        },
    );
    assert!(let Err(_) = result);
}

fn at(x: f32, y: f32, z: f32) -> VehicleState {
    VehicleState {
        position_m: [x, y, z].into(),
        ..Default::default()
    }
}

/// The waypoint a hold flies, and its arm state.
fn hold_target(hold: &mut HoldCommander, state: &VehicleState) -> (WaypointCommand, bool) {
    let command = hold.next_command(DT, state);
    let ControlInput::Waypoint(waypoint) = command.input else {
        panic!("a hold flies a waypoint, got {:?}", command.input);
    };
    (waypoint, command.arm)
}

#[test]
fn hold_latches_where_it_was_engaged_and_never_arms() {
    // (case, engaged at, armed, then flown from, expected target (x, y, altitude))
    let cases = [
        (
            "armed hover",
            at(3.0, 4.0, 10.0),
            true,
            at(5.0, 5.0, 12.0),
            (3.0, 4.0, 10.0),
        ),
        (
            "disarmed on the ground",
            at(1.0, 2.0, 0.0),
            false,
            at(1.0, 2.0, 0.0),
            (1.0, 2.0, 0.0),
        ),
    ];
    for (case, engaged_at, armed, flown_from, (x, y, altitude)) in cases {
        let mut hold = HoldCommander::new(1.5).expect("valid speed");
        hold.engage(&engaged_at, armed);
        let (target, arm) = hold_target(&mut hold, &flown_from);
        check!(arm == armed, "{case}: keeps the arm state");
        check!(
            (target.position_m.x, target.position_m.y, target.altitude_m) == (x, y, altitude),
            "{case}: latched where engaged, not where it drifted"
        );
        check!(target.cruise_speed_mps == 1.5, "{case}");
        check!(target.velocity_mps.is_none(), "{case}: stops on the point");
    }
}

#[test]
fn hold_retries_the_latch_until_the_position_is_finite() {
    let mut hold = HoldCommander::default();
    hold.engage(&at(f32::NAN, 0.0, 5.0), true);

    let (target, arm) = hold_target(&mut hold, &at(0.0, f32::NAN, 5.0));
    check!(arm);
    check!(
        target.position_m.x.is_nan(),
        "no position: a NaN waypoint (level, hover thrust)"
    );
    check!(target.altitude_m.is_nan());

    let (target, _) = hold_target(&mut hold, &at(7.0, 8.0, 9.0));
    check!((target.position_m.x, target.position_m.y, target.altitude_m) == (7.0, 8.0, 9.0));
    let (target, _) = hold_target(&mut hold, &at(0.0, 0.0, 0.0));
    check!(target.altitude_m == 9.0, "latched once finite");
}

#[test]
fn hold_rejects_a_speed_that_isnt_positive() {
    for speed in [0.0, -1.0, f32::NAN, f32::INFINITY] {
        check!(HoldCommander::new(speed).is_err(), "{speed}");
    }
    check!(HoldCommander::default().cruise_speed_mps == DEFAULT_HOLD_SPEED_MPS);
}
