use std::cell::Cell;
use std::rc::Rc;

use assert2::{assert, check};

use super::*;
use crate::{AbsoluteAttitudeCommand, AttitudeCommand, LandCommand, RateCommand, Vec3};

const DT: f32 = 0.1;

/// What the scripted link reports each tick, shared with the test.
#[derive(Debug, Default)]
struct LinkScript {
    since_s: Cell<Option<f32>>,
    /// The latched arm state: set by the test (an `Arm` message) or by `sync_arm`.
    arm: Cell<bool>,
    /// What it flies; `None` is a neutral rate command (sticks).
    input: Cell<Option<ControlInput>>,
    /// The last `sync_arm` call's argument.
    synced: Cell<Option<bool>>,
    ticks: Cell<u32>,
}

/// A [`LinkedCommander`] whose freshness, arm state, and input the test sets.
struct Link(Rc<LinkScript>);

impl Commander for Link {
    fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
        self.0.ticks.set(self.0.ticks.get() + 1);
        let input = self
            .0
            .input
            .get()
            .unwrap_or(ControlInput::Rate(RateCommand::default()));
        Command::new(self.0.arm.get(), input)
    }
}

impl LinkedCommander for Link {
    fn since_update_s(&self) -> Option<f32> {
        self.0.since_s.get()
    }

    fn sync_arm(&mut self, armed: bool) {
        self.0.arm.set(armed);
        self.0.synced.set(Some(armed));
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
    fn engage(&mut self, _state: &VehicleState, last: &Command) {
        self.armed = last.arm;
        self.script.engaged.set(self.script.engaged.get() + 1);
        self.script.engaged_armed.set(Some(last.arm));
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
            takeover_stick_delta: 0.1,
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
            tick(fresh, true, P, false), // 0.3 s: the primary flies; the arm sent during the
            // fallback is dropped at the handover (#235)
            tick(fresh, true, P, true), // a new arm request arms
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
        tick(fresh, true, P, false), // handover drops the arm sent while recovering
        tick(fresh, true, P, true),  // the link re-sends it: armed
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
            tick(fresh, true, P, false), // handover drops the arm sent while recovering
            tick(fresh, true, P, true),  // the link re-sends it: armed
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
        (
            FailsafeConfig {
                takeover_stick_delta: 0.0,
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
        hold.engage(&engaged_at, &Command::new(armed, hover_at(20.0)));
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
    hold.engage(&at(f32::NAN, 0.0, 5.0), &Command::new(true, hover_at(20.0)));

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
fn hold_finishes_an_armed_land_and_holds_otherwise() {
    let land_at = ControlInput::Land(LandCommand {
        position_m: Vec2::new(1.0, 2.0),
    });
    // (case, last command, lands)
    let cases = [
        ("armed land", Command::new(true, land_at), true),
        (
            "disarmed land: nothing to finish",
            Command::new(false, land_at),
            false,
        ),
        ("armed hover", Command::new(true, hover_at(20.0)), false),
    ];
    for (case, last, lands) in cases {
        let mut hold = HoldCommander::default();
        hold.engage(&at(0.0, 0.0, 0.0), &Command::new(true, land_at));
        hold.engage(&at(3.0, 4.0, 10.0), &last); // re-engaged: forgets the earlier land
        check!(hold.is_landing() == lands, "{case}");
        let command = hold.next_command(DT, &at(5.0, 5.0, 8.0));
        check!(command.arm == last.arm, "{case}: keeps the arm state");
        if lands {
            check!(command.input == land_at, "{case}: the same land");
        } else {
            let (target, _) = hold_target(&mut hold, &at(5.0, 5.0, 8.0));
            check!(
                (target.position_m.x, target.position_m.y, target.altitude_m) == (3.0, 4.0, 10.0),
                "{case}: holds where engaged"
            );
        }
    }
}

#[test]
fn hold_rejects_a_speed_that_isnt_positive() {
    for speed in [0.0, -1.0, f32::NAN, f32::INFINITY] {
        check!(HoldCommander::new(speed).is_err(), "{speed}");
    }
    check!(HoldCommander::default().cruise_speed_mps == DEFAULT_HOLD_SPEED_MPS);
}

/// One tick of a handover scenario: the link's freshness, an `Arm` message if any (`None`
/// leaves the latched state), and its input if changed.
fn step(
    commander: &mut FailsafeCommander<Link, Recorder>,
    link: &LinkScript,
    since_s: Option<f32>,
    arm: Option<bool>,
    input: Option<ControlInput>,
) -> Command {
    link.since_s.set(since_s);
    if let Some(arm) = arm {
        link.arm.set(arm);
    }
    if input.is_some() {
        link.input.set(input);
    }
    commander.next_command(DT, &VehicleState::default())
}

fn hover_at(altitude_m: f32) -> ControlInput {
    ControlInput::Waypoint(WaypointCommand {
        altitude_m,
        ..Default::default()
    })
}

fn sticks(throttle: f32, roll: f32) -> ControlInput {
    ControlInput::Attitude(AttitudeCommand {
        throttle,
        roll,
        ..Default::default()
    })
}

#[test]
fn drops_an_arm_latched_while_the_fallback_flew() {
    use FailsafeState::{Fallback as F, Primary as P};
    let fresh = Some(0.0);
    let (mut commander, link, _) = failsafe();
    // (case, since, Arm message, expected state, expected arm)
    let ticks = [
        ("never heard", None, Some(false), F, false),
        ("arm sent while recovering", fresh, Some(true), F, false),
        ("still recovering", fresh, None, F, false),
        ("handover: the stale arm is dropped", fresh, None, P, false),
        ("stays disarmed", fresh, None, P, false),
        ("a new arm arms", fresh, Some(true), P, true),
    ];
    for (case, since_s, arm, state, armed) in ticks {
        let command = step(&mut commander, &link, since_s, arm, Some(hover_at(5.0)));
        check!(commander.failsafe() == Some(state), "{case}");
        check!(command.arm == armed, "{case}");
    }
    check!(link.synced.get() == Some(false), "synced to the vehicle");
}

/// A failsafe whose link armed the vehicle and flew `input`, then went stale: the fallback
/// holds, armed.
fn armed_then_lost(input: ControlInput) -> (FailsafeCommander<Link, Recorder>, Rc<LinkScript>) {
    let (mut commander, link, _) = failsafe();
    let fresh = Some(0.0);
    for _ in 0..4 {
        step(&mut commander, &link, fresh, Some(true), Some(input));
    }
    let command = step(&mut commander, &link, fresh, None, None);
    assert!(commander.failsafe() == Some(FailsafeState::Primary) && command.arm);
    let command = step(&mut commander, &link, Some(0.6), None, None);
    assert!(commander.failsafe() == Some(FailsafeState::Fallback) && command.arm);
    (commander, link)
}

/// Waypoints and offboard absolute attitudes aren't sticks: a recovered link flying one takes
/// back an armed vehicle at once, unchanged (an app streaming a constant hover never "moves").
#[test]
fn a_waypoint_or_absolute_attitude_takes_back_an_armed_vehicle_at_once_and_stays_armed() {
    let absolute = ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
        thrust: 0.45,
        ..Default::default()
    });
    for input in [hover_at(5.0), absolute] {
        let (mut commander, link) = armed_then_lost(input);
        let fresh = Some(0.0);
        for _ in 0..2 {
            step(&mut commander, &link, fresh, None, None);
        }
        let command = step(&mut commander, &link, fresh, None, None); // recovered: 0.3 s
        check!(
            commander.failsafe() == Some(FailsafeState::Primary),
            "{input:?}"
        );
        check!(command.arm, "{input:?}: armed through the hold");
        check!(command.input == input, "{input:?}");
        check!(link.synced.get() == Some(true), "{input:?}");
    }
}

#[test]
fn sticks_take_back_an_armed_vehicle_only_once_they_move() {
    use FailsafeState::{AwaitingSticks as A, Fallback as F, Primary as P};
    let (mut commander, link) = armed_then_lost(sticks(0.6, 0.0));
    let fresh = Some(0.0);
    // (case, sticks sent, expected state)
    let ticks = [
        ("recovering", sticks(0.2, 0.0), F), // the joystick comes back at low throttle
        ("still recovering", sticks(0.2, 0.0), F),
        ("recovered, sticks unchanged: holds", sticks(0.2, 0.0), A),
        ("still unchanged", sticks(0.2, 0.0), A),
        ("moved less than the delta", sticks(0.25, 0.05), A),
        ("moved past it: the pilot has them", sticks(0.5, 0.0), P),
    ];
    for (i, (case, input, state)) in ticks.into_iter().enumerate() {
        let command = step(&mut commander, &link, fresh, None, Some(input));
        check!(commander.failsafe() == Some(state), "tick {i}: {case}");
        check!(command.arm, "tick {i}: {case}: stays armed");
    }
}

#[test]
fn a_dropout_while_waiting_restarts_the_stick_reference() {
    use FailsafeState::{AwaitingSticks as A, Fallback as F, Primary as P};
    let (mut commander, link) = armed_then_lost(sticks(0.6, 0.0));
    let fresh = Some(0.0);
    // (case, since, sticks sent, expected state)
    let ticks = [
        ("recovering", fresh, sticks(0.2, 0.0), F),
        ("still recovering", fresh, sticks(0.2, 0.0), F),
        (
            "recovered at 0.2: the reference",
            fresh,
            sticks(0.2, 0.0),
            A,
        ),
        ("dropout", Some(0.6), sticks(0.2, 0.0), F),
        ("recovering again", fresh, sticks(0.5, 0.0), F),
        ("still recovering again", fresh, sticks(0.5, 0.0), F),
        (
            "recovered at 0.5: the new reference",
            fresh,
            sticks(0.5, 0.0),
            A,
        ),
        ("moved from 0.5", fresh, sticks(0.7, 0.0), P),
    ];
    for (i, (case, since_s, input, state)) in ticks.into_iter().enumerate() {
        step(&mut commander, &link, since_s, None, Some(input));
        check!(commander.failsafe() == Some(state), "tick {i}: {case}");
    }
}

#[test]
fn stick_movement_counts_any_axis_and_a_change_of_kind() {
    let rate = |throttle: f32, yaw_rate: f32| {
        ControlInput::Rate(RateCommand {
            throttle,
            yaw_rate,
            ..Default::default()
        })
    };
    // (case, reference, now, moved)
    let cases = [
        ("same", sticks(0.5, 0.0), sticks(0.5, 0.0), false),
        (
            "throttle within delta",
            sticks(0.5, 0.0),
            sticks(0.55, 0.0),
            false,
        ),
        ("roll past delta", sticks(0.5, 0.0), sticks(0.5, -0.2), true),
        ("rate yaw past delta", rate(0.5, 0.0), rate(0.5, 0.3), true),
        ("attitude to rate", sticks(0.5, 0.0), rate(0.5, 0.0), true),
        (
            "NaN never moves",
            sticks(f32::NAN, 0.0),
            sticks(f32::NAN, 0.0),
            false,
        ),
    ];
    for (case, reference, now, moved) in cases {
        check!(sticks_moved(&reference, &now, 0.1) == moved, "{case}");
    }
}

fn land() -> ControlInput {
    ControlInput::Land(LandCommand::default())
}

fn sinking(vertical_speed_mps: f32) -> VehicleState {
    VehicleState {
        velocity_mps: Vec3::new(0.0, 0.0, vertical_speed_mps),
        ..Default::default()
    }
}

#[test]
fn disarms_on_touchdown_only_while_the_primary_flies_the_land() {
    use FailsafeState::{Fallback as F, Primary as P};
    // 0.35 s ticks: the detector needs 1 s stopped after the descent, so three stopped ticks;
    // one fresh tick passes the 0.3 s recovery.
    const TICK_S: f32 = 0.35;
    let (fresh, stale) = (Some(0.0), Some(0.6));
    let (mut commander, link, _) = failsafe();
    // (case, since, Arm message, input if changed, vertical speed, expected state, arm)
    let ticks = [
        (
            "hears the link: hands over",
            fresh,
            None,
            Some(hover_at(5.0)),
            0.0,
            P,
            false,
        ),
        ("arms", fresh, Some(true), None, 0.0, P, true),
        ("land", fresh, None, Some(land()), 0.0, P, true),
        ("descending", fresh, None, None, -0.7, P, true),
        (
            "the link drops: the hold flies",
            stale,
            None,
            None,
            0.0,
            F,
            true,
        ),
        ("hovering in the hold", stale, None, None, 0.0, F, true),
        ("still hovering", stale, None, None, 0.0, F, true),
        ("still hovering", stale, None, None, 0.0, F, true),
        (
            "the link is back: no disarm at the handover",
            fresh,
            None,
            None,
            0.0,
            P,
            true,
        ),
        (
            "stopped, no descent since the land resumed",
            fresh,
            None,
            None,
            0.0,
            P,
            true,
        ),
        ("stopped", fresh, None, None, 0.0, P, true),
        ("stopped", fresh, None, None, 0.0, P, true),
        ("stopped", fresh, None, None, 0.0, P, true),
        ("descending", fresh, None, None, -0.7, P, true),
        ("stopped", fresh, None, None, 0.0, P, true),
        ("stopped 0.7 s", fresh, None, None, 0.0, P, true),
        (
            "stopped 1.05 s: down, disarmed",
            fresh,
            None,
            None,
            0.0,
            P,
            false,
        ),
        ("stays disarmed", fresh, None, None, 0.0, P, false),
        ("a new arm", fresh, Some(true), None, 0.0, P, true),
        (
            "re-armed on the ground: no new descent",
            fresh,
            None,
            None,
            0.0,
            P,
            true,
        ),
        ("still armed", fresh, None, None, 0.0, P, true),
        ("still armed", fresh, None, None, 0.0, P, true),
    ];
    for (i, (case, since_s, arm, input, vertical_speed_mps, state, armed)) in
        ticks.into_iter().enumerate()
    {
        link.since_s.set(since_s);
        if let Some(arm) = arm {
            link.arm.set(arm);
        }
        if input.is_some() {
            link.input.set(input);
        }
        let command = commander.next_command(TICK_S, &sinking(vertical_speed_mps));
        check!(commander.failsafe() == Some(state), "tick {i}: {case}");
        check!(command.arm == armed, "tick {i}: {case}");
        if case.ends_with("down, disarmed") {
            check!(link.synced.get() == Some(false), "the primary synced");
        }
    }
}

/// The link drops during an armed land: the hold finishes it, and the failsafe disarms on
/// touchdown, keeping the descent it saw before the drop (#263).
#[test]
fn the_hold_finishes_a_land_the_link_dropped() {
    use FailsafeState::{Fallback as F, Primary as P};
    // As above: 0.35 s ticks, so three stopped ticks touch down and one fresh tick recovers.
    const TICK_S: f32 = 0.35;
    let (fresh, stale) = (Some(0.0), Some(0.6));
    // (case, since, Arm message, input if changed, vertical speed, expected state, arm)
    type Row = (
        &'static str,
        Option<f32>,
        Option<bool>,
        Option<ControlInput>,
        f32,
        FailsafeState,
        bool,
    );
    let armed_land: [Row; 4] = [
        (
            "hears the link",
            fresh,
            None,
            Some(hover_at(5.0)),
            0.0,
            P,
            false,
        ),
        ("arms", fresh, Some(true), None, 0.0, P, true),
        ("land", fresh, None, Some(land()), 0.0, P, true),
        ("descending", fresh, None, None, -0.7, P, true),
    ];
    let scenarios: [(&str, &[Row]); 2] = [
        (
            "dropped mid-descent",
            &[
                ("the link drops: lands on", stale, None, None, -0.7, F, true),
                ("stopped", stale, None, None, 0.0, F, true),
                ("stopped 0.7 s", stale, None, None, 0.0, F, true),
                (
                    "stopped 1.05 s: down, disarmed",
                    stale,
                    None,
                    None,
                    0.0,
                    F,
                    false,
                ),
                ("stays disarmed", stale, None, None, 0.0, F, false),
                (
                    "the link is back, disarmed",
                    fresh,
                    None,
                    None,
                    0.0,
                    P,
                    false,
                ),
            ],
        ),
        (
            "dropped on the ground before touchdown",
            &[
                ("stopped", fresh, None, None, 0.0, P, true),
                ("stopped 0.7 s", fresh, None, None, 0.0, P, true),
                (
                    "the link drops at 1.05 s: down, disarmed",
                    stale,
                    None,
                    None,
                    0.0,
                    F,
                    false,
                ),
                ("stays disarmed", stale, None, None, 0.0, F, false),
            ],
        ),
    ];
    for (scenario, rows) in scenarios {
        let link = Rc::new(LinkScript::default());
        let mut commander = FailsafeCommander::new(
            Link(link.clone()),
            HoldCommander::default(),
            FailsafeConfig {
                timeout_s: 0.5,
                recover_after_s: 0.3,
                takeover_stick_delta: 0.1,
            },
        )
        .expect("valid config");
        for (i, (case, since_s, arm, input, vertical_speed_mps, state, armed)) in
            armed_land.iter().chain(rows).enumerate()
        {
            link.since_s.set(*since_s);
            if let Some(arm) = arm {
                link.arm.set(*arm);
            }
            if input.is_some() {
                link.input.set(*input);
            }
            let command = commander.next_command(TICK_S, &sinking(*vertical_speed_mps));
            check!(
                commander.failsafe() == Some(*state),
                "{scenario}: tick {i}: {case}"
            );
            check!(command.arm == *armed, "{scenario}: tick {i}: {case}");
            if i >= 2 {
                check!(
                    command.input == land(),
                    "{scenario}: tick {i}: {case}: lands"
                );
            }
        }
        check!(commander.fallback().is_landing(), "{scenario}");
        check!(
            link.synced.get() == Some(false),
            "{scenario}: the primary synced"
        );
    }
}
