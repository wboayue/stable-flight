use core::time::Duration;
use std::cell::Cell;
use std::rc::Rc;

use assert2::{assert, check};

use super::*;
use crate::commanders::AutotuneHoldConfig;
use crate::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use crate::mixers::TableMixer;
use crate::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use crate::testing::NullVehicle;
use crate::tuning::autotune::{AutotuneAbort, AutotuneStatus, Envelope, StagePlan};
use crate::{AttitudeCommand, Command, ControlInput, VehicleState};

const DT: f32 = 0.004;

/// Flies whatever its cell holds.
struct Sticks(Rc<Cell<Command>>);

impl Commander for Sticks {
    fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
        self.0.get()
    }
}

type TestLoop = FlightLoop<
    AutotuneHold<Sticks>,
    NullVehicle,
    CopterGuidance,
    CopterStabilizer,
    TableMixer<4>,
    (),
>;

fn sticks(armed: bool, roll: f32) -> Command {
    let command = Command::armed(ControlInput::Attitude(AttitudeCommand {
        throttle: 0.5,
        roll,
        ..Default::default()
    }));
    Command {
        arm: armed,
        ..command
    }
}

/// A copter at 10 m flying the sticks in the returned cell, one tick flown, and a supervisor.
fn hovering(armed: bool) -> (TestLoop, Rc<Cell<Command>>, Autotune) {
    let command = Rc::new(Cell::new(sticks(armed, 0.0)));
    let hold = AutotuneHold::new(Sticks(Rc::clone(&command)), AutotuneHoldConfig::default())
        .expect("valid");
    let state = VehicleState {
        position_m: [0.0, 0.0, 10.0].into(),
        ..Default::default()
    };
    let mut flight_loop = FlightLoop::new(
        hold,
        NullVehicle::with_state(state),
        CopterGuidance::new(CopterGuidanceConfig::default()),
        CopterStabilizer::new(CopterStabilizerConfig::default()),
        TableMixer::quad_x(),
        (),
    );
    flight_loop.prime();
    tick(&mut flight_loop);
    let autotune =
        Autotune::new(&StagePlan::copter_design_search(), Envelope::default()).expect("valid");
    (flight_loop, command, autotune)
}

fn tick(flight_loop: &mut TestLoop) {
    flight_loop.step(Duration::from_secs_f32(DT));
}

/// An armed start engages the hold; a refused one leaves it off.
#[test]
fn a_start_engages_the_hold_unless_refused() {
    // (armed, start's result)
    for (armed, expected) in [(false, Err(StartError::NotArmed)), (true, Ok(()))] {
        let (mut flight_loop, _, mut autotune) = hovering(armed);
        check!(
            start(&mut autotune, &mut flight_loop) == expected,
            "armed {armed}"
        );
        check!(
            flight_loop.commander().is_engaged() == armed,
            "armed {armed}"
        );
    }
}

/// A stop hands the vehicle back and reverts the run; a second finds nothing to undo.
#[test]
fn a_stop_releases_the_hold_and_reverts() {
    let (mut flight_loop, _, mut autotune) = hovering(true);
    assert!(start(&mut autotune, &mut flight_loop).is_ok());

    check!(stop(&mut autotune, &mut flight_loop) == Some(AutotuneEvent::Reverted));
    check!(!flight_loop.commander().is_engaged());
    check!(autotune.status() == AutotuneStatus::Reverted);
    check!(stop(&mut autotune, &mut flight_loop).is_none());
}

/// A roll stick mid-run takes over from the hold, and the run reverts.
#[test]
fn a_take_over_mid_run_reverts() {
    let (mut flight_loop, command, mut autotune) = hovering(true);
    assert!(start(&mut autotune, &mut flight_loop).is_ok());
    check!(!step(&mut autotune, &mut flight_loop, DT).reverted);

    command.set(sticks(true, 0.5));
    tick(&mut flight_loop);
    let step = step(&mut autotune, &mut flight_loop, DT);
    check!(step.reverted);
    check!(step.event.is_none());
    check!(autotune.status() == AutotuneStatus::Reverted);
    check!(!flight_loop.commander().is_engaged());
}

/// A run that ends keeps holding the vehicle; a take-over after it hands back without
/// reverting, so the gains it ended on stay.
#[test]
fn an_ended_run_keeps_the_hold_and_its_gains() {
    let (mut flight_loop, command, mut autotune) = hovering(true);
    assert!(start(&mut autotune, &mut flight_loop).is_ok());
    step(&mut autotune, &mut flight_loop, DT);
    assert!(
        autotune
            .abort(&mut flight_loop, AutotuneAbort::HostFault)
            .is_some()
    );
    let failed = autotune.status();
    assert!(matches!(failed, AutotuneStatus::Failed { .. }));

    for _ in 0..10 {
        tick(&mut flight_loop);
        step(&mut autotune, &mut flight_loop, DT);
    }
    check!(flight_loop.commander().is_engaged());

    command.set(sticks(true, 0.5));
    tick(&mut flight_loop);
    let step = step(&mut autotune, &mut flight_loop, DT);
    check!(!step.reverted);
    check!(autotune.status() == failed);
    check!(!flight_loop.commander().is_engaged());
}
