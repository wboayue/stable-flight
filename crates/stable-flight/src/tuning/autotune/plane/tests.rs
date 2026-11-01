use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::Duration;

use assert2::{assert, check};

use super::*;
use crate::controllers::{PlaneRateControllerConfig, RateGains};
use crate::stabilizers::plane::PlaneStabilizer;
use crate::tuning::Tunable;
use crate::tuning::autotune::{AutotuneEvent, AutotuneStatus, Outcome};
use crate::tuning::plant::{CONTROL_RADPS2, DAMPING_PER_S, Wing};
use crate::{
    AbsoluteAttitudeCommand, Actuator, ActuatorCommand, AttitudeSetpoint, Command, Commander,
    Configurable, ControlInput, ControlOutput, FlightLoop, FlightRegime, Guidance, Mixer,
    PlaneSetpoint, StateSource, VehicleState,
};

/// The flight loop's tick: 250 Hz.
const DT: f32 = 0.004;
/// Where the wing flies, in m.
const ALTITUDE_M: f32 = 100.0;

/// Knobs the tests turn while the loop flies.
#[derive(Clone)]
struct Controls {
    wing: Rc<RefCell<Wing>>,
    airspeed_mps: Rc<Cell<Option<f32>>>,
    altitude_m: Rc<Cell<f32>>,
    /// Guidance flies a tailsitter transition ramp.
    transition: Rc<Cell<bool>>,
}

impl Default for Controls {
    fn default() -> Self {
        Self {
            wing: Rc::default(),
            airspeed_mps: Rc::new(Cell::new(Some(15.0))),
            altitude_m: Rc::new(Cell::new(ALTITUDE_M)),
            transition: Rc::default(),
        }
    }
}

/// Armed, wings level.
struct Hold;

impl Commander for Hold {
    fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
        Command::new(
            true,
            ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
                thrust: 0.5,
                ..Default::default()
            }),
        )
    }
}

/// A level plane attitude at half thrust, or a transition ramp (`Controls::transition`).
struct LevelGuidance(Controls);

impl Guidance for LevelGuidance {
    fn guide(
        &mut self,
        _dt: f32,
        _regime: FlightRegime,
        _input: &ControlInput,
        _state: &VehicleState,
    ) -> Setpoint {
        Setpoint::Plane(PlaneSetpoint {
            thrust: 0.5,
            transition: self.0.transition.get(),
            ..Default::default()
        })
    }
}

/// The [`Wing`] as a vehicle: moments in through `servos[0..3]`.
struct WingVehicle(Controls);

impl StateSource for WingVehicle {
    fn state(&mut self, _dt: Duration) -> VehicleState {
        let mut state = self.0.wing.borrow().state();
        state.airspeed_mps = self.0.airspeed_mps.get();
        state.position_m.z = self.0.altitude_m.get();
        state
    }
}

impl Actuator for WingVehicle {
    fn actuate(&mut self, command: &ActuatorCommand) {
        let servos = command.servos;
        let moments = Vec3::new(servos[0], servos[1], servos[2]);
        self.0.wing.borrow_mut().step(DT, moments);
    }
}

/// Passes the stabilizer's moments straight to the wing.
struct MomentMixer;

impl Mixer for MomentMixer {
    fn mix(&mut self, controls: &ControlOutput) -> ActuatorCommand {
        let mut command = ActuatorCommand::default();
        command.servos[..3].copy_from_slice(controls.moments.as_slice());
        command
    }
}

type WingLoop = FlightLoop<Hold, WingVehicle, LevelGuidance, PlaneStabilizer, MomentMixer, ()>;

/// Gains that fly the [`Wing`]: feedforward matching its damping, a little P and I.
fn config() -> PlaneStabilizerConfig {
    let rate = RateGains {
        p: 0.1,
        i: 0.1,
        ff: DAMPING_PER_S / CONTROL_RADPS2,
    };
    PlaneStabilizerConfig {
        rate: PlaneRateControllerConfig {
            roll: rate,
            pitch: rate,
            ..Default::default()
        },
        ..Default::default()
    }
}

/// An armed loop flying the wing level for a second.
fn flying() -> (WingLoop, Controls) {
    let controls = Controls::default();
    let mut flight_loop = FlightLoop::new(
        Hold,
        WingVehicle(controls.clone()),
        LevelGuidance(controls.clone()),
        PlaneStabilizer::new(config()),
        MomentMixer,
        (),
    );
    flight_loop.prime();
    for _ in 0..250 {
        flight_loop.step(Duration::from_secs_f32(DT));
    }
    (flight_loop, controls)
}

fn started(flight_loop: &WingLoop, plan: &[StagePlan<PlaneSide>]) -> PlaneAutotune {
    let mut autotune =
        PlaneAutotune::new(plan, PlaneEnvelope::for_plane(&config())).expect("valid plan");
    autotune.start(flight_loop).expect("armed and idle");
    autotune
}

type Event = AutotuneEvent<PlaneLoop, PlaneAxisGains>;

/// Steps the loop and the supervisor until the run ends or `max_s` passes, calling `poke`
/// before each tick; returns the events in order.
fn run(
    flight_loop: &mut WingLoop,
    autotune: &mut PlaneAutotune,
    max_s: f32,
    mut poke: impl FnMut(&PlaneAutotune),
) -> Vec<Event> {
    let mut events = Vec::new();
    for _ in 0..(max_s / DT) as usize {
        poke(autotune);
        flight_loop.step(Duration::from_secs_f32(DT));
        if let Some(event) = autotune.update(flight_loop, DT) {
            events.push(event);
        }
        if !autotune.is_running() {
            break;
        }
    }
    events
}

const LOOPS: [PlaneLoop; 4] = [
    PlaneLoop::RollRate,
    PlaneLoop::PitchRate,
    PlaneLoop::Roll,
    PlaneLoop::Pitch,
];

#[test]
fn a_step_test_measures_each_loop_and_keeps_its_gains() {
    let (mut flight_loop, _) = flying();
    let plan = LOOPS.map(|target| StagePlan::plane_step_test(target, &config()));
    let mut autotune = started(&flight_loop, &plan);
    let events = run(&mut flight_loop, &mut autotune, 200.0, |_| {});

    let done: Vec<_> = events
        .iter()
        .filter_map(|event| match event {
            AutotuneEvent::StageDone {
                target,
                gains,
                outcome,
                steps,
                ..
            } => Some((*target, *gains, *outcome, steps.map(|s| s.count()))),
            _ => None,
        })
        .collect();
    assert!(done.len() == LOOPS.len(), "{events:?}");
    for ((target, gains, outcome, steps), expected) in done.into_iter().zip(LOOPS) {
        check!(target == expected);
        check!(gains == expected.gains(&config()), "{target:?}");
        check!(outcome == Outcome::Measured, "{target:?}");
        check!(steps == Some(6), "{target:?}");
    }
    check!(events.last() == Some(&AutotuneEvent::Complete));
    check!(*flight_loop.stabilizer().config() == config());
}

/// Each limit fails the stage it breaks: the wing banked or pitched past the envelope, its
/// airspeed out of the band, or sunk past the altitude loss. A missing airspeed breaks
/// nothing.
#[test]
fn breaches_fail_the_stage() {
    type Poke = fn(&Controls);
    let rows: [(&str, Poke, Option<AutotuneAbort>); 6] = [
        (
            "bank",
            |c| c.wing.borrow_mut().angles_rad.x = 1.2,
            Some(AutotuneAbort::Tilt),
        ),
        (
            "pitch",
            |c| c.wing.borrow_mut().angles_rad.y = -0.6,
            Some(AutotuneAbort::Tilt),
        ),
        (
            "slow",
            |c| c.airspeed_mps.set(Some(8.0)),
            Some(AutotuneAbort::AirspeedLow),
        ),
        (
            "fast",
            |c| c.airspeed_mps.set(Some(30.0)),
            Some(AutotuneAbort::AirspeedHigh),
        ),
        (
            "sink",
            |c| c.altitude_m.set(ALTITUDE_M - 50.0),
            Some(AutotuneAbort::AltitudeLoss),
        ),
        ("no airspeed", |c| c.airspeed_mps.set(None), None),
    ];
    for (case, poke, reason) in rows {
        let (mut flight_loop, controls) = flying();
        let mut autotune = started(
            &flight_loop,
            &[StagePlan::plane_step_test(PlaneLoop::Roll, &config())],
        );
        poke(&controls);
        let events = run(&mut flight_loop, &mut autotune, 1.0, |_| {});
        let failed = events.iter().find_map(|event| match event {
            AutotuneEvent::StageFailed { reason, .. } => Some(*reason),
            _ => None,
        });
        check!(failed == reason, "{case}: {events:?}");
        check!(*flight_loop.stabilizer().config() == config(), "{case}");
    }
}

#[test]
fn a_transition_mid_stage_stops_it() {
    let (mut flight_loop, controls) = flying();
    let mut autotune = started(
        &flight_loop,
        &[StagePlan::plane_step_test(PlaneLoop::Roll, &config())],
    );
    let events = run(&mut flight_loop, &mut autotune, 30.0, |autotune| {
        if matches!(autotune.status(), AutotuneStatus::Running { .. }) {
            controls.transition.set(true);
        }
    });
    check!(
        events.last()
            == Some(&AutotuneEvent::StageFailed {
                target: PlaneLoop::Roll,
                reason: AutotuneAbort::Interrupted,
            }),
        "{events:?}"
    );
}

#[test]
fn the_settle_reads_the_held_attitude() {
    let target = PlaneSide::attitude_target(&Setpoint::Plane(PlaneSetpoint {
        roll_rad: 0.3,
        pitch_rad: f32::NAN,
        ..Default::default()
    }));
    check!(target == Vec2::new(0.3, 0.0));
    let hover = Setpoint::Attitude(AttitudeSetpoint {
        roll_rad: 0.3,
        ..Default::default()
    });
    check!(PlaneSide::attitude_target(&hover) == Vec2::zeros());
}

#[test]
fn rejects_an_invalid_envelope() {
    let good = PlaneEnvelope::for_plane(&config());
    let rows: [(&str, PlaneEnvelope); 4] = [
        (
            "max_airspeed_mps",
            PlaneEnvelope {
                max_airspeed_mps: good.min_airspeed_mps,
                ..good
            },
        ),
        (
            "max_bank_rad",
            PlaneEnvelope {
                max_bank_rad: 0.0,
                ..good
            },
        ),
        (
            "settle.tilt_rad",
            PlaneEnvelope {
                max_pitch_rad: good.settle.tilt_rad,
                ..good
            },
        ),
        (
            "recovery_s",
            PlaneEnvelope {
                recovery_s: -1.0,
                ..good
            },
        ),
    ];
    check!(good.validate().is_ok());
    for (field, envelope) in rows {
        let error = envelope.validate().map_err(|e| e.field);
        check!(error == Err(field));
    }
}

#[test]
fn vtol_configs_hold_their_plane_side() {
    let mut quadplane = QuadPlaneStabilizerConfig::default();
    SideConfig::<PlaneSide>::side_mut(&mut quadplane).trim_airspeed_mps = 21.0;
    check!(quadplane.plane.trim_airspeed_mps == 21.0);
    let mut tailsitter = TailsitterStabilizerConfig::default();
    SideConfig::<PlaneSide>::side_mut(&mut tailsitter).trim_airspeed_mps = 19.0;
    check!(SideConfig::<PlaneSide>::side(&tailsitter).trim_airspeed_mps == 19.0);
}

/// While the commander maneuvers no probe starts and the settle doesn't time out; once it
/// stops, the stage runs.
#[test]
fn maneuvering_holds_probes_off() {
    let (mut flight_loop, _) = flying();
    let mut autotune = started(
        &flight_loop,
        &[StagePlan::plane_step_test(PlaneLoop::Roll, &config())],
    );
    autotune.set_maneuvering(true);
    let timeout_s = PlaneEnvelope::for_plane(&config()).settle_timeout_s;
    let events = run(&mut flight_loop, &mut autotune, timeout_s + 5.0, |_| {});
    check!(
        matches!(autotune.status(), AutotuneStatus::Settling { .. }),
        "{events:?}"
    );
    check!(Tunable::<PlaneLoop>::probe(flight_loop.stabilizer()).is_none());

    autotune.set_maneuvering(false);
    let events = run(&mut flight_loop, &mut autotune, 60.0, |_| {});
    check!(
        events.last() == Some(&AutotuneEvent::Complete),
        "{events:?}"
    );
}
