use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::Duration;

use assert2::{assert, check};

use super::*;
use crate::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use crate::stabilizers::copter::CopterStabilizer;
use crate::tuning::plant::{Axes, PlantParams};
use crate::tuning::{StepAbort, StepMetrics};
use crate::{
    AbsoluteAttitudeCommand, Actuator, ActuatorCommand, Command, Commander, ControlInput,
    ControlOutput, Mixer, RateCommand, StateSource, Vec3,
};

/// The flight loop's tick: 250 Hz.
const DT: f32 = 0.004;
/// Where the sim hovers, in m.
const ALTITUDE_M: f32 = 10.0;

/// Knobs the tests turn while the loop flies.
#[derive(Clone, Default)]
struct Controls {
    arm: Rc<Cell<bool>>,
    input: Rc<Cell<Option<ControlInput>>>,
    axes: Rc<RefCell<Option<Axes>>>,
    altitude_m: Rc<Cell<f32>>,
    /// Added to the measured roll rate, in rad/s.
    rate_spike_radps: Rc<Cell<f32>>,
    /// The host's link to the vehicle is down: the state holds at the last one seen, and the
    /// sim flies on unseen under [`OUTAGE_MOMENT`] instead of the commands.
    outage: Rc<Cell<bool>>,
    /// The last state the vehicle reported.
    seen: Rc<RefCell<Option<VehicleState>>>,
}

/// The roll moment the sim flies on during an outage, as a host link's last commands might
/// leave it.
const OUTAGE_MOMENT: f32 = 0.5;

/// Arms (per `Controls::arm`) and holds level at heading 0, or flies `Controls::input`.
struct TestCommander(Controls);

impl Commander for TestCommander {
    fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
        let input =
            self.0
                .input
                .get()
                .unwrap_or(ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
                    thrust: 0.5,
                    ..Default::default()
                }));
        Command::new(self.0.arm.get(), input)
    }
}

/// The `Axes` sim as a vehicle: moments in through the servos (see [`MomentMixer`]).
struct SimVehicle(Controls);

impl StateSource for SimVehicle {
    fn state(&mut self, _dt: Duration) -> VehicleState {
        if self.0.outage.get()
            && let Some(seen) = self.0.seen.borrow().clone()
        {
            return seen;
        }
        let mut axes = self.0.axes.borrow_mut();
        let Some(axes) = axes.as_mut() else {
            return VehicleState::default();
        };
        let mut state = axes.state();
        state.position_m.z = self.0.altitude_m.get();
        state.angular_velocity_radps.x += self.0.rate_spike_radps.get();
        *self.0.seen.borrow_mut() = Some(state.clone());
        state
    }
}

impl Actuator for SimVehicle {
    fn actuate(&mut self, command: &ActuatorCommand) {
        if let Some(axes) = self.0.axes.borrow_mut().as_mut() {
            let servos = command.servos;
            let moments = if self.0.outage.get() {
                Vec3::new(OUTAGE_MOMENT, 0.0, 0.0)
            } else {
                Vec3::new(servos[0], servos[1], servos[2])
            };
            axes.step(moments);
        }
    }
}

/// Passes the stabilizer's moments straight to the sim, in `servos[0..3]`.
struct MomentMixer;

impl Mixer for MomentMixer {
    fn mix(&mut self, controls: &ControlOutput) -> ActuatorCommand {
        let mut command = ActuatorCommand::default();
        command.servos[..3].copy_from_slice(controls.moments.as_slice());
        command.motors[0] = controls.thrust;
        command
    }
}

/// Disturbs the sim through its controls.
type Disturbance = fn(&Controls);

type TestLoop =
    FlightLoop<TestCommander, SimVehicle, CopterGuidance, CopterStabilizer, MomentMixer, ()>;

/// Hand-tune-like gains that fly the sim, with the rate loops' P scaled by `rate_p_scale`.
fn stabilizer_config(rate_p_scale: f32) -> CopterStabilizerConfig {
    let mut config = CopterStabilizerConfig::default();
    let rate = Gains {
        p: 0.5 * rate_p_scale,
        i: 0.2,
        d: 0.004,
    };
    config.body_rate.roll_gains = rate;
    config.body_rate.pitch_gains = rate;
    config.body_rate.yaw_gains = Gains {
        p: 0.6 * rate_p_scale,
        i: 0.6,
        d: 0.0,
    };
    let attitude = Gains {
        p: 3.0,
        i: 0.0,
        d: 0.1,
    };
    config.attitude.roll_gains = attitude;
    config.attitude.pitch_gains = attitude;
    config.yaw.gains = Gains {
        p: 2.5,
        i: 0.0,
        d: 0.06,
    };
    config
}

/// An armed loop hovering level over the sim for a second.
fn hovering(rate_p_scale: f32) -> (TestLoop, Controls) {
    hovering_on(rate_p_scale, PlantParams::rate_axis())
}

/// [`hovering`] over axes with `params`.
fn hovering_on(rate_p_scale: f32, params: PlantParams) -> (TestLoop, Controls) {
    let controls = Controls::default();
    controls.arm.set(true);
    controls.altitude_m.set(ALTITUDE_M);
    *controls.axes.borrow_mut() = Some(Axes::new([params; 3], Vec3::zeros(), DT));
    let mut flight_loop = FlightLoop::new(
        TestCommander(controls.clone()),
        SimVehicle(controls.clone()),
        CopterGuidance::new(CopterGuidanceConfig::default()),
        CopterStabilizer::new(stabilizer_config(rate_p_scale)),
        MomentMixer,
        (),
    );
    flight_loop.prime();
    for _ in 0..250 {
        flight_loop.step(Duration::from_secs_f32(DT));
    }
    (flight_loop, controls)
}

/// Steps the loop and the supervisor until the run ends or `max_s` passes, calling `poke`
/// before each tick; returns the events in order.
fn run(
    flight_loop: &mut TestLoop,
    autotune: &mut Autotune,
    max_s: f32,
    mut poke: impl FnMut(&Autotune, &TestLoop),
) -> Vec<AutotuneEvent> {
    let mut events = Vec::new();
    for _ in 0..(max_s / DT) as usize {
        poke(autotune, flight_loop);
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

/// A design stage on `target` with the copter defaults.
fn design_stage(target: TunedLoop) -> StagePlan {
    StagePlan {
        target,
        method: Method::Design(DesignPlan::copter()),
    }
}

/// A search stage on `target` with the copter defaults (`SearchPlan::copter`).
fn search_stage(target: TunedLoop) -> StagePlan {
    StagePlan {
        target,
        method: Method::Search(SearchPlan::copter(target)),
    }
}

fn started(flight_loop: &TestLoop, plan: &[StagePlan]) -> Autotune {
    let mut autotune = Autotune::new(plan, Envelope::default()).expect("valid plan");
    autotune.start(flight_loop).expect("armed and idle");
    autotune
}

/// Roll and pitch rate, then angle: the loops a design tunes.
const DESIGNED: [TunedLoop; 4] = [
    TunedLoop::RollRate,
    TunedLoop::PitchRate,
    TunedLoop::Roll,
    TunedLoop::Pitch,
];

/// Every designed loop, rate loops first, tunes in order and commits each stage's gains from
/// its own fit; nothing else in the config changes.
#[test]
fn full_run_tunes_every_loop_in_order() {
    let (mut flight_loop, _controls) = hovering(1.0);
    let baseline = *flight_loop.stabilizer().config();
    let mut autotune = started(&flight_loop, &DESIGNED.map(design_stage));

    let events = run(&mut flight_loop, &mut autotune, 300.0, |_, _| {});

    check!(autotune.status() == AutotuneStatus::Complete);
    let started: Vec<_> = events
        .iter()
        .filter_map(|event| match *event {
            AutotuneEvent::StageStarted(target) => Some(target),
            _ => None,
        })
        .collect();
    check!(started == DESIGNED);
    let done: Vec<_> = events
        .iter()
        .filter_map(|event| match *event {
            AutotuneEvent::StageDone {
                target,
                gains,
                outcome,
                ..
            } => Some((target, gains, outcome)),
            _ => None,
        })
        .collect();
    let mut expected = baseline;
    for (index, (target, gains, outcome)) in done.iter().copied().enumerate() {
        check!(target == DESIGNED[index]);
        check!(outcome == Outcome::Designed, "{target:?}");
        check!(gains != target.gains(&baseline), "{target:?}");
        *target.gains_mut(&mut expected) = gains;
    }
    check!(done.len() == DESIGNED.len());
    check!(events.last() == Some(&AutotuneEvent::Complete));
    check!(*flight_loop.stabilizer().config() == expected);
}

/// Breaking the envelope mid-stage stops it and fails the run: the config goes back to the
/// last commit, so the failing loop keeps its gains and the earlier stage's stay committed.
#[test]
fn an_envelope_breach_ends_the_run_keeping_earlier_stages() {
    let plan = [
        search_stage(TunedLoop::RollRate),
        StagePlan::copter_step_test(TunedLoop::PitchRate),
    ];
    // (reason, how to cause it)
    let cases: [(AutotuneAbort, Disturbance); 3] = [
        (AutotuneAbort::Tilt, |c| {
            if let Some(axes) = c.axes.borrow_mut().as_mut() {
                axes.tilt(1, 0.8);
            }
        }),
        (AutotuneAbort::Rate, |c| c.rate_spike_radps.set(5.0)),
        (AutotuneAbort::AltitudeLoss, |c| {
            c.altitude_m.set(ALTITUDE_M - 4.0)
        }),
    ];
    for (reason, cause) in cases {
        let (mut flight_loop, controls) = hovering_on(1.0, delayed_axis());
        let baseline = *flight_loop.stabilizer().config();
        let mut autotune = started(&flight_loop, &plan);

        let events = run(&mut flight_loop, &mut autotune, 300.0, |autotune, _| {
            if matches!(autotune.status(), AutotuneStatus::Running { stage: 1, .. }) {
                cause(&controls);
            }
        });

        check!(
            events.last()
                == Some(&AutotuneEvent::StageFailed {
                    target: TunedLoop::PitchRate,
                    reason
                }),
            "{reason:?}"
        );
        check!(autotune.status() == AutotuneStatus::Failed { stage: 1, reason });
        let config = flight_loop.stabilizer().config();
        check!(
            config.body_rate.roll_gains != baseline.body_rate.roll_gains,
            "{reason:?}"
        );
        check!(
            config.body_rate.pitch_gains == baseline.body_rate.pitch_gains,
            "{reason:?}"
        );
        check!(flight_loop.probe() == None, "{reason:?}");
    }
}

/// Disarming before completion reverts every stage to the baseline; after completion, the
/// tuned gains stay.
#[test]
fn disarm_reverts_until_complete() {
    let plan = [
        design_stage(TunedLoop::RollRate),
        design_stage(TunedLoop::PitchRate),
    ];
    // Mid-run: disarm once the second stage runs.
    let (mut flight_loop, controls) = hovering(1.0);
    let baseline = *flight_loop.stabilizer().config();
    let mut autotune = started(&flight_loop, &plan);
    let events = run(&mut flight_loop, &mut autotune, 60.0, |autotune, _| {
        if matches!(autotune.status(), AutotuneStatus::Running { stage: 1, .. }) {
            controls.arm.set(false);
        }
    });
    check!(events.last() == Some(&AutotuneEvent::Reverted));
    check!(autotune.status() == AutotuneStatus::Reverted);
    check!(*flight_loop.stabilizer().config() == baseline);

    // After completion: disarming changes nothing.
    let (mut flight_loop, controls) = hovering(1.0);
    let mut autotune = started(&flight_loop, &plan);
    run(&mut flight_loop, &mut autotune, 60.0, |_, _| {});
    let tuned = *flight_loop.stabilizer().config();
    controls.arm.set(false);
    flight_loop.step(Duration::from_secs_f32(DT));
    check!(autotune.update(&mut flight_loop, DT) == None);
    check!(*flight_loop.stabilizer().config() == tuned);
}

/// Holds the vehicle at `roll_rad` of roll through the commander.
fn hold_roll(controls: &Controls, roll_rad: f32) {
    controls.input.set(Some(ControlInput::AbsoluteAttitude(
        AbsoluteAttitudeCommand {
            roll_rad,
            thrust: 0.5,
            ..Default::default()
        },
    )));
}

/// A vehicle held past the settle limits fails a step stage before its first probe, and its
/// config stays.
#[test]
fn a_vehicle_that_never_settles_fails_before_the_first_step() {
    let (mut flight_loop, controls) = hovering(1.0);
    let baseline = *flight_loop.stabilizer().config();
    // 0.12 rad: past the step settle's 0.044 rad, inside max_tilt_rad.
    hold_roll(&controls, 0.12);
    let mut autotune = started(
        &flight_loop,
        &[StagePlan::copter_step_test(TunedLoop::RollRate)],
    );

    let events = run(&mut flight_loop, &mut autotune, 30.0, |_, _| {});

    let failed = AutotuneEvent::StageFailed {
        target: TunedLoop::RollRate,
        reason: AutotuneAbort::NotSettled,
    };
    check!(events == [AutotuneEvent::StageStarted(TunedLoop::RollRate), failed]);
    check!(*flight_loop.stabilizer().config() == baseline);
}

/// A probe abort or an interrupt fails the stage; its loop keeps its gains.
#[test]
fn probe_aborts_and_interrupts_fail_the_stage() {
    // Running ticks before the disturbance (a 1 rad/s step has about risen), and how long it
    // lasts: long enough to abort, short enough not to tilt the vehicle past the envelope.
    const RUNNING_TICKS: usize = 40;
    const DISTURBANCE_TICKS: usize = 10;
    // (plan, disturbance once running, expected reason)
    let cases: [(StagePlan, Disturbance, AutotuneAbort); 2] = [
        // Past three times the 1 rad/s step, inside the envelope's 4 rad/s.
        (
            StagePlan::copter_step_test(TunedLoop::RollRate),
            |c| c.rate_spike_radps.set(2.6),
            AutotuneAbort::Probe(ProbeAbort::Step(StepAbort::Runaway)),
        ),
        (
            StagePlan::copter_step_test(TunedLoop::Roll),
            |c| {
                c.input.set(Some(ControlInput::Rate(RateCommand {
                    throttle: 0.5,
                    ..Default::default()
                })))
            },
            AutotuneAbort::Interrupted,
        ),
    ];
    for (plan, disturb, reason) in cases {
        let (mut flight_loop, controls) = hovering(1.0);
        let baseline = *flight_loop.stabilizer().config();
        let mut autotune = started(&flight_loop, &[plan]);

        let mut running = 0;
        let events = run(&mut flight_loop, &mut autotune, 30.0, |autotune, _| {
            if matches!(autotune.status(), AutotuneStatus::Running { .. }) {
                running += 1;
                if (RUNNING_TICKS..RUNNING_TICKS + DISTURBANCE_TICKS).contains(&running) {
                    disturb(&controls);
                } else {
                    controls.rate_spike_radps.set(0.0);
                }
            }
        });

        let failed = AutotuneEvent::StageFailed {
            target: plan.target,
            reason,
        };
        check!(events.last() == Some(&failed), "{reason:?}");
        check!(*flight_loop.stabilizer().config() == baseline, "{reason:?}");
    }
}

/// `new` rejects a bad plan or envelope; `start` needs an armed loop and no run in progress.
#[test]
fn new_and_start_reject_bad_input() {
    let stage = design_stage(TunedLoop::RollRate);
    let bad_envelope = Envelope {
        settle: Settle {
            tilt_rad: 1.0, // looser than max_tilt_rad
            ..Envelope::default().settle
        },
        ..Envelope::default()
    };
    let step_test = |plan: StepTestPlan| StagePlan {
        target: TunedLoop::RollRate,
        method: Method::StepTest(plan),
    };
    let copter_step = StepTestPlan::copter(TunedLoop::RollRate);
    let bad_step = step_test(StepTestPlan {
        step: StepConfig {
            duration_s: 0.0,
            ..COPTER_RATE_STEP
        },
        ..copter_step
    });
    let no_pairs = step_test(StepTestPlan {
        pairs: 0,
        ..copter_step
    });
    let loose_settle = step_test(StepTestPlan {
        settle: Settle {
            hold_s: 20.0, // longer than settle_timeout_s
            ..COPTER_STEP_SETTLE
        },
        ..copter_step
    });
    let no_recovery = Envelope {
        recovery_s: -1.0,
        ..Envelope::default()
    };
    // (plan, envelope, section, field)
    let cases: [(&[StagePlan], Envelope, &str, &str); 7] = [
        (&[stage], no_recovery, "envelope", "recovery_s"),
        (&[], Envelope::default(), "plan", "stages"),
        (
            &[stage; MAX_STAGES + 1],
            Envelope::default(),
            "plan",
            "stages",
        ),
        (&[stage], bad_envelope, "envelope", "settle.tilt_rad"),
        (
            &[bad_step],
            Envelope::default(),
            "plan.step_test",
            "duration_s",
        ),
        (&[no_pairs], Envelope::default(), "plan.step_test", "pairs"),
        (
            &[loose_settle],
            Envelope::default(),
            "plan",
            "settle.hold_s",
        ),
    ];
    for (plan, envelope, section, field) in cases {
        let error = Autotune::new(plan, envelope).expect_err(field);
        check!(error.section == Some(section));
        check!(error.field == field);
    }

    let (mut flight_loop, controls) = hovering(1.0);
    let mut autotune = Autotune::new(&[stage], Envelope::default()).expect("valid");
    controls.arm.set(false);
    flight_loop.step(Duration::from_secs_f32(DT));
    check!(autotune.start(&flight_loop) == Err(StartError::NotArmed));
    controls.arm.set(true);
    flight_loop.step(Duration::from_secs_f32(DT));
    check!(autotune.start(&flight_loop) == Ok(()));
    check!(autotune.start(&flight_loop) == Err(StartError::Running));
    check!(StartError::NotArmed.to_string() == "not armed");
    check!(StartError::Running.to_string() == "already running");
}

/// An idle supervisor does nothing.
#[test]
fn update_before_start_does_nothing() {
    let (mut flight_loop, _controls) = hovering(1.0);
    let baseline = *flight_loop.stabilizer().config();
    let mut autotune =
        Autotune::new(&DESIGNED.map(design_stage), Envelope::default()).expect("valid");

    check!(autotune.update(&mut flight_loop, DT) == None);
    assert!(autotune.status() == AutotuneStatus::Idle);
    check!(*flight_loop.stabilizer().config() == baseline);
}

/// The `Measured` step metrics of `events`, in order.
fn step_metrics(events: &[AutotuneEvent]) -> Vec<StepMetrics> {
    events
        .iter()
        .filter_map(|event| match event {
            AutotuneEvent::Measured {
                measurement: Measurement::Step(metrics),
                ..
            } => Some(*metrics),
            _ => None,
        })
        .collect()
}

/// A step-test stage measures alternating steps on its loop and keeps the gains: each stage
/// reports `2·pairs` steps, then `StageDone` with the loop's gains unchanged.
#[test]
fn step_tests_measure_alternating_steps_and_keep_the_gains() {
    let (mut flight_loop, _controls) = hovering(1.0);
    let baseline = *flight_loop.stabilizer().config();
    let targets = [TunedLoop::RollRate, TunedLoop::Pitch];
    let mut autotune = started(&flight_loop, &targets.map(StagePlan::copter_step_test));

    let events = run(&mut flight_loop, &mut autotune, 60.0, |_, _| {});

    check!(autotune.status() == AutotuneStatus::Complete);
    check!(*flight_loop.stabilizer().config() == baseline);
    let pairs = StepTestPlan::copter(TunedLoop::RollRate).pairs;
    let mut stages = events.split(|event| matches!(event, AutotuneEvent::StageStarted(_)));
    stages.next(); // before the first stage: nothing
    for (target, stage) in targets.into_iter().zip(stages) {
        let metrics = step_metrics(stage);
        check!(metrics.len() == 2 * pairs, "{target:?}");
        for (index, step) in metrics.iter().enumerate() {
            let up = index % 2 == 0;
            check!((step.amplitude > 0.0) == up, "{target:?} step {index}");
            check!(
                (step.final_value - 1.0).abs() < 0.15,
                "{target:?} step {index}: {step:?}"
            );
        }
        let done = stage.iter().find_map(|event| match event {
            AutotuneEvent::StageDone {
                target: done,
                gains,
                outcome: Outcome::Measured,
                steps,
                ..
            } if *done == target => Some((*gains, *steps)),
            _ => None,
        });
        let Some((gains, Some(steps))) = done else {
            panic!("{target:?}: no StageDone with steps: {done:?}");
        };
        check!(gains == target.gains(&baseline), "{target:?}");
        check!(steps.count() == 2 * pairs, "{target:?}");
    }
}

/// A stall noted while a step runs makes that step run again; noted between probes, it's
/// ignored.
#[test]
fn a_stalled_step_runs_again() {
    let (mut flight_loop, _controls) = hovering(1.0);
    let plan = [StagePlan::copter_step_test(TunedLoop::RollRate)];
    let pairs = StepTestPlan::copter(TunedLoop::RollRate).pairs;
    let mut autotune = started(&flight_loop, &plan);
    autotune.note_stall(); // settling: no probe yet

    let mut noted = false;
    let mut events = Vec::new();
    for _ in 0..(60.0 / DT) as usize {
        flight_loop.step(Duration::from_secs_f32(DT));
        if !noted && matches!(autotune.status(), AutotuneStatus::Running { .. }) {
            autotune.note_stall();
            noted = true;
        }
        if let Some(event) = autotune.update(&mut flight_loop, DT) {
            events.push(event);
        }
        if !autotune.is_running() {
            break;
        }
    }

    check!(autotune.status() == AutotuneStatus::Complete);
    let metrics = step_metrics(&events);
    check!(metrics.len() == 2 * pairs);
    // The stalled first step, reported as stalled with its untimely metrics, and its rerun
    // both go up.
    let stalled: Vec<_> = events
        .iter()
        .filter_map(|event| match event {
            AutotuneEvent::Stalled {
                measurement: Some(Measurement::Step(metrics)),
                ..
            } => Some(*metrics),
            _ => None,
        })
        .collect();
    check!(stalled.len() == 1);
    check!(stalled[0].amplitude > 0.0 && metrics[0].amplitude > 0.0);
}

/// A step test under a host that turns slow for `slow_s` from the first step's start:
/// the events, and whether the run completed.
fn step_test_with_slow_host(slow_s: f32) -> (Vec<AutotuneEvent>, AutotuneStatus) {
    let (mut flight_loop, _controls) = hovering(1.0);
    let plan = [StagePlan::copter_step_test(TunedLoop::RollRate)];
    let mut autotune = started(&flight_loop, &plan);
    let mut slow_from_s = None;
    let mut events = Vec::new();
    for tick in 0..(120.0 / DT) as usize {
        let time_s = tick as f32 * DT;
        flight_loop.step(Duration::from_secs_f32(DT));
        if slow_from_s.is_none() && matches!(autotune.status(), AutotuneStatus::Running { .. }) {
            slow_from_s = Some(time_s);
        }
        let slow = slow_from_s.is_some_and(|from_s| time_s - from_s < slow_s);
        autotune.set_host_slow(slow);
        if let Some(event) = autotune.update(&mut flight_loop, DT) {
            events.push(event);
        }
        if !autotune.is_running() {
            break;
        }
    }
    (events, autotune.status())
}

/// A host that turns slow mid-step stops the step at once, unmeasured, and the step test
/// waits out the slow stretch, longer than the settle timeout, without failing: it then
/// measures every step, the stopped one again (#438).
#[test]
fn a_slow_host_stops_the_step_and_settling_waits() {
    // Longer than the settle timeout, which would fail the stage if settling counted it.
    let slow_s = Envelope::default().settle_timeout_s + 5.0;
    let (events, status) = step_test_with_slow_host(slow_s);

    check!(status == AutotuneStatus::Complete);
    let stopped = events
        .iter()
        .filter(|event| {
            matches!(
                event,
                AutotuneEvent::Stalled {
                    measurement: None,
                    ..
                }
            )
        })
        .count();
    check!(stopped == 1);
    let pairs = StepTestPlan::copter(TunedLoop::RollRate).pairs;
    let metrics = step_metrics(&events);
    check!(metrics.len() == 2 * pairs);
    check!(
        metrics[0].amplitude > 0.0,
        "the stopped step runs again, up"
    );
}

/// A host that stays slow stops only the step that was running: no step starts while it is
/// slow, so a slow stretch of any length doesn't run into the stall budget (`max_stalled`).
#[test]
fn a_long_slow_stretch_stops_one_step() {
    let (events, status) = step_test_with_slow_host(600.0);
    // Still waiting when the flight ends: neither failed nor done.
    check!(
        matches!(status, AutotuneStatus::Settling { .. }),
        "{status:?}"
    );
    let stopped = events
        .iter()
        .filter(|event| matches!(event, AutotuneEvent::Stalled { .. }))
        .count();
    check!(stopped == 1);
    check!(step_metrics(&events).is_empty());
}

/// A step test settles to its own, tighter limits: held 3.4° off level (inside the envelope's
/// 6°, outside the step settle's 2.5°), it never starts; a design stage, which settles to the
/// envelope's, does.
#[test]
fn a_step_test_settles_to_its_own_limits() {
    let (mut flight_loop, controls) = hovering(1.0);
    let baseline = *flight_loop.stabilizer().config();
    controls.input.set(Some(ControlInput::AbsoluteAttitude(
        AbsoluteAttitudeCommand {
            roll_rad: 0.06,
            thrust: 0.5,
            ..Default::default()
        },
    )));
    let mut autotune = started(
        &flight_loop,
        &[StagePlan::copter_step_test(TunedLoop::RollRate)],
    );

    let events = run(&mut flight_loop, &mut autotune, 30.0, |_, _| {});

    let failed = AutotuneEvent::StageFailed {
        target: TunedLoop::RollRate,
        reason: AutotuneAbort::NotSettled,
    };
    check!(events.last() == Some(&failed));
    check!(*flight_loop.stabilizer().config() == baseline);

    // A design settles to the envelope's 6°: the same hold starts it.
    let (mut flight_loop, controls) = hovering(1.0);
    controls.input.set(Some(ControlInput::AbsoluteAttitude(
        AbsoluteAttitudeCommand {
            roll_rad: 0.06,
            thrust: 0.5,
            ..Default::default()
        },
    )));
    let mut autotune = started(&flight_loop, &[design_stage(TunedLoop::PitchRate)]);
    run(&mut flight_loop, &mut autotune, 5.0, |_, _| {});
    check!(matches!(
        autotune.status(),
        AutotuneStatus::Running { .. } | AutotuneStatus::Complete
    ));
}

/// A yaw stage waits for the yaw rate to settle too: turning level at 0.3 rad/s (past the
/// step settle's 0.1), a yaw rate step test never starts, and a roll rate one, which leaves
/// yaw out, does (#296).
#[test]
fn a_yaw_stage_waits_for_yaw_to_settle() {
    const TURN_RADPS: f32 = 0.3;
    // (target, whether it starts)
    let rows = [(TunedLoop::YawRate, false), (TunedLoop::RollRate, true)];
    for (target, starts) in rows {
        let (mut flight_loop, controls) = hovering(1.0);
        let mut heading_rad = 0.0;
        let mut autotune = started(&flight_loop, &[StagePlan::copter_step_test(target)]);

        let events = run(&mut flight_loop, &mut autotune, 30.0, |_, _| {
            heading_rad += TURN_RADPS * DT;
            controls.input.set(Some(ControlInput::AbsoluteAttitude(
                AbsoluteAttitudeCommand {
                    heading_rad,
                    thrust: 0.5,
                    ..Default::default()
                },
            )));
        });

        let failed = AutotuneEvent::StageFailed {
            target,
            reason: AutotuneAbort::NotSettled,
        };
        check!((events.last() == Some(&failed)) == !starts, "{target:?}");
        check!(step_metrics(&events).is_empty() == !starts, "{target:?}");
    }
}

/// A yaw rate design holds its gains only once yaw is still too: turning level at 0.6 rad/s
/// (past the envelope settle's 0.3), the designed gains never hold and go back
/// ([`AutotuneEvent::Unsettled`]); flying straight, they're committed (#296).
#[test]
fn a_yaw_design_holds_only_once_yaw_is_still() {
    const TURN_RADPS: f32 = 0.6;
    // (turn rate in rad/s, whether the design holds)
    let rows = [(TURN_RADPS, false), (0.0, true)];
    for (turn_radps, holds) in rows {
        let (mut flight_loop, controls) = hovering(1.0);
        let mut heading_rad = 0.0;
        let mut autotune = started(&flight_loop, &[design_stage(TunedLoop::YawRate)]);

        let events = run(&mut flight_loop, &mut autotune, 60.0, |_, _| {
            heading_rad += turn_radps * DT;
            controls.input.set(Some(ControlInput::AbsoluteAttitude(
                AbsoluteAttitudeCommand {
                    heading_rad,
                    thrust: 0.5,
                    ..Default::default()
                },
            )));
        });

        let unsettled = events
            .iter()
            .any(|event| matches!(event, AutotuneEvent::Unsettled { .. }));
        let designed = events.iter().any(|event| {
            matches!(
                event,
                AutotuneEvent::StageDone {
                    outcome: Outcome::Designed,
                    ..
                }
            )
        });
        check!(unsettled == !holds, "turning {turn_radps}: {events:?}");
        check!(designed == holds, "turning {turn_radps}: {events:?}");
    }
}

/// Rate P multiplier that puts the start (P 3) well above [`lowering_search`]'s `max`.
const SCALE_ABOVE_MAX: f32 = 6.0;

/// A roll rate search whose `max` (P 1) is below a start at [`SCALE_ABOVE_MAX`].
fn lowering_search() -> StagePlan {
    StagePlan {
        target: TunedLoop::RollRate,
        method: Method::Search(SearchPlan {
            max: 1.0,
            ..SearchPlan::copter_rate()
        }),
    }
}

/// A rate axis with QuadCopterX-like lag: 12 ms of dead time.
fn delayed_axis() -> PlantParams {
    PlantParams {
        delay_s: 0.012,
        ..PlantParams::rate_axis()
    }
}

/// A rate P far above the search's `max` is lowered and committed before anything flies:
/// the vehicle settles on the lowered gains between probes, never on the start, and a kept
/// result is the lowered gains (#375).
#[test]
fn a_start_above_max_settles_on_lowered_gains() {
    let (mut flight_loop, _controls) = hovering_on(SCALE_ABOVE_MAX, delayed_axis());
    let start = TunedLoop::RollRate.gains(flight_loop.stabilizer().config());
    let mut autotune = started(&flight_loop, &[lowering_search()]);

    // The roll rate gains flown on each tick spent settling.
    let mut settling = Vec::new();
    let events = run(
        &mut flight_loop,
        &mut autotune,
        300.0,
        |autotune, flight_loop| {
            if matches!(autotune.status(), AutotuneStatus::Settling { .. }) {
                settling.push(TunedLoop::RollRate.gains(flight_loop.stabilizer().config()));
            }
        },
    );

    let Some(lowered) = events.iter().find_map(|event| match *event {
        AutotuneEvent::Lowered { gains, .. } => Some(gains),
        _ => None,
    }) else {
        panic!("no Lowered event: {events:?}");
    };
    check!(lowered.p < start.p);
    let Some(first) = settling.iter().position(|gains| *gains == lowered) else {
        panic!("never settled on the lowered gains");
    };
    check!(settling[first..].iter().all(|gains| *gains == lowered));
    let Some((gains, outcome)) = events.iter().find_map(|event| match *event {
        AutotuneEvent::StageDone { gains, outcome, .. } => Some((gains, outcome)),
        _ => None,
    }) else {
        panic!("no StageDone: {events:?}");
    };
    check!(gains != start);
    if matches!(outcome, Outcome::Kept(_)) {
        check!(gains == lowered);
    }
}

/// A stage that fails after lowering its start puts back the lowered gains, not the start
/// (#375).
#[test]
fn a_failure_after_lowering_keeps_the_lowered_gains() {
    let (mut flight_loop, _controls) = hovering_on(SCALE_ABOVE_MAX, delayed_axis());
    let start = TunedLoop::RollRate.gains(flight_loop.stabilizer().config());
    let mut autotune = started(&flight_loop, &[lowering_search()]);

    let mut lowered = None;
    for _ in 0..(60.0 / DT) as usize {
        flight_loop.step(Duration::from_secs_f32(DT));
        if let Some(AutotuneEvent::Lowered { gains, .. }) = autotune.update(&mut flight_loop, DT) {
            lowered = Some(gains);
        }
        if lowered.is_some() && matches!(autotune.status(), AutotuneStatus::Running { .. }) {
            break;
        }
    }
    let Some(lowered) = lowered else {
        panic!("no Lowered event");
    };
    check!(lowered != start);

    // Stopped from outside mid-probe: the stage fails as interrupted.
    flight_loop.stop_probe();
    flight_loop.step(Duration::from_secs_f32(DT));
    let failed = AutotuneEvent::StageFailed {
        target: TunedLoop::RollRate,
        reason: AutotuneAbort::Interrupted,
    };
    check!(autotune.update(&mut flight_loop, DT) == Some(failed));
    check!(TunedLoop::RollRate.gains(flight_loop.stabilizer().config()) == lowered);
}

/// The rate search tunes roll and pitch rate from a hand-tune-like start and from half its
/// P: each stage keeps a result that flies no worse than its baseline, with its start's D and
/// I at [`RATE_I_PER_P`] of a new P; the other loops keep their gains.
#[test]
fn rate_search_tunes_the_rate_loops() {
    for scale in [1.0, 0.5] {
        let (mut flight_loop, _controls) = hovering_on(scale, delayed_axis());
        let start = *flight_loop.stabilizer().config();
        let plan = [TunedLoop::RollRate, TunedLoop::PitchRate].map(search_stage);
        let mut autotune = started(&flight_loop, &plan);

        let events = run(&mut flight_loop, &mut autotune, 300.0, |_, _| {});

        check!(events.last() == Some(&AutotuneEvent::Complete), "{scale}");
        let tried = events
            .iter()
            .filter(|event| matches!(event, AutotuneEvent::Tried { .. }))
            .count();
        check!(
            tried >= 2 * plan.len(),
            "{scale}: baseline and a bracket point per stage"
        );
        let config = *flight_loop.stabilizer().config();
        for stage in plan {
            let target = stage.target;
            let done = events.iter().find_map(|event| match *event {
                AutotuneEvent::StageDone {
                    target: done,
                    gains,
                    outcome,
                    steps: Some(verify),
                    baseline: Some(baseline),
                } if done == target => Some((gains, outcome, verify, baseline)),
                _ => None,
            });
            let Some((gains, outcome, verify, baseline)) = done else {
                panic!("{scale} {target:?}: no StageDone: {events:?}");
            };
            check!(outcome == Outcome::Tuned, "{scale} {target:?}");
            check!(target.gains(&config) == gains, "{scale} {target:?}");
            let start_gains = target.gains(&start);
            let expected_i = if gains.p == start_gains.p {
                start_gains.i
            } else {
                RATE_I_PER_P * gains.p
            };
            check!(gains.i == expected_i, "{scale} {target:?}: I follows P");
            check!(
                gains.d == target.gains(&start).d,
                "{scale} {target:?}: D kept"
            );
            let (Some(verify_os), Some(baseline_os)) =
                (verify.overshoot.mean(), baseline.overshoot.mean())
            else {
                panic!("{scale} {target:?}: no overshoot");
            };
            check!(verify_os <= baseline_os + 0.05, "{scale} {target:?}");
        }
        for target in [
            TunedLoop::YawRate,
            TunedLoop::Roll,
            TunedLoop::Pitch,
            TunedLoop::Heading,
        ] {
            check!(
                target.gains(&config) == target.gains(&start),
                "{scale} {target:?}"
            );
        }
    }
}

/// Only a step flies a candidate: the vehicle settles on the committed gains, a candidate
/// goes in when its step starts and comes out when it ends. A gust past the envelope while a
/// candidate flies fails only that candidate (the run goes on); one that lasts past
/// `Envelope::recovery_s` on the committed gains fails the stage back to them.
#[test]
fn candidates_fly_only_during_their_step() {
    // (gust length in ticks, run completes)
    let cases = [(5, true), (usize::MAX, false)];
    for (gust_ticks, completes) in cases {
        let (mut flight_loop, controls) = hovering_on(1.0, delayed_axis());
        let start = *flight_loop.stabilizer().config();
        let plan = [search_stage(TunedLoop::RollRate)];
        let mut autotune = started(&flight_loop, &plan);

        let mut candidate_ticks = 0;
        let mut gust_left = 0;
        let mut events = Vec::new();
        for _ in 0..(300.0 / DT) as usize {
            flight_loop.step(Duration::from_secs_f32(DT));
            let flying = *flight_loop.stabilizer().config() != start;
            match autotune.status() {
                AutotuneStatus::Settling { .. } => {
                    check!(!flying, "settles on the committed gains");
                }
                AutotuneStatus::Running { .. } if flying => {
                    check!(flight_loop.probe().is_some(), "only under a probe");
                    candidate_ticks += 1;
                    // Well into the search: a gust past the envelope's rate.
                    if candidate_ticks == 200 {
                        gust_left = gust_ticks;
                    }
                }
                _ => {}
            }
            controls
                .rate_spike_radps
                .set(if gust_left > 0 { 10.0 } else { 0.0 });
            gust_left = gust_left.saturating_sub(1);
            events.extend(autotune.update(&mut flight_loop, DT));
            if !autotune.is_running() {
                break;
            }
        }

        check!(candidate_ticks >= 200, "{gust_ticks}");
        let failed_candidate = events.iter().any(|event| {
            matches!(event, AutotuneEvent::Tried { gains, passed: false, .. } if *gains != TunedLoop::RollRate.gains(&start))
        });
        check!(failed_candidate, "{gust_ticks}: the gusted candidate fails");
        if completes {
            check!(
                events.last() == Some(&AutotuneEvent::Complete),
                "{gust_ticks}"
            );
        } else {
            // The lasting gust rolls the vehicle past the tilt limit.
            let failed = AutotuneEvent::StageFailed {
                target: TunedLoop::RollRate,
                reason: AutotuneAbort::Tilt,
            };
            check!(events.last() == Some(&failed), "{gust_ticks}");
            check!(*flight_loop.stabilizer().config() == start, "{gust_ticks}");
        }
    }
}

/// A host turning slow while a candidate flies stops it and puts the committed gains back on
/// the same update; once the host keeps time, the same candidate flies again and the search
/// completes (#438).
#[test]
fn a_slow_host_stops_a_candidate_and_it_flies_again() {
    /// How long the host stays slow, in ticks (1 s).
    const SLOW_TICKS: usize = 250;
    let (mut flight_loop, _controls) = hovering_on(1.0, delayed_axis());
    let start = *flight_loop.stabilizer().config();
    let plan = [search_stage(TunedLoop::RollRate)];
    let mut autotune = started(&flight_loop, &plan);

    let mut stopped = None;
    let mut slow_left = 0;
    let mut flown_again = false;
    let mut events = Vec::new();
    for _ in 0..(300.0 / DT) as usize {
        flight_loop.step(Duration::from_secs_f32(DT));
        let config = *flight_loop.stabilizer().config();
        if stopped.is_none() && config != start && flight_loop.probe().is_some() {
            stopped = Some(config);
            slow_left = SLOW_TICKS;
        }
        autotune.set_host_slow(slow_left > 0);
        let event = autotune.update(&mut flight_loop, DT);
        if slow_left == SLOW_TICKS {
            check!(matches!(
                event,
                Some(AutotuneEvent::Stalled {
                    measurement: None,
                    ..
                })
            ));
            check!(
                *flight_loop.stabilizer().config() == start,
                "committed gains back"
            );
        }
        slow_left = slow_left.saturating_sub(1);
        if slow_left == 0 && stopped.is_some_and(|candidate| config == candidate) {
            flown_again = true;
        }
        events.extend(event);
        if !autotune.is_running() {
            break;
        }
    }

    check!(stopped.is_some(), "a candidate flew");
    check!(flown_again, "the stopped candidate flies again");
    check!(events.last() == Some(&AutotuneEvent::Complete));
}

/// A design stage runs through a slow host: its fit and hold go on, and it finishes with its
/// design while the host stays slow (#524).
#[test]
fn a_design_stage_runs_through_a_slow_host() {
    let (mut flight_loop, _controls) = hovering(1.0);
    let mut autotune = started(&flight_loop, &[design_stage(TunedLoop::RollRate)]);
    autotune.set_host_slow(true);

    let mut events = Vec::new();
    for _ in 0..(30.0 / DT) as usize {
        flight_loop.step(Duration::from_secs_f32(DT));
        events.extend(autotune.update(&mut flight_loop, DT));
        if !autotune.is_running() {
            break;
        }
    }

    let stalled = events
        .iter()
        .any(|event| matches!(event, AutotuneEvent::Stalled { .. }));
    check!(!stalled, "{events:?}");
    let designed = events.iter().any(|event| {
        matches!(
            event,
            AutotuneEvent::StageDone {
                outcome: Outcome::Designed,
                ..
            }
        )
    });
    check!(designed, "{events:?}");
    check!(autotune.status() == AutotuneStatus::Complete);
}

/// A breach in the second after a freeze ([`Autotune::note_stall`]) is the loop catching up
/// with what the vehicle did unseen and fails nothing; one with no freeze, or once
/// `Envelope::recovery_s` has passed, fails the stage (#524).
#[test]
fn a_breach_right_after_a_freeze_does_not_fail_the_stage() {
    /// How long each spike lasts, in ticks (0.1 s).
    const SPIKE_TICKS: usize = 25;
    // (row, ticks from the freeze to the spike, freeze noted, what fails the stage)
    let cases = [
        ("spike at a freeze", 0, true, None),
        ("spike ending inside the window", 200, true, None),
        (
            "spike after the window",
            300,
            true,
            Some(AutotuneAbort::Rate),
        ),
        ("spike, no freeze", 0, false, Some(AutotuneAbort::Rate)),
    ];
    for (row, delay_ticks, frozen, failure) in cases {
        let (mut flight_loop, controls) = hovering(1.0);
        let plan = [StagePlan::copter_step_test(TunedLoop::RollRate)];
        let mut autotune = started(&flight_loop, &plan);

        let mut since_freeze = None;
        let mut events = Vec::new();
        for _ in 0..(60.0 / DT) as usize {
            flight_loop.step(Duration::from_secs_f32(DT));
            if since_freeze.is_none() && matches!(autotune.status(), AutotuneStatus::Running { .. })
            {
                since_freeze = Some(0);
                if frozen {
                    autotune.note_stall();
                }
            }
            let spiking = since_freeze
                .is_some_and(|ticks| (delay_ticks..delay_ticks + SPIKE_TICKS).contains(&ticks));
            // Past the envelope's 4 rad/s.
            controls
                .rate_spike_radps
                .set(if spiking { 5.0 } else { 0.0 });
            since_freeze = since_freeze.map(|ticks| ticks + 1);
            events.extend(autotune.update(&mut flight_loop, DT));
            if !autotune.is_running() {
                break;
            }
        }

        check!(since_freeze.is_some(), "{row}");
        let expected = failure.map_or(AutotuneEvent::Complete, |reason| {
            AutotuneEvent::StageFailed {
                target: TunedLoop::RollRate,
                reason,
            }
        });
        check!(events.last() == Some(&expected), "{row}: {events:?}");
    }
}

/// A candidate that breaches right after a freeze comes out on that update as a stall, the
/// committed gains back, and flies again later: the breach says nothing about it (#524).
#[test]
fn a_candidate_breaching_after_a_freeze_comes_out_unjudged() {
    let (mut flight_loop, controls) = hovering_on(1.0, delayed_axis());
    let start = *flight_loop.stabilizer().config();
    let plan = [search_stage(TunedLoop::RollRate)];
    let mut autotune = started(&flight_loop, &plan);

    let mut frozen = None;
    let mut spiked = false;
    let mut flown_again = false;
    let mut events = Vec::new();
    for _ in 0..(300.0 / DT) as usize {
        flight_loop.step(Duration::from_secs_f32(DT));
        let config = *flight_loop.stabilizer().config();
        // The tick after the freeze: its state carries the spike.
        let checking = spiked;
        spiked = frozen.is_none() && config != start && flight_loop.probe().is_some();
        if spiked {
            frozen = Some(config);
            autotune.note_stall();
        }
        // Past the envelope's 4 rad/s, for the tick after the freeze only.
        controls
            .rate_spike_radps
            .set(if spiked { 5.0 } else { 0.0 });
        let event = autotune.update(&mut flight_loop, DT);
        if checking {
            check!(
                matches!(
                    event,
                    Some(AutotuneEvent::Stalled {
                        measurement: None,
                        ..
                    })
                ),
                "{event:?}"
            );
            check!(
                *flight_loop.stabilizer().config() == start,
                "committed gains back"
            );
        } else if !spiked && frozen.is_some_and(|candidate| config == candidate) {
            flown_again = true;
        }
        events.extend(event);
        if !autotune.is_running() {
            break;
        }
    }

    check!(frozen.is_some(), "a candidate flew");
    check!(flown_again, "the candidate flies again");
    check!(events.last() == Some(&AutotuneEvent::Complete));
}

/// A host link outage: the state holds at the last one seen while the vehicle flies on
/// unseen, then jumps. Told the state is stale, the supervisor stops the running step
/// unjudged, runs it again, and excuses the jump; untold, the jump fails the stage. Stale
/// past `Envelope::max_stale_s`, the stage fails `HostFault` (#309).
#[test]
fn a_stale_state_rides_through_its_jump_until_the_limit() {
    // (row, outage in ticks, told it's stale, how the run ends)
    let cases = [
        // RealFlight's first reconnect attempt comes 0.5 s into an outage.
        ("told, 0.5 s", 125, true, None),
        ("untold, 0.5 s", 125, false, Some(AutotuneAbort::Tilt)),
        (
            "told, past the limit",
            (2.1 / DT) as usize,
            true,
            Some(AutotuneAbort::HostFault),
        ),
    ];
    for (row, outage_ticks, told, failure) in cases {
        let (mut flight_loop, controls) = hovering(1.0);
        let plan = [StagePlan::copter_step_test(TunedLoop::RollRate)];
        let mut autotune = started(&flight_loop, &plan);

        let mut since_outage = None;
        let mut events = Vec::new();
        for _ in 0..(120.0 / DT) as usize {
            if since_outage.is_none() && flight_loop.probe().is_some() {
                since_outage = Some(0);
            }
            let stale = since_outage.is_some_and(|ticks| ticks < outage_ticks);
            controls.outage.set(stale);
            flight_loop.step(Duration::from_secs_f32(DT));
            autotune.set_state_stale(told && stale);
            since_outage = since_outage.map(|ticks| ticks + 1);
            events.extend(autotune.update(&mut flight_loop, DT));
            if !autotune.is_running() {
                break;
            }
        }

        check!(since_outage.is_some(), "{row}");
        let expected = failure.map_or(AutotuneEvent::Complete, |reason| {
            AutotuneEvent::StageFailed {
                target: TunedLoop::RollRate,
                reason,
            }
        });
        check!(events.last() == Some(&expected), "{row}: {events:?}");
        if told {
            check!(
                events.iter().any(|event| matches!(
                    event,
                    AutotuneEvent::Stalled {
                        measurement: None,
                        ..
                    }
                )),
                "{row}: the step under the outage runs again"
            );
        }
    }
}

/// Aborting a run with a candidate under test ends it there: the stage fails for the reason
/// given, the probe stops, and the committed gains fly. Before the start and after the end,
/// it does nothing (#309).
#[test]
fn an_abort_ends_the_run_on_the_kept_gains() {
    let (mut flight_loop, _controls) = hovering_on(1.0, delayed_axis());
    let start = *flight_loop.stabilizer().config();
    let plan = [search_stage(TunedLoop::RollRate)];
    let mut autotune = Autotune::new(&plan, Envelope::default()).expect("valid plan");
    check!(autotune.abort(&mut flight_loop, AutotuneAbort::HostFault) == None);
    autotune.start(&flight_loop).expect("armed and idle");

    let mut aborted = None;
    for _ in 0..(300.0 / DT) as usize {
        flight_loop.step(Duration::from_secs_f32(DT));
        autotune.update(&mut flight_loop, DT);
        if *flight_loop.stabilizer().config() != start && flight_loop.probe().is_some() {
            aborted = autotune.abort(&mut flight_loop, AutotuneAbort::HostFault);
            break;
        }
    }

    check!(
        aborted
            == Some(AutotuneEvent::StageFailed {
                target: TunedLoop::RollRate,
                reason: AutotuneAbort::HostFault,
            })
    );
    check!(
        autotune.status()
            == AutotuneStatus::Failed {
                stage: 0,
                reason: AutotuneAbort::HostFault
            }
    );
    check!(*flight_loop.stabilizer().config() == start);
    check!(flight_loop.probe() == None);
    check!(autotune.abort(&mut flight_loop, AutotuneAbort::HostFault) == None);
}

/// `DesignPlan::validate` rejects a negative or non-finite `rate_i_per_p`; the copter default
/// and `None` are valid.
#[test]
fn design_plan_rate_i_validation() {
    let good = DesignPlan::copter();
    check!(good.validate() == Ok(()));
    let none = DesignPlan {
        rate_i_per_p: None,
        ..good
    };
    check!(none.validate() == Ok(()));
    for ratio in [-0.1, f32::NAN, f32::INFINITY] {
        let bad = DesignPlan {
            rate_i_per_p: Some(ratio),
            ..good
        };
        let error = bad.validate().expect_err("rejected");
        check!(error.field == "rate_i_per_p", "{ratio}");
    }
}

/// `SearchPlan::validate` rejects each bad field; the copter default is valid.
#[test]
fn search_plan_validation() {
    let good = SearchPlan::copter_rate();
    let rows: [(SearchPlan, &str); 14] = [
        (SearchPlan { confirm: 0, ..good }, "confirm"),
        (
            SearchPlan {
                i_per_p: Some(-0.1),
                ..good
            },
            "i_per_p",
        ),
        (
            SearchPlan {
                term: Term::D,
                ..good
            },
            "i_per_p",
        ),
        (
            SearchPlan {
                test: StepTestPlan {
                    max_stalled: 0,
                    ..good.test
                },
                ..good
            },
            "max_stalled",
        ),
        (
            SearchPlan {
                test: StepTestPlan {
                    pairs: 0,
                    ..good.test
                },
                ..good
            },
            "pairs",
        ),
        (
            SearchPlan {
                criterion: Criterion::Overshoot {
                    max_overshoot: -0.1,
                    max_bounce_back: 0.1,
                    settled_past: None,
                },
                ..good
            },
            "criterion.max_overshoot",
        ),
        (
            SearchPlan {
                criterion: Criterion::Overshoot {
                    max_overshoot: 0.05,
                    max_bounce_back: f32::NAN,
                    settled_past: None,
                },
                ..good
            },
            "criterion.max_bounce_back",
        ),
        (
            SearchPlan {
                factor: 1.0,
                ..good
            },
            "factor",
        ),
        (
            SearchPlan {
                resolution: 1.0,
                ..good
            },
            "resolution",
        ),
        (
            SearchPlan {
                margin: 0.0,
                ..good
            },
            "margin",
        ),
        (
            SearchPlan {
                margin: 1.1,
                ..good
            },
            "margin",
        ),
        (SearchPlan { min: 0.0, ..good }, "min"),
        (
            SearchPlan {
                max: good.min,
                ..good
            },
            "max",
        ),
        (
            SearchPlan {
                acceptance: Acceptance {
                    settling_ratio: 0.9,
                    ..Acceptance::default()
                },
                ..good
            },
            "acceptance.settling_ratio",
        ),
    ];
    check!(good.validate() == Ok(()));
    for (plan, field) in rows {
        let error = plan.validate().expect_err(field);
        check!(error.field == field);
    }
    let stage = StagePlan {
        target: TunedLoop::RollRate,
        method: Method::Search(SearchPlan {
            factor: 0.5,
            ..good
        }),
    };
    let error = Autotune::new(&[stage], Envelope::default()).expect_err("bad factor");
    check!(error.section == Some("plan.search"));
}

/// The angle criterion judges overshoot from where each step ended, and fails a step that
/// ends further past the step than its settle band; the rate criterion judges from the step
/// (#564).
#[test]
fn the_angle_criterion_judges_overshoot_from_where_the_step_ended() {
    let angle = SearchPlan::copter_angle().criterion;
    let rate = SearchPlan::copter_rate().criterion;
    // (case, each step's peak and final value, angle passes, rate passes)
    type Row = (&'static str, &'static [(f32, f32)], bool, bool);
    let rows: [Row; 4] = [
        (
            "held past, no ringing",
            &[(1.06, 1.06), (1.06, 1.06)],
            true,
            false,
        ),
        (
            "rings past where it ends",
            &[(1.1, 1.0), (1.1, 1.0)],
            false,
            false,
        ),
        (
            "ends past the band",
            &[(1.12, 1.12), (1.0, 1.0)],
            false,
            false,
        ),
        ("on the step", &[(1.02, 1.0), (1.0, 1.0)], true, true),
    ];
    for (case, steps, angle_passes, rate_passes) in rows {
        let mut summary = StepSummary::default();
        for &(peak, final_value) in steps {
            summary.add(&StepMetrics {
                amplitude: 1.0,
                peak,
                overshoot: (peak - 1.0).max(0.0),
                final_value,
                ..Default::default()
            });
        }
        check!(angle.passes(&summary) == angle_passes, "{case}: angle");
        check!(rate.passes(&summary) == rate_passes, "{case}: rate");
    }
}

/// The criterion passes on its limits' means; acceptance compares the verify's means with the
/// baseline's, a missing baseline mean allowing anything.
#[test]
fn criterion_and_acceptance_judge_the_means() {
    // (overshoot, bounce-back, settling time)
    type Step = (f32, f32, Option<f32>);
    let summary = |steps: &[Step]| {
        let mut summary = StepSummary::default();
        for &(overshoot, bounce_back, settling_time_s) in steps {
            summary.add(&StepMetrics {
                amplitude: 1.0,
                overshoot,
                bounce_back,
                settling_time_s,
                ..Default::default()
            });
        }
        summary
    };
    let criterion = SearchPlan::copter_rate().criterion;
    // (steps, passes): limits 0.05 overshoot, 0.1 bounce-back
    let criteria: [(&[Step], bool); 5] = [
        (&[(0.05, 0.1, None)], true),
        (&[(0.0, 0.0, None), (0.1, 0.0, None)], true), // mean 0.05
        (&[(0.06, 0.0, None)], false),
        (&[(0.0, 0.11, None)], false),
        (&[], false),
    ];
    for (index, (steps, passes)) in criteria.into_iter().enumerate() {
        check!(criterion.passes(&summary(steps)) == passes, "row {index}");
    }

    let acceptance = Acceptance::default();
    let base = [(0.1, 0.0, Some(0.2))];
    // (baseline, verify, accepted)
    let rows: [(&[Step], &[Step], bool); 8] = [
        (&base, &[(0.15, 0.0, Some(0.22))], true), // at both limits
        (&base, &[(0.16, 0.0, Some(0.2))], false), // more overshoot
        (&base, &[(0.1, 0.0, Some(0.23))], false), // slower
        (&base, &[(0.1, 0.0, None)], false),       // unsettled
        (&base, &[(0.1, 0.11, Some(0.2))], false), // bounces past both
        (&[(0.1, 0.2, Some(0.2))], &[(0.1, 0.15, Some(0.2))], true), // less than baseline's
        (&[(0.1, 0.0, None)], &[(0.1, 0.0, Some(0.3))], true), // the baseline never settled
        (&base, &[], false),
    ];
    for (index, (baseline, verify, accepted)) in rows.into_iter().enumerate() {
        check!(
            acceptance.accepts(&criterion, &summary(baseline), &summary(verify)) == accepted,
            "row {index}"
        );
    }
}

/// A term reads and replaces its own gain only.
#[test]
fn terms_read_and_replace_their_gain() {
    let gains = Gains {
        p: 1.0,
        i: 2.0,
        d: 3.0,
    };
    check!(Term::P.get(gains) == 1.0);
    check!(Term::D.get(gains) == 3.0);
    check!(Term::P.with(gains, 9.0) == Gains { p: 9.0, ..gains });
    check!(Term::D.with(gains, 9.0) == Gains { d: 9.0, ..gains });
}

/// The full copter search, on axes with delay and noise: rate loops first, then angle; each
/// stage keeps a result no worse than its baseline, with its start's D, and the angle P stays
/// within the plan's bounds.
#[test]
fn searches_tune_rate_then_angle() {
    let params = PlantParams {
        delay_s: 0.02,
        noise: 0.02,
        ..PlantParams::rate_axis()
    };
    let (mut flight_loop, _controls) = hovering_on(1.0, params);
    let start = *flight_loop.stabilizer().config();
    let plan = DESIGNED.map(search_stage);
    let mut autotune = started(&flight_loop, &plan);

    let events = run(&mut flight_loop, &mut autotune, 600.0, |_, _| {});

    check!(events.last() == Some(&AutotuneEvent::Complete));
    let done: Vec<_> = events
        .iter()
        .filter_map(|event| match *event {
            AutotuneEvent::StageDone {
                target,
                gains,
                outcome,
                steps: Some(verify),
                baseline: Some(baseline),
            } => Some((target, gains, outcome, verify, baseline)),
            _ => None,
        })
        .collect();
    let targets: Vec<_> = done.iter().map(|(target, ..)| *target).collect();
    check!(targets == plan.map(|stage| stage.target));
    let angle = SearchPlan::copter_angle();
    for (target, gains, outcome, verify, baseline) in done {
        check!(outcome == Outcome::Tuned, "{target:?}");
        check!(gains.d == target.gains(&start).d, "{target:?}: D kept");
        if target.is_rate() {
            continue;
        }
        check!(
            verify.settling_time_s.mean().is_some() && baseline.settling_time_s.mean().is_some(),
            "{target:?}: settled"
        );
        check!((angle.min..=angle.max).contains(&gains.p), "{target:?}");
    }
}

/// Each loop's search steps as its step test does, allowing 10 clipped ticks (a step's
/// first ticks ask for more than full moment at tuned gains): 1 rad/s on roll and pitch rate,
/// 0.5 rad/s on yaw rate (prop drag's little yaw authority, #296), 5° on roll and pitch. The
/// yaw rate step test watches its step for 1 s, the search for 0.25 s (#558). Roll and pitch
/// rate searches carry I at a fifth of P, yaw rate at 0.3 of P.
#[test]
fn search_step_presets() {
    let rows = [
        (
            TunedLoop::RollRate,
            COPTER_RATE_STEP,
            COPTER_RATE_STEP,
            Some(RATE_I_PER_P),
        ),
        (
            TunedLoop::PitchRate,
            COPTER_RATE_STEP,
            COPTER_RATE_STEP,
            Some(RATE_I_PER_P),
        ),
        (
            TunedLoop::YawRate,
            COPTER_YAW_RATE_STEP,
            COPTER_YAW_RATE_TEST_STEP,
            Some(YAW_RATE_I_PER_P),
        ),
        (TunedLoop::Roll, COPTER_TILT_STEP, COPTER_TILT_STEP, None),
        (TunedLoop::Pitch, COPTER_TILT_STEP, COPTER_TILT_STEP, None),
    ];
    let ten_ticks = (0..10).fold(0.0_f32, |sum, _| sum + 0.004);
    for (target, search_step, test_step, i_per_p) in rows {
        let search = SearchPlan::copter(target);
        check!(search.test.step == search_step, "{target:?}");
        check!(StepTestPlan::copter(target).step == test_step, "{target:?}");
        check!(ten_ticks <= search.test.max_saturated_s, "{target:?}");
        check!(
            ten_ticks + 0.004 > search.test.max_saturated_s,
            "{target:?}"
        );
        check!(search.i_per_p == i_per_p, "{target:?}");
        check!(search.validate() == Ok(()), "{target:?}");
    }
}

/// The design search designs every roll and pitch loop, rate first, with every fit run on the
/// axis's rate loop, then searches each loop from its design in the same order,
/// then searches yaw rate from its start.
#[test]
fn copter_design_search_designs_then_searches_every_loop() {
    let params = PlantParams {
        delay_s: 0.008,
        ..PlantParams::rate_axis()
    };
    let (mut flight_loop, _controls) = hovering_on(1.0, params);
    let start = *flight_loop.stabilizer().config();
    let plan = StagePlan::copter_design_search();
    let mut autotune = started(&flight_loop, &plan);

    let events = run(&mut flight_loop, &mut autotune, 900.0, |_, _| {});

    check!(events.last() == Some(&AutotuneEvent::Complete));
    let fitted: Vec<_> = events
        .iter()
        .filter_map(|event| match event {
            AutotuneEvent::Measured {
                target,
                measurement: Measurement::Identify(_),
            } => Some(*target),
            _ => None,
        })
        .collect();
    let rate_loops = [
        TunedLoop::RollRate,
        TunedLoop::PitchRate,
        TunedLoop::RollRate,
        TunedLoop::PitchRate,
    ];
    check!(fitted == rate_loops);
    let done: Vec<_> = events
        .iter()
        .filter_map(|event| match *event {
            AutotuneEvent::StageDone {
                target,
                gains,
                outcome,
                steps,
                ..
            } => Some((target, gains, outcome, steps.is_some())),
            _ => None,
        })
        .collect();
    let targets: Vec<_> = done.iter().map(|(target, ..)| *target).collect();
    check!(targets == plan.map(|stage| stage.target));
    let tuned = *flight_loop.stabilizer().config();
    for (index, (target, gains, outcome, stepped)) in done.into_iter().enumerate() {
        let search = index >= 4;
        let expected = if search {
            Outcome::Tuned
        } else {
            Outcome::Designed
        };
        check!(outcome == expected, "stage {index}: {target:?}");
        check!(stepped == search, "stage {index}: {target:?}");
        check!(
            gains != target.gains(&start),
            "stage {index}: {target:?} tuned"
        );
        if search {
            check!(gains == target.gains(&tuned), "stage {index}: {target:?}");
        }
    }
}

/// A fit starts when the settle times out: a vehicle that can't get still still gets fitted,
/// since a fit only gains excitation from the ringing.
#[test]
fn an_identification_starts_when_the_settle_times_out() {
    let (mut flight_loop, controls) = hovering(1.0);
    hold_roll(&controls, 0.25);
    let plan = StagePlan {
        target: TunedLoop::PitchRate,
        method: Method::Design(DesignPlan::copter()),
    };
    let mut autotune = started(&flight_loop, &[plan]);
    let timeout_s = Envelope::default().settle_timeout_s;

    run(&mut flight_loop, &mut autotune, timeout_s - 0.1, |_, _| {});
    check!(matches!(autotune.status(), AutotuneStatus::Settling { .. }));
    run(&mut flight_loop, &mut autotune, 0.2, |_, _| {});
    check!(matches!(autotune.status(), AutotuneStatus::Running { .. }));
    check!(matches!(
        flight_loop.probe(),
        Some((TunedLoop::PitchRate, ProbeStatus::Running(_)))
    ));
}

/// A settle holds on tilt and the roll and pitch rates; a yaw rate, from the untuned yaw loop
/// ringing, doesn't hold it up (#518).
#[test]
fn a_settle_ignores_the_yaw_rate() {
    let settle = COPTER_STEP_SETTLE;
    let past = 2.0 * settle.rate_radps;
    // (case, roll rate, pitch rate, yaw rate in rad/s, roll in rad, holds)
    let rows = [
        ("still", 0.0, 0.0, 0.0, 0.0, true),
        ("yawing", 0.0, 0.0, past, 0.0, true),
        ("rolling", past, 0.0, 0.0, 0.0, false),
        ("pitching", 0.0, -past, 0.0, 0.0, false),
        ("tilted", 0.0, 0.0, 0.0, 2.0 * settle.tilt_rad, false),
    ];
    for (case, roll_radps, pitch_radps, yaw_radps, roll_rad, holds) in rows {
        let state = VehicleState {
            angular_velocity_radps: Vec3::new(roll_radps, pitch_radps, yaw_radps),
            attitude: nalgebra::UnitQuaternion::from_euler_angles(roll_rad, 0.0, 0.0),
            ..Default::default()
        };
        check!(
            settle.holds(&Motion::of(&state, state.angular_velocity_radps)) == holds,
            "{case}"
        );
    }
}

/// The settle reads the filtered rates, about every axis and each alone: a still vehicle whose
/// raw gyro carries noise past the settle holds it (#373).
#[test]
fn a_settle_reads_the_filtered_rates() {
    let settle = COPTER_STEP_SETTLE;
    let noisy = Vec3::repeat(2.0 * settle.rate_radps);
    let state = VehicleState {
        angular_velocity_radps: noisy,
        ..Default::default()
    };
    // (case, filtered rates, holds)
    let rows = [
        ("filtered still", Vec3::zeros(), true),
        ("filtered noisy", noisy, false),
    ];
    for (case, filtered, holds) in rows {
        let motion = Motion::of(&state, filtered);
        check!(settle.holds(&motion) == holds, "{case}");
        check!(settle.holds_axis(&motion, 0) == holds, "{case}: roll");
        check!(settle.holds_axis(&motion, 1) == holds, "{case}: pitch");
    }
}

/// `TunedLoop::inner` routes a fit on an angle stage to the rate loop under it, as PX4 adds its
/// excitation to the rate setpoint; rate stages and other probes run on their own loop (#561).
#[test]
fn fits_run_on_the_rate_loop_under_an_angle_stage() {
    let identify = ProbeConfig::Identify(COPTER_IDENTIFY);
    let step = ProbeConfig::Step(COPTER_RATE_STEP);
    // (stage target, probe, probed loop)
    let rows = [
        (TunedLoop::Roll, identify, TunedLoop::RollRate),
        (TunedLoop::Pitch, identify, TunedLoop::PitchRate),
        (TunedLoop::RollRate, identify, TunedLoop::RollRate),
        (TunedLoop::Roll, step, TunedLoop::Roll),
        (TunedLoop::RollRate, step, TunedLoop::RollRate),
    ];
    for (target, probe, probed) in rows {
        check!(
            probed_loop(target, &probe) == probed,
            "{target:?} {probe:?}"
        );
    }
}

/// A failure puts back every design its loop's search hasn't finished, and keeps the searched
/// loops: roll rate searched stays, pitch rate designed only goes back to its start.
#[test]
fn a_failure_puts_back_the_designs_not_yet_searched() {
    let (mut flight_loop, _controls) = hovering(1.0);
    let start = *flight_loop.stabilizer().config();
    // No angle P in [500, 600] keeps the margin: the roll design fails.
    let unreachable = DesignPlan {
        design: LoopShapingConfig {
            angle_p_min: 500.0,
            angle_p_max: 600.0,
            ..COPTER_LOOP_SHAPING
        },
        ..DesignPlan::copter()
    };
    let plan = [
        design_stage(TunedLoop::RollRate),
        design_stage(TunedLoop::PitchRate),
        search_stage(TunedLoop::RollRate),
        StagePlan {
            target: TunedLoop::Roll,
            method: Method::Design(unreachable),
        },
    ];
    let mut autotune = started(&flight_loop, &plan);

    let events = run(&mut flight_loop, &mut autotune, 600.0, |_, _| {});

    let searched = events.iter().find_map(|event| match *event {
        AutotuneEvent::StageDone {
            target: TunedLoop::RollRate,
            steps: Some(_),
            gains,
            ..
        } => Some(gains),
        _ => None,
    });
    check!(
        autotune.status()
            == AutotuneStatus::Failed {
                stage: 3,
                reason: AutotuneAbort::Design(DesignError::AngleUnstable),
            }
    );
    let config = *flight_loop.stabilizer().config();
    check!(Some(TunedLoop::RollRate.gains(&config)) == searched);
    check!(searched != Some(TunedLoop::RollRate.gains(&start)));
    check!(TunedLoop::PitchRate.gains(&config) == TunedLoop::PitchRate.gains(&start));
    check!(TunedLoop::Roll.gains(&config) == TunedLoop::Roll.gains(&start));
}

/// A design that breaches the envelope during its search's baseline fails the stage and goes
/// back to the loop's start, rather than staying on: here a rate limit the baseline's steps
/// pass (they peak at 0.67 rad/s on this sim) and the fit doesn't.
#[test]
fn a_design_that_breaches_during_its_search_goes_back_to_its_start() {
    let (mut flight_loop, _controls) = hovering(1.0);
    let start = *flight_loop.stabilizer().config();
    let envelope = Envelope {
        max_rate_radps: 0.5,
        ..Envelope::default()
    };
    let plan = [
        design_stage(TunedLoop::RollRate),
        search_stage(TunedLoop::RollRate),
    ];
    let mut autotune = Autotune::new(&plan, envelope).expect("valid plan");
    autotune.start(&flight_loop).expect("armed and idle");

    let events = run(&mut flight_loop, &mut autotune, 60.0, |_, _| {});

    check!(
        autotune.status()
            == AutotuneStatus::Failed {
                stage: 1,
                reason: AutotuneAbort::Rate,
            },
        "{events:?}"
    );
    check!(*flight_loop.stabilizer().config() == start);
}

/// A design above its search's `max` is lowered to it, and still goes back to the loop's start
/// when the search fails: the lowered design never passed a step. Here every step after the
/// lower stalls, so the search fails, `Stalled`, or `NotSettled` once the lowered P can't
/// level the vehicle between steps.
#[test]
fn a_lowered_design_goes_back_to_its_start_when_its_search_fails() {
    let (mut flight_loop, _controls) = hovering(1.0);
    let start = *flight_loop.stabilizer().config();
    let below_the_design = SearchPlan {
        min: 0.01,
        max: 0.05,
        ..SearchPlan::copter(TunedLoop::RollRate)
    };
    let plan = [
        design_stage(TunedLoop::RollRate),
        StagePlan {
            target: TunedLoop::RollRate,
            method: Method::Search(below_the_design),
        },
    ];
    let mut autotune = started(&flight_loop, &plan);

    let mut lowered = false;
    for _ in 0..(120.0 / DT) as usize {
        if lowered {
            autotune.note_stall();
        }
        flight_loop.step(Duration::from_secs_f32(DT));
        let event = autotune.update(&mut flight_loop, DT);
        lowered |= matches!(event, Some(AutotuneEvent::Lowered { .. }));
        if matches!(autotune.status(), AutotuneStatus::Failed { .. }) {
            break;
        }
    }

    check!(lowered);
    let status = autotune.status();
    check!(
        matches!(
            status,
            AutotuneStatus::Failed {
                stage: 1,
                reason: AutotuneAbort::Stalled | AutotuneAbort::NotSettled,
            }
        ),
        "{status:?}"
    );
    check!(*flight_loop.stabilizer().config() == start);
}

/// The large-copter search tunes roll and pitch rate, then angle, on the large steps, keeping
/// the start's rate I, and validates within its envelope (#351); yaw rate and heading, which it
/// doesn't tune, keep the copter plan.
#[test]
fn large_copter_search_steps_each_loop_on_large_steps() {
    let plan = StagePlan::large_copter_search();
    // (stage, target, step)
    let rows = [
        (0, TunedLoop::RollRate, LARGE_COPTER_RATE_STEP),
        (1, TunedLoop::PitchRate, LARGE_COPTER_RATE_STEP),
        (2, TunedLoop::Roll, LARGE_COPTER_TILT_STEP),
        (3, TunedLoop::Pitch, LARGE_COPTER_TILT_STEP),
    ];
    for (stage, target, step) in rows {
        check!(plan[stage].target == target);
        let Method::Search(search) = plan[stage].method else {
            panic!("stage {stage}: not a search");
        };
        check!(search.test.step == step, "{target:?}");
        check!(
            search.test.max_saturated_s == LARGE_COPTER_MAX_SATURATED_S,
            "{target:?}"
        );
        check!(search.i_per_p == None, "{target:?}");
    }
    check!(Autotune::new(&plan, Envelope::large_copter()).is_ok());
    for target in [TunedLoop::YawRate, TunedLoop::Heading] {
        check!(
            SearchPlan::large_copter(target) == SearchPlan::copter(target),
            "{target:?}"
        );
    }
}
