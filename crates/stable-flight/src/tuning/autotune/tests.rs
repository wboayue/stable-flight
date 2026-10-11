use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::Duration;

use assert2::{assert, check};

use super::*;
use crate::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use crate::stabilizers::copter::CopterStabilizer;
use crate::tuning::plant::{Axes, PlantParams};
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
}

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
        let mut axes = self.0.axes.borrow_mut();
        let Some(axes) = axes.as_mut() else {
            return VehicleState::default();
        };
        let mut state = axes.state();
        state.position_m.z = self.0.altitude_m.get();
        state.angular_velocity_radps.x += self.0.rate_spike_radps.get();
        state
    }
}

impl Actuator for SimVehicle {
    fn actuate(&mut self, command: &ActuatorCommand) {
        if let Some(axes) = self.0.axes.borrow_mut().as_mut() {
            let servos = command.servos;
            axes.step(Vec3::new(servos[0], servos[1], servos[2]));
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
    let controls = Controls::default();
    controls.arm.set(true);
    controls.altitude_m.set(ALTITUDE_M);
    *controls.axes.borrow_mut() = Some(Axes::new([PlantParams::rate_axis(); 3], Vec3::zeros(), DT));
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
    mut poke: impl FnMut(&Autotune),
) -> Vec<AutotuneEvent> {
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

fn started(flight_loop: &TestLoop, plan: &[StagePlan]) -> Autotune {
    let mut autotune = Autotune::new(plan, Envelope::default()).expect("valid plan");
    autotune.start(flight_loop).expect("armed and idle");
    autotune
}

/// The six-stage copter sequence tunes every loop in order and commits each stage's gains
/// from its own measurement; nothing else in the config changes.
#[test]
fn full_run_tunes_every_loop_in_order() {
    let (mut flight_loop, _controls) = hovering(1.0);
    let baseline = *flight_loop.stabilizer().config();
    let mut autotune = started(&flight_loop, &StagePlan::copter_sequence());

    let events = run(&mut flight_loop, &mut autotune, 300.0, |_| {});

    check!(autotune.status() == AutotuneStatus::Complete);
    let mut expected = baseline;
    let mut index = 0;
    for target in COPTER_SEQUENCE {
        check!(events.get(index) == Some(&AutotuneEvent::StageStarted(target)));
        let Some(&AutotuneEvent::StageDone {
            target: done,
            result,
            gains,
        }) = events.get(index + 1)
        else {
            panic!(
                "{target:?}: expected StageDone, got {:?}",
                events.get(index + 1)
            );
        };
        check!(done == target);
        // PD: the rule's P and D; the integral keeps the baseline's gain.
        let plan = StagePlan::copter(target);
        let rule = gains_from_ultimate(&result.point, plan.rule, plan.form);
        check!(gains.p == rule.p, "{target:?}");
        check!(gains.d == rule.d, "{target:?}");
        check!(gains.i == target.gains(&baseline).i, "{target:?}");
        *target.gains_mut(&mut expected) = gains;
        index += 2;
    }
    check!(events.get(index) == Some(&AutotuneEvent::Complete));
    check!(events.len() == index + 1);
    check!(*flight_loop.stabilizer().config() == expected);
}

/// A stage commits the rule's gains for the terms its form tunes and keeps the current gains
/// for the rest.
#[test]
fn a_stage_tunes_only_its_forms_terms() {
    let plan = StagePlan::copter(TunedLoop::RollRate);
    let result = RelayResult {
        point: crate::tuning::UltimatePoint { ku: 2.0, tu_s: 0.1 },
        relay: 0.2,
        cycle_amplitude: 0.1,
        bias: 0.0,
    };
    let current = Gains {
        p: 9.0,
        i: 8.0,
        d: 7.0,
    };
    // (form, tunes I, tunes D)
    let cases = [
        (ControllerForm::Pid, true, true),
        (ControllerForm::Pi, true, false),
        (ControllerForm::Pd, false, true),
        (ControllerForm::P, false, false),
    ];
    for (form, tunes_i, tunes_d) in cases {
        let plan = StagePlan { form, ..plan };
        let rule = gains_from_ultimate(&result.point, plan.rule, form);

        let gains = tuned_gains(&plan, &result, current);

        check!(gains.p == rule.p, "{form:?}");
        check!(
            gains.i == if tunes_i { rule.i } else { current.i },
            "{form:?}"
        );
        check!(
            gains.d == if tunes_d { rule.d } else { current.d },
            "{form:?}"
        );
    }
}

/// The relay ignores the gains of the loop it measures: from rate P at 0.3× and 3× the sane
/// value, the tuned rate gains agree within 10%.
#[test]
fn result_does_not_depend_on_the_starting_rate_gains() {
    let plan = [
        StagePlan::copter(TunedLoop::RollRate),
        StagePlan::copter(TunedLoop::PitchRate),
    ];
    let tuned = [0.3, 3.0].map(|scale| {
        let (mut flight_loop, _controls) = hovering(scale);
        let mut autotune = started(&flight_loop, &plan);
        run(&mut flight_loop, &mut autotune, 60.0, |_| {});
        check!(
            autotune.status() == AutotuneStatus::Complete,
            "scale {scale}"
        );
        flight_loop.stabilizer().config().body_rate
    });

    for (soft, stiff) in [
        (tuned[0].roll_gains, tuned[1].roll_gains),
        (tuned[0].pitch_gains, tuned[1].pitch_gains),
    ] {
        check!(
            (soft.p / stiff.p - 1.0).abs() < 0.1,
            "{soft:?} vs {stiff:?}"
        );
    }
}

/// Breaking the envelope mid-stage stops it and fails the run: the failing loop keeps its
/// gains (the stage never committed), the earlier stage's stay committed.
#[test]
fn an_envelope_breach_ends_the_run_keeping_earlier_stages() {
    let plan = [
        StagePlan::copter(TunedLoop::RollRate),
        StagePlan::copter(TunedLoop::PitchRate),
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
        let (mut flight_loop, controls) = hovering(1.0);
        let baseline = *flight_loop.stabilizer().config();
        let mut autotune = started(&flight_loop, &plan);

        let events = run(&mut flight_loop, &mut autotune, 60.0, |autotune| {
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
        check!(flight_loop.tuning() == None, "{reason:?}");
    }
}

/// Disarming before completion reverts every stage to the baseline; after completion, the
/// tuned gains stay.
#[test]
fn disarm_reverts_until_complete() {
    let plan = [
        StagePlan::copter(TunedLoop::RollRate),
        StagePlan::copter(TunedLoop::PitchRate),
    ];
    // Mid-run: disarm once the second stage runs.
    let (mut flight_loop, controls) = hovering(1.0);
    let baseline = *flight_loop.stabilizer().config();
    let mut autotune = started(&flight_loop, &plan);
    let events = run(&mut flight_loop, &mut autotune, 60.0, |autotune| {
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
    run(&mut flight_loop, &mut autotune, 60.0, |_| {});
    let tuned = *flight_loop.stabilizer().config();
    controls.arm.set(false);
    flight_loop.step(Duration::from_secs_f32(DT));
    check!(autotune.update(&mut flight_loop, DT) == None);
    check!(*flight_loop.stabilizer().config() == tuned);
}

/// A vehicle that never settles fails the run before any stage starts.
#[test]
fn a_vehicle_that_never_settles_fails_before_the_first_stage() {
    let (mut flight_loop, controls) = hovering(1.0);
    let baseline = *flight_loop.stabilizer().config();
    // Held at 0.2 rad of roll: past settle_tilt_rad, inside max_tilt_rad.
    controls.input.set(Some(ControlInput::AbsoluteAttitude(
        AbsoluteAttitudeCommand {
            roll_rad: 0.2,
            thrust: 0.5,
            ..Default::default()
        },
    )));
    let mut autotune = started(&flight_loop, &StagePlan::copter_sequence());

    let events = run(&mut flight_loop, &mut autotune, 30.0, |_| {});

    let failed = AutotuneEvent::StageFailed {
        target: TunedLoop::RollRate,
        reason: AutotuneAbort::NotSettled,
    };
    check!(events == [failed]);
    check!(*flight_loop.stabilizer().config() == baseline);
}

/// A relay abort or an interrupt fails the stage; its loop keeps its gains.
#[test]
fn relay_aborts_and_interrupts_fail_the_stage() {
    let short = StagePlan {
        relay: RelayConfig {
            timeout_s: 0.2,
            ..COPTER_RATE_RELAY
        },
        ..StagePlan::copter(TunedLoop::RollRate)
    };
    let acro = ControlInput::Rate(RateCommand {
        throttle: 0.5,
        ..Default::default()
    });
    // (plan, input once running, expected reason)
    let cases = [
        (short, None, AutotuneAbort::Relay(RelayAbort::Timeout)),
        (
            StagePlan::copter(TunedLoop::Roll),
            Some(acro),
            AutotuneAbort::Interrupted,
        ),
    ];
    for (plan, input, reason) in cases {
        let (mut flight_loop, controls) = hovering(1.0);
        let baseline = *flight_loop.stabilizer().config();
        let mut autotune = started(&flight_loop, &[plan]);

        let events = run(&mut flight_loop, &mut autotune, 30.0, |autotune| {
            if matches!(autotune.status(), AutotuneStatus::Running { .. }) {
                controls.input.set(input);
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
    let stage = StagePlan::copter(TunedLoop::RollRate);
    let bad_relay = StagePlan {
        relay: RelayConfig {
            cycles: 0,
            ..COPTER_RATE_RELAY
        },
        ..stage
    };
    let bad_envelope = Envelope {
        settle_tilt_rad: 1.0, // looser than max_tilt_rad
        ..Envelope::default()
    };
    // (plan, envelope, section, field)
    let cases: [(&[StagePlan], Envelope, &str, &str); 4] = [
        (&[], Envelope::default(), "plan", "stages"),
        (
            &[stage; MAX_STAGES + 1],
            Envelope::default(),
            "plan",
            "stages",
        ),
        (&[bad_relay], Envelope::default(), "plan.relay", "cycles"),
        (&[stage], bad_envelope, "envelope", "settle_tilt_rad"),
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
        Autotune::new(&StagePlan::copter_sequence(), Envelope::default()).expect("valid");

    check!(autotune.update(&mut flight_loop, DT) == None);
    assert!(autotune.status() == AutotuneStatus::Idle);
    check!(*flight_loop.stabilizer().config() == baseline);
}
