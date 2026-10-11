use assert2::check;

use super::*;
use crate::stabilizers::copter::TunedLoop;
use crate::tuning::{ControllerForm, RelayResult, StepMetrics, UltimatePoint, gains_from_ultimate};

const CURRENT: Gains = Gains {
    p: 9.0,
    i: 8.0,
    d: 7.0,
};

fn measured(ku: f32, tu_s: f32) -> Input {
    Input::Measured {
        measurement: Measurement::Relay(RelayResult {
            point: UltimatePoint { ku, tu_s },
            relay: 0.2,
            cycle_amplitude: 0.1,
            bias: 0.0,
        }),
        gains: CURRENT,
        stalled: false,
    }
}

/// A relay stage asks for its relay first, then finishes with the rule's gains for the
/// measured point.
#[test]
fn relay_probes_once_then_finishes_with_the_rules_gains() {
    let plan = RelayPlan::copter(TunedLoop::RollRate);
    let mut procedure = Procedure::new(Method::Relay(plan));

    let first = procedure.next(Input::Start { gains: CURRENT });
    check!(first == Action::probe(ProbeConfig::Relay(plan.relay)));

    let rule = gains_from_ultimate(&UltimatePoint { ku: 2.0, tu_s: 0.1 }, plan.rule, plan.form);
    let last = procedure.next(measured(2.0, 0.1));
    check!(
        last == Action::Finish {
            gains: Gains {
                p: rule.p,
                i: CURRENT.i,
                d: rule.d,
            },
            outcome: Outcome::Tuned,
            steps: None,
            baseline: None,
        }
    );
}

/// A relay stage sets the rule's gains for the terms its form tunes and keeps the current
/// gains for the rest.
#[test]
fn relay_tunes_only_its_forms_terms() {
    // (form, tunes I, tunes D)
    let cases = [
        (ControllerForm::Pid, true, true),
        (ControllerForm::Pi, true, false),
        (ControllerForm::Pd, false, true),
        (ControllerForm::P, false, false),
    ];
    for (form, tunes_i, tunes_d) in cases {
        let plan = RelayPlan {
            form,
            ..RelayPlan::copter(TunedLoop::RollRate)
        };
        let rule = gains_from_ultimate(&UltimatePoint { ku: 2.0, tu_s: 0.1 }, plan.rule, form);
        let mut procedure = Procedure::new(Method::Relay(plan));

        let Action::Finish { gains, .. } = procedure.next(measured(2.0, 0.1)) else {
            panic!("{form:?}: no gains");
        };

        check!(gains.p == rule.p, "{form:?}");
        check!(
            gains.i == if tunes_i { rule.i } else { CURRENT.i },
            "{form:?}"
        );
        check!(
            gains.d == if tunes_d { rule.d } else { CURRENT.d },
            "{form:?}"
        );
    }
}

/// A step's measurement: `saturated_s` clipped, `stalled` or not; its overshoot tells steps
/// apart.
fn stepped(saturated_s: f32, stalled: bool) -> Input {
    stepped_with(saturated_s, stalled, 0.0)
}

fn stepped_with(saturated_s: f32, stalled: bool, overshoot: f32) -> Input {
    Input::Measured {
        measurement: Measurement::Step(StepMetrics {
            saturated_s,
            overshoot,
            ..Default::default()
        }),
        gains: CURRENT,
        stalled,
    }
}

fn step_test(pairs: usize) -> (Procedure, StepTestPlan) {
    let plan = StepTestPlan {
        pairs,
        ..StepTestPlan::copter(TunedLoop::RollRate)
    };
    (Procedure::new(Method::StepTest(plan)), plan)
}

/// A step test alternates `+step` and `−step` for its pairs, then finishes with the gains it
/// flew (no change) and a summary of its valid steps only.
#[test]
fn step_test_alternates_then_finishes_with_the_same_gains() {
    let (mut procedure, plan) = step_test(2);
    let up = Action::probe(ProbeConfig::Step(plan.step));
    let down = Action::probe(ProbeConfig::Step(plan.step.reversed()));
    let clipped = plan.max_saturated_s + 0.01;

    let mut actions = vec![procedure.next(Input::Start { gains: CURRENT })];
    actions.push(procedure.next(stepped_with(clipped, false, 9.0))); // invalid: left out
    for overshoot in [0.1, 0.2, 0.3, 0.4] {
        actions.push(procedure.next(stepped_with(0.0, false, overshoot)));
    }

    let Some(Action::Finish {
        gains,
        outcome: Outcome::Measured,
        steps: Some(steps),
        baseline: None,
    }) = actions.pop()
    else {
        panic!("no summary: {actions:?}");
    };
    check!(actions == [up, up, down, up, down]);
    check!(gains == CURRENT);
    check!(steps.count() == 4);
    check!(steps.overshoot.max == 0.4);
}

/// A clipped or stalled step runs again in the same direction; `max_invalid` in a row fail the
/// stage, and a valid step resets the count.
#[test]
fn step_test_repeats_invalid_steps_and_fails_on_too_many() {
    let (mut procedure, plan) = step_test(3);
    let up = Action::probe(ProbeConfig::Step(plan.step));
    let down = Action::probe(ProbeConfig::Step(plan.step.reversed()));
    let clipped = plan.max_saturated_s + 0.01;

    procedure.next(Input::Start { gains: CURRENT });
    // (input, expected action)
    let script = [
        (stepped(plan.max_saturated_s, false), down), // at the limit: valid
        (stepped(clipped, false), down),
        (stepped(0.0, true), down),
        (stepped(0.0, false), up), // valid: count reset
        (stepped(clipped, false), up),
        (stepped(clipped, false), up),
        (stepped(0.0, true), Action::Fail(AutotuneAbort::Invalid)),
    ];
    check!(plan.max_invalid == 3);
    for (index, (input, expected)) in script.into_iter().enumerate() {
        check!(procedure.next(input) == expected, "step {index}");
    }
}

/// A measurement the method didn't ask for fails the stage.
#[test]
fn a_mismatched_measurement_fails() {
    let (mut step, _) = step_test(1);
    check!(step.next(measured(2.0, 0.1)) == Action::Fail(AutotuneAbort::Interrupted));
    let mut relay = Procedure::new(Method::Relay(RelayPlan::copter(TunedLoop::RollRate)));
    check!(relay.next(stepped(0.0, false)) == Action::Fail(AutotuneAbort::Interrupted));
}

/// A relay or a step test fails on a probe abort, and judges nothing (no verdicts).
#[test]
fn relay_and_step_test_fail_on_an_abort() {
    let abort = ProbeAbort::Step(crate::tuning::StepAbort::Runaway);
    let (step, _) = step_test(1);
    let relay = Procedure::new(Method::Relay(RelayPlan::copter(TunedLoop::RollRate)));
    for mut procedure in [step, relay] {
        procedure.next(Input::Start { gains: CURRENT });
        let failed = procedure.next(Input::Aborted {
            abort,
            gains: CURRENT,
        });
        check!(failed == Action::Fail(AutotuneAbort::Probe(abort)));
        check!(procedure.take_verdict() == None);
    }
}
