use assert2::check;

use super::*;
use crate::stabilizers::copter::TunedLoop;
use crate::tuning::autotune::DesignPlan;
use crate::tuning::{
    ControllerForm, DesignError, IdentifyAbort, RateModel, RelayResult, StepMetrics, UltimatePoint,
    design_angle_p, design_rate, gains_from_ultimate,
};

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

    let first = procedure.next(Input::start(CURRENT));
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

    let mut actions = vec![procedure.next(Input::start(CURRENT))];
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

/// A clipped or stalled step runs again in the same direction. `max_invalid` clipped steps in
/// a row fail the stage, and a valid step resets the count; stalls don't count toward it, but
/// `max_stalled` of them in a row fail the stage too.
#[test]
fn step_test_repeats_invalid_steps_and_fails_on_too_many() {
    let (mut procedure, plan) = step_test(3);
    let up = Action::probe(ProbeConfig::Step(plan.step));
    let down = Action::probe(ProbeConfig::Step(plan.step.reversed()));
    let clipped = plan.max_saturated_s + 0.01;

    procedure.next(Input::start(CURRENT));
    // (input, expected action)
    let script = [
        (stepped(plan.max_saturated_s, false), down), // at the limit: valid
        (stepped(clipped, false), down),
        (stepped(0.0, true), down), // a stall: doesn't count as clipped
        (stepped(clipped, false), down),
        (stepped(0.0, false), up), // valid: count reset
        (stepped(clipped, false), up),
        (stepped(clipped, false), up),
        (
            stepped(clipped, false),
            Action::Fail(AutotuneAbort::Invalid),
        ),
    ];
    check!(plan.max_invalid == 3);
    for (index, (input, expected)) in script.into_iter().enumerate() {
        check!(procedure.next(input) == expected, "step {index}");
    }

    let (mut procedure, plan) = step_test(3);
    procedure.next(Input::start(CURRENT));
    for index in 1..plan.max_stalled {
        check!(procedure.next(stepped(0.0, true)) == up, "stall {index}");
    }
    check!(procedure.next(stepped(0.0, true)) == Action::Fail(AutotuneAbort::Stalled));
}

/// A probe a slow host stopped, unmeasured: a relay runs again, `MAX_MEASURE_STOPS` in a row
/// failing the stage; a step test repeats the step
/// in the same direction, counting it against `max_stalled` along with stalled steps.
#[test]
fn a_stopped_probe_runs_again() {
    let plan = RelayPlan::copter(TunedLoop::RollRate);
    let mut procedure = Procedure::new(Method::Relay(plan));
    let relay = procedure.next(Input::start(CURRENT));
    check!(procedure.next(Input::Stalled { gains: CURRENT }) == relay);
    check!(matches!(
        procedure.next(measured(2.0, 0.1)),
        Action::Finish { .. }
    ));
    // `MAX_MEASURE_STOPS` in a row fail the stage: a host that keeps flapping mid-relay.
    let mut procedure = Procedure::new(Method::Relay(plan));
    procedure.next(Input::start(CURRENT));
    for stop in 1..MAX_MEASURE_STOPS {
        check!(
            procedure.next(Input::Stalled { gains: CURRENT }) == relay,
            "stop {stop}"
        );
    }
    check!(
        procedure.next(Input::Stalled { gains: CURRENT }) == Action::Fail(AutotuneAbort::Stalled)
    );

    let (mut procedure, plan) = step_test(3);
    let down = Action::probe(ProbeConfig::Step(plan.step.reversed()));
    procedure.next(Input::start(CURRENT));
    check!(procedure.next(stepped(0.0, false)) == down);
    check!(procedure.next(Input::Stalled { gains: CURRENT }) == down);
    for index in 2..plan.max_stalled {
        check!(procedure.next(stepped(0.0, true)) == down, "stall {index}");
    }
    check!(
        procedure.next(Input::Stalled { gains: CURRENT }) == Action::Fail(AutotuneAbort::Stalled)
    );
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
        procedure.next(Input::start(CURRENT));
        let failed = procedure.next(Input::Aborted {
            abort,
            gains: CURRENT,
        });
        check!(failed == Action::Fail(AutotuneAbort::Probe(abort)));
        check!(procedure.take_verdict() == None);
    }
}

/// The copter step tests count a step clipped for their allowance in 250 Hz ticks as valid
/// (rate loops 10, angle loops 5), summed in `f32` as the probe does (five 4 ms ticks come to
/// slightly over 20 ms); one more tick is too many.
#[test]
fn clipped_ticks_within_the_allowance_stay_valid() {
    // (loop, ticks allowed)
    let cases = [(TunedLoop::RollRate, 10), (TunedLoop::Roll, 5)];
    for (target, ticks) in cases {
        let plan = StepTestPlan::copter(target);
        let clipped = |ticks| (0..ticks).fold(0.0_f32, |sum, _| sum + 0.004);
        let up = Action::probe(ProbeConfig::Step(plan.step));
        let down = Action::probe(ProbeConfig::Step(plan.step.reversed()));

        let mut procedure = Procedure::new(Method::StepTest(plan));
        procedure.next(Input::start(CURRENT));
        check!(
            procedure.next(stepped(clipped(ticks), false)) == down,
            "{target:?}"
        );
        check!(
            procedure.next(stepped(clipped(ticks + 1), false)) == down,
            "{target:?}: reruns"
        );
        check!(procedure.next(stepped(0.0, false)) == up, "{target:?}");
    }
}

/// A relay or a step test never flies a candidate; a breach reported to one fails it.
#[test]
fn relay_and_step_test_fail_on_a_breach() {
    let (step, _) = step_test(1);
    let relay = Procedure::new(Method::Relay(RelayPlan::copter(TunedLoop::RollRate)));
    for mut procedure in [step, relay] {
        procedure.next(Input::start(CURRENT));
        let failed = procedure.next(Input::Breached {
            reason: AutotuneAbort::Tilt,
            gains: CURRENT,
        });
        check!(failed == Action::Fail(AutotuneAbort::Tilt));
    }
}

/// `quad_x`'s roll fit on the sim: a 30 ms lag, one tick of delay.
const QUAD_X_FIT: RateModel = RateModel {
    rate_weights: [1.87437, -0.87447],
    moment_weights: [0.0136, 0.0, 0.0],
    delay_ticks: 1,
    dt_s: 0.004,
};

fn fitted(model: RateModel, stalled: bool) -> Input {
    Input::Measured {
        measurement: Measurement::Identify(model),
        gains: CURRENT,
        stalled,
    }
}

/// `QUAD_X_FIT` with its moment scaled by `scale`: a plant `scale` times as responsive, so
/// its design's P is about `1 / scale` of `QUAD_X_FIT`'s.
fn scaled(scale: f32) -> RateModel {
    RateModel {
        moment_weights: QUAD_X_FIT.moment_weights.map(|weight| weight * scale),
        ..QUAD_X_FIT
    }
}

/// A fit the design can't use: its pole is no lag.
const NO_LAG: RateModel = RateModel {
    rate_weights: [1.0, 0.0],
    ..QUAD_X_FIT
};

/// Feeds a rate loop's design stage `fits` in turn, until it ends or runs out of them.
fn design_with(fits: &[RateModel]) -> Action {
    let plan = DesignPlan::copter();
    let mut procedure = Procedure::new(Method::Design(plan));
    let mut action = procedure.next(Input::start(CURRENT));
    for model in fits {
        check!(action == Action::probe(ProbeConfig::Identify(plan.identify)));
        action = procedure.next(fitted(*model, false));
    }
    action
}

fn tuned(gains: Gains) -> Action {
    Action::Finish {
        gains,
        outcome: Outcome::Tuned,
        steps: None,
        baseline: None,
    }
}

/// A design stage finishes once two fits design P within 25% of each other, on the lower P;
/// an unusable fit or one that agrees with no other fits again, and 4 fits without a pair
/// fail the stage with the last fit's reason.
#[test]
fn design_finishes_on_two_fits_that_agree() {
    let design = DesignPlan::copter().design;
    let rate = |model: RateModel| design_rate(&model, CURRENT, &design).expect("rate gains");
    let (base, near, far) = (scaled(1.0), scaled(1.2), scaled(1.6));
    check!(rate(near).p < rate(base).p);
    let identify = Action::probe(ProbeConfig::Identify(DesignPlan::copter().identify));
    // (case, fits, end)
    let rows = [
        ("one fit fits again", vec![base], identify),
        ("two alike", vec![base, base], tuned(rate(base))),
        (
            "within 25%: the lower P, fitted last",
            vec![base, near],
            tuned(rate(near)),
        ),
        (
            "within 25%: the lower P, fitted first",
            vec![near, base],
            tuned(rate(near)),
        ),
        (
            "an outlier between",
            vec![base, far, base],
            tuned(rate(base)),
        ),
        (
            "an unusable fit between",
            vec![base, NO_LAG, base],
            tuned(rate(base)),
        ),
        (
            "no two agree",
            vec![scaled(1.0), scaled(1.6), scaled(2.6), scaled(4.2)],
            Action::Fail(AutotuneAbort::FitsDisagree),
        ),
        (
            "out of fits on an unusable one",
            vec![base, far, scaled(2.6), NO_LAG],
            Action::Fail(AutotuneAbort::Design(DesignError::NotALag)),
        ),
    ];
    for (case, fits, end) in rows {
        check!(design_with(&fits) == end, "{case}");
    }
}

/// An angle loop's design stage designs P on the rate gains under it, with its own I and D.
#[test]
fn design_designs_an_angle_p_on_the_rate_loop_under_it() {
    let plan = DesignPlan::copter();
    let rate = design_rate(&QUAD_X_FIT, CURRENT, &plan.design).expect("rate gains");
    let angle_p = design_angle_p(&QUAD_X_FIT, rate, CURRENT, &plan.design).expect("angle P");
    let mut procedure = Procedure::new(Method::Design(plan));
    procedure.next(Input::Start {
        gains: CURRENT,
        inner: Some(rate),
        run_start: CURRENT,
    });
    procedure.next(fitted(QUAD_X_FIT, false));
    check!(
        procedure.next(fitted(QUAD_X_FIT, false))
            == tuned(Gains {
                p: angle_p,
                ..CURRENT
            })
    );
}

/// An aborted fit fails the stage with the probe's abort.
#[test]
fn design_fails_on_an_aborted_fit() {
    let abort = ProbeAbort::Identify(IdentifyAbort::Unexcited);
    let mut procedure = Procedure::new(Method::Design(DesignPlan::copter()));
    procedure.next(Input::start(CURRENT));
    check!(
        procedure.next(Input::Aborted {
            abort,
            gains: CURRENT
        }) == Action::Fail(AutotuneAbort::Probe(abort))
    );
}

/// A fit a slow host stopped or spoiled runs again, `MAX_MEASURE_STOPS` in a row failing the
/// stage.
#[test]
fn design_fits_again_after_a_stall() {
    let plan = DesignPlan::copter();
    let identify = Action::probe(ProbeConfig::Identify(plan.identify));
    let mut procedure = Procedure::new(Method::Design(plan));
    procedure.next(Input::start(CURRENT));
    check!(procedure.next(fitted(QUAD_X_FIT, true)) == identify);
    for stop in 2..MAX_MEASURE_STOPS {
        check!(
            procedure.next(Input::Stalled { gains: CURRENT }) == identify,
            "stop {stop}"
        );
    }
    check!(
        procedure.next(Input::Stalled { gains: CURRENT }) == Action::Fail(AutotuneAbort::Stalled)
    );
}

/// Stalls count neither as fits nor, once a fit the host kept time for ends them, toward the
/// next run of stalls.
#[test]
fn a_kept_fit_ends_a_run_of_stalls() {
    let plan = DesignPlan::copter();
    let identify = Action::probe(ProbeConfig::Identify(plan.identify));
    let mut procedure = Procedure::new(Method::Design(plan));
    procedure.next(Input::start(CURRENT));
    for fit in [scaled(1.0), scaled(1.6), scaled(2.6)] {
        for _ in 1..MAX_MEASURE_STOPS {
            check!(procedure.next(Input::Stalled { gains: CURRENT }) == identify);
        }
        check!(procedure.next(fitted(fit, false)) == identify);
    }
    check!(procedure.next(fitted(scaled(1.0), false)) != identify);
}
