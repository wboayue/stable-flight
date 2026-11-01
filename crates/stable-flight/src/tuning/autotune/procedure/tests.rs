use assert2::check;

use super::*;
use crate::controllers::RATE_I_PER_P;
use crate::stabilizers::copter::TunedLoop;
use crate::tuning::autotune::DesignPlan;
use crate::tuning::{
    DesignError, IdentifyAbort, RateModel, StepMetrics, design_angle_p, design_rate,
};

const CURRENT: Gains = Gains {
    p: 9.0,
    i: 8.0,
    d: 7.0,
};

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

/// A step a slow host stopped runs again in the same direction, counting against
/// `max_stalled` along with stalled steps.
#[test]
fn a_stopped_step_runs_again() {
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
    step.next(Input::start(CURRENT));
    check!(step.next(fitted(QUAD_X_FIT, false)) == Action::Fail(AutotuneAbort::Interrupted));
    let mut design = Procedure::new(Method::Design(DesignPlan::copter()));
    design.next(Input::start(CURRENT));
    check!(design.next(stepped(0.0, false)) == Action::Fail(AutotuneAbort::Interrupted));
}

/// A step test or a design fails on a probe abort, and judges nothing (no verdicts).
#[test]
fn step_test_and_design_fail_on_an_abort() {
    let abort = ProbeAbort::Step(crate::tuning::StepAbort::Runaway);
    let (step, _) = step_test(1);
    let design = Procedure::new(Method::Design(DesignPlan::copter()));
    for mut procedure in [step, design] {
        procedure.next(Input::start(CURRENT));
        let failed = procedure.next(Input::Aborted {
            abort,
            gains: CURRENT,
        });
        check!(failed == Action::Fail(AutotuneAbort::Probe(abort)));
        check!(procedure.take_verdict() == None);
    }
}

/// A step test or a design whose committed gains won't settle before a probe fails
/// `NotSettled`: only a search lowers its start.
#[test]
fn step_test_and_design_fail_when_unsettled() {
    let (step, _) = step_test(1);
    let design = Procedure::new(Method::Design(DesignPlan::copter()));
    for mut procedure in [step, design] {
        procedure.next(Input::start(CURRENT));
        let failed = procedure.next(Input::Unsettled { gains: CURRENT });
        check!(failed == Action::Fail(AutotuneAbort::NotSettled));
    }
}

/// The copter step tests count a step clipped for 10 ticks at 250 Hz as valid on every loop,
/// summed in `f32` as the probe does (ten 4 ms ticks come to slightly over 40 ms); one more
/// tick is too many. An angle step at the tuned angle P clips for its first ticks (#310).
#[test]
fn clipped_ticks_within_the_allowance_stay_valid() {
    let ticks = 10;
    for target in [TunedLoop::RollRate, TunedLoop::Roll, TunedLoop::Heading] {
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

/// A step test never flies a candidate; a breach reported to one fails it.
#[test]
fn step_test_fails_on_a_breach() {
    let (mut procedure, _) = step_test(1);
    procedure.next(Input::start(CURRENT));
    let failed = procedure.next(Input::Breached {
        reason: AutotuneAbort::Tilt,
        gains: CURRENT,
    });
    check!(failed == Action::Fail(AutotuneAbort::Tilt));
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

/// The rate gains `plan` designs from [`QUAD_X_FIT`]: P and D by `design_rate`, I at
/// [`RATE_I_PER_P`] of P.
fn designed_rate(plan: &DesignPlan) -> Gains {
    let rate = design_rate(&QUAD_X_FIT, CURRENT, DEFAULT_GYRO_CUTOFF_HZ, &plan.design)
        .expect("rate gains");
    Gains {
        i: RATE_I_PER_P * rate.p,
        ..rate
    }
}

/// A design stage fits once, holds the designed gains, then finishes with them: a rate
/// loop's P and D from the fit with I at [`RATE_I_PER_P`] of P, an angle loop's P on the rate
/// loop under it, with its own I and D.
#[test]
fn design_fits_holds_then_finishes_with_the_designed_gains() {
    let plan = DesignPlan::copter();
    let rate = designed_rate(&plan);
    let angle_p = design_angle_p(
        &QUAD_X_FIT,
        rate,
        CURRENT,
        DEFAULT_GYRO_CUTOFF_HZ,
        &plan.design,
    )
    .expect("angle P");
    // (case, inner gains, designed)
    let rows = [
        ("rate", None, rate),
        (
            "angle",
            Some(rate),
            Gains {
                p: angle_p,
                ..CURRENT
            },
        ),
    ];
    for (case, inner, designed) in rows {
        let mut procedure = Procedure::new(Method::Design(plan));
        let first = procedure.next(Input::Start {
            gains: CURRENT,
            inner,
            run_start: CURRENT,
            gyro_cutoff_hz: DEFAULT_GYRO_CUTOFF_HZ,
        });
        check!(
            first == Action::probe(ProbeConfig::Identify(plan.identify)),
            "{case}"
        );
        check!(
            procedure.next(fitted(QUAD_X_FIT, false)) == Action::Hold { gains: designed },
            "{case}"
        );
        check!(
            procedure.next(held(true))
                == Action::Finish {
                    gains: designed,
                    outcome: Outcome::Designed,
                    steps: None,
                    baseline: None,
                },
            "{case}"
        );
    }
}

fn held(settled: bool) -> Input {
    Input::Held {
        settled,
        gains: CURRENT,
    }
}

/// A fit the design can't use, or whose design doesn't hold the settle or breaches the
/// envelope, runs again, the third failing the stage with its reason; an aborted fit fails it
/// with the probe's.
#[test]
fn design_fits_again_after_an_unusable_fit_or_design() {
    let plan = DesignPlan::copter();
    let identify = Action::probe(ProbeConfig::Identify(plan.identify));
    let no_lag = fitted(
        RateModel {
            rate_weights: [2.2, -1.2],
            ..QUAD_X_FIT
        },
        false,
    );
    let usable = fitted(QUAD_X_FIT, false);
    let breached = Input::Breached {
        reason: AutotuneAbort::Rate,
        gains: CURRENT,
    };
    let rate = designed_rate(&plan);
    // (case, inputs after the start, end)
    let rows = [
        (
            "an unusable fit, then a design that holds",
            vec![no_lag, usable, held(true)],
            Action::Finish {
                gains: rate,
                outcome: Outcome::Designed,
                steps: None,
                baseline: None,
            },
        ),
        ("two unusable fits", vec![no_lag, no_lag], identify),
        (
            "three unusable fits",
            vec![no_lag, no_lag, no_lag],
            Action::Fail(AutotuneAbort::Design(DesignError::NotALag)),
        ),
        (
            "a design that doesn't hold",
            vec![usable, held(false)],
            identify,
        ),
        (
            "three designs that don't hold",
            vec![
                usable,
                held(false),
                usable,
                held(false),
                usable,
                held(false),
            ],
            Action::Fail(AutotuneAbort::NotSettled),
        ),
        ("a design that breaches", vec![usable, breached], identify),
        (
            "a design that breaches, then two unusable fits",
            vec![usable, breached, no_lag, no_lag],
            Action::Fail(AutotuneAbort::Design(DesignError::NotALag)),
        ),
    ];
    for (case, inputs, end) in rows {
        let mut procedure = Procedure::new(Method::Design(plan));
        let mut action = procedure.next(Input::start(CURRENT));
        for input in inputs {
            action = procedure.next(input);
        }
        check!(action == end, "{case}");
    }

    let abort = ProbeAbort::Identify(IdentifyAbort::Unexcited);
    let mut procedure = Procedure::new(Method::Design(plan));
    procedure.next(Input::start(CURRENT));
    check!(
        procedure.next(Input::Aborted {
            abort,
            gains: CURRENT
        }) == Action::Fail(AutotuneAbort::Probe(abort))
    );
}

/// A rate design whose fit saturates under noise lowers the start, every gain `÷ 1.3`, commits
/// it, and fits again on it, three times; the fourth fails the stage, as does the first in an
/// angle stage (#551).
#[test]
fn design_lowers_a_start_whose_fit_saturates() {
    let plan = DesignPlan::copter();
    let saturated = ProbeAbort::Identify(IdentifyAbort::Saturated);
    let mut procedure = Procedure::new(Method::Design(plan));
    procedure.next(Input::start(CURRENT));
    let mut gains = CURRENT;
    for lowering in 1..=3 {
        let lowered = Gains {
            p: gains.p / 1.3,
            i: gains.i / 1.3,
            d: gains.d / 1.3,
        };
        let action = procedure.next(Input::Aborted {
            abort: saturated,
            gains,
        });
        let expected = Action::Probe {
            probe: ProbeConfig::Identify(plan.identify),
            gains: None,
            commit: Some(lowered),
        };
        check!(action == expected, "lowering {lowering}");
        gains = lowered;
    }
    let action = procedure.next(Input::Aborted {
        abort: saturated,
        gains,
    });
    check!(action == Action::Fail(AutotuneAbort::Probe(saturated)));

    // An angle stage's clipped moment is its rate loop's: it fails rather than lower its own.
    let mut angle = Procedure::new(Method::Design(plan));
    angle.next(Input::Start {
        gains: CURRENT,
        inner: Some(CURRENT),
        run_start: CURRENT,
        gyro_cutoff_hz: DEFAULT_GYRO_CUTOFF_HZ,
    });
    let action = angle.next(Input::Aborted {
        abort: saturated,
        gains: CURRENT,
    });
    check!(action == Action::Fail(AutotuneAbort::Probe(saturated)));
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

/// A fit that ran, even one the design can't use, clears the stall count: stalls fail the
/// stage only in a row.
#[test]
fn design_counts_only_stalls_in_a_row() {
    let plan = DesignPlan::copter();
    let identify = Action::probe(ProbeConfig::Identify(plan.identify));
    let no_lag = RateModel {
        rate_weights: [2.2, -1.2],
        ..QUAD_X_FIT
    };
    let mut procedure = Procedure::new(Method::Design(plan));
    procedure.next(Input::start(CURRENT));
    for stop in 1..MAX_MEASURE_STOPS {
        check!(
            procedure.next(Input::Stalled { gains: CURRENT }) == identify,
            "stop {stop}"
        );
    }
    check!(procedure.next(fitted(no_lag, false)) == identify);
    for stop in 1..MAX_MEASURE_STOPS {
        check!(
            procedure.next(Input::Stalled { gains: CURRENT }) == identify,
            "stop {stop} after the fit"
        );
    }
}
