use std::cell::RefCell;

use assert2::{assert, check};

use super::*;
use crate::tuning::autotune::{Criterion, Method, Procedure, Term};
use crate::tuning::{
    RelayAbort, RelayResult, StepAbort, StepMetrics, UltimatePoint, gains_from_ultimate,
};

/// A hand-tune-like rate loop.
const START: Gains = Gains {
    p: 0.5,
    i: 0.2,
    d: 0.004,
};

/// A rate loop's ultimate point (QuadCopterX's, about).
const POINT: UltimatePoint = UltimatePoint {
    ku: 2.0,
    tu_s: 0.07,
};

/// The most probes a test lets a search ask for.
const MAX_PROBES: usize = 200;

/// A step's metrics; settling at 0.1 s.
fn metrics(overshoot: f32, bounce_back: f32) -> StepMetrics {
    StepMetrics {
        overshoot,
        bounce_back,
        settling_time_s: Some(0.1),
        ..Default::default()
    }
}

/// A relay measurement of `point`.
fn relay_input(point: UltimatePoint, stalled: bool) -> Input {
    Input::Measured {
        measurement: Measurement::Relay(RelayResult {
            point,
            relay: 0.2,
            cycle_amplitude: 0.1,
            bias: 0.0,
        }),
        gains: START,
        stalled,
    }
}

fn relay_abort() -> Input {
    Input::Aborted {
        abort: ProbeAbort::Relay(RelayAbort::Timeout),
        gains: START,
    }
}

/// A scripted relay reply.
type Reply = fn() -> Input;

/// What a scripted search did.
#[derive(Debug)]
struct Run {
    /// The gains each step flew, in order.
    flown: Vec<Gains>,
    /// The verdicts, in order.
    verdicts: Vec<Verdict>,
    /// How it ended.
    end: Action,
}

impl Run {
    /// The searched term's gain on each judged bracket point (the baseline first).
    fn tried(&self, term: Term) -> Vec<f32> {
        self.verdicts.iter().map(|v| term.get(v.gains)).collect()
    }
}

/// Runs `plan` from [`START`]: relays get `relays` in turn, steps `step(gains flown)`
/// (`None`: the step aborts).
fn drive(
    plan: SearchPlan,
    mut relays: impl FnMut() -> Input,
    step: impl Fn(Gains) -> Option<StepMetrics>,
) -> Run {
    let mut procedure = Procedure::new(Method::Search(plan));
    let mut flown = Vec::new();
    let mut verdicts = Vec::new();
    let mut action = procedure.next(Input::Start { gains: START });
    for _ in 0..MAX_PROBES {
        verdicts.extend(procedure.take_verdict());
        let input = match action {
            Action::Probe {
                probe: ProbeConfig::Relay(_),
                gains,
            } => {
                assert!(gains.is_none());
                relays()
            }
            Action::Probe {
                probe: ProbeConfig::Step(_),
                gains,
            } => {
                let gains = gains.unwrap_or(START);
                flown.push(gains);
                match step(gains) {
                    Some(metrics) => Input::Measured {
                        measurement: Measurement::Step(metrics),
                        gains: START,
                        stalled: false,
                    },
                    None => Input::Aborted {
                        abort: ProbeAbort::Step(StepAbort::Runaway),
                        gains: START,
                    },
                }
            }
            end => {
                return Run {
                    flown,
                    verdicts,
                    end,
                };
            }
        };
        action = procedure.next(input);
    }
    panic!("no end after {MAX_PROBES} probes");
}

/// A D search without a relay, judged on bounce-back alone: it starts from [`START`].
fn d_plan() -> SearchPlan {
    SearchPlan {
        term: Term::D,
        criterion: Criterion::Overshoot {
            max_overshoot: 1.0,
            max_bounce_back: 0.1,
        },
        min: 0.001,
        max: 0.03,
        relay: None,
        ..SearchPlan::copter_rate()
    }
}

/// Bounce-back grows with D past `threshold` (0.1 at it).
fn damping(threshold: f32) -> impl Fn(Gains) -> Option<StepMetrics> {
    move |gains| Some(metrics(0.0, 0.1 * gains.d / threshold))
}

fn no_relays() -> Input {
    panic!("no ceiling planned")
}

/// The gains a run finished with, and how.
fn finished(run: &Run) -> (Gains, Outcome) {
    let Action::Finish { gains, outcome, .. } = run.end else {
        panic!("not finished: {:?}", run.end);
    };
    (gains, outcome)
}

/// From a passing or failing start, the search brackets and bisects to within `resolution` of
/// the threshold, then verifies `margin ×` its largest pass and keeps it.
#[test]
fn search_converges_below_the_threshold() {
    let plan = d_plan();
    // (threshold, start passes)
    let cases = [(0.01, true), (0.02, true), (0.003, false), (0.0015, false)];
    for (threshold, start_passes) in cases {
        let run = drive(plan, no_relays, damping(threshold));

        let (gains, outcome) = finished(&run);
        check!(outcome == Outcome::Tuned, "{threshold}");
        let lowest = plan.margin * threshold / plan.resolution;
        let highest = plan.margin * threshold;
        check!(
            (lowest..=highest).contains(&gains.d),
            "{threshold}: {}",
            gains.d
        );
        check!(
            gains
                == Gains {
                    d: gains.d,
                    ..START
                },
            "{threshold}: other terms kept"
        );
        check!(run.verdicts[0].passed == start_passes, "{threshold}");
        check!(
            run.verdicts.len() <= 20,
            "{threshold}: {:?}",
            run.tried(Term::D)
        );
    }
}

/// The baseline flies the committed gains; bracket points and the verify fly the candidate,
/// one pair each for the bracket, `test.pairs` for the baseline and verify.
#[test]
fn only_candidates_fly_new_gains() {
    let plan = d_plan();
    let run = drive(plan, no_relays, damping(0.01));

    let baseline = 2 * plan.test.pairs;
    check!(run.flown[..baseline].iter().all(|gains| *gains == START));
    // Each bracket point is one pair on one gain.
    let tried = run.tried(Term::D);
    for (index, gain) in tried.iter().enumerate().skip(1) {
        let first = baseline + 2 * (index - 1);
        check!(run.flown[first].d == *gain);
        check!(run.flown[first + 1].d == *gain);
    }
    let (gains, _) = finished(&run);
    let verify = &run.flown[run.flown.len() - baseline..];
    check!(verify.iter().all(|flown| *flown == gains));
    check!(run.flown.len() == baseline + 2 * (tried.len() - 1) + baseline);
}

/// Nothing passes down to `min`: the start gains stay.
#[test]
fn no_pass_keeps_the_start() {
    let plan = d_plan();
    let run = drive(plan, no_relays, |_| Some(metrics(0.0, 1.0)));

    check!(finished(&run) == (START, Outcome::Kept(KeepReason::NoPass)));
    let tried = run.tried(Term::D);
    check!(tried.last() == Some(&plan.min));
    check!(
        tried.windows(2).all(|pair| pair[1] < pair[0]),
        "down only: {tried:?}"
    );
}

/// A step that aborts fails its pair, like a bad response.
#[test]
fn an_aborted_step_fails_its_pair() {
    let threshold = 0.01;
    let run = drive(d_plan(), no_relays, |gains| {
        (gains.d <= threshold).then(|| metrics(0.0, 0.0))
    });

    let (gains, outcome) = finished(&run);
    check!(outcome == Outcome::Tuned);
    check!(gains.d <= d_plan().margin * threshold);
    check!(
        run.verdicts
            .iter()
            .all(|v| v.passed == (v.gains.d <= threshold))
    );
}

/// A result that flies worse than the baseline (here: slower to settle) leaves the start.
#[test]
fn a_worse_result_keeps_the_start() {
    let run = drive(d_plan(), no_relays, |gains| {
        Some(StepMetrics {
            settling_time_s: Some(if gains == START { 0.1 } else { 0.2 }),
            ..metrics(0.0, 0.0)
        })
    });

    check!(finished(&run) == (START, Outcome::Kept(KeepReason::Worse)));
    let Action::Finish {
        steps: Some(verify),
        baseline: Some(baseline),
        ..
    } = run.end
    else {
        panic!("no summaries: {:?}", run.end);
    };
    check!(verify.settling_time_s.mean() == Some(0.2));
    check!(baseline.settling_time_s.mean() == Some(0.1));
}

/// The rate P search runs its relay first: the rule's D and P seed it (the bracket starts at
/// the rule's P, the baseline flies the start gains and isn't a bracket point), and it never
/// tries above `ceiling · Ku`.
#[test]
fn the_relay_seeds_and_caps_the_search() {
    let plan = SearchPlan::copter_rate();
    let Some(relay) = plan.relay else {
        panic!("the rate search has a relay");
    };
    let point = POINT;
    let rule = gains_from_ultimate(&point, relay.plan.rule, relay.plan.form);
    let cap = relay.ceiling * point.ku;
    let run = drive(
        plan,
        || relay_input(point, false),
        |_| Some(metrics(0.0, 0.0)),
    );

    check!(run.verdicts[0].gains == START);
    let tried = run.tried(Term::P);
    check!(tried[1] == rule.p);
    check!(tried.iter().all(|gain| *gain <= cap), "{tried:?}");
    check!(tried.last() == Some(&cap));
    let (gains, outcome) = finished(&run);
    check!(outcome == Outcome::Tuned);
    check!(
        gains
            == Gains {
                p: plan.margin * cap,
                i: START.i,
                d: rule.d
            }
    );
}

/// A relay that stalls or aborts runs again; after `max_invalid` failures the search goes on
/// from the start gains, without a ceiling.
#[test]
fn a_failing_relay_is_retried_then_skipped() {
    let plan = SearchPlan::copter_rate();
    let point = POINT;
    let good = || relay_input(POINT, false);
    let stalled = || relay_input(POINT, true);
    // (relay replies in order, relay used)
    let cases: [(&[Reply], bool); 3] = [
        (&[stalled, good], true),
        (&[relay_abort, good], true),
        (&[stalled, relay_abort, stalled], false),
    ];
    check!(plan.test.max_invalid == 3);
    for (index, (replies, used)) in cases.into_iter().enumerate() {
        let mut replies = replies.iter();
        let run = drive(
            plan,
            || replies.next().expect("no more relays than scripted")(),
            |_| Some(metrics(0.0, 0.0)),
        );

        let (gains, _) = finished(&run);
        let top = run.tried(Term::P).into_iter().fold(0.0, f32::max);
        if used {
            check!(top == 0.6 * point.ku, "case {index}");
            check!(gains.d != START.d, "case {index}: the rule's D");
        } else {
            check!(top == plan.max, "case {index}");
            check!(gains.d == START.d, "case {index}: the start's D");
        }
    }
}

/// A gain that passes above one that failed (noise) can't widen the bracket: the search still
/// ends, within the first bracket.
#[test]
fn a_non_monotone_response_stays_bounded() {
    // Fails only in a narrow band below a higher passing region.
    let run = drive(d_plan(), no_relays, |gains| {
        let bad = (0.0065..0.007).contains(&gains.d) || gains.d > 0.02;
        Some(metrics(0.0, if bad { 1.0 } else { 0.0 }))
    });

    let (gains, outcome) = finished(&run);
    check!(outcome == Outcome::Tuned);
    check!(gains.d < 0.0065);
}

/// Invalid steps in a row fail a bracket point (it clips: it can't be shown to pass). On the
/// baseline they leave no baseline but don't end the stage: the search goes on below (#361).
#[test]
fn invalid_steps_fail_a_point_not_the_stage() {
    let plan = d_plan();
    let clipped = |clips: bool| {
        Some(StepMetrics {
            saturated_s: if clips { 1.0 } else { 0.0 },
            ..metrics(0.0, 0.0)
        })
    };
    let baseline_count = |run: &Run| {
        let Action::Finish {
            baseline: Some(baseline),
            ..
        } = run.end
        else {
            panic!("no baseline: {:?}", run.end);
        };
        baseline.count()
    };

    let above_start = drive(plan, no_relays, |gains| clipped(gains.d > START.d));
    let (gains, outcome) = finished(&above_start);
    check!(outcome == Outcome::Tuned);
    check!(gains.d == plan.margin * START.d);
    check!(
        above_start
            .verdicts
            .iter()
            .all(|v| v.passed == (v.gains.d <= START.d))
    );

    // Clipping from the start gains up: the start fails, the search finds a gain below.
    let from_start = drive(plan, no_relays, |gains| clipped(gains.d >= START.d));
    let (gains, outcome) = finished(&from_start);
    check!(outcome == Outcome::Tuned, "clipped from the start up");
    check!(gains.d < START.d);
    check!(baseline_count(&from_start) == 0);
    check!(!from_start.verdicts[0].passed);

    let everywhere = drive(plan, no_relays, |_| clipped(true));
    check!(finished(&everywhere) == (START, Outcome::Kept(KeepReason::NoPass)));
    check!(baseline_count(&everywhere) == 0);
}

/// A start the search would never pick (its term above `max`) flies no baseline: the search
/// goes straight to its bracket, and the verify meets the criterion itself (#361).
#[test]
fn a_start_above_max_flies_no_baseline() {
    let plan = SearchPlan {
        max: 0.5 * START.d,
        ..d_plan()
    };
    let run = drive(plan, no_relays, damping(0.001));

    check!(run.flown.iter().all(|gains| *gains != START));
    let (gains, outcome) = finished(&run);
    check!(outcome == Outcome::Tuned);
    check!(gains.d < plan.max);
}

/// A baseline that fails the criterion isn't a standard: a result that meets the criterion
/// is kept though it settles slower (a too-high rate P settles fast by overshooting within
/// the settle band, #361). A passing baseline still holds the result to no worse.
#[test]
fn a_failing_baseline_judges_by_the_criterion() {
    // The start overshoots past the criterion but settles at 0.1 s; below it, clean steps
    // that settle at 0.2 s.
    let respond = |start_overshoot: f32| {
        move |gains: Gains| {
            Some(if gains == START {
                metrics(start_overshoot, 0.0)
            } else {
                StepMetrics {
                    settling_time_s: Some(0.2),
                    ..metrics(0.0, 0.0)
                }
            })
        }
    };
    let plan = SearchPlan {
        criterion: Criterion::Overshoot {
            max_overshoot: 0.05,
            max_bounce_back: 0.1,
        },
        ..d_plan()
    };
    // (start overshoot, outcome)
    let cases = [
        (0.08, Outcome::Tuned),
        (0.0, Outcome::Kept(KeepReason::Worse)),
    ];
    for (start_overshoot, expected) in cases {
        let run = drive(plan, no_relays, respond(start_overshoot));
        let (_, outcome) = finished(&run);
        check!(outcome == expected, "start overshoot {start_overshoot}");
    }
}

/// An input the search didn't ask for fails it.
#[test]
fn a_mismatched_input_fails() {
    let mut search = Procedure::new(Method::Search(d_plan()));
    search.next(Input::Start { gains: START });
    let relay = relay_input(POINT, false);
    check!(search.next(relay) == Action::Fail(AutotuneAbort::Interrupted));
}

/// With no baseline to compare (the start's steps abort), the verify must meet the criterion
/// itself: a result that bounces back too far leaves the start.
#[test]
fn without_a_baseline_the_verify_meets_the_criterion() {
    let plan = d_plan();
    // The start aborts; below it everything passes, except a bouncing `verify` gain.
    let respond = |verify: Option<f32>| {
        move |gains: Gains| {
            let bounce = if Some(gains.d) == verify { 1.0 } else { 0.0 };
            (gains != START).then(|| metrics(0.0, bounce))
        }
    };
    let clean = drive(plan, no_relays, respond(None));
    let (tuned, outcome) = finished(&clean);
    check!(outcome == Outcome::Tuned);
    let Action::Finish {
        baseline: Some(baseline),
        ..
    } = clean.end
    else {
        panic!("no baseline: {:?}", clean.end);
    };
    check!(baseline.count() == 0);

    let bouncing = drive(plan, no_relays, respond(Some(tuned.d)));
    check!(finished(&bouncing) == (START, Outcome::Kept(KeepReason::Worse)));
}

/// A bracket point passes only on `confirm` passing pairs in a row: a gain that passes one
/// pair and fails the next (a lucky pair near a sharp limit) fails.
#[test]
fn a_point_needs_confirming_pairs() {
    let plan = d_plan();
    check!(plan.confirm == 2);
    // Above 0.0045, pairs alternate pass and fail on each gain.
    let steps_at = RefCell::new(Vec::<(f32, usize)>::new());
    let run = drive(plan, no_relays, |gains| {
        let mut steps_at = steps_at.borrow_mut();
        let count = match steps_at.iter_mut().find(|(gain, _)| *gain == gains.d) {
            Some((_, count)) => count,
            None => {
                steps_at.push((gains.d, 0));
                &mut steps_at.last_mut().expect("just pushed").1
            }
        };
        let pair = *count / 2;
        *count += 1;
        let flaky = gains.d > 0.0045 && pair % 2 == 1;
        Some(metrics(0.0, if flaky { 1.0 } else { 0.0 }))
    });

    let (gains, outcome) = finished(&run);
    check!(outcome == Outcome::Tuned);
    check!(gains.d < plan.margin * 0.0045, "{}", gains.d);
    // Each flaky gain flew a passing pair, then a failing one.
    let flaky: Vec<_> = run
        .verdicts
        .iter()
        .filter(|v| v.gains.d > 0.0045)
        .map(|v| v.passed)
        .collect();
    check!(!flaky.is_empty());
    check!(
        flaky.chunks(2).all(|pair| pair == [true, false]),
        "{flaky:?}"
    );
}

/// A breach while a candidate flies fails that candidate, and the search goes on; one
/// reported on the baseline (committed gains) fails the stage.
#[test]
fn a_breach_fails_the_candidate_only() {
    let breached = |gains| Input::Breached {
        reason: AutotuneAbort::Rate,
        gains,
    };
    let mut search = Procedure::new(Method::Search(d_plan()));
    let Action::Probe { gains: None, .. } = search.next(Input::Start { gains: START }) else {
        panic!("the baseline flies the committed gains");
    };
    check!(search.next(breached(START)) == Action::Fail(AutotuneAbort::Rate));

    let mut search = Procedure::new(Method::Search(d_plan()));
    let mut action = search.next(Input::Start { gains: START });
    // The baseline passes; then the first bracket point breaches.
    for _ in 0..2 * d_plan().test.pairs {
        action = search.next(Input::Measured {
            measurement: Measurement::Step(metrics(0.0, 0.0)),
            gains: START,
            stalled: false,
        });
    }
    let Action::Probe {
        gains: Some(candidate),
        ..
    } = action
    else {
        panic!("a bracket point: {action:?}");
    };
    let next = search.next(breached(START));
    let verdict = search.take_verdict().expect("judged");
    check!(verdict.gains == candidate);
    check!(!verdict.passed);
    let Action::Probe {
        gains: Some(lower), ..
    } = next
    else {
        panic!("searches on: {next:?}");
    };
    check!(lower.d < candidate.d);
}

/// Stalled steps never fail a candidate: they run again until `max_stalled` in a row, which
/// fails the stage (the host can't keep time).
#[test]
fn stalls_rerun_and_never_fail_a_candidate() {
    let plan = d_plan();
    let stalled_step = |stalled| Input::Measured {
        measurement: Measurement::Step(metrics(0.0, 0.0)),
        gains: START,
        stalled,
    };
    let mut search = Procedure::new(Method::Search(plan));
    search.next(Input::Start { gains: START });
    for _ in 0..2 * plan.test.pairs {
        search.next(stalled_step(false));
    }
    // On the first bracket point: stalls below the budget, then a valid pair passes it.
    for _ in 1..plan.test.max_stalled {
        let Action::Probe { gains: Some(_), .. } = search.next(stalled_step(true)) else {
            panic!("reruns the candidate");
        };
    }
    search.next(stalled_step(false));
    search.next(stalled_step(false));
    check!(search.take_verdict().map(|v| v.passed) == Some(true));
    for _ in 1..plan.test.max_stalled {
        search.next(stalled_step(true));
    }
    check!(search.next(stalled_step(true)) == Action::Fail(AutotuneAbort::Stalled));
}
