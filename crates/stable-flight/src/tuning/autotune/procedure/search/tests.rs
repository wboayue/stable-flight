use crate::controllers::GyroFilters;
use std::cell::RefCell;

use assert2::check;

use super::*;
use crate::controllers::RATE_I_PER_P;
use crate::tuning::autotune::procedure::Procedure;
use crate::tuning::autotune::{Criterion, Method, Term};
use crate::tuning::{Measurement, ProbeAbort, ProbeConfig, StepAbort, StepMetrics};

/// A hand-tune-like rate loop.
const START: Gains = Gains {
    p: 0.5,
    i: 0.2,
    d: 0.004,
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

/// A probe a slow host stopped, unmeasured.
fn stopped() -> Input {
    Input::Stalled { gains: START }
}

/// What a scripted search did.
#[derive(Debug)]
struct Run {
    /// The gains each step flew, in order.
    flown: Vec<Gains>,
    /// The verdicts, in order.
    verdicts: Vec<Verdict>,
    /// The gains each commit set, in order.
    committed: Vec<Gains>,
    /// How it ended.
    end: Action,
}

impl Run {
    /// The searched term's gain on each judged bracket point (the baseline first).
    fn tried(&self, term: Term) -> Vec<f32> {
        self.verdicts.iter().map(|v| term.get(v.gains)).collect()
    }
}

/// Runs `plan` from [`START`], also the run's start: steps get `step(gains flown)` (`None`:
/// the step aborts).
fn drive(plan: SearchPlan, step: impl Fn(Gains) -> Option<StepMetrics>) -> Run {
    drive_from(plan, START, step)
}

/// As [`drive`], with the loop's gains at the run's start `run_start`.
fn drive_from(
    plan: SearchPlan,
    run_start: Gains,
    step: impl Fn(Gains) -> Option<StepMetrics>,
) -> Run {
    drive_unsettled(plan, run_start, 0, step)
}

/// As [`drive_from`], the first `unsettled` probes timing out in the settle before them.
fn drive_unsettled(
    plan: SearchPlan,
    run_start: Gains,
    mut unsettled: usize,
    step: impl Fn(Gains) -> Option<StepMetrics>,
) -> Run {
    let mut procedure = Procedure::new(Method::Search(plan));
    let mut flown = Vec::new();
    let mut verdicts = Vec::new();
    let mut committed = Vec::new();
    let mut action = procedure.next(Input::Start {
        gains: START,
        inner: None,
        run_start,
        gyro: GyroFilters::low_pass(0.0),
    });
    for _ in 0..MAX_PROBES {
        verdicts.extend(procedure.take_verdict());
        if let Action::Probe {
            commit: Some(gains),
            ..
        } = action
        {
            committed.push(gains);
        }
        let current = committed.last().copied().unwrap_or(START);
        let input = match action {
            Action::Probe { .. } if unsettled > 0 => {
                unsettled -= 1;
                Input::Unsettled { gains: current }
            }
            Action::Probe {
                probe: ProbeConfig::Step(_),
                gains,
                ..
            } => {
                let gains = gains.unwrap_or(current);
                flown.push(gains);
                match step(gains) {
                    Some(metrics) => Input::Measured {
                        measurement: Measurement::Step(metrics),
                        gains: current,
                        stalled: false,
                    },
                    None => Input::Aborted {
                        abort: ProbeAbort::Step(StepAbort::Runaway),
                        gains: current,
                    },
                }
            }
            end => {
                return Run {
                    flown,
                    verdicts,
                    committed,
                    end,
                };
            }
        };
        action = procedure.next(input);
    }
    panic!("no end after {MAX_PROBES} probes");
}

/// A D search judged on bounce-back alone: it starts from [`START`].
fn d_plan() -> SearchPlan {
    SearchPlan {
        term: Term::D,
        criterion: Criterion::Overshoot {
            max_overshoot: 1.0,
            max_bounce_back: 0.1,
            settled_past: None,
        },
        min: 0.001,
        max: 0.03,
        i_per_p: None,
        ..SearchPlan::copter_rate()
    }
}

/// Bounce-back grows with D past `threshold` (0.1 at it).
fn damping(threshold: f32) -> impl Fn(Gains) -> Option<StepMetrics> {
    move |gains| Some(metrics(0.0, 0.1 * gains.d / threshold))
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
        let run = drive(plan, damping(threshold));

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
    let run = drive(plan, damping(0.01));

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
    let run = drive(plan, |_| Some(metrics(0.0, 1.0)));

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
    let run = drive(d_plan(), |gains| {
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
    let run = drive(d_plan(), |gains| {
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

/// A scripted step reply per gains flown (`None`: the step aborts).
type StepReply = fn(Gains) -> Option<StepMetrics>;

/// A search that keeps no result keeps its start only if the start's steps met the criterion;
/// a start that missed it, or whose steps aborted (a design the search was seeded with), gives
/// way to the loop's gains at the run's start.
#[test]
fn a_start_that_never_passed_gives_way_to_the_run_start() {
    let run_start = Gains { d: 0.002, ..START };
    // (case, step reply per gains flown, kept, outcome)
    let rows: [(&str, StepReply, Gains, Outcome); 3] = [
        (
            "nothing passes",
            |_| Some(metrics(0.0, 1.0)),
            run_start,
            Outcome::Kept(KeepReason::NoPass),
        ),
        (
            "the start aborts, nothing else passes",
            |gains| (gains != START).then(|| metrics(0.0, 1.0)),
            run_start,
            Outcome::Kept(KeepReason::NoPass),
        ),
        (
            "the start passes, the result flies worse",
            |gains| {
                Some(StepMetrics {
                    settling_time_s: Some(if gains == START { 0.1 } else { 0.2 }),
                    ..metrics(0.0, 0.0)
                })
            },
            START,
            Outcome::Kept(KeepReason::Worse),
        ),
    ];
    for (case, step, kept, outcome) in rows {
        let run = drive_from(d_plan(), run_start, step);
        check!(finished(&run) == (kept, outcome), "{case}");
    }
}

/// A P search with `i_per_p` flies each candidate P with I at that share of it, and the
/// result keeps it; without, every candidate keeps the start's I. The start flies as it is.
#[test]
fn candidates_carry_i_at_the_plans_share_of_p() {
    // Overshoot grows with P, past the criterion's 0.05 above P 1.
    let overshoot = |gains: Gains| Some(metrics(0.05 * gains.p, 0.0));
    for i_per_p in [Some(0.2), None] {
        let candidate_i = |p: f32| i_per_p.map_or(START.i, |ratio| ratio * p);
        let plan = SearchPlan {
            i_per_p,
            ..SearchPlan::copter_rate()
        };
        let run = drive(plan, overshoot);
        for gains in &run.flown {
            let expected = if *gains == START {
                START.i
            } else {
                candidate_i(gains.p)
            };
            check!(gains.i == expected, "{i_per_p:?}: {gains:?}");
        }
        let (kept, outcome) = finished(&run);
        check!(outcome == Outcome::Tuned, "{i_per_p:?}");
        check!(kept.i == candidate_i(kept.p), "{i_per_p:?}: {kept:?}");
    }
}

/// A start above `max` is lowered and committed either way, but only the run's own start stays
/// lowered when nothing passes; a seeded start (a design) gives way to the run's start.
#[test]
fn only_the_runs_own_start_stays_lowered() {
    let plan = SearchPlan {
        max: START.d / 2.0,
        ..d_plan()
    };
    let lowered = Gains {
        d: plan.max,
        ..START
    };
    let run_start = Gains { d: 0.0015, ..START };
    // (case, run's start, kept)
    let rows = [
        ("the run's own start", START, lowered),
        ("a seeded start", run_start, run_start),
    ];
    for (case, run_start, kept) in rows {
        let run = drive_from(plan, run_start, |_| Some(metrics(0.0, 1.0)));
        check!(run.committed == [lowered], "{case}");
        check!(
            finished(&run) == (kept, Outcome::Kept(KeepReason::NoPass)),
            "{case}"
        );
    }
}

/// A start the vehicle won't settle on is lowered by `factor` each time, with I at the plan's
/// share of the lowered P, committed, and its baseline stepped on it; below `min`, or past
/// `MAX_START_LOWERINGS`, the search fails `NotSettled` (#552).
#[test]
fn an_unsettled_start_is_lowered_until_it_settles() {
    let plan = SearchPlan {
        min: 0.2,
        ..SearchPlan::copter_rate()
    };
    // Divided once per lowering, as the search does.
    let lowered = |times: usize| {
        let p = (0..times).fold(START.p, |p, _| p / plan.factor);
        Gains {
            p,
            i: RATE_I_PER_P * p,
            ..START
        }
    };
    let passing = |_| Some(metrics(0.0, 0.0));
    // (unsettled settles, commits)
    let rows = [
        (1, vec![lowered(1)]),
        (3, vec![lowered(1), lowered(2), lowered(3)]),
    ];
    for (unsettled, commits) in rows {
        let run = drive_unsettled(plan, START, unsettled, passing);
        check!(run.committed == commits, "{unsettled}");
        check!(
            run.flown.first() == commits.last(),
            "{unsettled}: baseline on the lowered start"
        );
        check!(finished(&run).1 == Outcome::Tuned, "{unsettled}");
    }
    // (min, unsettled settles, commits before the failure): a fourth lowering takes 0.5 below
    // `min` 0.2 (0.5 / 1.3⁴); at `min` 0.01 the cap ends it.
    let failing = [
        (0.2, 4, 3),
        (0.01, MAX_START_LOWERINGS + 1, MAX_START_LOWERINGS),
    ];
    for (min, unsettled, commits) in failing {
        let run = drive_unsettled(SearchPlan { min, ..plan }, START, unsettled, passing);
        check!(run.committed.len() == commits, "min {min}");
        check!(
            run.end == Action::Fail(AutotuneAbort::NotSettled),
            "min {min}"
        );
    }
}

/// Past the baseline, committed gains that won't settle fail the search: only the baseline
/// lowers its start.
#[test]
fn an_unsettled_bracket_fails() {
    let plan = d_plan();
    let mut search = Procedure::new(Method::Search(plan));
    let mut action = search.next(Input::start(START));
    for _ in 0..2 * plan.test.pairs {
        check!(matches!(action, Action::Probe { gains: None, .. }));
        action = search.next(Input::Measured {
            measurement: Measurement::Step(metrics(0.0, 0.0)),
            gains: START,
            stalled: false,
        });
    }
    check!(matches!(action, Action::Probe { gains: Some(_), .. }));
    let failed = search.next(Input::Unsettled { gains: START });
    check!(failed == Action::Fail(AutotuneAbort::NotSettled));
}

/// A gain that passes above one that failed (noise) can't widen the bracket: the search still
/// ends, within the first bracket.
#[test]
fn a_non_monotone_response_stays_bounded() {
    // Fails only in a narrow band below a higher passing region.
    let run = drive(d_plan(), |gains| {
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

    let above_start = drive(plan, |gains| clipped(gains.d > START.d));
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
    let from_start = drive(plan, |gains| clipped(gains.d >= START.d));
    let (gains, outcome) = finished(&from_start);
    check!(outcome == Outcome::Tuned, "clipped from the start up");
    check!(gains.d < START.d);
    check!(baseline_count(&from_start) == 0);
    check!(!from_start.verdicts[0].passed);

    let everywhere = drive(plan, |_| clipped(true));
    check!(finished(&everywhere) == (START, Outcome::Kept(KeepReason::NoPass)));
    check!(baseline_count(&everywhere) == 0);
}

/// A start the search would never pick (its term above `max`) flies no baseline: it's lowered
/// to `max` and committed, the search goes straight to its bracket, and the verify meets the
/// criterion itself (#361).
#[test]
fn a_start_above_max_flies_no_baseline() {
    let plan = SearchPlan {
        max: 0.5 * START.d,
        ..d_plan()
    };
    let run = drive(plan, damping(0.001));

    check!(
        run.committed
            == [Gains {
                d: plan.max,
                ..START
            }]
    );
    check!(run.flown.iter().all(|gains| *gains != START));
    let (gains, outcome) = finished(&run);
    check!(outcome == Outcome::Tuned);
    check!(gains.d < plan.max);
}

/// A start at or below `max` commits nothing.
#[test]
fn a_start_within_max_commits_nothing() {
    let run = drive(d_plan(), damping(0.01));
    check!(run.committed.is_empty());
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
            settled_past: None,
        },
        ..d_plan()
    };
    // (start overshoot, outcome)
    let cases = [
        (0.08, Outcome::Tuned),
        (0.0, Outcome::Kept(KeepReason::Worse)),
    ];
    for (start_overshoot, expected) in cases {
        let run = drive(plan, respond(start_overshoot));
        let (_, outcome) = finished(&run);
        check!(outcome == expected, "start overshoot {start_overshoot}");
    }
}

/// An input the search didn't ask for fails it.
#[test]
fn a_mismatched_input_fails() {
    let mut search = Procedure::new(Method::Search(d_plan()));
    search.next(Input::start(START));
    let held = Input::Held {
        settled: true,
        gains: START,
    };
    check!(search.next(held) == Action::Fail(AutotuneAbort::Interrupted));
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
    let clean = drive(plan, respond(None));
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

    let bouncing = drive(plan, respond(Some(tuned.d)));
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
    let run = drive(plan, |gains| {
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
    let Action::Probe { gains: None, .. } = search.next(Input::start(START)) else {
        panic!("the baseline flies the committed gains");
    };
    check!(search.next(breached(START)) == Action::Fail(AutotuneAbort::Rate));

    let mut search = Procedure::new(Method::Search(d_plan()));
    let mut action = search.next(Input::start(START));
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
    search.next(Input::start(START));
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

/// A step a slow host stopped runs again on the same gains, and stops share the stall budget
/// with stalled steps: `max_stalled` in a row fail the stage.
#[test]
fn a_stopped_step_runs_again_and_counts_as_stalled() {
    let plan = d_plan();
    let mut procedure = Procedure::new(Method::Search(plan));
    let first = procedure.next(Input::start(START));
    check!(matches!(
        first,
        Action::Probe {
            probe: ProbeConfig::Step(_),
            ..
        }
    ));
    let stalled = Input::Measured {
        measurement: Measurement::Step(metrics(0.0, 0.0)),
        gains: START,
        stalled: true,
    };
    for index in 1..plan.test.max_stalled {
        let input = if index % 2 == 0 { stalled } else { stopped() };
        check!(procedure.next(input) == first, "stall {index}");
    }
    check!(procedure.next(stopped()) == Action::Fail(AutotuneAbort::Stalled));
}
