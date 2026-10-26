use assert2::check;

use super::*;
use crate::stabilizers::copter::TunedLoop;
use crate::tuning::autotune::{Method, Procedure};
use crate::tuning::{StepAbort, StepMetrics};

/// A designed rate loop, committed.
const DESIGNED: Gains = Gains {
    p: 1.2,
    i: 0.2,
    d: 0.036,
};

/// The rate loop at the run's start.
const RUN_START: Gains = Gains {
    p: 0.5,
    i: 0.2,
    d: 0.004,
};

/// The most probes a test lets a verify ask for.
const MAX_PROBES: usize = 20;

/// A step's reply: its metrics, an abort, or a breach of the envelope.
#[derive(Clone, Copy, Debug)]
enum Reply {
    Step { overshoot: f32, settling_s: f32 },
    Aborted,
    Breached,
}

/// A step settling at `settling_s` with `overshoot` and no bounce-back.
const fn step(overshoot: f32, settling_s: f32) -> Reply {
    Reply::Step {
        overshoot,
        settling_s,
    }
}

/// What a scripted verify did.
#[derive(Debug)]
struct Run {
    /// The gains each step flew (`None`: the committed gains), in order.
    flown: Vec<Option<Gains>>,
    /// How it ended.
    end: Action,
}

/// Runs a rate loop's copter verify on `committed` from `run_start`: each step on the
/// committed gains gets `verify`, each on the run's start `baseline`.
fn drive(committed: Gains, run_start: Gains, verify: Reply, baseline: Reply) -> Run {
    let plan = VerifyPlan::copter(TunedLoop::RollRate);
    let mut procedure = Procedure::new(Method::Verify(plan));
    let mut flown = Vec::new();
    let mut action = procedure.next(Input::Start {
        gains: committed,
        inner: None,
        run_start,
    });
    for _ in 0..MAX_PROBES {
        let Action::Probe { gains, commit, .. } = action else {
            return Run { flown, end: action };
        };
        check!(commit.is_none());
        flown.push(gains);
        let reply = if gains.is_some() { baseline } else { verify };
        let input = match reply {
            Reply::Step {
                overshoot,
                settling_s,
            } => Input::Measured {
                measurement: Measurement::Step(StepMetrics {
                    overshoot,
                    settling_time_s: Some(settling_s),
                    ..Default::default()
                }),
                gains: committed,
                stalled: false,
            },
            Reply::Aborted => Input::Aborted {
                abort: ProbeAbort::Step(StepAbort::Reversed),
                gains: committed,
            },
            Reply::Breached => Input::Breached {
                reason: AutotuneAbort::Tilt,
                gains: committed,
            },
        };
        action = procedure.next(input);
    }
    panic!("no end after {MAX_PROBES} probes: {flown:?}");
}

/// Which gains stay and whether a baseline flew, per verify and baseline reply (the copter
/// rate criterion: overshoot at most 0.05; acceptance: 0.05 more overshoot, 10% slower
/// settling).
#[test]
fn keeps_the_committed_gains_unless_they_fly_worse_than_the_run_start() {
    // (case, committed, verify reply, baseline reply, gains kept, outcome, baseline flown)
    let rows = [
        (
            "verify meets the criterion",
            DESIGNED,
            step(0.02, 0.1),
            step(0.0, 0.1),
            DESIGNED,
            Outcome::Measured,
            false,
        ),
        (
            "verify aborts",
            DESIGNED,
            Reply::Aborted,
            step(0.0, 0.1),
            RUN_START,
            Outcome::Restored,
            false,
        ),
        (
            "misses, no worse than the run start",
            DESIGNED,
            step(0.08, 0.1),
            step(0.06, 0.1),
            DESIGNED,
            Outcome::Measured,
            true,
        ),
        (
            "misses, worse than the run start",
            DESIGNED,
            step(0.08, 0.2),
            step(0.0, 0.1),
            RUN_START,
            Outcome::Restored,
            true,
        ),
        (
            "misses, the run start aborts",
            DESIGNED,
            step(0.08, 0.2),
            Reply::Aborted,
            DESIGNED,
            Outcome::Measured,
            true,
        ),
        (
            "misses, the run start breaches",
            DESIGNED,
            step(0.08, 0.2),
            Reply::Breached,
            DESIGNED,
            Outcome::Measured,
            true,
        ),
        (
            "misses on gains the run never changed",
            RUN_START,
            step(0.08, 0.2),
            step(0.0, 0.1),
            RUN_START,
            Outcome::Measured,
            false,
        ),
    ];
    for (case, committed, verify, baseline, kept, outcome, baseline_flown) in rows {
        let run = drive(committed, RUN_START, verify, baseline);
        let Action::Finish {
            gains,
            outcome: ended,
            baseline: summary,
            ..
        } = run.end
        else {
            check!(false, "{case}: {:?}", run.end);
            continue;
        };
        check!(gains == kept, "{case}");
        check!(ended == outcome, "{case}");
        check!(summary.is_some() == baseline_flown, "{case}");
        let candidates = run.flown.iter().filter(|gains| gains.is_some()).count();
        check!(
            (candidates > 0) == baseline_flown,
            "{case}: {:?}",
            run.flown
        );
        check!(
            run.flown.iter().flatten().all(|gains| *gains == RUN_START),
            "{case}: the baseline flies the run's start"
        );
    }
}

/// The verify and the baseline each fly the plan's pairs.
#[test]
fn verify_and_baseline_fly_their_pairs() {
    let run = drive(DESIGNED, RUN_START, step(0.08, 0.1), step(0.06, 0.1));
    let pairs = VerifyPlan::copter(TunedLoop::RollRate).test.pairs;
    check!(run.flown.len() == 4 * pairs);
    check!(run.flown[..2 * pairs].iter().all(Option::is_none));
    check!(
        run.flown[2 * pairs..]
            .iter()
            .all(|gains| *gains == Some(RUN_START))
    );
}

/// A breach on the committed gains, or a slow host past its stall budget, fails the stage.
#[test]
fn a_committed_breach_or_endless_stalls_fail_the_stage() {
    let run = drive(DESIGNED, RUN_START, Reply::Breached, step(0.0, 0.1));
    check!(run.end == Action::Fail(AutotuneAbort::Tilt));

    let plan = VerifyPlan::copter(TunedLoop::RollRate);
    let mut procedure = Procedure::new(Method::Verify(plan));
    procedure.next(Input::start(DESIGNED));
    let mut end = None;
    for _ in 0..plan.test.max_stalled {
        end = Some(procedure.next(Input::Stalled { gains: DESIGNED }));
    }
    check!(end == Some(Action::Fail(AutotuneAbort::Stalled)));
}
