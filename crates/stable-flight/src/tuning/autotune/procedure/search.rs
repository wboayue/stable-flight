//! The gain search: see [`Search`].

use super::{Action, Batch, Input, Pairs, Verdict};
use crate::controllers::Gains;
use crate::tuning::StepSummary;
use crate::tuning::autotune::{AutotuneAbort, KeepReason, Outcome, SearchPlan, with_i_per_p};

#[cfg(test)]
mod tests;

/// A [`SearchPlan`] as a pure state machine: baseline, bracket and bisect, verify.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(in crate::tuning::autotune) struct Search {
    plan: SearchPlan,
    leg: Leg,
    /// The loop's gains at the stage's start.
    start: Gains,
    /// What stays when the search keeps no result and the start's steps missed the criterion:
    /// the loop's gains at the run's start, lowered to `max` when the stage starts from them.
    fallback: Gains,
    /// The largest gain that passed.
    pass: Option<f32>,
    /// The smallest gain that failed.
    fail: Option<f32>,
    /// The steps on the gain under test.
    pairs: Pairs,
    /// The steps on the start gains: empty when they weren't flown (the start is above `max`)
    /// or couldn't be measured (a step aborted or kept clipping).
    baseline: StepSummary,
    /// The last judgment, until taken.
    verdict: Option<Verdict>,
}

/// Where a search is.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Leg {
    /// Stepping on the start gains.
    Baseline,
    /// Stepping on a bracket point that has passed `passes` pairs in a row.
    Try { gain: f32, passes: usize },
    /// Stepping on the result.
    Verify(f32),
}

impl Search {
    pub(super) fn new(plan: SearchPlan) -> Self {
        Self {
            plan,
            leg: Leg::Baseline,
            start: Gains::default(),
            fallback: Gains::default(),
            pass: None,
            fail: None,
            pairs: Pairs::new(plan.test, plan.test.pairs),
            baseline: StepSummary::default(),
            verdict: None,
        }
    }

    pub(super) fn next(&mut self, input: Input) -> Action {
        if let Some(batch) = self.pairs.take(&input) {
            return match batch {
                Batch::Next => self.step(),
                Batch::Ended { complete } => self.judge(complete),
                Batch::Fail(abort) => Action::Fail(abort),
            };
        }
        match input {
            Input::Start {
                gains, run_start, ..
            } => {
                self.start = gains;
                self.fallback = run_start;
                self.baseline()
            }
            // A candidate that took the vehicle past the envelope fails.
            Input::Breached { .. } if matches!(self.leg, Leg::Try { .. } | Leg::Verify(_)) => {
                self.judge(false)
            }
            Input::Breached { reason, .. } => Action::Fail(reason),
            _ => Action::Fail(AutotuneAbort::Interrupted),
        }
    }

    pub(super) fn take_verdict(&mut self) -> Option<Verdict> {
        self.verdict.take()
    }

    /// Steps on the start gains, or, when the searched term starts above `max` (a gain the
    /// search would never pick), goes on without a baseline rather than fly it (#361) and
    /// commits the start with its term at `max` in its place: a start that high can ring once
    /// disturbed and never settle (#375). Those gains are what stays if nothing better passes.
    fn baseline(&mut self) -> Action {
        if self.plan.term.get(self.start) > self.plan.max {
            let lowered = self.with(self.plan.max);
            // The run's own start too high to fly stays lowered; a seeded start goes back to it.
            if self.fallback == self.start {
                self.fallback = lowered;
            }
            self.start = lowered;
            return match self.advance() {
                Action::Probe { probe, gains, .. } => Action::Probe {
                    probe,
                    gains,
                    commit: Some(self.start),
                },
                end => end,
            };
        }
        self.leg = Leg::Baseline;
        self.step()
    }

    /// The next step on the gain under test.
    fn step(&self) -> Action {
        let gains = match self.leg {
            Leg::Try { gain, .. } | Leg::Verify(gain) => Some(self.with(gain)),
            Leg::Baseline => None,
        };
        Action::Probe {
            probe: self.pairs.probe(),
            gains,
            commit: None,
        }
    }

    /// The start gains with the searched term at `gain`, and I at the plan's
    /// [`i_per_p`](crate::tuning::autotune::SearchPlan::i_per_p) of P if it sets one.
    fn with(&self, gain: f32) -> Gains {
        with_i_per_p(self.plan.term.with(self.start, gain), self.plan.i_per_p)
    }

    /// Judges the batch on the gain under test (`complete`: every step ran; else one
    /// aborted, too many were invalid, or the vehicle breached the envelope, which fails it)
    /// and moves on. A bracket point needs `confirm` passing pairs in a row.
    fn judge(&mut self, complete: bool) -> Action {
        let steps = self.pairs.summary;
        let passed = complete && self.plan.criterion.passes(&steps);
        let (gains, bracketed) = match self.leg {
            // The baseline is the first bracket point. Incomplete, it's no baseline: the start
            // gains aborted or clipped.
            Leg::Baseline => {
                self.baseline = if complete {
                    steps
                } else {
                    StepSummary::default()
                };
                (self.start, true)
            }
            Leg::Try { gain, .. } => (self.with(gain), true),
            Leg::Verify(gain) => return self.verified(gain, complete, steps),
        };
        self.verdict = Some(Verdict {
            gains,
            passed,
            steps,
        });
        // A passing pair short of `confirm` in a row: another pair on the same gain.
        if let Leg::Try { gain, passes } = self.leg
            && passed
            && passes + 1 < self.plan.confirm
        {
            self.leg = Leg::Try {
                gain,
                passes: passes + 1,
            };
            self.pairs.restart(1);
            return self.step();
        }
        if bracketed {
            let gain = self.plan.term.get(gains);
            if passed {
                self.pass = Some(self.pass.map_or(gain, |pass| pass.max(gain)));
            } else {
                self.fail = Some(self.fail.map_or(gain, |fail| fail.min(gain)));
            }
        }
        self.advance()
    }

    /// Keeps the verified `gain` if its steps (`complete`: none aborted or kept clipping) fly
    /// no worse than a baseline that met the criterion, or, with no such baseline (the start's
    /// steps weren't flown, aborted, clipped, or failed the criterion), meet the criterion
    /// themselves; else the [`kept`](Self::kept) gains stay. A start that fails the criterion
    /// isn't a standard to hold the result to: a too-high rate P settles fast because it
    /// overshoots within the settle band (#361).
    fn verified(&self, gain: f32, complete: bool, steps: StepSummary) -> Action {
        let SearchPlan {
            criterion,
            acceptance,
            ..
        } = self.plan;
        let accepted = complete
            // An empty baseline doesn't pass the criterion.
            && if criterion.passes(&self.baseline) {
                acceptance.accepts(&criterion, &self.baseline, &steps)
            } else {
                criterion.passes(&steps)
            };
        if accepted {
            self.finish(self.with(gain), Outcome::Tuned, Some(steps))
        } else {
            self.finish(self.kept(), Outcome::Kept(KeepReason::Worse), Some(steps))
        }
    }

    /// Tries the next bracket point, or verifies the result once the bracket is narrow
    /// enough.
    fn advance(&mut self) -> Action {
        let SearchPlan {
            factor,
            resolution,
            margin,
            min,
            max,
            ..
        } = self.plan;
        let next = match (self.pass, self.fail) {
            (Some(pass), Some(fail)) => {
                (fail > pass * resolution).then(|| libm::sqrtf(pass * fail))
            }
            (Some(pass), None) => (pass < max).then(|| (pass * factor).min(max)),
            (None, Some(fail)) if fail > min => Some((fail / factor).max(min)),
            (None, Some(_)) => {
                return self.finish(self.kept(), Outcome::Kept(KeepReason::NoPass), None);
            }
            // Nothing tried yet: the start's gain.
            (None, None) => Some(self.plan.term.get(self.start).clamp(min, max)),
        };
        let (leg, pairs) = match next {
            Some(gain) => (Leg::Try { gain, passes: 0 }, 1),
            None => {
                let pass = self.pass.unwrap_or(min);
                let result = (margin * pass).clamp(min, max);
                (Leg::Verify(result), self.plan.test.pairs)
            }
        };
        self.leg = leg;
        self.pairs.restart(pairs);
        self.step()
    }

    /// The gains that stay when the search keeps no result: the start, if its steps met the
    /// criterion, else the fallback. A start the search was seeded with (a design) and never
    /// saw pass doesn't outlive the search.
    fn kept(&self) -> Gains {
        if self.plan.criterion.passes(&self.baseline) {
            self.start
        } else {
            self.fallback
        }
    }

    fn finish(&self, gains: Gains, outcome: Outcome, steps: Option<StepSummary>) -> Action {
        Action::Finish {
            gains,
            outcome,
            steps,
            baseline: Some(self.baseline),
        }
    }
}
