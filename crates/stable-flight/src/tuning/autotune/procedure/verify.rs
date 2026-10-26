//! The verify after a design: see [`Verify`].

use super::{Action, Batch, Input, Pairs, Verdict};
use crate::controllers::Gains;
use crate::tuning::StepSummary;
use crate::tuning::autotune::{AutotuneAbort, Outcome, VerifyPlan};

#[cfg(test)]
mod tests;

/// A [`VerifyPlan`] as a pure state machine: steps on the committed gains, then, only when
/// they miss the criterion, steps on the run's start gains to judge them against.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(in crate::tuning::autotune) struct Verify {
    plan: VerifyPlan,
    leg: Leg,
    /// The loop's committed gains: what the verify flies.
    committed: Gains,
    /// The loop's gains at the run's start: what the baseline flies.
    run_start: Gains,
    /// The steps of the leg under way.
    pairs: Pairs,
    /// The verify's steps, kept while the baseline flies.
    verify: StepSummary,
    /// The last judgment, until taken.
    verdict: Option<Verdict>,
}

/// Where a verify is.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Leg {
    /// Stepping on the committed gains.
    Verify,
    /// Stepping on the run's start gains, after a verify that missed the criterion.
    Baseline,
}

impl Verify {
    pub(super) fn new(plan: VerifyPlan) -> Self {
        Self {
            plan,
            leg: Leg::Verify,
            committed: Gains::default(),
            run_start: Gains::default(),
            pairs: Pairs::new(plan.test, plan.test.pairs),
            verify: StepSummary::default(),
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
        match (self.leg, input) {
            (
                _,
                Input::Start {
                    gains, run_start, ..
                },
            ) => {
                self.committed = gains;
                self.run_start = run_start;
                self.step()
            }
            // The run's start gains, flown as a candidate, took the vehicle past the envelope.
            (Leg::Baseline, Input::Breached { .. }) => self.judge(false),
            (_, Input::Breached { reason, .. }) => Action::Fail(reason),
            _ => Action::Fail(AutotuneAbort::Interrupted),
        }
    }

    pub(super) fn take_verdict(&mut self) -> Option<Verdict> {
        self.verdict.take()
    }

    /// The next step: on the committed gains, or, for the baseline, on the run's start gains
    /// as a candidate.
    fn step(&self) -> Action {
        let gains = match self.leg {
            Leg::Verify => None,
            Leg::Baseline => Some(self.run_start),
        };
        Action::Probe {
            probe: self.pairs.probe(),
            gains,
            commit: None,
        }
    }

    /// Judges the leg's steps (`complete`: every step ran; else one aborted, too many
    /// clipped, or the baseline breached the envelope) and moves on.
    fn judge(&mut self, complete: bool) -> Action {
        let steps = self.pairs.summary;
        let VerifyPlan {
            criterion,
            acceptance,
            ..
        } = self.plan;
        let passed = complete && criterion.passes(&steps);
        match self.leg {
            Leg::Verify => {
                self.verdict = Some(Verdict {
                    gains: self.committed,
                    passed,
                    steps,
                });
                // Gains the run never changed have nothing to be judged against, nor to go
                // back to.
                if passed || self.committed == self.run_start {
                    return self.finish(self.committed, Outcome::Measured, steps, None);
                }
                if !complete {
                    return self.finish(self.run_start, Outcome::Restored, steps, None);
                }
                self.verify = steps;
                self.leg = Leg::Baseline;
                self.pairs.restart(self.plan.test.pairs);
                self.step()
            }
            Leg::Baseline => {
                self.verdict = Some(Verdict {
                    gains: self.run_start,
                    passed,
                    steps,
                });
                let baseline = if complete {
                    steps
                } else {
                    StepSummary::default()
                };
                // A baseline that couldn't be flown shows nothing to be worse than.
                let accepted = baseline.count() == 0
                    || acceptance.accepts(&criterion, &baseline, &self.verify);
                let (gains, outcome) = if accepted {
                    (self.committed, Outcome::Measured)
                } else {
                    (self.run_start, Outcome::Restored)
                };
                self.finish(gains, outcome, self.verify, Some(baseline))
            }
        }
    }

    fn finish(
        &self,
        gains: Gains,
        outcome: Outcome,
        steps: StepSummary,
        baseline: Option<StepSummary>,
    ) -> Action {
        Action::Finish {
            gains,
            outcome,
            steps: Some(steps),
            baseline,
        }
    }
}
