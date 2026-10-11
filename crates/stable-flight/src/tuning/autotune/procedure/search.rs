//! The gain search: see [`Search`].

use super::{Action, Input, Pairs, Verdict};
use crate::controllers::Gains;
use crate::tuning::autotune::{AutotuneAbort, KeepReason, Outcome, SearchPlan};
use crate::tuning::{Measurement, ProbeAbort, ProbeConfig, StepSummary};

#[cfg(test)]
mod tests;

/// A [`SearchPlan`] as a pure state machine: relay, baseline, bracket and bisect, verify.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(in crate::tuning::autotune) struct Search {
    plan: SearchPlan,
    leg: Leg,
    /// The loop's gains at the stage's start.
    start: Gains,
    /// The gains candidates change one term of: the relay rule's, or `start`.
    seed: Gains,
    /// The largest gain to try: `plan.max`, lowered by the relay's ceiling.
    max: f32,
    /// The largest gain that passed.
    pass: Option<f32>,
    /// The smallest gain that failed.
    fail: Option<f32>,
    /// The steps on the gain under test.
    pairs: Pairs,
    /// The steps on the start gains.
    baseline: StepSummary,
    /// The last judgment, until taken.
    verdict: Option<Verdict>,
}

/// Where a search is.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Leg {
    /// Measuring the ultimate point; `tries` relays failed so far.
    Relay { tries: usize },
    /// Stepping on the start gains.
    Baseline,
    /// Stepping on a bracket point.
    Try(f32),
    /// Stepping on the result.
    Verify(f32),
}

impl Search {
    pub(super) fn new(plan: SearchPlan) -> Self {
        Self {
            plan,
            leg: Leg::Baseline,
            start: Gains::default(),
            seed: Gains::default(),
            max: plan.max,
            pass: None,
            fail: None,
            pairs: Pairs::new(plan.test, plan.test.pairs),
            baseline: StepSummary::default(),
            verdict: None,
        }
    }

    pub(super) fn next(&mut self, input: Input) -> Action {
        match (self.leg, input) {
            (_, Input::Start { gains }) => {
                self.start = gains;
                self.seed = gains;
                self.retry_relay(0)
            }
            (
                Leg::Relay { .. },
                Input::Measured {
                    measurement: Measurement::Relay(result),
                    stalled: false,
                    ..
                },
            ) => {
                if let Some(relay) = self.plan.relay {
                    self.seed = relay.plan.gains(&result, self.start);
                    let cap = relay.ceiling * result.point.ku;
                    self.max = self.max.min(cap).max(self.plan.min);
                }
                self.leg = Leg::Baseline;
                self.step()
            }
            (
                Leg::Relay { tries },
                Input::Measured {
                    measurement: Measurement::Relay(_),
                    stalled: true,
                    ..
                }
                | Input::Aborted {
                    abort: ProbeAbort::Relay(_),
                    ..
                },
            ) => self.retry_relay(tries + 1),
            (
                Leg::Baseline | Leg::Try(_) | Leg::Verify(_),
                Input::Measured {
                    measurement: Measurement::Step(metrics),
                    stalled,
                    ..
                },
            ) => match self.pairs.record(&metrics, stalled) {
                // The start gains can't be measured: nothing to compare with.
                Err(abort) if self.leg == Leg::Baseline => Action::Fail(abort),
                // A candidate whose steps keep clipping can't be shown to pass.
                Err(_) => self.judge(false),
                Ok(()) if self.pairs.done() => self.judge(true),
                Ok(()) => self.step(),
            },
            (
                Leg::Baseline | Leg::Try(_) | Leg::Verify(_),
                Input::Aborted {
                    abort: ProbeAbort::Step(_),
                    ..
                },
            ) => self.judge(false),
            _ => Action::Fail(AutotuneAbort::Interrupted),
        }
    }

    pub(super) fn take_verdict(&mut self) -> Option<Verdict> {
        self.verdict.take()
    }

    /// Runs the relay (again), or goes on without it: none planned, or `max_invalid`
    /// failures.
    fn retry_relay(&mut self, tries: usize) -> Action {
        match self.plan.relay {
            Some(relay) if tries < self.plan.test.max_invalid => {
                self.leg = Leg::Relay { tries };
                Action::probe(ProbeConfig::Relay(relay.plan.relay))
            }
            _ => {
                self.leg = Leg::Baseline;
                self.step()
            }
        }
    }

    /// The next step on the gain under test.
    fn step(&self) -> Action {
        let gains = match self.leg {
            Leg::Try(gain) | Leg::Verify(gain) => Some(self.with(gain)),
            Leg::Relay { .. } | Leg::Baseline => None,
        };
        Action::Probe {
            probe: self.pairs.probe(),
            gains,
        }
    }

    /// The seed gains with the searched term at `gain`.
    fn with(&self, gain: f32) -> Gains {
        self.plan.term.with(self.seed, gain)
    }

    /// Judges the batch on the gain under test (`complete`: every step ran; else one
    /// aborted or too many were invalid, which fails it) and moves on.
    fn judge(&mut self, complete: bool) -> Action {
        let steps = self.pairs.summary;
        let passed = complete && self.plan.criterion.passes(&steps);
        let (gains, bracketed) = match self.leg {
            // The baseline is a bracket point only when the search starts from it.
            Leg::Baseline => {
                self.baseline = steps;
                (self.start, self.seed == self.start)
            }
            Leg::Try(gain) => (self.with(gain), true),
            Leg::Verify(gain) => return self.verified(gain, complete, steps),
            Leg::Relay { .. } => return Action::Fail(AutotuneAbort::Interrupted),
        };
        self.verdict = Some(Verdict {
            gains,
            passed,
            steps,
        });
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
    /// no worse than the baseline, or, with no baseline steps to compare (the start's
    /// aborted), meet the criterion; else the start gains stay.
    fn verified(&self, gain: f32, complete: bool, steps: StepSummary) -> Action {
        let SearchPlan {
            criterion,
            acceptance,
            ..
        } = self.plan;
        let accepted = complete
            && if self.baseline.count() > 0 {
                acceptance.accepts(&criterion, &self.baseline, &steps)
            } else {
                criterion.passes(&steps)
            };
        if accepted {
            self.finish(self.with(gain), Outcome::Tuned, Some(steps))
        } else {
            self.finish(self.start, Outcome::Kept(KeepReason::Worse), Some(steps))
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
            ..
        } = self.plan;
        let next = match (self.pass, self.fail) {
            (Some(pass), Some(fail)) => {
                (fail > pass * resolution).then(|| libm::sqrtf(pass * fail))
            }
            (Some(pass), None) => (pass < self.max).then(|| (pass * factor).min(self.max)),
            (None, Some(fail)) if fail > min => Some((fail / factor).max(min)),
            (None, Some(_)) => {
                return self.finish(self.start, Outcome::Kept(KeepReason::NoPass), None);
            }
            // Nothing tried yet: the seed's gain.
            (None, None) => Some(self.plan.term.get(self.seed).clamp(min, self.max)),
        };
        let (leg, pairs) = match next {
            Some(gain) => (Leg::Try(gain), 1),
            None => {
                let pass = self.pass.unwrap_or(min);
                let result = (margin * pass).clamp(min, self.max);
                (Leg::Verify(result), self.plan.test.pairs)
            }
        };
        self.leg = leg;
        self.pairs.restart(pairs);
        self.step()
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
