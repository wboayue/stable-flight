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
    /// The steps on the start gains: empty when they weren't flown (the start is above `max`)
    /// or couldn't be measured (a step aborted or kept clipping).
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
            (_, Input::Start { gains, .. }) => {
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
                self.baseline()
            }
            (
                Leg::Relay { tries },
                Input::Measured {
                    measurement: Measurement::Relay(_),
                    stalled: true,
                    ..
                }
                | Input::Stalled { .. }
                | Input::Aborted {
                    abort: ProbeAbort::Relay(_),
                    ..
                },
            ) => self.retry_relay(tries + 1),
            (
                Leg::Baseline | Leg::Try { .. } | Leg::Verify(_),
                Input::Measured {
                    measurement: Measurement::Step(metrics),
                    stalled,
                    ..
                },
            ) => match self.pairs.record(&metrics, stalled) {
                // Nothing can be measured while the host keeps stalling.
                Err(abort @ AutotuneAbort::Stalled) => Action::Fail(abort),
                // Steps that keep clipping measure the limit, not the loop: a candidate can't be
                // shown to pass, and the start gains leave no baseline (#361).
                Err(_) => self.judge(false),
                Ok(()) if self.pairs.done() => self.judge(true),
                Ok(()) => self.step(),
            },
            (Leg::Baseline | Leg::Try { .. } | Leg::Verify(_), Input::Stalled { .. }) => {
                match self.pairs.stall() {
                    Err(abort) => Action::Fail(abort),
                    Ok(()) => self.step(),
                }
            }
            (
                Leg::Baseline | Leg::Try { .. } | Leg::Verify(_),
                Input::Aborted {
                    abort: ProbeAbort::Step(_),
                    ..
                },
            ) => self.judge(false),
            // A candidate that took the vehicle past the envelope fails.
            (Leg::Try { .. } | Leg::Verify(_), Input::Breached { .. }) => self.judge(false),
            (_, Input::Breached { reason, .. }) => Action::Fail(reason),
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
            _ => self.baseline(),
        }
    }

    /// Steps on the start gains, or, when the searched term starts above `max` (a gain the
    /// search would never pick, which the relay's ceiling may have shown is too high), goes on
    /// without a baseline rather than fly it (#361) and commits the seed, its term at most
    /// `max`, in the start's place: a start above the relay's ceiling can ring once disturbed
    /// and never settle (#375). Those gains are what stays if nothing better passes.
    fn baseline(&mut self) -> Action {
        if self.plan.term.get(self.start) > self.max {
            self.start = self.with(self.plan.term.get(self.seed).min(self.max));
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
            Leg::Relay { .. } | Leg::Baseline => None,
        };
        Action::Probe {
            probe: self.pairs.probe(),
            gains,
            commit: None,
        }
    }

    /// The seed gains with the searched term at `gain`.
    fn with(&self, gain: f32) -> Gains {
        self.plan.term.with(self.seed, gain)
    }

    /// Judges the batch on the gain under test (`complete`: every step ran; else one
    /// aborted, too many were invalid, or the vehicle breached the envelope, which fails it)
    /// and moves on. A bracket point needs `confirm` passing pairs in a row.
    fn judge(&mut self, complete: bool) -> Action {
        let steps = self.pairs.summary;
        let passed = complete && self.plan.criterion.passes(&steps);
        let (gains, bracketed) = match self.leg {
            // The baseline is a bracket point only when the search starts from it. Incomplete,
            // it's no baseline: the start gains aborted or clipped.
            Leg::Baseline => {
                self.baseline = if complete {
                    steps
                } else {
                    StepSummary::default()
                };
                (self.start, self.seed == self.start)
            }
            Leg::Try { gain, .. } => (self.with(gain), true),
            Leg::Verify(gain) => return self.verified(gain, complete, steps),
            Leg::Relay { .. } => return Action::Fail(AutotuneAbort::Interrupted),
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
    /// themselves; else the start gains stay. A start that fails the criterion isn't a
    /// standard to hold the result to: a too-high rate P settles fast because it overshoots
    /// within the settle band (#361).
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
            Some(gain) => (Leg::Try { gain, passes: 0 }, 1),
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
