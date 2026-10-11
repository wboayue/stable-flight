//! A stage's decisions, apart from the flying: see [`Procedure`].

use super::{AutotuneAbort, Method, Outcome, RelayPlan, StepTestPlan};
use crate::controllers::Gains;
use crate::tuning::{Measurement, ProbeAbort, ProbeConfig, StepMetrics, StepSummary};

use search::Search;

mod search;
#[cfg(test)]
mod tests;

/// What a procedure is told.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Input {
    /// The stage starts; `gains` are its loop's current gains.
    Start { gains: Gains },
    /// A probe finished; `gains` are the loop's committed gains. `stalled`: the host fell
    /// behind during it ([`Autotune::note_stall`](super::Autotune::note_stall)), so its timing
    /// can't be trusted.
    Measured {
        measurement: Measurement,
        gains: Gains,
        stalled: bool,
    },
    /// A probe gave up; `gains` are the loop's committed gains.
    Aborted { abort: ProbeAbort, gains: Gains },
    /// A probe on candidate gains took the vehicle past the envelope; `gains` are the
    /// loop's committed gains, back in place.
    Breached { reason: AutotuneAbort, gains: Gains },
}

/// What the supervisor does next for the stage.
#[expect(
    clippy::large_enum_variant,
    reason = "returned once per decision, between ticks; the core has no allocator to box with"
)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Action {
    /// Settle on the committed gains, then run `probe` on the stage's loop, flying `gains`
    /// for that probe only when given.
    Probe {
        probe: ProbeConfig,
        gains: Option<Gains>,
    },
    /// Commit these gains for the stage's loop and end the stage; a step test reports its
    /// valid steps, a search its verify and baseline.
    Finish {
        gains: Gains,
        outcome: Outcome,
        steps: Option<StepSummary>,
        baseline: Option<StepSummary>,
    },
    /// End the run: the stage can't go on.
    Fail(AutotuneAbort),
}

impl Action {
    /// Runs `probe` on the committed gains.
    fn probe(probe: ProbeConfig) -> Self {
        Self::Probe { probe, gains: None }
    }
}

/// A search's judgment of a pair of steps (or its baseline) on `gains`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Verdict {
    pub(super) gains: Gains,
    pub(super) passed: bool,
    pub(super) steps: StepSummary,
}

/// A stage's method as a pure state machine: fed what happened, it returns what to do next.
/// It never sees the flight loop; the supervisor settles, probes, guards the envelope, and
/// commits, so a procedure is tested by feeding it inputs.
#[expect(
    clippy::large_enum_variant,
    reason = "one per supervisor, never moved per tick; the core has no allocator to box with"
)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Procedure {
    /// One relay measurement, then the rule's gains.
    Relay(RelayPlan),
    /// Alternating steps, then the same gains.
    StepTest(Pairs),
    /// A gain search.
    Search(Search),
}

impl Procedure {
    pub(super) fn new(method: Method) -> Self {
        match method {
            Method::Relay(plan) => Self::Relay(plan),
            Method::StepTest(plan) => Self::StepTest(Pairs::new(plan, plan.pairs)),
            Method::Search(plan) => Self::Search(Search::new(plan)),
        }
    }

    pub(super) fn next(&mut self, input: Input) -> Action {
        match self {
            Self::Relay(plan) => match input {
                Input::Start { .. } => Action::probe(ProbeConfig::Relay(plan.relay)),
                Input::Measured {
                    measurement: Measurement::Relay(result),
                    gains,
                    ..
                } => Action::Finish {
                    gains: plan.gains(&result, gains),
                    outcome: Outcome::Tuned,
                    steps: None,
                    baseline: None,
                },
                Input::Aborted { abort, .. } => Action::Fail(AutotuneAbort::Probe(abort)),
                Input::Breached { reason, .. } => Action::Fail(reason),
                Input::Measured { .. } => Action::Fail(AutotuneAbort::Interrupted),
            },
            Self::StepTest(pairs) => match input {
                Input::Start { .. } => Action::probe(pairs.probe()),
                Input::Measured {
                    measurement: Measurement::Step(metrics),
                    gains,
                    stalled,
                } => match pairs.record(&metrics, stalled) {
                    Err(abort) => Action::Fail(abort),
                    Ok(()) if pairs.done() => Action::Finish {
                        gains,
                        outcome: Outcome::Measured,
                        steps: Some(pairs.summary),
                        baseline: None,
                    },
                    Ok(()) => Action::probe(pairs.probe()),
                },
                Input::Aborted { abort, .. } => Action::Fail(AutotuneAbort::Probe(abort)),
                Input::Breached { reason, .. } => Action::Fail(reason),
                Input::Measured { .. } => Action::Fail(AutotuneAbort::Interrupted),
            },
            Self::Search(search) => search.next(input),
        }
    }

    /// The verdict the last [`next`](Self::next) reached, if any; taking it clears it.
    pub(super) fn take_verdict(&mut self) -> Option<Verdict> {
        match self {
            Self::Search(search) => search.take_verdict(),
            Self::Relay(_) | Self::StepTest(_) => None,
        }
    }
}

/// A batch of alternating steps: `+step` after an even count of valid steps, `−step` after an
/// odd one. A step clipped longer than the plan allows, or stalled, is invalid: it is left out
/// and runs again in the same direction, and `max_invalid` in a row fail the stage.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Pairs {
    plan: StepTestPlan,
    /// Pairs in this batch.
    pairs: usize,
    /// Invalid steps since the last valid one.
    invalid: usize,
    /// The valid steps.
    summary: StepSummary,
}

impl Pairs {
    fn new(plan: StepTestPlan, pairs: usize) -> Self {
        Self {
            plan,
            pairs,
            invalid: 0,
            summary: StepSummary::default(),
        }
    }

    /// A fresh batch of `pairs` pairs.
    fn restart(&mut self, pairs: usize) {
        *self = Self::new(self.plan, pairs);
    }

    /// The next step.
    fn probe(&self) -> ProbeConfig {
        let step = if self.summary.count().is_multiple_of(2) {
            self.plan.step
        } else {
            self.plan.step.reversed()
        };
        ProbeConfig::Step(step)
    }

    /// Adds a step's metrics, or counts it invalid.
    ///
    /// # Errors
    ///
    /// [`AutotuneAbort::Invalid`] at `max_invalid` invalid steps in a row.
    fn record(&mut self, metrics: &StepMetrics, stalled: bool) -> Result<(), AutotuneAbort> {
        if stalled || metrics.saturated_s > self.plan.max_saturated_s {
            self.invalid += 1;
            if self.invalid >= self.plan.max_invalid {
                return Err(AutotuneAbort::Invalid);
            }
            return Ok(());
        }
        self.invalid = 0;
        self.summary.add(metrics);
        Ok(())
    }

    /// Whether the batch has all its valid steps.
    fn done(&self) -> bool {
        self.summary.count() >= 2 * self.pairs
    }
}
