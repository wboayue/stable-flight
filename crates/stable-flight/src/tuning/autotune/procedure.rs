//! A stage's decisions, apart from the flying: see [`Procedure`].

use super::{AutotuneAbort, Method, RelayPlan, StepTestPlan};
use crate::controllers::Gains;
use crate::tuning::{Measurement, ProbeConfig, StepSummary};

#[cfg(test)]
mod tests;

/// What a procedure is told.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Input {
    /// The stage starts; `gains` are its loop's current gains.
    Start { gains: Gains },
    /// A probe finished; `gains` are the loop's gains it flew on. `stalled`: the host fell
    /// behind during it ([`Autotune::note_stall`](super::Autotune::note_stall)), so its timing
    /// can't be trusted.
    Measured {
        measurement: Measurement,
        gains: Gains,
        stalled: bool,
    },
}

/// What the supervisor does next for the stage.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Action {
    /// Settle, then run this probe on the stage's loop.
    Probe(ProbeConfig),
    /// Commit these gains for the stage's loop and end the stage; a step test also reports
    /// its valid steps.
    Finish {
        gains: Gains,
        steps: Option<StepSummary>,
    },
    /// End the run: the stage can't go on.
    Fail(AutotuneAbort),
}

/// A stage's method as a pure state machine: fed what happened, it returns what to do next.
/// It never sees the flight loop; the supervisor settles, probes, guards the envelope, and
/// commits, so a procedure is tested by feeding it inputs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Procedure {
    /// One relay measurement, then the rule's gains.
    Relay(RelayPlan),
    /// Alternating steps, then the same gains.
    StepTest(StepTest),
}

/// A step test's progress.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct StepTest {
    plan: StepTestPlan,
    /// Invalid steps since the last valid one.
    invalid: usize,
    /// The valid steps.
    summary: StepSummary,
}

impl Procedure {
    pub(super) fn new(method: Method) -> Self {
        match method {
            Method::Relay(plan) => Self::Relay(plan),
            Method::StepTest(plan) => Self::StepTest(StepTest {
                plan,
                invalid: 0,
                summary: StepSummary::default(),
            }),
        }
    }

    pub(super) fn next(&mut self, input: Input) -> Action {
        match self {
            Self::Relay(plan) => match input {
                Input::Start { .. } => Action::Probe(ProbeConfig::Relay(plan.relay)),
                Input::Measured {
                    measurement: Measurement::Relay(result),
                    gains,
                    ..
                } => Action::Finish {
                    gains: plan.gains(&result, gains),
                    steps: None,
                },
                Input::Measured { .. } => Action::Fail(AutotuneAbort::Interrupted),
            },
            Self::StepTest(test) => test.next(input),
        }
    }
}

impl StepTest {
    fn next(&mut self, input: Input) -> Action {
        let (metrics, gains, stalled) = match input {
            Input::Start { .. } => return self.probe(),
            Input::Measured {
                measurement: Measurement::Step(metrics),
                gains,
                stalled,
            } => (metrics, gains, stalled),
            Input::Measured { .. } => return Action::Fail(AutotuneAbort::Interrupted),
        };
        if stalled || metrics.saturated_s > self.plan.max_saturated_s {
            self.invalid += 1;
            if self.invalid >= self.plan.max_invalid {
                return Action::Fail(AutotuneAbort::Invalid);
            }
            return self.probe();
        }
        self.invalid = 0;
        self.summary.add(&metrics);
        if self.summary.count() >= 2 * self.plan.pairs {
            return Action::Finish {
                gains,
                steps: Some(self.summary),
            };
        }
        self.probe()
    }

    /// The next step: positive after an even count of valid steps, negative after an odd one
    /// (an invalid step runs again in the same direction).
    fn probe(&self) -> Action {
        let step = if self.summary.count().is_multiple_of(2) {
            self.plan.step
        } else {
            self.plan.step.reversed()
        };
        Action::Probe(ProbeConfig::Step(step))
    }
}
