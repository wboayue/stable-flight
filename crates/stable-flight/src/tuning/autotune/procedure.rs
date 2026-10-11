//! A stage's decisions, apart from the flying: see [`Procedure`].

use super::{Method, RelayPlan};
use crate::controllers::Gains;
use crate::tuning::{Measurement, ProbeConfig};

#[cfg(test)]
mod tests;

/// What a procedure is told.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Input {
    /// The stage starts; `gains` are its loop's current gains.
    Start { gains: Gains },
    /// A probe finished; `gains` are the loop's gains it flew on.
    Measured {
        measurement: Measurement,
        gains: Gains,
    },
}

/// What the supervisor does next for the stage.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Action {
    /// Settle, then run this probe on the stage's loop.
    Probe(ProbeConfig),
    /// Commit these gains for the stage's loop and end the stage.
    Finish(Gains),
}

/// A stage's method as a pure state machine: fed what happened, it returns what to do next.
/// It never sees the flight loop; the supervisor settles, probes, guards the envelope, and
/// commits, so a procedure is tested by feeding it inputs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Procedure {
    /// One relay measurement, then the rule's gains.
    Relay(RelayPlan),
}

impl Procedure {
    pub(super) fn new(method: Method) -> Self {
        match method {
            Method::Relay(plan) => Self::Relay(plan),
        }
    }

    pub(super) fn next(&mut self, input: Input) -> Action {
        match self {
            Self::Relay(plan) => match input {
                Input::Start { .. } => Action::Probe(ProbeConfig::Relay(plan.relay)),
                Input::Measured {
                    measurement: Measurement::Relay(result),
                    gains,
                } => Action::Finish(plan.gains(&result, gains)),
            },
        }
    }
}
