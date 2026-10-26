//! A design stage: see [`Design`].

use super::{Action, Input, measure_again};
use crate::controllers::Gains;
use crate::tuning::autotune::{AutotuneAbort, DesignPlan, Outcome};
use crate::tuning::{Measurement, ProbeConfig};

/// A [`DesignPlan`] as a pure state machine: a fit, then the gains designed from it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(in crate::tuning::autotune) struct Design {
    plan: DesignPlan,
    /// The rate loop's gains under an angle loop (`None` for a rate loop).
    inner: Option<Gains>,
    /// Fits a slow host stopped in a row.
    stops: usize,
}

impl Design {
    pub(super) fn new(plan: DesignPlan) -> Self {
        Self {
            plan,
            inner: None,
            stops: 0,
        }
    }

    pub(super) fn next(&mut self, input: Input) -> Action {
        let identify = ProbeConfig::Identify(self.plan.identify);
        match input {
            Input::Start { inner, .. } => {
                self.inner = inner;
                Action::probe(identify)
            }
            // A design stage commits nothing before it finishes: `gains` are its start.
            Input::Measured {
                measurement: Measurement::Identify(model),
                gains,
                stalled: false,
            } => match self.plan.gains(&model, gains, self.inner) {
                Ok(gains) => Action::Finish {
                    gains,
                    outcome: Outcome::Tuned,
                    steps: None,
                    baseline: None,
                },
                Err(error) => Action::Fail(AutotuneAbort::Design(error)),
            },
            // The fit's timing can't be trusted, or nothing was fitted.
            Input::Measured {
                measurement: Measurement::Identify(_),
                stalled: true,
                ..
            }
            | Input::Stalled { .. } => measure_again(&mut self.stops, identify),
            Input::Aborted { abort, .. } => Action::Fail(AutotuneAbort::Probe(abort)),
            Input::Breached { reason, .. } => Action::Fail(reason),
            Input::Measured { .. } => Action::Fail(AutotuneAbort::Interrupted),
        }
    }
}
