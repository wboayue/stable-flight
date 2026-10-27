//! A design stage: see [`Design`].

use super::{Action, Input, measure_again};
use crate::controllers::Gains;
use crate::tuning::autotune::{AutotuneAbort, DesignPlan, Outcome};
use crate::tuning::{Measurement, ProbeConfig};

/// Fits the design can't use a design stage may take before it fails: on QuadCopterX in
/// RealFlight a fit with a negative acceleration came up in 4 of 10 runs, never twice in a
/// stage (#518).
const MAX_UNUSABLE_FITS: usize = 3;

/// A [`DesignPlan`] as a pure state machine: a fit, then the gains designed from it. A fit
/// the design can't use runs again, up to [`MAX_UNUSABLE_FITS`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub(in crate::tuning::autotune) struct Design {
    plan: DesignPlan,
    /// The rate loop's gains under an angle loop (`None` for a rate loop).
    inner: Option<Gains>,
    /// Fits a slow host stopped in a row.
    stops: usize,
    /// Fits the design couldn't use.
    unusable: usize,
}

impl Design {
    pub(super) fn new(plan: DesignPlan) -> Self {
        Self {
            plan,
            inner: None,
            stops: 0,
            unusable: 0,
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
            } => {
                // A fit that ran clears the slow host's count: only stalls in a row fail.
                self.stops = 0;
                match self.plan.gains(&model, gains, self.inner) {
                    Ok(gains) => Action::Finish {
                        gains,
                        outcome: Outcome::Designed,
                        steps: None,
                        baseline: None,
                    },
                    Err(error) => {
                        self.unusable += 1;
                        if self.unusable >= MAX_UNUSABLE_FITS {
                            Action::Fail(AutotuneAbort::Design(error))
                        } else {
                            Action::probe(identify)
                        }
                    }
                }
            }
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
