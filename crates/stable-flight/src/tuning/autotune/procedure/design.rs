//! A design stage: see [`Design`].

use super::{Action, Input, measure_again};
use crate::controllers::Gains;
use crate::tuning::autotune::{AutotuneAbort, DesignPlan, Outcome};
use crate::tuning::{DesignError, Measurement, ProbeConfig};

/// Fits a design stage may take before it fails, usable or not: the two that must agree plus
/// two spares, for one unusable fit and one that disagrees.
const MAX_FITS: usize = 4;

/// How far apart two designs' P may be, as a fraction of the larger, and still agree. On
/// QuadCopterX in RealFlight the good fits of one axis designed rate P within 0.86–1.04 of
/// each other, and a bad roll fit 0.41 (#518).
const AGREEMENT: f32 = 0.25;

/// A [`DesignPlan`] as a pure state machine: fits until two of them design gains that agree,
/// then the more cautious of the pair (the lower P). A fit gets the design's error when it's
/// unusable (a fit across noise, a negative acceleration) and runs again; a stage whose
/// [`MAX_FITS`] fits never agree fails.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(in crate::tuning::autotune) struct Design {
    plan: DesignPlan,
    /// The rate loop's gains under an angle loop (`None` for a rate loop).
    inner: Option<Gains>,
    /// Fits a slow host stopped in a row.
    stops: usize,
    /// Fits the host kept time for, usable or not.
    fits: usize,
    /// The gains each usable fit designed, none agreeing with another.
    designs: [Option<Gains>; MAX_FITS],
}

impl Design {
    pub(super) fn new(plan: DesignPlan) -> Self {
        Self {
            plan,
            inner: None,
            stops: 0,
            fits: 0,
            designs: [None; MAX_FITS],
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
                self.stops = 0;
                let designed = self.plan.gains(&model, gains, self.inner);
                self.judge(designed)
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

    /// Finishes on the first earlier design `designed` agrees with, or fits again until
    /// [`MAX_FITS`].
    fn judge(&mut self, designed: Result<Gains, DesignError>) -> Action {
        let fit = self.fits;
        self.fits += 1;
        let reason = match designed {
            Ok(gains) => {
                let pair = self
                    .designs
                    .iter()
                    .flatten()
                    .find(|kept| agree(**kept, gains));
                if let Some(kept) = pair {
                    let cautious = if kept.p <= gains.p { *kept } else { gains };
                    return Action::Finish {
                        gains: cautious,
                        outcome: Outcome::Tuned,
                        steps: None,
                        baseline: None,
                    };
                }
                self.designs[fit] = Some(gains);
                AutotuneAbort::FitsDisagree
            }
            Err(error) => AutotuneAbort::Design(error),
        };
        if self.fits >= MAX_FITS {
            Action::Fail(reason)
        } else {
            Action::probe(ProbeConfig::Identify(self.plan.identify))
        }
    }
}

/// Whether two designs' P is within [`AGREEMENT`] of the larger.
fn agree(a: Gains, b: Gains) -> bool {
    (a.p - b.p).abs() <= AGREEMENT * a.p.max(b.p)
}
