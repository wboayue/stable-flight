//! A design stage: see [`Design`].

use super::{Action, Input, measure_again};
use crate::controllers::Gains;
use crate::tuning::autotune::{AutotuneAbort, DesignPlan, Outcome};
use crate::tuning::{IdentifyAbort, Measurement, ProbeAbort, ProbeConfig};

/// Fits the design can't use a design stage may take before it fails: on QuadCopterX in
/// RealFlight a fit with a negative acceleration came up in 4 of 10 runs, never twice in a
/// stage (#518).
const MAX_UNUSABLE_FITS: usize = 3;

/// What a design stage divides its start's gains by when its fit saturates under noise, as a
/// search lowers a start that won't settle (the copter brackets' factor, #552).
const SATURATED_FACTOR: f32 = 1.3;

/// Most times a design stage lowers a start whose fit saturates: three cuts halve it (÷ 2.2),
/// as a search's do (#552). Up to six didn't unclip the sim's combined rows from three times
/// a quad's rate gains, where the D turns the 90 Hz gyro tone into clipped moments (#551).
const MAX_SATURATED_LOWERINGS: usize = 3;

/// A [`DesignPlan`] as a pure state machine: a fit, the gains designed from it, then a hold
/// on them. A fit the design can't use, or whose design doesn't hold, runs again, up to
/// [`MAX_UNUSABLE_FITS`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub(in crate::tuning::autotune) struct Design {
    plan: DesignPlan,
    /// The rate loop's gains under an angle loop (`None` for a rate loop).
    inner: Option<Gains>,
    /// The stabilizer's gyro low-pass, in Hz, which the design models.
    gyro_cutoff_hz: f32,
    /// The design on hold.
    designed: Gains,
    /// Fits a slow host stopped in a row.
    stops: usize,
    /// Fits the design couldn't use, or whose design didn't hold.
    unusable: usize,
    /// Times the start was lowered for a fit that saturated.
    lowerings: usize,
}

impl Design {
    pub(super) fn new(plan: DesignPlan) -> Self {
        Self {
            plan,
            inner: None,
            gyro_cutoff_hz: 0.0,
            designed: Gains::default(),
            stops: 0,
            unusable: 0,
            lowerings: 0,
        }
    }

    pub(super) fn next(&mut self, input: Input) -> Action {
        let identify = ProbeConfig::Identify(self.plan.identify);
        match input {
            Input::Start {
                inner,
                gyro_cutoff_hz,
                ..
            } => {
                self.inner = inner;
                self.gyro_cutoff_hz = gyro_cutoff_hz;
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
                match self
                    .plan
                    .gains(&model, gains, self.inner, self.gyro_cutoff_hz)
                {
                    Ok(designed) => {
                        self.designed = designed;
                        Action::Hold { gains: designed }
                    }
                    Err(error) => self.refit(AutotuneAbort::Design(error)),
                }
            }
            // The fit's timing can't be trusted, or nothing was fitted.
            Input::Measured {
                measurement: Measurement::Identify(_),
                stalled: true,
                ..
            }
            | Input::Stalled { .. } => measure_again(&mut self.stops, identify),
            Input::Held { settled: true, .. } => Action::Finish {
                gains: self.designed,
                outcome: Outcome::Designed,
                steps: None,
                baseline: None,
            },
            // A design that rings or breaches came from a fit that misread the axis: on
            // QuadCopterX in RealFlight, fits reading 8 and 19 rad/s² against 25–71 designed
            // pitch rate P 3.93 and 2.30, which rang at 0.6 rad/s and kept the stages after
            // them from settling (#518).
            Input::Held { settled: false, .. } => self.refit(AutotuneAbort::NotSettled),
            Input::Breached { reason, .. } => self.refit(reason),
            Input::Aborted {
                abort: ProbeAbort::Identify(IdentifyAbort::Saturated),
                gains,
            } => self.lower_saturated(gains),
            Input::Aborted { abort, .. } => Action::Fail(AutotuneAbort::Probe(abort)),
            Input::Unsettled { .. } => Action::Fail(AutotuneAbort::NotSettled),
            Input::Measured { .. } => Action::Fail(AutotuneAbort::Interrupted),
        }
    }

    /// Lowers a start whose fit saturated under noise ([`IdentifyAbort::Saturated`]), every
    /// gain `÷ SATURATED_FACTOR`, commits it, and fits again on it; fails after
    /// [`MAX_SATURATED_LOWERINGS`]. Under gyro noise a start at three times a quad's rate gains
    /// rings at the rate loop's limit, and its fit designed rate P 8.1 against 1.9 (#551).
    fn lower_saturated(&mut self, gains: Gains) -> Action {
        if self.lowerings >= MAX_SATURATED_LOWERINGS {
            return Action::Fail(AutotuneAbort::Probe(ProbeAbort::Identify(
                IdentifyAbort::Saturated,
            )));
        }
        self.lowerings += 1;
        let lowered = Gains {
            p: gains.p / SATURATED_FACTOR,
            i: gains.i / SATURATED_FACTOR,
            d: gains.d / SATURATED_FACTOR,
        };
        Action::Probe {
            probe: ProbeConfig::Identify(self.plan.identify),
            gains: None,
            commit: Some(lowered),
        }
    }

    /// Counts an unusable fit: fit again, or fail with `abort` at [`MAX_UNUSABLE_FITS`].
    fn refit(&mut self, abort: AutotuneAbort) -> Action {
        self.unusable += 1;
        if self.unusable >= MAX_UNUSABLE_FITS {
            Action::Fail(abort)
        } else {
            Action::probe(ProbeConfig::Identify(self.plan.identify))
        }
    }
}
