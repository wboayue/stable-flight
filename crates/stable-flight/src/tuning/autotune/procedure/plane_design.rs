//! A plane design stage: see [`PlaneDesign`].

use super::{Action, MAX_MEASURE_STOPS};
use crate::stabilizers::plane::PlaneAxisGains;
use crate::tuning::autotune::side::Decide;
use crate::tuning::autotune::{AutotuneAbort, Outcome, PlaneDesignPlan};
use crate::tuning::wing::design::meets_targets;
use crate::tuning::{
    DesignError, Measurement, ProbeAbort, ProbeConfig, StepMetrics, StepSummary, WingDesign,
    WingDesignConfig, WingDesignStatus,
};

#[cfg(test)]
mod tests;

/// Fits a design stage may lose before it fails: a fit that doesn't solve, or a model no gains
/// fly, runs again (the copter design's three, `MAX_UNUSABLE_FITS`).
const MAX_UNUSABLE_FITS: usize = 3;

/// What a sagging pair multiplies I by, at least [`MIN_SAG_I`]: a held attitude whose airspeed
/// bleeds off needs more I than the fit at constant airspeed asks for (#533).
const SAG_I_FACTOR: f32 = 1.5;

/// The I a sagging pair's raise starts from when the design had none.
const MIN_SAG_I: f32 = 0.05;

/// The most of its overshoot a step that holds past its target gives back, as a share: one
/// that ends this close to its peak isn't swinging past, it's short of a steady moment the
/// fit didn't see, which I supplies. The Titan Cobra's 30° roll steps in RealFlight peak
/// 0.10–0.15 past and give back 0.01–0.03, against a 0.08 aileron moment the 22 m turn needs
/// (#635).
const HOLD_PAST_BOUNCE_SHARE: f32 = 0.5;

/// What a pair that overshoots or clips multiplies the time constant by: a slower attitude loop
/// asks for less rate, so less overshoot and less deflection (#533).
const OVERSHOOT_TAU_FACTOR: f32 = 1.15;

/// What a pair that settles but rises too slowly multiplies P by: one step of the design's
/// grid.
const SLOW_P_FACTOR: f32 = 1.66;

/// A [`PlaneDesignPlan`] as a pure state machine: a wing fit of the stage's axis, gains
/// designed on it over several updates, then attitude pairs flown on the candidate until one
/// meets the targets, each miss nudging the candidate (a sag or a hold past the target raises
/// I, a slow rise P, an overshoot or clip the time constant).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(in crate::tuning::autotune) struct PlaneDesign {
    plan: PlaneDesignPlan,
    step: Step,
    /// Probes a slow host stopped in a row.
    stops: usize,
    /// Fits the design couldn't use.
    unusable: usize,
}

#[expect(
    clippy::large_enum_variant,
    reason = "one per stage, replaced between stages; the core has no allocator to box with"
)]
#[derive(Clone, Copy, Debug, PartialEq)]
enum Step {
    Fitting,
    Designing(WingDesign),
    /// Flying pair `pair` (from 0) on `candidate`; `first` is the pair's first step, once
    /// measured.
    Verifying {
        candidate: PlaneAxisGains,
        pair: usize,
        first: Option<StepMetrics>,
    },
}

impl PlaneDesign {
    pub(super) fn new(plan: PlaneDesignPlan) -> Self {
        Self {
            plan,
            step: Step::Fitting,
            stops: 0,
            unusable: 0,
        }
    }

    pub(super) fn next(&mut self, on: Decide) -> Action<PlaneAxisGains> {
        match (self.step, on) {
            (Step::Fitting, Decide::Start) => self.fit(),
            (
                Step::Fitting,
                Decide::Measured {
                    measurement: Measurement::Wing(model),
                    stalled: false,
                },
            ) => {
                self.stops = 0;
                self.step = Step::Designing(WingDesign::new(model, self.plan.design));
                Action::Compute
            }
            (
                Step::Fitting,
                Decide::Measured {
                    measurement: Measurement::Wing(_),
                    stalled: true,
                },
            )
            | (Step::Fitting, Decide::Stalled) => self.again(Action::probe(self.identify())),
            (
                Step::Fitting,
                Decide::Aborted {
                    abort: ProbeAbort::Identify(abort),
                },
            ) => self.refit(AutotuneAbort::Probe(ProbeAbort::Identify(abort))),
            (Step::Designing(mut design), Decide::Computing) => {
                match design.advance(self.plan.candidates_per_update) {
                    WingDesignStatus::Running => {
                        self.step = Step::Designing(design);
                        Action::Compute
                    }
                    WingDesignStatus::Done(candidate) => self.verify(candidate, 0),
                    WingDesignStatus::Failed => {
                        self.refit(AutotuneAbort::Design(DesignError::NoWingDesign))
                    }
                }
            }
            // Settled on the candidate: step it.
            (
                Step::Verifying {
                    candidate, first, ..
                },
                Decide::Held { settled: true },
            ) => Action::Probe {
                probe: self.verify_step(first.is_some()),
                gains: Some(candidate),
                commit: None,
            },
            (
                Step::Verifying {
                    candidate,
                    pair,
                    first,
                },
                Decide::Measured {
                    measurement: Measurement::Step(metrics),
                    stalled,
                },
            ) => {
                if stalled {
                    return self.again(Action::Hold { gains: candidate });
                }
                self.stops = 0;
                match first {
                    None => {
                        self.step = Step::Verifying {
                            candidate,
                            pair,
                            first: Some(metrics),
                        };
                        Action::Hold { gains: candidate }
                    }
                    Some(first) => self.judge(candidate, pair, [first, metrics]),
                }
            }
            (Step::Verifying { candidate, .. }, Decide::Stalled) => {
                self.again(Action::Hold { gains: candidate })
            }
            // A candidate that won't hold still, runs away, or leaves the envelope asks for too
            // much: slower.
            (
                Step::Verifying {
                    candidate, pair, ..
                },
                Decide::Held { settled: false }
                | Decide::Aborted {
                    abort: ProbeAbort::Step(_),
                }
                | Decide::Breached { .. },
            ) => self.verify(slower(candidate, &self.plan), pair + 1),
            (_, Decide::Breached { reason }) => Action::Fail(reason),
            (_, Decide::Unsettled) => Action::Fail(AutotuneAbort::NotSettled),
            (_, Decide::Aborted { abort }) => Action::Fail(AutotuneAbort::Probe(abort)),
            _ => Action::Fail(AutotuneAbort::Interrupted),
        }
    }

    fn identify(&self) -> ProbeConfig {
        ProbeConfig::IdentifyWing(self.plan.identify)
    }

    /// Starts a fit.
    fn fit(&mut self) -> Action<PlaneAxisGains> {
        self.step = Step::Fitting;
        Action::probe(self.identify())
    }

    /// A fit the design couldn't use: fit again, or fail with `abort` at
    /// [`MAX_UNUSABLE_FITS`].
    fn refit(&mut self, abort: AutotuneAbort) -> Action<PlaneAxisGains> {
        self.unusable += 1;
        if self.unusable >= MAX_UNUSABLE_FITS {
            return Action::Fail(abort);
        }
        self.fit()
    }

    /// A probe a slow host spoiled: `retry`, or fail the stage at `MAX_MEASURE_STOPS` in a
    /// row.
    fn again(&mut self, retry: Action<PlaneAxisGains>) -> Action<PlaneAxisGains> {
        self.stops += 1;
        if self.stops >= MAX_MEASURE_STOPS {
            return Action::Fail(AutotuneAbort::Stalled);
        }
        retry
    }

    /// Starts pair `pair` on `candidate`, or fails the stage past the plan's pairs. Each step
    /// first holds the candidate until its axis settles on it: the rate loop's integral then
    /// holds what the candidate needs, where one carried over from the committed gains biases
    /// the step. From three times its gains, the airplane's committed roll loop settled with
    /// an I term its candidate's tenth-as-large I took seconds to unwind, and every pair's
    /// second step rose in 3.6 s against the first's 1.1 (#534).
    fn verify(&mut self, candidate: PlaneAxisGains, pair: usize) -> Action<PlaneAxisGains> {
        if pair >= self.plan.max_verify_pairs {
            return Action::Fail(AutotuneAbort::Unverified);
        }
        self.step = Step::Verifying {
            candidate,
            pair,
            first: None,
        };
        Action::Hold { gains: candidate }
    }

    /// The verify step: the plan's, or reversed for a pair's second.
    fn verify_step(&self, second: bool) -> ProbeConfig {
        let step = self.plan.verify;
        ProbeConfig::Step(if second { step.reversed() } else { step })
    }

    /// Finishes on `candidate` when both of `pair`'s steps meet the targets; else flies the
    /// next pair on it nudged: more I for a sag (a step that doesn't settle) or a step that
    /// holds past its target (overshoots and gives little of it back), more P for a
    /// step that settles but rises too slowly (the rate loop lags its target), else (an
    /// overshoot or clip) a slower time constant. A slower time constant doesn't speed a slow
    /// rise: the airplane's roll rose in 2.0, 2.5, 3.0, and 3.5 time constants at 0.56, 0.64,
    /// 0.74, and 0.85 s (#534).
    fn judge(
        &mut self,
        candidate: PlaneAxisGains,
        pair: usize,
        steps: [StepMetrics; 2],
    ) -> Action<PlaneAxisGains> {
        let tau_s = candidate.time_constant_s;
        let design = &self.plan.design;
        if steps.iter().all(|m| meets_targets(m, tau_s, design)) {
            let mut summary = StepSummary::default();
            steps.iter().for_each(|m| summary.add(m));
            return Action::Finish {
                gains: candidate,
                outcome: Outcome::Tuned,
                steps: Some(summary),
                baseline: None,
            };
        }
        let sags = steps
            .iter()
            .any(|m| m.settling_time_s.is_none() || holds_past(m, design));
        let max_rise_s = design.rise_per_time_constant * tau_s;
        let slow = steps
            .iter()
            .any(|m| m.rise_time_s.is_none_or(|rise_s| rise_s > max_rise_s));
        let wide = steps
            .iter()
            .any(|m| m.overshoot > design.max_overshoot || m.saturated_s > design.max_saturated_s);
        let mut next = candidate;
        if sags {
            next.rate.i = (candidate.rate.i * SAG_I_FACTOR).max(MIN_SAG_I);
        } else if slow && !wide {
            next.rate.p = candidate.rate.p * SLOW_P_FACTOR;
        } else {
            next = slower(candidate, &self.plan);
        }
        self.verify(next, pair + 1)
    }
}

/// Whether a step overshoots past `design`'s limit and keeps most of it: an offset a slower
/// time constant only widens (the error then carries the moment), where a swing past the target
/// gives most of its overshoot back.
fn holds_past(metrics: &StepMetrics, design: &WingDesignConfig) -> bool {
    metrics.overshoot > design.max_overshoot
        && metrics.bounce_back < HOLD_PAST_BOUNCE_SHARE * metrics.overshoot
}

/// `candidate` with its time constant raised by [`OVERSHOOT_TAU_FACTOR`], up to the plan's
/// slowest.
fn slower(candidate: PlaneAxisGains, plan: &PlaneDesignPlan) -> PlaneAxisGains {
    let most_s = plan.design.time_constant_range_s[1];
    PlaneAxisGains {
        time_constant_s: (candidate.time_constant_s * OVERSHOOT_TAU_FACTOR).min(most_s),
        ..candidate
    }
}
