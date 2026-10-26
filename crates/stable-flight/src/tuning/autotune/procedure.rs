//! A stage's decisions, apart from the flying: see [`Procedure`].

use super::{AutotuneAbort, DesignPlan, Method, Outcome, RelayPlan, StepTestPlan};
use crate::controllers::Gains;
use crate::tuning::{Measurement, ProbeAbort, ProbeConfig, StepMetrics, StepSummary};

use search::Search;
use verify::Verify;

mod search;
#[cfg(test)]
mod tests;
mod verify;

/// Relays or fits in a row a slow host may stop before the stage fails: the step tests' clip
/// budget (`StepTestPlan::max_invalid`), as a search's relay leg retries.
const MAX_MEASURE_STOPS: usize = 3;

/// What a procedure is told.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Input {
    /// The stage starts; `gains` are its loop's current gains, `inner` those of the rate loop
    /// under an angle or heading loop (`None` for a rate loop), `run_start` its loop's gains at
    /// [`Autotune::start`](super::Autotune::start).
    Start {
        gains: Gains,
        inner: Option<Gains>,
        run_start: Gains,
    },
    /// A probe finished; `gains` are the loop's committed gains. `stalled`: the host fell
    /// behind during it ([`Autotune::note_stall`](super::Autotune::note_stall)), so its timing
    /// can't be trusted.
    Measured {
        measurement: Measurement,
        gains: Gains,
        stalled: bool,
    },
    /// A probe stopped because the host turned slow
    /// ([`Autotune::set_host_slow`](super::Autotune::set_host_slow)), with no measurement;
    /// `gains` are the loop's committed gains.
    Stalled { gains: Gains },
    /// A probe gave up; `gains` are the loop's committed gains.
    Aborted { abort: ProbeAbort, gains: Gains },
    /// A probe on candidate gains took the vehicle past the envelope; `gains` are the
    /// loop's committed gains, back in place.
    Breached { reason: AutotuneAbort, gains: Gains },
}

#[cfg(test)]
impl Input {
    /// The start of a stage on `gains`, which are also the run's start, on a rate loop.
    fn start(gains: Gains) -> Self {
        Self::Start {
            gains,
            inner: None,
            run_start: gains,
        }
    }
}

/// What the supervisor does next for the stage.
#[expect(
    clippy::large_enum_variant,
    reason = "returned once per decision, between ticks; the core has no allocator to box with"
)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Action {
    /// Commit `commit` first when given (it stays if the stage fails), settle on the
    /// committed gains, then run `probe` on the stage's loop, flying `gains` for that probe
    /// only when given.
    Probe {
        probe: ProbeConfig,
        gains: Option<Gains>,
        commit: Option<Gains>,
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
        Self::Probe {
            probe,
            gains: None,
            commit: None,
        }
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
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Procedure {
    /// One relay measurement, then the rule's gains; `stops` counts relays a slow host
    /// stopped in a row.
    Relay { plan: RelayPlan, stops: usize },
    /// Alternating steps, then the same gains.
    StepTest(Pairs),
    /// A gain search.
    Search(Search),
    /// A fit, then the gains designed from it; `stops` counts fits a slow host stopped in a
    /// row.
    Design {
        plan: DesignPlan,
        start: Gains,
        inner: Option<Gains>,
        stops: usize,
    },
    /// Steps on the committed gains, against the run's start when they miss.
    Verify(Verify),
}

impl Procedure {
    pub(super) fn new(method: Method) -> Self {
        match method {
            Method::Relay(plan) => Self::Relay { plan, stops: 0 },
            Method::StepTest(plan) => Self::StepTest(Pairs::new(plan, plan.pairs)),
            Method::Search(plan) => Self::Search(Search::new(plan)),
            Method::Design(plan) => Self::Design {
                plan,
                start: Gains::default(),
                inner: None,
                stops: 0,
            },
            Method::Verify(plan) => Self::Verify(Verify::new(plan)),
        }
    }

    pub(super) fn next(&mut self, input: Input) -> Action {
        match self {
            Self::Relay { plan, stops } => match input {
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
                // Nothing was measured: measure again, at most `MAX_MEASURE_STOPS` in a row.
                Input::Stalled { .. } => {
                    *stops += 1;
                    if *stops >= MAX_MEASURE_STOPS {
                        Action::Fail(AutotuneAbort::Stalled)
                    } else {
                        Action::probe(ProbeConfig::Relay(plan.relay))
                    }
                }
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
                Input::Stalled { .. } => match pairs.stall() {
                    Err(abort) => Action::Fail(abort),
                    Ok(()) => Action::probe(pairs.probe()),
                },
                Input::Aborted { abort, .. } => Action::Fail(AutotuneAbort::Probe(abort)),
                Input::Breached { reason, .. } => Action::Fail(reason),
                Input::Measured { .. } => Action::Fail(AutotuneAbort::Interrupted),
            },
            Self::Search(search) => search.next(input),
            Self::Design {
                plan,
                start,
                inner,
                stops,
            } => match input {
                Input::Start {
                    gains,
                    inner: under,
                    ..
                } => {
                    *start = gains;
                    *inner = under;
                    Action::probe(ProbeConfig::Identify(plan.identify))
                }
                Input::Measured {
                    measurement: Measurement::Identify(model),
                    stalled: false,
                    ..
                } => match plan.gains(&model, *start, *inner) {
                    Ok(gains) => Action::Finish {
                        gains,
                        outcome: Outcome::Tuned,
                        steps: None,
                        baseline: None,
                    },
                    Err(error) => Action::Fail(AutotuneAbort::Design(error)),
                },
                // The fit's timing can't be trusted, or nothing was fitted: fit again, at most
                // `MAX_MEASURE_STOPS` in a row.
                Input::Measured {
                    measurement: Measurement::Identify(_),
                    stalled: true,
                    ..
                }
                | Input::Stalled { .. } => {
                    *stops += 1;
                    if *stops >= MAX_MEASURE_STOPS {
                        Action::Fail(AutotuneAbort::Stalled)
                    } else {
                        Action::probe(ProbeConfig::Identify(plan.identify))
                    }
                }
                Input::Aborted { abort, .. } => Action::Fail(AutotuneAbort::Probe(abort)),
                Input::Breached { reason, .. } => Action::Fail(reason),
                Input::Measured { .. } => Action::Fail(AutotuneAbort::Interrupted),
            },
            Self::Verify(verify) => verify.next(input),
        }
    }

    /// The verdict the last [`next`](Self::next) reached, if any; taking it clears it.
    pub(super) fn take_verdict(&mut self) -> Option<Verdict> {
        match self {
            Self::Search(search) => search.take_verdict(),
            Self::Verify(verify) => verify.take_verdict(),
            Self::Relay { .. } | Self::StepTest(_) | Self::Design { .. } => None,
        }
    }
}

/// A batch of alternating steps: `+step` after an even count of valid steps, `−step` after an
/// odd one. A step clipped longer than the plan allows, or stalled, is left out and runs again
/// in the same direction; `max_invalid` clipped or `max_stalled` stalled in a row fail the
/// batch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Pairs {
    plan: StepTestPlan,
    /// Pairs in this batch.
    pairs: usize,
    /// Clipped steps in a row.
    invalid: usize,
    /// Stalled steps in a row.
    stalled: usize,
    /// The valid steps.
    summary: StepSummary,
}

impl Pairs {
    fn new(plan: StepTestPlan, pairs: usize) -> Self {
        Self {
            plan,
            pairs,
            invalid: 0,
            stalled: 0,
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

    /// Adds a step's metrics, or counts it stalled or clipped.
    ///
    /// # Errors
    ///
    /// [`AutotuneAbort::Stalled`] at `max_stalled` stalled steps in a row,
    /// [`AutotuneAbort::Invalid`] at `max_invalid` clipped steps in a row.
    fn record(&mut self, metrics: &StepMetrics, stalled: bool) -> Result<(), AutotuneAbort> {
        if stalled {
            return self.stall();
        }
        self.stalled = 0;
        if metrics.saturated_s > self.plan.max_saturated_s {
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

    /// Counts a stalled step, which runs again in the same direction.
    ///
    /// # Errors
    ///
    /// [`AutotuneAbort::Stalled`] at `max_stalled` stalled steps in a row.
    fn stall(&mut self) -> Result<(), AutotuneAbort> {
        self.stalled += 1;
        if self.stalled >= self.plan.max_stalled {
            return Err(AutotuneAbort::Stalled);
        }
        Ok(())
    }

    /// Whether the batch has all its valid steps.
    fn done(&self) -> bool {
        self.summary.count() >= 2 * self.pairs
    }
}
