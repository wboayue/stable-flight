//! A stage's decisions, apart from the flying: see [`Procedure`].

use super::side::Decide;
use super::{AutotuneAbort, Method, Outcome, PlaneMethod, StepTestPlan};
use crate::controllers::{Gains, GyroFilters};
use crate::stabilizers::plane::PlaneAxisGains;
use crate::tuning::{Measurement, ProbeAbort, ProbeConfig, StepMetrics, StepSummary};

use design::Design;
use search::Search;

mod design;
mod search;
#[cfg(test)]
mod tests;

/// Fits in a row a slow host may stop before the stage fails: the step tests' clip budget
/// (`StepTestPlan::max_invalid`).
const MAX_MEASURE_STOPS: usize = 3;

/// Most times a stage lowers its start: a search's that won't settle, a rate design's whose fit
/// saturates under noise. Each cut is 1.3 on the copter, so eight take the start to an eighth
/// (÷ 8.2): 80 s for a search at the copter envelope's 10 s settle timeout, about 104 s for a
/// design (a settle and a 3 s fit each). The retry on a smaller start is ArduPilot's on a
/// smaller twitch: `AC_AutoTune` shrinks its twitch (`step_scaler`) ×0.9 per retry, down to a
/// fifth. Under gyro noise and 10 ms of delay, three times a quad's rate gains ring at the rate
/// loop's limit until lowered 6 times (`quad_x`) or 7 (`heavy_quad`). `racer`'s default yaw at
/// 25 ms of delay settles after one, and capped anywhere from 1 to 10 the robustness rows that
/// pass passed the same (#552, #598).
const MAX_START_LOWERINGS: usize = 8;

/// What a procedure is told.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Input {
    /// The stage starts; `gains` are its loop's current gains, `inner` those of the rate loop
    /// under an angle or heading loop (`None` for a rate loop), `run_start` its loop's gains at
    /// [`Autotune::start`](super::Autotune::start), and `gyro` the stabilizer's gyro filters,
    /// which a design models.
    Start {
        gains: Gains,
        inner: Option<Gains>,
        run_start: Gains,
        gyro: GyroFilters,
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
    /// A probe or a hold on candidate gains took the vehicle past the envelope; `gains` are
    /// the loop's committed gains, back in place.
    Breached { reason: AutotuneAbort, gains: Gains },
    /// A hold ended: `settled` when the vehicle held the settle on the candidate, which still
    /// flies; else it timed out and the committed `gains` are back.
    Held { settled: bool, gains: Gains },
    /// The vehicle didn't hold the settle on the committed `gains` within
    /// [`Envelope::settle_timeout_s`](super::Envelope::settle_timeout_s), before a probe.
    Unsettled { gains: Gains },
}

impl Input {
    /// What happened, as the supervisor saw it, and the loop's gains.
    fn split(self) -> (Decide, Gains) {
        match self {
            Self::Start { gains, .. } => (Decide::Start, gains),
            Self::Measured {
                measurement,
                gains,
                stalled,
            } => (
                Decide::Measured {
                    measurement,
                    stalled,
                },
                gains,
            ),
            Self::Stalled { gains } => (Decide::Stalled, gains),
            Self::Aborted { abort, gains } => (Decide::Aborted { abort }, gains),
            Self::Breached { reason, gains } => (Decide::Breached { reason }, gains),
            Self::Held { settled, gains } => (Decide::Held { settled }, gains),
            Self::Unsettled { gains } => (Decide::Unsettled, gains),
        }
    }
}

#[cfg(test)]
use crate::stabilizers::copter::DEFAULT_GYRO_CUTOFF_HZ;

#[cfg(test)]
impl Input {
    /// The start of a stage on `gains`, which are also the run's start, on a rate loop.
    fn start(gains: Gains) -> Self {
        Self::Start {
            gains,
            inner: None,
            run_start: gains,
            gyro: GyroFilters::low_pass(DEFAULT_GYRO_CUTOFF_HZ),
        }
    }
}

/// What the supervisor does next for the stage.
#[expect(
    clippy::large_enum_variant,
    reason = "returned once per decision, between ticks; the core has no allocator to box with"
)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Action<G = Gains> {
    /// Commit `commit` first when given (it stays if the stage fails), settle on the
    /// committed gains, then run `probe` on the stage's loop, flying `gains` for that probe
    /// only when given.
    Probe {
        probe: ProbeConfig,
        gains: Option<G>,
        commit: Option<G>,
    },
    /// Commit these gains for the stage's loop and end the stage; a step test reports its
    /// valid steps, a search its verify and baseline.
    Finish {
        gains: G,
        outcome: Outcome,
        steps: Option<StepSummary>,
        baseline: Option<StepSummary>,
    },
    /// Fly `gains` until the stage's axis holds the settle on them, or the settle timeout;
    /// [`Input::Held`] reports which.
    Hold { gains: G },
    /// End the run: the stage can't go on.
    Fail(AutotuneAbort),
}

impl<G> Action<G> {
    /// Runs `probe` on the committed gains.
    pub(super) fn probe(probe: ProbeConfig) -> Self {
        Self::Probe {
            probe,
            gains: None,
            commit: None,
        }
    }
}

/// A search's judgment of a pair of steps (or its baseline) on `gains`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Verdict<G = Gains> {
    pub(super) gains: G,
    pub(super) passed: bool,
    pub(super) steps: StepSummary,
}

/// A stage's method as a pure state machine: fed what happened, it returns what to do next.
/// It never sees the flight loop; the supervisor settles, probes, guards the envelope, and
/// commits, so a procedure is tested by feeding it inputs.
#[expect(
    clippy::large_enum_variant,
    reason = "one per run, replaced between stages; the core has no allocator to box with"
)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Procedure {
    /// Alternating steps, then the same gains.
    StepTest(Pairs),
    /// A gain search.
    Search(Search),
    /// A fit, then the gains designed from it.
    Design(Design),
}

impl Procedure {
    pub(super) fn new(method: Method) -> Self {
        match method {
            Method::StepTest(plan) => Self::StepTest(Pairs::new(plan, plan.pairs)),
            Method::Search(plan) => Self::Search(Search::new(plan)),
            Method::Design(plan) => Self::Design(Design::new(plan)),
        }
    }

    pub(super) fn next(&mut self, input: Input) -> Action {
        match self {
            Self::StepTest(pairs) => {
                let (on, gains) = input.split();
                pairs.step_test(on, gains)
            }
            Self::Search(search) => search.next(input),
            Self::Design(design) => design.next(input),
        }
    }

    /// The verdict the last [`next`](Self::next) reached, if any; taking it clears it.
    pub(super) fn take_verdict(&mut self) -> Option<Verdict> {
        match self {
            Self::Search(search) => search.take_verdict(),
            Self::StepTest(_) | Self::Design(_) => None,
        }
    }
}

/// A plane [`Procedure`]: a plane stage's method as a pure state machine, on a plane axis's
/// gains.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlaneProcedure(PlaneSteps);

#[derive(Clone, Copy, Debug, PartialEq)]
enum PlaneSteps {
    /// Alternating steps, then the same gains.
    StepTest(Pairs),
}

impl PlaneProcedure {
    pub(super) fn new(method: PlaneMethod) -> Self {
        match method {
            PlaneMethod::StepTest(plan) => Self(PlaneSteps::StepTest(Pairs::new(plan, plan.pairs))),
        }
    }

    pub(super) fn next(&mut self, on: Decide, gains: PlaneAxisGains) -> Action<PlaneAxisGains> {
        match &mut self.0 {
            PlaneSteps::StepTest(pairs) => pairs.step_test(on, gains),
        }
    }
}

/// A measurement a slow host spoiled, the `stops`-th in a row: run `probe` again, or fail the
/// stage at `MAX_MEASURE_STOPS`.
fn measure_again(stops: &mut usize, probe: ProbeConfig) -> Action {
    *stops += 1;
    if *stops >= MAX_MEASURE_STOPS {
        Action::Fail(AutotuneAbort::Stalled)
    } else {
        Action::probe(probe)
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

    /// A step test's next action on `on`, the loop's committed `gains` unchanged: the next
    /// step, the batch's steps once it has them all, or a failure. Any side's gains.
    fn step_test<G>(&mut self, on: Decide, gains: G) -> Action<G> {
        match on {
            Decide::Start => Action::probe(self.probe()),
            Decide::Measured {
                measurement: Measurement::Step(metrics),
                stalled,
            } => match self.record(&metrics, stalled) {
                Err(abort) => Action::Fail(abort),
                Ok(()) if self.done() => Action::Finish {
                    gains,
                    outcome: Outcome::Measured,
                    steps: Some(self.summary),
                    baseline: None,
                },
                Ok(()) => Action::probe(self.probe()),
            },
            Decide::Stalled => match self.stall() {
                Err(abort) => Action::Fail(abort),
                Ok(()) => Action::probe(self.probe()),
            },
            Decide::Aborted { abort } => Action::Fail(AutotuneAbort::Probe(abort)),
            Decide::Breached { reason } => Action::Fail(reason),
            Decide::Unsettled => Action::Fail(AutotuneAbort::NotSettled),
            Decide::Measured { .. } | Decide::Held { .. } => {
                Action::Fail(AutotuneAbort::Interrupted)
            }
        }
    }

    /// What a step's `input` means for the batch: a measured step or a stall is counted, and
    /// a step that aborts or keeps clipping ends the batch incomplete (it measures the limit,
    /// not the loop). `None` for an input that isn't a step's.
    fn take(&mut self, input: &Input) -> Option<Batch> {
        let batch = match *input {
            Input::Measured {
                measurement: Measurement::Step(metrics),
                stalled,
                ..
            } => match self.record(&metrics, stalled) {
                // Nothing can be measured while the host keeps stalling.
                Err(abort @ AutotuneAbort::Stalled) => Batch::Fail(abort),
                Err(_) => Batch::Ended { complete: false },
                Ok(()) if self.done() => Batch::Ended { complete: true },
                Ok(()) => Batch::Next,
            },
            Input::Stalled { .. } => match self.stall() {
                Err(abort) => Batch::Fail(abort),
                Ok(()) => Batch::Next,
            },
            Input::Aborted {
                abort: ProbeAbort::Step(_),
                ..
            } => Batch::Ended { complete: false },
            _ => return None,
        };
        Some(batch)
    }
}

/// Where a batch stands after a step ([`Pairs::take`]).
#[derive(Clone, Copy, Debug, PartialEq)]
enum Batch {
    /// Run the next step.
    Next,
    /// The batch is over: `complete` when every step ran.
    Ended { complete: bool },
    /// The stage can't go on.
    Fail(AutotuneAbort),
}
