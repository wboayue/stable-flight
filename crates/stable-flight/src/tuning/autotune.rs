//! The copter autotune supervisor: see [`Autotune`].

use core::fmt;

use crate::controllers::{
    Gains, InvalidConfig, check, check_limit, check_positive, check_unit_interval,
};
use crate::math::euler_angles;
use crate::stabilizers::copter::{CopterStabilizerConfig, TunedLoop};
use crate::{Configurable, FlightLoop, VehicleState};

use super::{
    ControllerForm, Measurement, ProbeAbort, ProbeConfig, ProbeStatus, RelayConfig, RelayResult,
    StepConfig, StepSummary, Tunable, TuningRule, gains_from_ultimate,
};

use procedure::{Action, Input, Procedure};

mod procedure;
#[cfg(test)]
mod tests;

/// The most stages a plan may hold: a tune and a step test of every [`TunedLoop`].
pub const MAX_STAGES: usize = 12;

/// Relay for a copter rate loop: normalized moment out, rad/s in. RealFlight's QuadCopterX
/// cycles in about 70 ms (17 ticks) with cycle-to-cycle jitter, so it averages 8 cycles
/// within 25% and moves the bias a fifth of the way per cycle.
pub const COPTER_RATE_RELAY: RelayConfig = RelayConfig {
    initial_relay: 0.05,
    max_relay: 0.4,
    relay_step: 1.5,
    hysteresis: 0.005,
    min_cycle_amplitude: 0.1,
    max_cycle_amplitude: 2.0,
    max_half_cycle_s: 0.3,
    max_bias: 0.5,
    bias_gain: 0.2,
    cycles: 8,
    tolerance: 0.25,
    timeout_s: 20.0,
};

/// Relay for a copter angle or heading loop: rad/s out, rad in; averaging as
/// [`COPTER_RATE_RELAY`]. The relay stays small (at most 0.6 rad/s) so the rate loop under it
/// doesn't saturate its moment: a saturated inner loop doesn't grow its cycle with the relay,
/// which reads `Ku` high (on QuadCopterX a 1.5 rad/s relay gave angle P 15, which oscillated).
/// Cycles of half a degree count.
pub const COPTER_ANGLE_RELAY: RelayConfig = RelayConfig {
    initial_relay: 0.1,
    max_relay: 0.6,
    relay_step: 1.5,
    hysteresis: 0.0005,
    min_cycle_amplitude: 0.01,
    max_cycle_amplitude: 0.35,
    max_half_cycle_s: 1.0,
    max_bias: 0.3,
    bias_gain: 0.2,
    cycles: 8,
    tolerance: 0.25,
    timeout_s: 30.0,
};

/// The loops [`StagePlan::copter_sequence`] tunes by relay: roll and pitch rate, the loops a
/// bad start breaks. The rest stay out (measured on QuadCopterX, #295):
///
/// - Yaw rate: the relay can't measure it within the yaw authority the mixer leaves at hover:
///   TODO(#296).
/// - Roll and pitch angle: the relay measures a high ultimate gain (about 60: an integrator
///   around a fast rate loop), and the rule's P oscillates once a few degrees of error saturate
///   the rate setpoint and the moments. [`StagePlan::copter_search`] tunes them by step
///   response instead, which sees the saturation.
/// - Heading: both of the above.
///
/// Their relay stages still run when planned explicitly (`StagePlan::copter`).
pub const COPTER_SEQUENCE: [TunedLoop; 2] = [TunedLoop::RollRate, TunedLoop::PitchRate];

/// One stage of an autotune: which loop, and the method that tunes it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StagePlan {
    /// The loop to tune.
    pub target: TunedLoop,
    /// How to tune it.
    pub method: Method,
}

impl StagePlan {
    /// The copter defaults for `target`: relay feedback, [`RelayPlan::copter`].
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    /// use stable_flight::tuning::autotune::{Method, RelayPlan, StagePlan};
    ///
    /// let plan = StagePlan::copter(TunedLoop::RollRate);
    /// assert_eq!(plan.method, Method::Relay(RelayPlan::copter(TunedLoop::RollRate)));
    /// ```
    pub fn copter(target: TunedLoop) -> Self {
        Self {
            target,
            method: Method::Relay(RelayPlan::copter(target)),
        }
    }

    /// The loops in [`COPTER_SEQUENCE`], in order, with the [`copter`](Self::copter) defaults.
    pub fn copter_sequence() -> [Self; COPTER_SEQUENCE.len()] {
        COPTER_SEQUENCE.map(Self::copter)
    }

    /// A step test of `target` with the copter defaults ([`StepTestPlan::copter`]).
    pub fn copter_step_test(target: TunedLoop) -> Self {
        Self {
            target,
            method: Method::StepTest(StepTestPlan::copter(target)),
        }
    }

    /// The copter rate search: each of [`COPTER_SEQUENCE`]'s loops by
    /// [`SearchPlan::copter_rate`].
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    /// use stable_flight::tuning::autotune::StagePlan;
    ///
    /// let plan = StagePlan::copter_rate_search();
    /// assert_eq!(plan.map(|stage| stage.target), [TunedLoop::RollRate, TunedLoop::PitchRate]);
    /// ```
    pub fn copter_rate_search() -> [Self; COPTER_SEQUENCE.len()] {
        COPTER_SEQUENCE.map(|target| Self {
            target,
            method: Method::Search(SearchPlan::copter_rate()),
        })
    }

    /// The copter step-response tune: [`copter_rate_search`](Self::copter_rate_search), then
    /// roll and pitch angle by [`SearchPlan::copter_angle`], so the angle search flies on the
    /// tuned rate loops.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    /// use stable_flight::tuning::autotune::StagePlan;
    ///
    /// let targets = StagePlan::copter_search().map(|stage| stage.target);
    /// assert_eq!(
    ///     targets,
    ///     [TunedLoop::RollRate, TunedLoop::PitchRate, TunedLoop::Roll, TunedLoop::Pitch]
    /// );
    /// ```
    pub fn copter_search() -> [Self; 4] {
        let [roll_rate, pitch_rate] = Self::copter_rate_search();
        let angle = |target| Self {
            target,
            method: Method::Search(SearchPlan::copter_angle()),
        };
        [
            roll_rate,
            pitch_rate,
            angle(TunedLoop::Roll),
            angle(TunedLoop::Pitch),
        ]
    }
}

/// How a stage tunes its loop: which probes it runs and what gains follow from them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Method {
    /// One relay measurement, turned into gains by a tuning rule.
    Relay(RelayPlan),
    /// Alternating step responses, measured and reported; the gains don't change.
    StepTest(StepTestPlan),
    /// A search for the largest gain of one term whose step responses meet a criterion, from
    /// gains a relay may seed, kept only if it flies no worse than a start that met the
    /// criterion.
    Search(SearchPlan),
}

impl Method {
    /// Checks the method's own fields and probe configs (its settle is checked against the
    /// envelope by [`Autotune::new`]).
    ///
    /// # Errors
    ///
    /// When the relay is invalid ([`RelayConfig::validate`]), the step test is
    /// ([`StepTestPlan::validate`]), or the search is ([`SearchPlan::validate`]).
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        match self {
            Self::Relay(plan) => plan.relay.validate(),
            Self::StepTest(plan) => plan.validate(),
            Self::Search(plan) => plan.validate(),
        }
    }
    /// The settle this method asks for before each probe; `None` uses the envelope's.
    pub fn settle(&self) -> Option<Settle> {
        match self {
            Self::Relay(_) => None,
            Self::StepTest(plan) => Some(plan.settle),
            Self::Search(plan) => Some(plan.test.settle),
        }
    }

    /// The section [`Autotune::new`] reports this method's errors in.
    fn plan_section(&self) -> &'static str {
        match self {
            Self::Relay(_) => "plan.relay",
            Self::StepTest(_) => "plan.step_test",
            Self::Search(_) => "plan.search",
        }
    }
}

/// Step response of a copter rate loop: 1 rad/s (57°/s) held for 0.25 s, about four of
/// QuadCopterX's 67 ms ultimate periods (#295), which tilts it about 14°; filtered at 40 Hz.
pub const COPTER_RATE_STEP: StepConfig = StepConfig {
    amplitude: 1.0,
    duration_s: 0.25,
    settle_band: 0.1,
    filter_hz: 40.0,
};

/// Step response of a copter angle loop: 10° held for 1.5 s, filtered at 20 Hz. QuadCopterX's
/// hand-tuned angle loop (P 3) reaches only 89% of the step in 0.8 s (#298), so the window
/// covers about four of its time constants; the vehicle drifts about 2 m meanwhile.
pub const COPTER_ANGLE_STEP: StepConfig = StepConfig {
    amplitude: 0.175,
    duration_s: 1.5,
    settle_band: 0.1,
    filter_hz: 20.0,
};

/// A copter angle search's step: [`COPTER_ANGLE_STEP`] at half the size (5°). At 10°, an angle
/// P of about 6 asks the tuned rate loop (P about 1) for a full moment; at 5°, about 12 (#304).
pub const COPTER_ANGLE_SEARCH_STEP: StepConfig = StepConfig {
    amplitude: COPTER_ANGLE_STEP.amplitude / 2.0,
    ..COPTER_ANGLE_STEP
};

/// Level before each copter step: within 2.5° and 0.1 rad/s for 0.5 s (ArduPilot's
/// `AUTOTUNE_LEVEL_ANGLE_CD` and `AUTOTUNE_REQUIRED_LEVEL_TIME_MS`), so each step starts from
/// rest.
pub const COPTER_STEP_SETTLE: Settle = Settle {
    tilt_rad: 0.044,
    rate_radps: 0.1,
    hold_s: 0.5,
};

/// A step test: `pairs` pairs of steps, alternating `+step` and `−step` so a trim or CG offset
/// shows as asymmetry rather than bias, each from a [`Settle`]. A step clipped for longer than
/// `max_saturated_s` (a clipped step measures the limit, not the loop) is invalid and runs
/// again; `max_invalid` in a row fail the stage. A step flagged stalled
/// ([`Autotune::note_stall`]) runs again too, on its own budget: a stall says nothing about the
/// loop, so `max_stalled` in a row (the host can't keep time) fail the stage.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepTestPlan {
    /// The step, in its positive direction.
    pub step: StepConfig,
    /// Pairs of steps to measure. At least 1.
    pub pairs: usize,
    /// Level before each step.
    pub settle: Settle,
    /// How long a step may be clipped and still count, in s. Non-negative and finite: a D kick
    /// clips the first tick or so of a rate step.
    pub max_saturated_s: f32,
    /// Clipped steps in a row that fail the stage. At least 1.
    pub max_invalid: usize,
    /// Stalled steps in a row that fail the stage. At least 1.
    pub max_stalled: usize,
}

impl StepTestPlan {
    /// The copter defaults for `target`: three pairs of [`COPTER_RATE_STEP`] (rate loops) or
    /// [`COPTER_ANGLE_STEP`] (angle and heading loops) from [`COPTER_STEP_SETTLE`], allowing
    /// 3 clipped steps in a row and 10 stalled steps in a row (RealFlight's proxy has frozen
    /// for 100 ms dozens of times a run). A step may clip for 10 ticks at 250 Hz on a rate loop
    /// and 5 on an angle loop, each half a tick clear of `f32` rounding (clipped time sums `dt`;
    /// five 4 ms ticks come to slightly over 0.02). Above P 1, a 1 rad/s rate step asks for
    /// more than full moment until the rate catches up: a real limit, not a bad response, and
    /// tuned rate loops reach P 1.3 on QuadCopterX.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    /// use stable_flight::tuning::autotune::{COPTER_ANGLE_STEP, StepTestPlan};
    ///
    /// assert_eq!(StepTestPlan::copter(TunedLoop::Roll).step, COPTER_ANGLE_STEP);
    /// ```
    pub fn copter(target: TunedLoop) -> Self {
        let (step, max_saturated_s) = if target.is_rate() {
            (COPTER_RATE_STEP, 0.042)
        } else {
            (COPTER_ANGLE_STEP, 0.022)
        };
        Self {
            step,
            pairs: 3,
            settle: COPTER_STEP_SETTLE,
            max_saturated_s,
            max_invalid: 3,
            max_stalled: 10,
        }
    }

    /// Checks the step ([`StepConfig::validate`]) and the counts and clip allowance.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        self.step.validate()?;
        check("pairs", self.pairs >= 1, "at least 1")?;
        check_limit("max_saturated_s", self.max_saturated_s)?;
        check("max_invalid", self.max_invalid >= 1, "at least 1")?;
        check("max_stalled", self.max_stalled >= 1, "at least 1")
    }
}

/// Relay feedback: measure the loop's ultimate point, then set the gains for the terms `form`
/// tunes by `rule`; the other terms keep their current gains.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RelayPlan {
    /// The relay that measures the loop.
    pub relay: RelayConfig,
    /// The tuning rule for its gains.
    pub rule: TuningRule,
    /// Which terms its gains set.
    pub form: ControllerForm,
}

impl RelayPlan {
    /// The copter defaults for `target`: Tyreus–Luyben PD throughout, so the integral keeps its
    /// current gain. A rate loop's I holds trim against steady disturbances rather than
    /// shaping its response, and the rule's integral time (`2.2·Tu`) is a fraction of a second
    /// on these fast loops: about 30× a flown hand tune on QuadCopterX. Rate loops use
    /// [`COPTER_RATE_RELAY`], angle and heading loops [`COPTER_ANGLE_RELAY`].
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    /// use stable_flight::tuning::ControllerForm;
    /// use stable_flight::tuning::autotune::RelayPlan;
    ///
    /// assert_eq!(RelayPlan::copter(TunedLoop::RollRate).form, ControllerForm::Pd);
    /// ```
    pub fn copter(target: TunedLoop) -> Self {
        let relay = if target.is_rate() {
            COPTER_RATE_RELAY
        } else {
            COPTER_ANGLE_RELAY
        };
        Self {
            relay,
            rule: TuningRule::TyreusLuyben,
            form: ControllerForm::Pd,
        }
    }

    /// The gains a measurement sets: the rule's for the terms the form tunes, `current` for
    /// the rest.
    fn gains(&self, result: &RelayResult, current: Gains) -> Gains {
        let tuned = gains_from_ultimate(&result.point, self.rule, self.form);
        Gains {
            p: tuned.p,
            i: if self.form.has_integral() {
                tuned.i
            } else {
                current.i
            },
            d: if self.form.has_derivative() {
                tuned.d
            } else {
                current.d
            },
        }
    }
}

/// A gain search on one term of a loop: the largest gain whose step responses meet
/// `criterion`, found by bracketing and bisection, then backed off by `margin` and verified.
///
/// 1. **Relay** (optional): measures the loop's ultimate point. `ceiling · Ku` caps `max`,
///    and the relay's rule seeds the gains the search starts from (e.g. a PD rule sets D,
///    and P, the searched term, starts at the rule's). A relay that aborts or stalls
///    `test.max_invalid` times leaves `max` and the start gains as they are.
/// 2. **Baseline:** `test.pairs` pairs at the start gains; if they meet the criterion, their
///    summary is what the result must not be worse than. A start above `max` (a gain the
///    search would never pick) isn't flown: it's lowered to the seed, its term at most
///    `max`, and committed ([`AutotuneEvent::Lowered`]), so the vehicle settles on it and
///    it stays if nothing better passes or the stage fails. A start whose steps abort or keep
///    clipping leaves no baseline.
/// 3. **Bracket** from the seed's gain: `× factor` per point while points pass, up to `max`;
///    `÷ factor` while they fail, down to `min` (nothing passing: the start gains stay). A
///    point passes when `confirm` pairs in a row pass, so one lucky pair near a sharp limit
///    doesn't (ArduPilot's `ignore_next`); the first failing pair fails it.
/// 4. **Bisect** (geometric mean) until the bracket is within `resolution`.
/// 5. **Verify** `margin ×` the largest pass with `test.pairs` pairs, and keep it only if it
///    passes [`Acceptance`] against a baseline that met the criterion, or, without one, meets
///    the criterion itself; else the start gains stay.
///
/// A pair is judged on its two steps' means, so a trim offset doesn't bias it. A step that
/// aborts (reversed or runaway) fails its pair, as do `test.max_invalid` clipped steps in a
/// row (a gain whose steps keep clipping measures the limit) and an [`Envelope`] breach while
/// the candidate flies; on the baseline, aborts and clipping leave no baseline, and a breach
/// fails the stage. Stalled steps only run again
/// (`test.max_stalled`). Only the step under test flies a candidate;
/// the vehicle settles on the committed gains between steps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SearchPlan {
    /// The term searched.
    pub term: Term,
    /// What a pair must meet to pass.
    pub criterion: Criterion,
    /// The steps, their settle, clip allowance, and invalid limit; `pairs` sizes the baseline
    /// and the verify.
    pub test: StepTestPlan,
    /// Pairs in a row a bracket point must pass. At least 1.
    pub confirm: usize,
    /// The bracket's growth per pair. Above 1.
    pub factor: f32,
    /// The bracket's ratio at which bisection stops. Above 1.
    pub resolution: f32,
    /// The fraction of the largest pass to verify and keep. In (0, 1].
    pub margin: f32,
    /// The least gain tried. Positive.
    pub min: f32,
    /// The largest gain tried. Above `min`.
    pub max: f32,
    /// A relay measurement that caps `max` and seeds the gains, if any.
    pub relay: Option<SearchRelay>,
    /// How much worse than the baseline the result may fly.
    pub acceptance: Acceptance,
}

impl SearchPlan {
    /// The copter defaults for a rate loop's P: a [`RelayPlan::copter`] relay (Tyreus–Luyben
    /// PD, so it sets D and seeds P; I keeps its gain) capping P at `0.6 · Ku`; then
    /// [`Criterion::Overshoot`] at ArduPilot's default aggressiveness (0.1: overshoot 0.05,
    /// bounce-back 0.1) in [0.1, 3]. Steps of [`COPTER_RATE_STEP`], two pairs for the baseline
    /// and the verify; a bracket point passes on two pairs in a row; brackets of `× 1.3`
    /// bisected to 10%, a 0.9 margin. Steps clip for up to 10 ticks ([`StepTestPlan::copter`]):
    /// with 5, clipping rather than overshoot caps P on QuadCopterX, and a 0.5 rad/s step
    /// doubles the noise relative to the step (#304).
    ///
    /// ArduPilot's D search (largest D before bounce-back) doesn't bound D on closed-loop
    /// steps: with the rate I, overshoot decays without falling back, so D climbs until the
    /// response overdamps. The relay's D flies well on QuadCopterX (#295, #301).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::autotune::{SearchPlan, Term};
    ///
    /// let plan = SearchPlan::copter_rate();
    /// assert_eq!(plan.term, Term::P);
    /// assert!(plan.relay.is_some());
    /// ```
    pub fn copter_rate() -> Self {
        Self {
            term: Term::P,
            criterion: Criterion::Overshoot {
                max_overshoot: 0.05,
                max_bounce_back: 0.1,
            },
            test: StepTestPlan {
                pairs: 2,
                ..StepTestPlan::copter(TunedLoop::RollRate)
            },
            confirm: 2,
            factor: 1.3,
            resolution: 1.1,
            margin: 0.9,
            min: 0.1,
            max: 3.0,
            relay: Some(SearchRelay {
                plan: RelayPlan::copter(TunedLoop::RollRate),
                ceiling: 0.6,
            }),
            acceptance: Acceptance::default(),
        }
    }

    /// The copter defaults for a roll or pitch angle loop's P (ArduPilot's angle-P stage): no
    /// relay (an angle loop's `Ku` is high, about 60 on QuadCopterX, so `0.6 · Ku` caps
    /// nothing), from the start gains in [1, 10] (above about 10, 5° steps clip the moment on
    /// QuadCopterX); overshoot and bounce-back at most 0.05
    /// (ArduPilot's aggressiveness halved, as for its angle P). Bounce-back is the overshoot
    /// above where the response levels off, so it counts even when a disturbance holds the
    /// response short of the step (about 8% on QuadCopterX). Steps of
    /// [`COPTER_ANGLE_SEARCH_STEP`]; otherwise as [`copter_rate`](Self::copter_rate).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::autotune::SearchPlan;
    ///
    /// assert!(SearchPlan::copter_angle().relay.is_none());
    /// ```
    pub fn copter_angle() -> Self {
        Self {
            criterion: Criterion::Overshoot {
                max_overshoot: 0.05,
                max_bounce_back: 0.05,
            },
            test: StepTestPlan {
                step: COPTER_ANGLE_SEARCH_STEP,
                pairs: 2,
                ..StepTestPlan::copter(TunedLoop::Roll)
            },
            min: 1.0,
            max: 10.0,
            relay: None,
            ..Self::copter_rate()
        }
    }

    /// Checks the steps ([`StepTestPlan::validate`]), the criterion, the bracket's factors and
    /// bounds, the relay ([`RelayConfig::validate`]) and its ceiling, and the acceptance.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        self.test.validate()?;
        self.criterion.validate()?;
        check("confirm", self.confirm >= 1, "at least 1")?;
        check(
            "factor",
            self.factor > 1.0 && self.factor.is_finite(),
            "above 1",
        )?;
        check(
            "resolution",
            self.resolution > 1.0 && self.resolution.is_finite(),
            "above 1",
        )?;
        check_unit_interval("margin", self.margin)?;
        check_positive("min", self.min)?;
        check(
            "max",
            self.max > self.min && self.max.is_finite(),
            "above min and finite",
        )?;
        if let Some(relay) = self.relay {
            relay.plan.relay.validate()?;
            check_positive("relay.ceiling", relay.ceiling)?;
        }
        self.acceptance.validate()
    }
}

/// A relay measurement before a search: it caps the search and seeds its gains.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SearchRelay {
    /// The relay, and the rule whose gains seed the search.
    pub plan: RelayPlan,
    /// The fraction of `Ku` the searched gain may reach. Positive (0.6: Ziegler–Nichols P).
    pub ceiling: f32,
}

/// A term of a PID loop's [`Gains`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Term {
    /// The proportional gain.
    P,
    /// The derivative gain.
    D,
}

impl Term {
    /// This term's gain in `gains`.
    pub fn get(self, gains: Gains) -> f32 {
        match self {
            Self::P => gains.p,
            Self::D => gains.d,
        }
    }

    /// `gains` with this term set to `gain`.
    pub fn with(self, gains: Gains, gain: f32) -> Gains {
        match self {
            Self::P => Gains { p: gain, ..gains },
            Self::D => Gains { d: gain, ..gains },
        }
    }
}

/// What a search's pair of steps must meet, on the means of [`StepSummary`]; limits are
/// fractions of the step (ArduPilot's aggressiveness).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Criterion {
    /// The response overshoots at most `max_overshoot` and falls back at most
    /// `max_bounce_back` after its peak: for a rate loop's P.
    Overshoot {
        /// The most overshoot. Non-negative.
        max_overshoot: f32,
        /// The most bounce-back. Non-negative.
        max_bounce_back: f32,
    },
}

impl Criterion {
    /// Whether `steps` meet it: their means within the limits. No steps don't.
    pub fn passes(&self, steps: &StepSummary) -> bool {
        let (Some(overshoot), Some(bounce_back)) =
            (steps.overshoot.mean(), steps.bounce_back.mean())
        else {
            return false;
        };
        match *self {
            Self::Overshoot {
                max_overshoot,
                max_bounce_back,
            } => overshoot <= max_overshoot && bounce_back <= max_bounce_back,
        }
    }

    /// The most bounce-back it allows.
    pub fn max_bounce_back(&self) -> f32 {
        match *self {
            Self::Overshoot {
                max_bounce_back, ..
            } => max_bounce_back,
        }
    }

    fn validate(&self) -> Result<(), InvalidConfig> {
        match *self {
            Self::Overshoot {
                max_overshoot,
                max_bounce_back,
            } => {
                check_limit("criterion.max_overshoot", max_overshoot)?;
                check_limit("criterion.max_bounce_back", max_bounce_back)
            }
        }
    }
}

/// How a search's verify compares with a baseline that met the criterion: the result is kept
/// unless it is worse.
/// Worse is more overshoot than the baseline's plus `overshoot_margin`, a mean settling time
/// above the baseline's times `settling_ratio`, more steps unsettled, or more bounce-back than
/// both the baseline's and the criterion's limit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Acceptance {
    /// Overshoot allowed above the baseline's. Non-negative.
    pub overshoot_margin: f32,
    /// Settling time allowed, as a multiple of the baseline's. At least 1.
    pub settling_ratio: f32,
}

impl Default for Acceptance {
    /// 0.05 more overshoot, 10% slower settling.
    fn default() -> Self {
        Self {
            overshoot_margin: 0.05,
            settling_ratio: 1.1,
        }
    }
}

impl Acceptance {
    /// Whether `verify` flies no worse than `baseline` (see [`Acceptance`]); a mean the
    /// baseline lacks (no valid steps, none settled) can't be worse.
    pub fn accepts(
        &self,
        criterion: &Criterion,
        baseline: &StepSummary,
        verify: &StepSummary,
    ) -> bool {
        verify.count() > 0
            && no_worse(baseline.overshoot.mean(), verify.overshoot.mean(), |base| {
                base + self.overshoot_margin
            })
            && no_worse(
                baseline.settling_time_s.mean(),
                verify.settling_time_s.mean(),
                |base| base * self.settling_ratio,
            )
            && verify.not_settled <= baseline.not_settled
            && no_worse(
                baseline.bounce_back.mean(),
                verify.bounce_back.mean(),
                |base| base.max(criterion.max_bounce_back()),
            )
    }

    fn validate(&self) -> Result<(), InvalidConfig> {
        check_limit("acceptance.overshoot_margin", self.overshoot_margin)?;
        check(
            "acceptance.settling_ratio",
            self.settling_ratio >= 1.0 && self.settling_ratio.is_finite(),
            "at least 1 and finite",
        )
    }
}

/// Whether `value` is within `limit(base)`: a missing baseline allows anything, a missing
/// value against a present baseline is worse.
fn no_worse(base: Option<f32>, value: Option<f32>, limit: impl FnOnce(f32) -> f32) -> bool {
    match (base, value) {
        (Some(base), Some(value)) => value <= limit(base),
        (Some(_), None) => false,
        (None, _) => true,
    }
}

/// How level and still the vehicle must hold before a probe starts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settle {
    /// Roll and pitch within this, in rad.
    pub tilt_rad: f32,
    /// Every body rate within this, in rad/s.
    pub rate_radps: f32,
    /// For this long, in s.
    pub hold_s: f32,
}

impl Settle {
    /// Whether `motion` is level and still enough.
    fn holds(&self, motion: &Motion) -> bool {
        motion.tilt_rad < self.tilt_rad && motion.rate_radps < self.rate_radps
    }
}

/// Limits that end a stage, and how still the vehicle must be before a probe starts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Envelope {
    /// Roll or pitch beyond this aborts the stage, in rad.
    pub max_tilt_rad: f32,
    /// Any body rate beyond this aborts the stage, in rad/s.
    pub max_rate_radps: f32,
    /// Sinking more than this below the altitude at [`Autotune::start`] aborts, in m.
    pub max_altitude_loss_m: f32,
    /// Level and still enough to start a probe, unless the stage's method sets its own
    /// ([`Method::settle`]).
    pub settle: Settle,
    /// How long to wait for a settle, in s. At the timeout a step fails the stage and a relay
    /// starts anyway: it replaces the loop's output, so a ringing loop doesn't stop it.
    pub settle_timeout_s: f32,
    /// After a probe on candidate gains ends, how long the vehicle has on the committed gains
    /// to come back inside the limits before a breach fails the stage, in s. A breach while
    /// the candidate flies fails only the candidate. Non-negative.
    pub recovery_s: f32,
}

impl Default for Envelope {
    /// Copter limits: 35° tilt, 4 rad/s, 3 m sink; settle within 6° and 0.3 rad/s for 1 s,
    /// waiting up to 10 s; 1 s to recover from a candidate.
    fn default() -> Self {
        Self {
            max_tilt_rad: 0.6,
            max_rate_radps: 4.0,
            max_altitude_loss_m: 3.0,
            settle: Settle {
                tilt_rad: 0.1,
                rate_radps: 0.3,
                hold_s: 1.0,
            },
            settle_timeout_s: 10.0,
            recovery_s: 1.0,
        }
    }
}

impl Envelope {
    /// Checks that every limit and time is positive and finite, and that settling
    /// ([`check_settle`](Self::check_settle)) is tighter than aborting.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_positive("max_tilt_rad", self.max_tilt_rad)?;
        check_positive("max_rate_radps", self.max_rate_radps)?;
        check_positive("max_altitude_loss_m", self.max_altitude_loss_m)?;
        check_positive("settle_timeout_s", self.settle_timeout_s)?;
        check_limit("recovery_s", self.recovery_s)?;
        self.check_settle(&self.settle)
    }

    /// Checks that `settle` is positive and tighter than the abort limits, and that its hold
    /// fits in `settle_timeout_s`.
    pub fn check_settle(&self, settle: &Settle) -> Result<(), InvalidConfig> {
        check(
            "settle.tilt_rad",
            settle.tilt_rad > 0.0 && settle.tilt_rad < self.max_tilt_rad,
            "positive and below max_tilt_rad",
        )?;
        check(
            "settle.rate_radps",
            settle.rate_radps > 0.0 && settle.rate_radps < self.max_rate_radps,
            "positive and below max_rate_radps",
        )?;
        check(
            "settle.hold_s",
            settle.hold_s > 0.0 && settle.hold_s < self.settle_timeout_s,
            "positive and below settle_timeout_s",
        )
    }

    /// The limit `motion` breaks, if any. NaN never counts as a breach.
    fn breach(&self, motion: &Motion, start_altitude_m: f32) -> Option<AutotuneAbort> {
        if motion.tilt_rad > self.max_tilt_rad {
            Some(AutotuneAbort::Tilt)
        } else if motion.rate_radps > self.max_rate_radps {
            Some(AutotuneAbort::Rate)
        } else if start_altitude_m - motion.altitude_m > self.max_altitude_loss_m {
            Some(AutotuneAbort::AltitudeLoss)
        } else {
            None
        }
    }
}

/// What the envelope checks, read once per update from the state.
struct Motion {
    /// The larger of |roll| and |pitch|, in rad.
    tilt_rad: f32,
    /// The largest body rate magnitude, in rad/s.
    rate_radps: f32,
    altitude_m: f32,
}

impl Motion {
    fn of(state: &VehicleState) -> Self {
        let (roll, pitch, _) = euler_angles(&state.attitude);
        Self {
            tilt_rad: roll.abs().max(pitch.abs()),
            rate_radps: state.angular_velocity_radps.amax(),
            altitude_m: state.position_m.z,
        }
    }
}

/// Why a stage failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AutotuneAbort {
    /// Roll or pitch passed [`Envelope::max_tilt_rad`].
    Tilt,
    /// A body rate passed [`Envelope::max_rate_radps`].
    Rate,
    /// The vehicle sank more than [`Envelope::max_altitude_loss_m`].
    AltitudeLoss,
    /// The vehicle didn't settle within [`Envelope::settle_timeout_s`].
    NotSettled,
    /// Too many clipped steps in a row ([`StepTestPlan::max_invalid`]).
    Invalid,
    /// Too many steps in a row during which the host stalled
    /// ([`StepTestPlan::max_stalled`]): it can't keep time for a measurement.
    Stalled,
    /// The probe gave up.
    Probe(ProbeAbort),
    /// The stabilizer idled the loop under test (e.g. a rate setpoint), or the probe was
    /// stopped from outside.
    Interrupted,
    /// The stabilizer rejected the tuned config.
    Rejected,
}

/// What an [`Autotune::update`] did, for the caller to log.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum AutotuneEvent {
    /// A stage started: it settles, then probes its loop.
    StageStarted(TunedLoop),
    /// A probe on the stage's loop finished.
    Measured {
        /// The probed loop.
        target: TunedLoop,
        /// What the probe measured.
        measurement: Measurement,
    },
    /// The host's timing spoiled a probe, which runs again: it was stopped as the host
    /// turned slow ([`Autotune::set_host_slow`], no measurement), or it finished across a
    /// stall ([`Autotune::note_stall`]).
    Stalled {
        /// The probed loop.
        target: TunedLoop,
        /// What the probe measured, if it finished; its timing can't be trusted.
        measurement: Option<Measurement>,
    },
    /// A search judged a pair of steps (or its baseline) on a gain.
    Tried {
        /// The searched loop.
        target: TunedLoop,
        /// The gains the steps flew.
        gains: Gains,
        /// Whether they met the criterion.
        passed: bool,
        /// The valid steps.
        steps: StepSummary,
    },
    /// A stage finished and committed its loop's gains.
    StageDone {
        /// The tuned loop.
        target: TunedLoop,
        /// The gains committed for it.
        gains: Gains,
        /// What the stage did to them.
        outcome: Outcome,
        /// A step test's valid steps, or a search's verify (invalid ones, which ran again,
        /// left out).
        steps: Option<StepSummary>,
        /// A search's baseline: its steps at the start gains.
        baseline: Option<StepSummary>,
    },
    /// A search lowered its loop's gains, which started above its `max` (lowered by the
    /// relay's ceiling when it has a relay), and committed them: the stage settles on them, and
    /// they stay if it fails or nothing better passes.
    Lowered {
        /// The searched loop.
        target: TunedLoop,
        /// The gains committed.
        gains: Gains,
    },
    /// A stage failed: the config went back to the last committed one (earlier stages stay),
    /// and the run ends.
    StageFailed {
        /// The loop the stage was for.
        target: TunedLoop,
        /// Why.
        reason: AutotuneAbort,
    },
    /// Every stage committed; the run is over and its gains stay.
    Complete,
    /// Disarmed before completion: every stage reverted to the config at
    /// [`Autotune::start`].
    Reverted,
}

/// What a finished stage did to its loop's gains.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Set them from its measurements.
    Tuned,
    /// Measured only; they didn't change.
    Measured,
    /// Kept the start gains.
    Kept(KeepReason),
}

/// Why a search kept the start gains.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeepReason {
    /// No gain down to the search's `min` passed.
    NoPass,
    /// The result flew worse than the start ([`Acceptance`]), or its steps aborted or kept
    /// clipping.
    Worse,
}

/// Where a run stands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AutotuneStatus {
    /// Not started.
    Idle,
    /// Stage `stage` (an index into the plan) is waiting for the vehicle to settle before its
    /// next probe.
    Settling {
        /// The stage.
        stage: usize,
    },
    /// Stage `stage` is probing `target`, or deciding on what a probe measured.
    Running {
        /// The stage index.
        stage: usize,
        /// Its loop.
        target: TunedLoop,
    },
    /// Every stage committed.
    Complete,
    /// Stage `stage` failed; earlier stages stay committed.
    Failed {
        /// The failed stage's index.
        stage: usize,
        /// Why.
        reason: AutotuneAbort,
    },
    /// Disarmed before completion; everything reverted.
    Reverted,
}

/// Why [`Autotune::start`] refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartError {
    /// The last tick wasn't armed.
    NotArmed,
    /// A run is already in progress.
    Running,
}

impl fmt::Display for StartError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::NotArmed => "not armed",
            Self::Running => "already running",
        })
    }
}

impl core::error::Error for StartError {}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    Idle,
    /// Stage `stage` starts on the next update.
    Starting {
        stage: usize,
    },
    /// The stage's procedure decides what follows `on`.
    Deciding {
        stage: usize,
        on: Decide,
    },
    /// Waiting for the vehicle to settle before running `request`.
    Settling {
        stage: usize,
        request: Request,
        level_s: f32,
        waited_s: f32,
    },
    /// A probe runs; `candidate`: on gains the procedure asked for, which go back to the
    /// committed ones when it ends.
    Probing {
        stage: usize,
        candidate: bool,
    },
    /// The last stage committed; the next update reports `Complete`.
    Finishing,
    Complete,
    Failed {
        stage: usize,
        reason: AutotuneAbort,
    },
    Reverted,
}

impl Phase {
    /// The stage a run in progress is on; `None` before, between (`Finishing`), and after
    /// runs.
    fn active_stage(&self) -> Option<usize> {
        match *self {
            Phase::Starting { stage }
            | Phase::Deciding { stage, .. }
            | Phase::Settling { stage, .. }
            | Phase::Probing { stage, .. } => Some(stage),
            Phase::Idle
            | Phase::Finishing
            | Phase::Complete
            | Phase::Failed { .. }
            | Phase::Reverted => None,
        }
    }
}

/// What a stage's procedure decides on.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Decide {
    /// The stage's start.
    Start,
    /// A finished probe's measurement, and whether the host stalled during it.
    Measured {
        measurement: Measurement,
        stalled: bool,
    },
    /// A probe stopped because the host turned slow, with no measurement.
    Stalled,
    /// A probe that gave up.
    Aborted { abort: ProbeAbort },
    /// A probe on candidate gains that took the vehicle past the envelope.
    Breached { reason: AutotuneAbort },
}

/// A probe a procedure asked for, and the gains to fly it on (`None`: the committed ones).
#[derive(Clone, Copy, Debug, PartialEq)]
struct Request {
    probe: ProbeConfig,
    gains: Option<Gains>,
}

/// Runs a copter autotune between ticks: each [`StagePlan`]'s procedure decides
/// which probes to run on its loop ([`Tunable::start_probe`]) and what gains follow from
/// them; the supervisor executes it.
///
/// - **Start** ([`start`](Self::start)) when armed and hovering; the caller owns the trigger.
///   The config and altitude at start are the baseline.
/// - **Settle:** before each probe, the vehicle must hold the method's [`Settle`]
///   ([`Method::settle`]), or the [`Envelope`]'s.
/// - **Probe and decide:** a finished probe's [`Measurement`] goes to the stage's procedure,
///   which asks for another probe or finishes with gains for its loop, committed through
///   [`set_stabilizer_config`](FlightLoop::set_stabilizer_config); the next stage flies them.
///   With [`Method::Relay`], one relay measurement and its rule's gains; with
///   [`Method::StepTest`], alternating steps and the gains unchanged; with
///   [`Method::Search`], a gain search verified against a baseline
///   ([`AutotuneEvent::Tried`] per judgment, the [`Outcome`] in `StageDone`).
/// - **Candidates:** a procedure may ask for a probe on other gains; they go in once the
///   vehicle has settled on the committed ones and come out when the probe ends, so only the
///   probe flies them. An envelope breach while they fly fails the candidate, not the stage;
///   the committed gains then have [`Envelope::recovery_s`] to bring the vehicle back.
/// - **Stalls:** [`note_stall`](Self::note_stall) marks the running probe's measurement as
///   untimely; a step test runs that step again. While the host is slow
///   ([`set_host_slow`](Self::set_host_slow)), the running probe stops at once (a step held
///   for its ticks at half the tick rate would tilt twice as far) and counts as stalled, and
///   settling waits: it gains no level time and doesn't time out.
/// - **Fail:** an envelope breach, a probe abort the procedure can't use (a search counts a
///   step's abort against its gain), an interrupt, or a rejected config stops the stage, puts
///   back the last committed config (the earlier stages' gains stay), and ends the
///   run.
/// - **Disarm** before completion reverts every stage to the baseline.
/// - **Complete:** the gains stay; read them from the stabilizer's config.
///
/// The procedures are pure (they see measurements and gains, never the loop), so the
/// supervisor alone flies, guards, and commits. [`update`](Self::update) returns an
/// [`AutotuneEvent`] when something happens, for the caller to log; the supervisor has no
/// clock (the caller passes `dt`) and does no I/O.
///
/// # Examples
///
/// ```no_run
/// # use stable_flight::FlightLoop;
/// # use stable_flight::stabilizers::copter::CopterStabilizer;
/// # use stable_flight::guidance::copter::CopterGuidance;
/// # use stable_flight::mixers::TableMixer;
/// # use stable_flight::testing::{ConstantCommander, NullVehicle};
/// use stable_flight::tuning::autotune::{Autotune, AutotuneEvent, Envelope, StagePlan};
///
/// # fn fly(flight_loop: &mut FlightLoop<ConstantCommander, NullVehicle, CopterGuidance, CopterStabilizer, TableMixer<4>, ()>) {
/// let mut autotune = Autotune::new(&StagePlan::copter_sequence(), Envelope::default())
///     .expect("valid plan");
/// autotune.start(flight_loop).expect("armed");
/// loop {
///     flight_loop.step(std::time::Duration::from_millis(4));
///     if let Some(event) = autotune.update(flight_loop, 0.004) {
///         println!("{event:?}");
///         if matches!(event, AutotuneEvent::Complete | AutotuneEvent::Reverted) {
///             break;
///         }
///     }
/// }
/// # }
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct Autotune {
    plan: [Option<StagePlan>; MAX_STAGES],
    stages: usize,
    envelope: Envelope,
    phase: Phase,
    /// The current stage's procedure.
    procedure: Procedure,
    /// The config at start: what a disarm reverts to.
    baseline: Option<CopterStabilizerConfig>,
    /// The config as of the last commit: what a failure reverts to.
    last_good: Option<CopterStabilizerConfig>,
    start_altitude_m: f32,
    /// The host stalled during the running probe ([`note_stall`](Self::note_stall)).
    stalled: bool,
    /// The host runs slow ([`set_host_slow`](Self::set_host_slow)).
    host_slow: bool,
    /// Time left, in s, for the vehicle to come back inside the envelope after a candidate
    /// ([`Envelope::recovery_s`]); breaches meanwhile don't fail the stage.
    recovering_s: f32,
}

/// What the supervisor needs from the stabilizer: copter probes and config.
pub trait CopterTunable:
    Tunable<Target = TunedLoop> + Configurable<Config = CopterStabilizerConfig>
{
}

impl<S> CopterTunable for S where
    S: Tunable<Target = TunedLoop> + Configurable<Config = CopterStabilizerConfig>
{
}

impl Autotune {
    /// A supervisor for `plan` (1 to [`MAX_STAGES`] stages) within `envelope`.
    ///
    /// # Errors
    ///
    /// When the plan is empty or too long (`plan`), a stage's method is invalid
    /// (`plan.<method>`, e.g. `plan.relay`; [`Method::validate`]), or the envelope is
    /// (`envelope`).
    pub fn new(plan: &[StagePlan], envelope: Envelope) -> Result<Self, InvalidConfig> {
        check(
            "stages",
            (1..=MAX_STAGES).contains(&plan.len()),
            "1 to MAX_STAGES",
        )
        .map_err(|e| e.in_section("plan"))?;
        let Some(first) = plan.first() else {
            unreachable!("the plan was checked non-empty above");
        };
        for stage in plan {
            stage
                .method
                .validate()
                .map_err(|e| e.in_section(stage.method.plan_section()))?;
        }
        envelope.validate().map_err(|e| e.in_section("envelope"))?;
        for settle in plan.iter().filter_map(|stage| stage.method.settle()) {
            envelope
                .check_settle(&settle)
                .map_err(|e| e.in_section("plan"))?;
        }

        let mut stages = [None; MAX_STAGES];
        for (slot, stage) in stages.iter_mut().zip(plan) {
            *slot = Some(*stage);
        }
        Ok(Self {
            plan: stages,
            stages: plan.len(),
            envelope,
            phase: Phase::Idle,
            procedure: Procedure::new(first.method),
            baseline: None,
            last_good: None,
            start_altitude_m: 0.0,
            stalled: false,
            host_slow: false,
            recovering_s: 0.0,
        })
    }

    /// Starts a run: records the stabilizer's config and the altitude as the baseline; the
    /// first stage starts on the next update. A finished run can be started again.
    ///
    /// # Errors
    ///
    /// [`StartError::NotArmed`] unless the last tick was armed; [`StartError::Running`]
    /// while a run is in progress.
    pub fn start<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &FlightLoop<C, V, G, S, M, T>,
    ) -> Result<(), StartError> {
        if self.is_running() {
            return Err(StartError::Running);
        }
        if !flight_loop.armed() {
            return Err(StartError::NotArmed);
        }
        let config = *flight_loop.stabilizer().config();
        self.baseline = Some(config);
        self.last_good = Some(config);
        self.start_altitude_m = flight_loop.state().position_m.z;
        self.recovering_s = 0.0;
        self.phase = Phase::Starting { stage: 0 };
        Ok(())
    }

    /// Advances the run by `dt` seconds; call it between ticks (after
    /// [`step`](FlightLoop::step)). Returns what happened, if anything: at most one event per
    /// call.
    ///
    /// In order: a finished run reports [`Complete`](AutotuneEvent::Complete); a disarm
    /// reverts everything; an envelope breach fails the stage; then the run starts a stage,
    /// lets its procedure decide, settles, or polls the running probe.
    pub fn update<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        dt: f32,
    ) -> Option<AutotuneEvent> {
        if !self.is_running() {
            return None;
        }
        if self.phase == Phase::Finishing {
            self.phase = Phase::Complete;
            return Some(AutotuneEvent::Complete);
        }
        if !flight_loop.armed() {
            flight_loop.stop_probe();
            if let Some(baseline) = self.baseline {
                restore(flight_loop, baseline);
            }
            self.phase = Phase::Reverted;
            return Some(AutotuneEvent::Reverted);
        }
        let motion = Motion::of(flight_loop.state());
        self.recovering_s = (self.recovering_s - dt).max(0.0);
        if let Some(stage) = self.phase.active_stage()
            && let Some(reason) = self.envelope.breach(&motion, self.start_altitude_m)
        {
            // A candidate's breach fails the candidate; the committed gains get time to
            // recover from it before a breach fails the stage.
            if let Phase::Probing {
                candidate: true, ..
            } = self.phase
            {
                self.end_probe(flight_loop, stage, true, Decide::Breached { reason });
                return None;
            }
            if self.recovering_s <= 0.0 {
                return Some(self.fail(flight_loop, stage, reason));
            }
        }
        match self.phase {
            Phase::Starting { stage } => self.begin(stage),
            Phase::Deciding { stage, on } => self.decide(flight_loop, stage, on),
            // A slow host holds the settle where it is, with no level time: the vehicle
            // settles again once the host keeps time.
            Phase::Settling {
                stage,
                request,
                waited_s,
                ..
            } if self.host_slow => {
                self.phase = Phase::Settling {
                    stage,
                    request,
                    level_s: 0.0,
                    waited_s,
                };
                None
            }
            Phase::Probing { stage, candidate } if self.host_slow => {
                self.end_probe(flight_loop, stage, candidate, Decide::Stalled);
                Some(AutotuneEvent::Stalled {
                    target: self.target(stage),
                    measurement: None,
                })
            }
            Phase::Settling {
                stage,
                request,
                level_s,
                waited_s,
            } => self.settle(
                flight_loop,
                &motion,
                (stage, request),
                level_s + dt,
                waited_s + dt,
            ),
            Phase::Probing { stage, candidate } => self.poll_probe(flight_loop, stage, candidate),
            Phase::Idle
            | Phase::Finishing
            | Phase::Complete
            | Phase::Failed { .. }
            | Phase::Reverted => None,
        }
    }

    /// Tells the supervisor whether the host runs slow: its ticks come further apart than
    /// their `dt`, for a stretch (e.g. RealFlight at half its rate, sitl's `TickRate`). While
    /// it does, a running probe stops and runs again later, and settling waits (see the
    /// type's **Stalls**). Takes effect on the next [`update`](Self::update).
    pub fn set_host_slow(&mut self, slow: bool) {
        self.host_slow = slow;
    }

    /// Tells the supervisor the host fell behind (e.g. a stalled simulator exchange,
    /// sitl's `RunStats::skipped_ticks` growing): the running probe's timing can't be trusted,
    /// so its procedure treats the measurement as invalid. A no-op unless a probe runs.
    pub fn note_stall(&mut self) {
        if matches!(self.phase, Phase::Probing { .. }) {
            self.stalled = true;
        }
    }

    /// Where the run stands.
    pub fn status(&self) -> AutotuneStatus {
        match self.phase {
            Phase::Idle => AutotuneStatus::Idle,
            Phase::Starting { stage }
            | Phase::Settling { stage, .. }
            | Phase::Deciding {
                stage,
                on: Decide::Start,
            } => AutotuneStatus::Settling { stage },
            Phase::Probing { stage, .. }
            | Phase::Deciding {
                stage,
                on:
                    Decide::Measured { .. }
                    | Decide::Stalled
                    | Decide::Aborted { .. }
                    | Decide::Breached { .. },
            } => AutotuneStatus::Running {
                stage,
                target: self.target(stage),
            },
            Phase::Finishing | Phase::Complete => AutotuneStatus::Complete,
            Phase::Failed { stage, reason } => AutotuneStatus::Failed { stage, reason },
            Phase::Reverted => AutotuneStatus::Reverted,
        }
    }

    /// Whether a run is in progress (started, not yet complete, failed, or reverted).
    pub fn is_running(&self) -> bool {
        !matches!(
            self.phase,
            Phase::Idle | Phase::Complete | Phase::Failed { .. } | Phase::Reverted
        )
    }

    /// Starts `stage` with a fresh procedure, which decides on the next update.
    fn begin(&mut self, stage: usize) -> Option<AutotuneEvent> {
        let plan = self.stage(stage)?;
        self.procedure = Procedure::new(plan.method);
        self.phase = Phase::Deciding {
            stage,
            on: Decide::Start,
        };
        Some(AutotuneEvent::StageStarted(plan.target))
    }

    /// Hands the procedure the stage's start or its last probe's result, with the loop's
    /// committed gains, and carries out what it asks for; reports a search's verdict.
    fn decide<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
        on: Decide,
    ) -> Option<AutotuneEvent> {
        let gains = self.target(stage).gains(flight_loop.stabilizer().config());
        let input = match on {
            Decide::Start => Input::Start { gains },
            Decide::Measured {
                measurement,
                stalled,
            } => Input::Measured {
                measurement,
                gains,
                stalled,
            },
            Decide::Stalled => Input::Stalled { gains },
            Decide::Aborted { abort } => Input::Aborted { abort, gains },
            Decide::Breached { reason } => Input::Breached { reason, gains },
        };
        match self.procedure.next(input) {
            Action::Probe {
                probe,
                gains,
                commit,
            } => {
                let lowered = commit.map(|gains| self.lower(flight_loop, stage, gains));
                if let Some(failed @ AutotuneEvent::StageFailed { .. }) = lowered {
                    return Some(failed);
                }
                self.phase = Phase::Settling {
                    stage,
                    request: Request { probe, gains },
                    level_s: 0.0,
                    waited_s: 0.0,
                };
                let tried = self
                    .procedure
                    .take_verdict()
                    .map(|verdict| AutotuneEvent::Tried {
                        target: self.target(stage),
                        gains: verdict.gains,
                        passed: verdict.passed,
                        steps: verdict.steps,
                    });
                lowered.or(tried)
            }
            Action::Finish {
                gains,
                outcome,
                steps,
                baseline,
            } => Some(self.commit(flight_loop, stage, gains, (outcome, steps, baseline))),
            Action::Fail(reason) => Some(self.fail(flight_loop, stage, reason)),
        }
    }

    /// Waits for the vehicle to hold the stage's [`Settle`] (`level_s` counts it, restarting
    /// when it moves), then sets the request's gains, if any, and starts its probe for
    /// `stage`; fails after `settle_timeout_s`, except a relay, which starts then instead: it
    /// replaces the loop's output, so a loop that rings on its own gains doesn't keep it from
    /// measuring (#375).
    fn settle<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        motion: &Motion,
        (stage, request): (usize, Request),
        level_s: f32,
        waited_s: f32,
    ) -> Option<AutotuneEvent> {
        let settle = self.settle_for(stage);
        let relay = matches!(request.probe, ProbeConfig::Relay(_));
        let level_s = if settle.holds(motion) { level_s } else { 0.0 };
        let timed_out = waited_s >= self.envelope.settle_timeout_s;
        if level_s >= settle.hold_s || (relay && timed_out) {
            if let Some(gains) = request.gains
                && self.set_gains(flight_loop, stage, gains).is_none()
            {
                return Some(self.fail(flight_loop, stage, AutotuneAbort::Rejected));
            }
            flight_loop.start_probe(self.target(stage), request.probe);
            self.phase = Phase::Probing {
                stage,
                candidate: request.gains.is_some(),
            };
            self.stalled = false;
            return None;
        }
        if timed_out {
            return Some(self.fail(flight_loop, stage, AutotuneAbort::NotSettled));
        }
        self.phase = Phase::Settling {
            stage,
            request,
            level_s,
            waited_s,
        };
        None
    }

    /// Checks the running probe: a measurement or an abort goes to the procedure on the next
    /// update, after a candidate's gains go back to the committed ones; a stopped probe fails
    /// the run.
    fn poll_probe<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
        candidate: bool,
    ) -> Option<AutotuneEvent> {
        let (on, event) = match flight_loop.probe() {
            Some((_, ProbeStatus::Running(_))) => return None,
            Some((target, ProbeStatus::Done(measurement))) => (
                Decide::Measured {
                    measurement,
                    stalled: self.stalled,
                },
                Some(if self.stalled {
                    AutotuneEvent::Stalled {
                        target,
                        measurement: Some(measurement),
                    }
                } else {
                    AutotuneEvent::Measured {
                        target,
                        measurement,
                    }
                }),
            ),
            Some((_, ProbeStatus::Aborted(abort))) => (Decide::Aborted { abort }, None),
            Some((_, ProbeStatus::Interrupted)) | None => {
                return Some(self.fail(flight_loop, stage, AutotuneAbort::Interrupted));
            }
        };
        self.end_probe(flight_loop, stage, candidate, on);
        event
    }

    /// Stops `stage`'s probe and hands `on` to its procedure on the next update; a
    /// candidate's gains go back to the committed ones, which get [`Envelope::recovery_s`] to
    /// bring the vehicle back inside the envelope.
    fn end_probe<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
        candidate: bool,
        on: Decide,
    ) {
        flight_loop.stop_probe();
        if candidate && let Some(last_good) = self.last_good {
            restore(flight_loop, last_good);
            self.recovering_s = self.envelope.recovery_s;
        }
        self.phase = Phase::Deciding { stage, on };
    }

    /// The settle before `stage`'s probes: its method's, or the envelope's.
    fn settle_for(&self, stage: usize) -> Settle {
        self.stage(stage)
            .and_then(|plan| plan.method.settle())
            .unwrap_or(self.envelope.settle)
    }

    fn stage(&self, stage: usize) -> Option<StagePlan> {
        self.plan.get(stage).copied().flatten()
    }

    /// The target of `stage`; `stage` is always in the plan (the phases only hold indices
    /// below `stages`).
    fn target(&self, stage: usize) -> TunedLoop {
        self.stage(stage)
            .map_or(TunedLoop::RollRate, |plan| plan.target)
    }

    /// Commits `gains` lowered mid-stage (a search's start above its `max`): the stage
    /// settles on them and a failure restores them. `StageFailed` if the stabilizer rejected
    /// them, else `Lowered`.
    fn lower<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
        gains: Gains,
    ) -> AutotuneEvent {
        let Some(config) = self.set_gains(flight_loop, stage, gains) else {
            return self.fail(flight_loop, stage, AutotuneAbort::Rejected);
        };
        self.last_good = Some(config);
        AutotuneEvent::Lowered {
            target: self.target(stage),
            gains,
        }
    }

    /// Commits a finished stage's gains and moves on.
    fn commit<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
        gains: Gains,
        (outcome, steps, baseline): (Outcome, Option<StepSummary>, Option<StepSummary>),
    ) -> AutotuneEvent {
        let Some(config) = self.set_gains(flight_loop, stage, gains) else {
            return self.fail(flight_loop, stage, AutotuneAbort::Rejected);
        };
        self.last_good = Some(config);
        self.phase = if stage + 1 < self.stages {
            Phase::Starting { stage: stage + 1 }
        } else {
            Phase::Finishing
        };
        AutotuneEvent::StageDone {
            target: self.target(stage),
            gains,
            outcome,
            steps,
            baseline,
        }
    }

    /// Sets `gains` on `stage`'s loop; the config flown, or `None` if the stabilizer
    /// rejected it.
    fn set_gains<C, V, G, S: CopterTunable, M, T>(
        &self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
        gains: Gains,
    ) -> Option<CopterStabilizerConfig> {
        let mut config = *flight_loop.stabilizer().config();
        *self.target(stage).gains_mut(&mut config) = gains;
        flight_loop
            .set_stabilizer_config(config)
            .ok()
            .map(|()| config)
    }

    /// Ends the run at `stage`: stops its probe and puts back the last committed config.
    fn fail<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
        reason: AutotuneAbort,
    ) -> AutotuneEvent {
        flight_loop.stop_probe();
        if let Some(last_good) = self.last_good {
            restore(flight_loop, last_good);
        }
        self.phase = Phase::Failed { stage, reason };
        AutotuneEvent::StageFailed {
            target: self.target(stage),
            reason,
        }
    }
}

/// Puts back a config the stabilizer flew before. It was valid then and validation doesn't
/// change between calls, so it can't be rejected.
fn restore<C, V, G, S: CopterTunable, M, T>(
    flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
    config: CopterStabilizerConfig,
) {
    let _ = flight_loop.set_stabilizer_config(config);
}
