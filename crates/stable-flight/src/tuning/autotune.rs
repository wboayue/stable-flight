//! The copter autotune supervisor: see [`Autotune`].

use core::fmt;

use crate::controllers::low_pass::LowPass;
use crate::controllers::{
    Gains, InvalidConfig, RATE_I_PER_P, YAW_RATE_I_PER_P, check, check_limit, check_positive,
    check_unit_interval,
};
use crate::math::euler_angles;
use crate::stabilizers::copter::{CopterStabilizerConfig, TunedLoop};
use crate::{Configurable, FlightLoop, Vec3, VehicleState};
use nalgebra::UnitQuaternion;

use super::{
    DesignError, IdentifyConfig, LoopShapingConfig, Measurement, ProbeAbort, ProbeConfig,
    ProbeStatus, RateModel, StepConfig, StepSummary, Tunable, design_angle_p, design_rate,
};

use procedure::{Action, Input, Procedure};

mod procedure;
#[cfg(test)]
mod tests;

/// The most stages a plan may hold: [`StagePlan::copter_design_search`] and a step test of
/// each loop it tunes.
pub const MAX_STAGES: usize = 14;

/// One stage of an autotune: which loop, and the method that tunes it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StagePlan {
    /// The loop to tune.
    pub target: TunedLoop,
    /// How to tune it.
    pub method: Method,
}

impl StagePlan {
    /// A step test of `target` with the copter defaults ([`StepTestPlan::copter`]).
    pub fn copter_step_test(target: TunedLoop) -> Self {
        Self {
            target,
            method: Method::StepTest(StepTestPlan::copter(target)),
        }
    }

    /// A step test of `target` on a full-size multirotor's steps
    /// ([`StepTestPlan::large_copter`]): the steps
    /// [`large_copter_search`](Self::large_copter_search) judges, so its results read against
    /// the search's.
    pub fn large_copter_step_test(target: TunedLoop) -> Self {
        Self {
            target,
            method: Method::StepTest(StepTestPlan::large_copter(target)),
        }
    }

    /// The copter design search: [`DesignPlan::copter`] for roll and pitch rate, then roll
    /// and pitch angle (each angle design flies on its designed rate loop), then
    /// [`SearchPlan::copter`] for each in the same order, starting from its design, then
    /// [`SearchPlan::copter_yaw_rate`] from the start's yaw gains.
    /// The design spares the search most of its climb from the start gains; the search keeps
    /// only gains that passed steps, so a design from a bad fit doesn't outlive it (on
    /// QuadCopterX in RealFlight one fit designed roll rate P 0.41 against 0.63–1.04, #518).
    /// Every loop is designed before any is searched: a loop still on its start gains can
    /// ring and keep another's steps from settling (pitch on `racer`'s high start with 25 ms
    /// of delay, #518). On the sim its roll and pitch stages took 152–380 s against the
    /// search's 308–693 s, and tuned the high starts with delay the search can't (#518); with
    /// the yaw search, the passing robustness rows take 202–280 s (#296).
    ///
    /// Yaw rate is searched without a design: the design's yaw P (5–8 on the sim's quads)
    /// clips 0.5 rad/s steps for 44–116 ms, and the search from its D (0.22–0.25) left steps
    /// short of 90% in the window (#296).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    /// use stable_flight::tuning::autotune::{Method, StagePlan};
    ///
    /// let plan = StagePlan::copter_design_search();
    /// assert!(matches!(plan[0].method, Method::Design(_)));
    /// assert!(matches!(plan[4].method, Method::Search(_)));
    /// assert_eq!(plan[4].target, TunedLoop::RollRate);
    /// assert_eq!(plan[8].target, TunedLoop::YawRate);
    /// ```
    pub fn copter_design_search() -> [Self; 9] {
        let loops = [
            TunedLoop::RollRate,
            TunedLoop::PitchRate,
            TunedLoop::Roll,
            TunedLoop::Pitch,
        ];
        let design = loops.map(|target| Self {
            target,
            method: Method::Design(DesignPlan::copter()),
        });
        let search = loops.map(|target| Self {
            target,
            method: Method::Search(SearchPlan::copter(target)),
        });
        let [roll_rate, pitch_rate, roll, pitch] = design;
        let [
            search_roll_rate,
            search_pitch_rate,
            search_roll,
            search_pitch,
        ] = search;
        [
            roll_rate,
            pitch_rate,
            roll,
            pitch,
            search_roll_rate,
            search_pitch_rate,
            search_roll,
            search_pitch,
            Self {
                target: TunedLoop::YawRate,
                method: Method::Search(SearchPlan::copter_yaw_rate()),
            },
        ]
    }

    /// The step search for a full-size multirotor: [`SearchPlan::copter`] for roll and pitch
    /// rate, then roll and pitch angle, each on the slower steps a large aircraft needs
    /// ([`LARGE_COPTER_RATE_STEP`], [`LARGE_COPTER_TILT_STEP`]), clipping for up to
    /// [`LARGE_COPTER_MAX_SATURATED_S`], and keeping the start's rate I: with I at
    /// [`RATE_I_PER_P`] of P, the Alia 250's rate steps (no rate D, rising in 0.5 s) overshoot
    /// by 10% and its pitch rate search tilts out of the envelope (#374). Fly it within
    /// [`Envelope::large_copter`].
    ///
    /// The design search doesn't fit a full-size aircraft yet: on the sim's Alia it designs
    /// rate P about 19× the step search's, and the search after it tilts out of the envelope
    /// (#572). From rate P 1 and 3 this search tunes the Alia's sim model to within 25% of its
    /// hand tune in 11–15 min of sim time (#572).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    /// use stable_flight::tuning::autotune::{LARGE_COPTER_RATE_STEP, Method, StagePlan};
    ///
    /// let plan = StagePlan::large_copter_search();
    /// assert_eq!(plan[0].target, TunedLoop::RollRate);
    /// let Method::Search(search) = plan[0].method else { unreachable!() };
    /// assert_eq!(search.test.step, LARGE_COPTER_RATE_STEP);
    /// assert_eq!(search.i_per_p, None);
    /// ```
    pub fn large_copter_search() -> [Self; 4] {
        [
            TunedLoop::RollRate,
            TunedLoop::PitchRate,
            TunedLoop::Roll,
            TunedLoop::Pitch,
        ]
        .map(|target| Self {
            target,
            method: Method::Search(SearchPlan::large_copter(target)),
        })
    }
}

/// How a stage tunes its loop: which probes it runs and what gains follow from them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Method {
    /// Alternating step responses, measured and reported; the gains don't change.
    StepTest(StepTestPlan),
    /// A search for the largest gain of one term whose step responses meet a criterion, from
    /// the stage's start gains, kept only if it flies no worse than a start that met the
    /// criterion.
    Search(SearchPlan),
    /// Gains designed from a fitted model of the loop's axis, committed without steps.
    Design(DesignPlan),
}

impl Method {
    /// Checks the method's own fields and probe configs (its settle is checked against the
    /// envelope by [`Autotune::new`]).
    ///
    /// # Errors
    ///
    /// When the step test is invalid ([`StepTestPlan::validate`]), the search is
    /// ([`SearchPlan::validate`]), or the design is ([`DesignPlan::validate`]).
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        match self {
            Self::StepTest(plan) => plan.validate(),
            Self::Search(plan) => plan.validate(),
            Self::Design(plan) => plan.validate(),
        }
    }

    /// The settle this method asks for before each probe; `None` uses the envelope's.
    pub fn settle(&self) -> Option<Settle> {
        match self {
            Self::Design(_) => None,
            Self::StepTest(plan) => Some(plan.settle),
            Self::Search(plan) => Some(plan.test.settle),
        }
    }

    /// The section [`Autotune::new`] reports this method's errors in.
    fn plan_section(&self) -> &'static str {
        match self {
            Self::StepTest(_) => "plan.step_test",
            Self::Search(_) => "plan.search",
            Self::Design(_) => "plan.design",
        }
    }
}

/// Identification of a copter rate axis: a 0.7 rad/s square wave added to its rate setpoint
/// for 3 s, washed out at 0.2 s (PX4's `MC_AT_SYSID_AMP` and time constant). On the sim 3 s
/// fit the same model as 20 s on every airframe, from any start (#312). On the rate setpoint
/// the excitation doesn't scale with the angle P and no edge kicks the angle D (#561).
pub const COPTER_IDENTIFY: IdentifyConfig = IdentifyConfig {
    amplitude: 0.7,
    duration_s: 3.0,
    washout_tc_s: 0.2,
};

/// Share of a search's largest pass it keeps, and of loop shaping's largest angle P: 10% off,
/// as ArduPilot backs off its autotune result.
const SEARCH_BACK_OFF: f32 = 0.9;

/// The copter's angle P range, in rad/s of rate per rad of angle, searched or designed in.
const COPTER_ANGLE_P_MIN: f32 = 1.0;
const COPTER_ANGLE_P_MAX: f32 = 10.0;

/// Loop shaping of a copter's roll and pitch: 65° of phase margin on the rate and angle loops,
/// angle P within the angle search's range (1–10) less its 10% margin
/// ([`SearchPlan::copter_angle`]). On the sim 65° met the step spec on every clean and delayed
/// row of the three copter airframes, and put angle P at the range's top as the angle search
/// does (#312, #517).
pub const COPTER_LOOP_SHAPING: LoopShapingConfig = LoopShapingConfig {
    rate_margin_deg: 65.0,
    angle_margin_deg: 65.0,
    angle_p_min: COPTER_ANGLE_P_MIN,
    angle_p_max: COPTER_ANGLE_P_MAX,
    angle_back_off: SEARCH_BACK_OFF,
};

/// Step response of a copter rate loop: 1 rad/s (57°/s) held for 0.25 s, about four of the
/// 67 ms period QuadCopterX's rate loops oscillate at on the edge of stability (#295), which
/// tilts it about 14°; filtered at 40 Hz.
pub const COPTER_RATE_STEP: StepConfig = StepConfig {
    amplitude: 1.0,
    duration_s: 0.25,
    settle_band: 0.1,
    filter_hz: 40.0,
};

/// Step response of a copter yaw rate loop, searched: [`COPTER_RATE_STEP`] at half the size
/// (0.5 rad/s). Prop drag gives a quad little yaw authority (about 7 rad/s² per unit moment on
/// the sim's `quad_x`), so 1 rad/s steps clip at any P that rises within PX4's yaw target,
/// 0.2 s; the sim yaw search clipped 4–10× as long at 1 rad/s and settled on a lower P (#296).
/// QuadCopterX has half `quad_x`'s yaw authority, and its steps reach only about half the step
/// in the 0.25 s window, so its search stops on clipping rather than on the criterion. On a
/// sim plant fitted to it that bound lands on the same P from every start (1.11–1.15), whose
/// steps overshoot at most 0.03 and settle within 0.56 s. Longer windows fail: at 1 s a start
/// with I as high as P overshoots on the I alone, and the search falls to P 0.51 (#558).
pub const COPTER_YAW_RATE_STEP: StepConfig = StepConfig {
    amplitude: 0.5,
    ..COPTER_RATE_STEP
};

/// Step response of a copter yaw rate loop, step-tested: [`COPTER_YAW_RATE_STEP`] held for
/// 1 s, so a yaw as slow as QuadCopterX's rises and settles within it (within 0.50 and 0.56 s
/// on the sim plant fitted to it, #558).
pub const COPTER_YAW_RATE_TEST_STEP: StepConfig = StepConfig {
    duration_s: 1.0,
    ..COPTER_YAW_RATE_STEP
};

/// Step response of a copter roll or pitch loop, searched and step-tested: 5° held for 1.5 s,
/// filtered at 20 Hz. QuadCopterX's hand-tuned angle loop (P 3) reaches only 89% of a step in
/// 0.8 s (#298), so the window covers about four of its time constants; the vehicle drifts
/// about 1 m meanwhile. At 10°, an angle P of about 6 asks the tuned rate loop (P about 1) for
/// a full moment; at 5°, about 12 (#304). At the tuned angle P 9 on the sim, `quad_x`'s 10°
/// steps clip for 13 ticks and `heavy_quad`'s for 15, past [`COPTER_MAX_SATURATED_S`]; 5°
/// steps clip for 5 (#310).
pub const COPTER_TILT_STEP: StepConfig = StepConfig {
    amplitude: 0.0875,
    duration_s: 1.5,
    settle_band: 0.1,
    filter_hz: 20.0,
};

/// Step response of a copter heading loop: [`COPTER_TILT_STEP`] at twice the size (10°).
pub const COPTER_HEADING_STEP: StepConfig = StepConfig {
    amplitude: 2.0 * COPTER_TILT_STEP.amplitude,
    ..COPTER_TILT_STEP
};

/// Level before each copter step: within 2.5° and 0.1 rad/s for 0.5 s (ArduPilot's
/// `AUTOTUNE_LEVEL_ANGLE_CD` and `AUTOTUNE_REQUIRED_LEVEL_TIME_MS`), so each step starts from
/// rest.
pub const COPTER_STEP_SETTLE: Settle = Settle {
    tilt_rad: 0.044,
    rate_radps: 0.1,
    hold_s: 0.5,
};

/// Step response of a full-size multirotor's rate loop: 0.2 rad/s held for 1.5 s, rising in
/// about 0.6 s on the Alia 250's sim model (motor lag 0.17 s). On [`COPTER_RATE_STEP`] the
/// Alia's rate steps never rise within the window, so the overshoot criterion passes any gain
/// (#351).
pub const LARGE_COPTER_RATE_STEP: StepConfig = StepConfig {
    amplitude: 0.2,
    duration_s: 1.5,
    ..COPTER_RATE_STEP
};

/// Step response of a full-size multirotor's roll or pitch loop: [`COPTER_TILT_STEP`] held for
/// 2.5 s, rising in about 1.2 s on the Alia 250's sim model (#351).
pub const LARGE_COPTER_TILT_STEP: StepConfig = StepConfig {
    duration_s: 2.5,
    ..COPTER_TILT_STEP
};

/// How long a full-size multirotor's step may clip, in s: its rate step's moment saturates for
/// the first few ticks while the slow motors catch up (#351).
pub const LARGE_COPTER_MAX_SATURATED_S: f32 = 0.2;

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

/// How long a copter step may clip and still count, in s: 10 ticks at 250 Hz, half a tick
/// clear of `f32` rounding (clipped time sums `dt`; ten 4 ms ticks come to slightly over
/// 0.04). A step's first ticks ask for more than full moment until the loop catches up: a
/// real limit, not a bad response. Above P 1, a 1 rad/s rate step does (tuned rate loops reach
/// P 1.3 on QuadCopterX); so does a 5° angle step at the tuned angle P 9, for 5 ticks on the
/// sim's `quad_x` and 6–10 under its trim rows. A 5-tick allowance fails those rows, and in
/// RealFlight left 3 of 16 angle searches on their start after points whose every step
/// clipped (#310). Heading steps share it as one rule; it isn't measured on them.
pub const COPTER_MAX_SATURATED_S: f32 = 0.042;

impl StepTestPlan {
    /// The copter defaults for `target`: three pairs of [`COPTER_RATE_STEP`] (roll and pitch
    /// rate), [`COPTER_YAW_RATE_TEST_STEP`] (yaw rate),
    /// [`COPTER_TILT_STEP`] (roll and pitch), or [`COPTER_HEADING_STEP`] (heading), from
    /// [`COPTER_STEP_SETTLE`], each clipping for up to [`COPTER_MAX_SATURATED_S`], allowing
    /// 3 clipped steps in a row and 10 stalled steps in a row (RealFlight's proxy has frozen
    /// for 100 ms dozens of times a run).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    /// use stable_flight::tuning::autotune::{COPTER_TILT_STEP, StepTestPlan};
    ///
    /// assert_eq!(StepTestPlan::copter(TunedLoop::Roll).step, COPTER_TILT_STEP);
    /// ```
    pub fn copter(target: TunedLoop) -> Self {
        let step = match target {
            TunedLoop::RollRate | TunedLoop::PitchRate => COPTER_RATE_STEP,
            TunedLoop::YawRate => COPTER_YAW_RATE_TEST_STEP,
            TunedLoop::Roll | TunedLoop::Pitch => COPTER_TILT_STEP,
            TunedLoop::Heading => COPTER_HEADING_STEP,
        };
        Self {
            step,
            pairs: 3,
            settle: COPTER_STEP_SETTLE,
            max_saturated_s: COPTER_MAX_SATURATED_S,
            max_invalid: 3,
            max_stalled: 10,
        }
    }

    /// [`copter`](Self::copter) for roll or pitch, on a full-size multirotor's steps
    /// ([`LARGE_COPTER_RATE_STEP`] or [`LARGE_COPTER_TILT_STEP`]), clipping for up to
    /// [`LARGE_COPTER_MAX_SATURATED_S`]. Yaw rate and heading are [`copter`](Self::copter)'s
    /// unchanged.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    /// use stable_flight::tuning::autotune::{LARGE_COPTER_TILT_STEP, StepTestPlan};
    ///
    /// assert_eq!(StepTestPlan::large_copter(TunedLoop::Roll).step, LARGE_COPTER_TILT_STEP);
    /// ```
    pub fn large_copter(target: TunedLoop) -> Self {
        let mut plan = Self::copter(target);
        plan.step = match target {
            TunedLoop::RollRate | TunedLoop::PitchRate => LARGE_COPTER_RATE_STEP,
            TunedLoop::Roll | TunedLoop::Pitch => LARGE_COPTER_TILT_STEP,
            TunedLoop::YawRate | TunedLoop::Heading => return plan,
        };
        plan.max_saturated_s = LARGE_COPTER_MAX_SATURATED_S;
        plan
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

/// A gain search on one term of a loop: the largest gain whose step responses meet
/// `criterion`, found by bracketing and bisection, then backed off by `margin` and verified.
///
/// 1. **Baseline:** `test.pairs` pairs at the start gains; if they meet the criterion, their
///    summary is what the result must not be worse than. A start above `max` (a gain the
///    search would never pick) isn't flown: its term is lowered to `max` and committed
///    ([`AutotuneEvent::Lowered`]), so the vehicle settles on it and it stays if nothing
///    better passes or the stage fails. A start whose steps abort or keep clipping leaves no
///    baseline.
/// 2. **Bracket** from the start's gain: `× factor` per point while points pass, up to `max`;
///    `÷ factor` while they fail, down to `min` (nothing passing: see below). A
///    point passes when `confirm` pairs in a row pass, so one lucky pair near a sharp limit
///    doesn't (ArduPilot's `ignore_next`); the first failing pair fails it.
/// 3. **Bisect** (geometric mean) until the bracket is within `resolution`.
/// 4. **Verify** `margin ×` the largest pass with `test.pairs` pairs, and keep it only if it
///    passes [`Acceptance`] against a baseline that met the criterion, or, without one, meets
///    the criterion itself.
///
/// A search that keeps no result keeps its start gains if their steps met the criterion, and
/// otherwise puts back the loop's gains at the run's start ([`Autotune::start`]): a start the
/// search was seeded with (a design, [`StagePlan::copter_design_search`]) that never passed
/// doesn't outlive it. The run's start lowered to `max` stays lowered.
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
    /// How much worse than the baseline the result may fly.
    pub acceptance: Acceptance,
    /// With [`Term::P`] only: I at this share of each P tried, in 1/s, so every P carries the
    /// I that holds the trim moment ([`RATE_I_PER_P`] on roll and pitch, [`YAW_RATE_I_PER_P`]
    /// on yaw); `None` keeps the start's I. The
    /// baseline flies the start's gains as they are, so from a start whose I isn't at this
    /// share, a candidate differs from the baseline in I as well as P. Non-negative and finite.
    pub i_per_p: Option<f32>,
}

/// `gains` with I at `i_per_p` of P, in 1/s; unchanged for `None`
/// ([`SearchPlan::i_per_p`], [`DesignPlan::rate_i_per_p`]).
pub(crate) fn with_i_per_p(gains: Gains, i_per_p: Option<f32>) -> Gains {
    Gains {
        i: i_per_p.map_or(gains.i, |ratio| ratio * gains.p),
        ..gains
    }
}

/// `Ok` when `i_per_p` is `None` or non-negative and finite.
fn check_i_per_p(field: &'static str, i_per_p: Option<f32>) -> Result<(), InvalidConfig> {
    i_per_p.map_or(Ok(()), |ratio| check_limit(field, ratio))
}

impl SearchPlan {
    /// The copter defaults for a rate loop's P, with I at [`RATE_I_PER_P`] of it and D as it
    /// starts (a design sets it, [`StagePlan::copter_design_search`]):
    /// [`Criterion::Overshoot`] at ArduPilot's default aggressiveness (0.1: overshoot 0.05,
    /// bounce-back 0.1) in [0.1, 3]. Steps of [`COPTER_RATE_STEP`], two pairs for the baseline
    /// and the verify; a bracket point passes on two pairs in a row; brackets of `× 1.3`
    /// bisected to 10%, a 0.9 margin. Steps clip for up to 10 ticks ([`StepTestPlan::copter`]):
    /// with 5, clipping rather than overshoot caps P on QuadCopterX, and a 0.5 rad/s step
    /// doubles the noise relative to the step (#304).
    ///
    /// ArduPilot's D search (largest D before bounce-back) doesn't bound D on closed-loop
    /// steps: with the rate I, overshoot decays without falling back, so D climbs until the
    /// response overdamps. The design's D comes from the fitted lag instead (#517).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::autotune::{SearchPlan, Term};
    ///
    /// let plan = SearchPlan::copter_rate();
    /// assert_eq!(plan.term, Term::P);
    /// ```
    pub fn copter_rate() -> Self {
        Self {
            term: Term::P,
            criterion: Criterion::Overshoot {
                max_overshoot: 0.05,
                max_bounce_back: 0.1,
                settled_past: None,
            },
            test: StepTestPlan {
                pairs: 2,
                ..StepTestPlan::copter(TunedLoop::RollRate)
            },
            confirm: 2,
            factor: 1.3,
            resolution: 1.1,
            margin: SEARCH_BACK_OFF,
            min: 0.1,
            max: 3.0,
            acceptance: Acceptance::default(),
            i_per_p: Some(RATE_I_PER_P),
        }
    }

    /// The copter defaults for a roll or pitch angle loop's P (ArduPilot's angle-P stage),
    /// from the start gains in [1, 10] (above about 10, 5° steps clip the moment on
    /// QuadCopterX); overshoot and bounce-back at most 0.05
    /// (ArduPilot's aggressiveness halved, as for its angle P). Bounce-back is the overshoot
    /// above where the response levels off, so it counts even when a disturbance holds the
    /// response short of the step (about 8% on QuadCopterX). Overshoot counts from where the
    /// response ends, within the settle band past the step, so a disturbance holding it past
    /// doesn't count either: on the Titan Cobra every P from 6.5 down to 1.7 ended 5–7% past
    /// without ringing, and judged from the step the search walked P down to 1.3 (#564). Steps
    /// of [`COPTER_TILT_STEP`]; otherwise as [`copter_rate`](Self::copter_rate).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::autotune::SearchPlan;
    ///
    /// assert_eq!(SearchPlan::copter_angle().max, 10.0);
    /// ```
    pub fn copter_angle() -> Self {
        Self {
            criterion: Criterion::Overshoot {
                max_overshoot: 0.05,
                max_bounce_back: 0.05,
                settled_past: Some(COPTER_TILT_STEP.settle_band),
            },
            test: StepTestPlan {
                pairs: 2,
                ..StepTestPlan::copter(TunedLoop::Roll)
            },
            min: COPTER_ANGLE_P_MIN,
            max: COPTER_ANGLE_P_MAX,
            i_per_p: None,
            ..Self::copter_rate()
        }
    }

    /// The copter yaw rate search: [`copter_rate`](Self::copter_rate) on steps of
    /// [`COPTER_YAW_RATE_STEP`], D as it starts (0 on yaw by default, as ArduPilot's
    /// `ATC_RAT_YAW_D` and PX4's `MC_YAWRATE_D`), I at [`YAW_RATE_I_PER_P`] of P. That I holds
    /// a yaw trim only up to `yaw_max_integral` (`BodyRateControllerConfig`); with the start's
    /// I kept instead, small P overshoots on the I alone, the bracket stops there, and the
    /// result depends on the start: 0.35 against 0.91 on the sim's `racer` with a 0.40 yaw
    /// trim (#296).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::autotune::{COPTER_YAW_RATE_STEP, SearchPlan};
    ///
    /// assert_eq!(SearchPlan::copter_yaw_rate().test.step, COPTER_YAW_RATE_STEP);
    /// ```
    pub fn copter_yaw_rate() -> Self {
        Self {
            test: StepTestPlan {
                step: COPTER_YAW_RATE_STEP,
                pairs: 2,
                ..StepTestPlan::copter(TunedLoop::YawRate)
            },
            i_per_p: Some(YAW_RATE_I_PER_P),
            ..Self::copter_rate()
        }
    }

    /// The copter search for `target` from a design ([`StagePlan::copter_design_search`]):
    /// [`copter_rate`](Self::copter_rate) for roll or pitch rate, whose D the design set,
    /// [`copter_yaw_rate`](Self::copter_yaw_rate), or [`copter_angle`](Self::copter_angle).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    /// use stable_flight::tuning::autotune::SearchPlan;
    ///
    /// assert_eq!(SearchPlan::copter(TunedLoop::RollRate), SearchPlan::copter_rate());
    /// assert_eq!(SearchPlan::copter(TunedLoop::Roll), SearchPlan::copter_angle());
    /// ```
    pub fn copter(target: TunedLoop) -> Self {
        match target {
            TunedLoop::RollRate | TunedLoop::PitchRate => Self::copter_rate(),
            TunedLoop::YawRate => Self::copter_yaw_rate(),
            TunedLoop::Roll | TunedLoop::Pitch | TunedLoop::Heading => Self::copter_angle(),
        }
    }

    /// [`copter`](Self::copter) for roll or pitch, on a full-size multirotor's steps
    /// ([`StepTestPlan::large_copter`], in [`copter`](Self::copter)'s pairs), keeping the
    /// start's rate I ([`StagePlan::large_copter_search`] says why). Yaw rate and heading,
    /// which the large search doesn't tune, are [`copter`](Self::copter)'s unchanged.
    pub fn large_copter(target: TunedLoop) -> Self {
        let plan = Self::copter(target);
        if matches!(target, TunedLoop::YawRate | TunedLoop::Heading) {
            return plan;
        }
        Self {
            test: StepTestPlan {
                pairs: plan.test.pairs,
                ..StepTestPlan::large_copter(target)
            },
            i_per_p: None,
            ..plan
        }
    }

    /// Checks the steps ([`StepTestPlan::validate`]), the criterion, the bracket's factors and
    /// bounds, the acceptance, and `i_per_p`.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check(
            "i_per_p",
            self.i_per_p.is_none() || self.term == Term::P,
            "None unless term is P",
        )?;
        check_i_per_p("i_per_p", self.i_per_p)?;
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
        self.acceptance.validate()
    }
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
    /// `max_bounce_back` after its peak: for a P search, a rate loop's judging overshoot from
    /// the step and an angle loop's from where its steps end ([`settled_past`]).
    ///
    /// [`settled_past`]: Criterion::Overshoot::settled_past
    Overshoot {
        /// The most overshoot. Non-negative.
        max_overshoot: f32,
        /// The most bounce-back. Non-negative.
        max_bounce_back: f32,
        /// When set, overshoot counts from where each step ended
        /// ([`StepSummary::overshoot_past_end`]) rather than from the step, and no step may
        /// end more than this past the step. For a window that outlasts the rise, as an angle
        /// step's 1.5 s does (the slowest pass on the Titan Cobra, P 1.43, rose in 812 ms,
        /// #564): a disturbance holding the response past the step, without ringing,
        /// isn't overshoot that a lower P removes (#564). `None` judges from the step, for a
        /// window that may end mid-rise (a 0.25 s yaw rate step). Non-negative.
        settled_past: Option<f32>,
    },
}

impl Criterion {
    /// Whether `steps` meet it: their means within the limits. No steps don't.
    pub fn passes(&self, steps: &StepSummary) -> bool {
        let Self::Overshoot {
            max_overshoot,
            max_bounce_back,
            settled_past,
        } = *self;
        let overshoot = match settled_past {
            Some(_) => steps.overshoot_past_end.mean(),
            None => steps.overshoot.mean(),
        };
        let (Some(overshoot), Some(bounce_back)) = (overshoot, steps.bounce_back.mean()) else {
            return false;
        };
        let ended_past = steps.final_value.max - 1.0;
        overshoot <= max_overshoot
            && bounce_back <= max_bounce_back
            && settled_past.is_none_or(|limit| ended_past <= limit)
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
                settled_past,
            } => {
                check_limit("criterion.max_overshoot", max_overshoot)?;
                check_limit("criterion.max_bounce_back", max_bounce_back)?;
                settled_past.map_or(Ok(()), |limit| check_limit("criterion.settled_past", limit))
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

/// Gains for one loop from a fitted model of its axis, committed without steps: search from
/// them once every loop is designed ([`StagePlan::copter_design_search`]).
///
/// 1. **Identify:** `identify` excites the axis on its rate loop's setpoint (an angle loop's
///    probe runs on the rate loop under it) and fits a [`RateModel`]. The fit starts at the
///    settle timeout even when the vehicle hasn't settled: a loop that rings on its start gains only
///    adds excitation (#356). A slow host may stop it 3 times in a row before the stage fails.
/// 2. **Design:** a rate loop's P and D by [`design_rate`], its I at `rate_i_per_p` of P; an
///    angle loop's P by [`design_angle_p`] on the rate loop's committed gains, its I and D
///    kept. A fit the design can't use runs again; the third fails the stage
///    ([`AutotuneAbort::Design`]).
/// 3. **Hold:** the design flies until the vehicle holds the settle about the loop's axis on
///    it. One that doesn't within the settle timeout ([`AutotuneEvent::Unsettled`]), or
///    breaches the envelope, goes back and counts as an unusable fit. The design flies from
///    then on ([`Outcome::Designed`]), but a failure before its loop's search finishes puts
///    the loop's earlier gains back.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DesignPlan {
    /// The excitation and fit.
    pub identify: IdentifyConfig,
    /// The design's margins and angle P range.
    pub design: LoopShapingConfig,
    /// A rate design's I at this share of its P, in 1/s ([`SearchPlan::i_per_p`]); `None`
    /// keeps the start's I. Non-negative and finite.
    pub rate_i_per_p: Option<f32>,
}

impl DesignPlan {
    /// The copter defaults: [`COPTER_IDENTIFY`], [`COPTER_LOOP_SHAPING`], and rate I at
    /// [`RATE_I_PER_P`].
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::autotune::{COPTER_LOOP_SHAPING, DesignPlan};
    ///
    /// assert_eq!(DesignPlan::copter().design, COPTER_LOOP_SHAPING);
    /// ```
    pub fn copter() -> Self {
        Self {
            identify: COPTER_IDENTIFY,
            design: COPTER_LOOP_SHAPING,
            rate_i_per_p: Some(RATE_I_PER_P),
        }
    }

    /// Checks the fit ([`IdentifyConfig::validate`]), the design
    /// ([`LoopShapingConfig::validate`]), and `rate_i_per_p`.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_i_per_p("rate_i_per_p", self.rate_i_per_p)?;
        self.identify.validate()?;
        self.design.validate()
    }

    /// The gains `model` designs from the loop's `start`: a rate loop's (`inner` `None`), or
    /// an angle loop's P flown on the `inner` rate gains, through a gyro low-pass at
    /// `gyro_cutoff_hz`.
    fn gains(
        &self,
        model: &RateModel,
        start: Gains,
        inner: Option<Gains>,
        gyro_cutoff_hz: f32,
    ) -> Result<Gains, DesignError> {
        match inner {
            None => design_rate(model, start, gyro_cutoff_hz, &self.design)
                .map(|gains| with_i_per_p(gains, self.rate_i_per_p)),
            Some(rate) => design_angle_p(model, rate, start, gyro_cutoff_hz, &self.design)
                .map(|p| Gains { p, ..start }),
        }
    }
}

/// How level and still the vehicle must hold before a probe starts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settle {
    /// Roll and pitch within this, in rad.
    pub tilt_rad: f32,
    /// Roll and pitch rates within this, in rad/s, read through the stabilizer's gyro
    /// low-pass; the yaw rate too for a yaw rate or heading stage, as ArduPilot's level check
    /// does (`AC_AutoTune::currently_level`, `AUTOTUNE_LEVEL_RATE_Y_CD`, 7.5°/s). Roll and
    /// pitch stages leave yaw out: with 25 ms of delay it rings on its start gains at ±0.18
    /// rad/s on `racer` without tilting the vehicle (#518).
    pub rate_radps: f32,
    /// For this long, in s.
    pub hold_s: f32,
}

impl Settle {
    /// Whether `motion` is level and still enough.
    fn holds(&self, motion: &Motion) -> bool {
        motion.tilt_rad < self.tilt_rad && motion.tilt_rate_radps < self.rate_radps
    }

    /// Whether `motion` is level and still enough about `axis` alone (0 roll, 1 pitch).
    fn holds_axis(&self, motion: &Motion, axis: usize) -> bool {
        motion.axis_tilt_rad[axis] < self.tilt_rad && motion.axis_rate_radps[axis] < self.rate_radps
    }

    /// Whether `motion` is level and still enough, and turning slowly enough too.
    fn holds_yaw(&self, motion: &Motion) -> bool {
        self.holds(motion) && motion.yaw_rate_radps < self.rate_radps
    }

    /// Whether `motion` holds this settle for a probe on `target`: before a step on it the
    /// vehicle is still about that loop's axis, so a yaw rate or heading stage waits for yaw
    /// too.
    fn holds_for(&self, motion: &Motion, target: TunedLoop) -> bool {
        match tilt_axis(target) {
            Some(_) => self.holds(motion),
            None => self.holds_yaw(motion),
        }
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
    /// How long to wait for a settle, in s. At the timeout a step fails the stage, and an
    /// identification starts anyway: a fit only gains excitation from a ringing loop.
    pub settle_timeout_s: f32,
    /// After a probe on candidate gains ends, how long the vehicle has on the committed gains
    /// to come back inside the limits before a breach fails the stage, in s. A breach while
    /// the candidate flies fails only the candidate. After a freeze
    /// ([`Autotune::note_stall`]), breaches don't count for as long. Non-negative.
    pub recovery_s: f32,
    /// How long the host may go without a fresh vehicle state
    /// ([`Autotune::set_state_stale`]) before the run fails
    /// [`HostFault`](AutotuneAbort::HostFault), in s: meanwhile the vehicle flies on unseen.
    /// Positive.
    pub max_stale_s: f32,
}

impl Default for Envelope {
    /// Copter limits: 35° tilt, 4 rad/s, 3 m sink; settle within 6° and 0.3 rad/s for 1 s,
    /// waiting up to 10 s; 1 s to recover from a candidate; 2 s without a fresh state.
    /// RealFlight's reconnects come 0.5, 1.5, and 3.5 s into an outage, so the run rides
    /// through the first two; a QuadCopterX unseen for 3.5 s mid-step rolled over (#309).
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
            max_stale_s: 2.0,
        }
    }
}

impl Envelope {
    /// [`Default`]'s limits, waiting up to 30 s to settle: a full-size multirotor brakes a
    /// step's sideways speed slowly, and the Alia 250's sim model doesn't level within the
    /// default 10 s (#351).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::autotune::Envelope;
    ///
    /// assert_eq!(Envelope::large_copter().settle_timeout_s, 30.0);
    /// ```
    pub fn large_copter() -> Self {
        Self {
            settle_timeout_s: 30.0,
            ..Self::default()
        }
    }

    /// Checks that every limit and time is positive and finite, and that settling
    /// ([`check_settle`](Self::check_settle)) is tighter than aborting.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_positive("max_tilt_rad", self.max_tilt_rad)?;
        check_positive("max_rate_radps", self.max_rate_radps)?;
        check_positive("max_altitude_loss_m", self.max_altitude_loss_m)?;
        check_positive("settle_timeout_s", self.settle_timeout_s)?;
        check_limit("recovery_s", self.recovery_s)?;
        check_positive("max_stale_s", self.max_stale_s)?;
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

/// What the envelope and the settle check, read once per update from the state. The settle's
/// rates are filtered through the stabilizer's gyro low-pass, the rates its loops fly on: read
/// raw, a still vehicle's gyro noise and vibration hold them over the settle (0.2 rad/s on the
/// sim's noise rows, the settle 0.1), so no stage starts (#373). The envelope reads the raw
/// rate, so a breach shows on its first tick. Angles and rates are in the probed loops' frame
/// ([`Tunable::probe_frame`]): view axes for a tailsitter's hover, where nose-up reads level.
struct Motion {
    /// The larger of |roll| and |pitch|, in rad.
    tilt_rad: f32,
    /// The largest raw body rate magnitude, in rad/s.
    rate_radps: f32,
    /// The larger of the filtered |roll| and |pitch| rates, in rad/s.
    tilt_rate_radps: f32,
    /// |roll| and |pitch|, in rad.
    axis_tilt_rad: [f32; 2],
    /// The filtered |roll| and |pitch| rates, in rad/s.
    axis_rate_radps: [f32; 2],
    /// The filtered |yaw| rate, in rad/s.
    yaw_rate_radps: f32,
    altitude_m: f32,
}

impl Motion {
    /// `state`'s motion in `frame` (the probed loops' frame, in the body frame), with
    /// `filtered_radps` its body rates through the gyro low-pass.
    fn of(state: &VehicleState, frame: &UnitQuaternion<f32>, filtered_radps: Vec3) -> Self {
        // The view as `VehicleState::rotate_body` builds it, without cloning the state.
        let (roll, pitch, _) = euler_angles(&(state.attitude * frame));
        let raw_radps = frame.inverse_transform_vector(&state.angular_velocity_radps);
        let filtered_radps = frame.inverse_transform_vector(&filtered_radps);
        Self {
            tilt_rad: roll.abs().max(pitch.abs()),
            rate_radps: raw_radps.amax(),
            tilt_rate_radps: filtered_radps.xy().amax(),
            axis_tilt_rad: [roll.abs(), pitch.abs()],
            axis_rate_radps: [filtered_radps.x.abs(), filtered_radps.y.abs()],
            yaw_rate_radps: filtered_radps.z.abs(),
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
    /// The design found no gains for the fitted model.
    Design(DesignError),
    /// The host lost the vehicle for too long to go on: its state stayed stale past
    /// [`Envelope::max_stale_s`] (e.g. a simulator link down for seconds), or the host ended
    /// the run with [`Autotune::abort`].
    HostFault,
}

/// What an [`Autotune::update`] did, for the caller to log.
// `StageDone` carries two step summaries (about 450 bytes); boxing them would allocate, which
// core doesn't, and the caller handles one event per tick.
#[allow(clippy::large_enum_variant)]
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
        /// The judged loop.
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
        /// A step test's valid steps, or a search's verify (invalid ones, which ran again, left
        /// out).
        steps: Option<StepSummary>,
        /// A search's baseline, when flown: its steps at the start gains.
        baseline: Option<StepSummary>,
    },
    /// A search lowered its loop's gains, which started above its `max`, and committed them: the stage settles on them, and
    /// they stay if it fails or nothing better passes.
    Lowered {
        /// The searched loop.
        target: TunedLoop,
        /// The gains committed.
        gains: Gains,
    },
    /// A design didn't hold the settle on its loop's axis within the settle timeout; the
    /// committed gains are back, and the design stage fits again.
    Unsettled {
        /// The designed loop.
        target: TunedLoop,
        /// The gains that didn't hold.
        gains: Gains,
    },
    /// A stage failed: the config went back to the last kept one (earlier stages stay, except
    /// designs not yet searched), and the run ends.
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
    /// Designed from a fit: they fly, but a failure puts the loop's kept gains back until its
    /// search finishes ([`Method::Design`]).
    Designed,
    /// Kept no result: the start gains, or the run's ([`SearchPlan`]).
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
    /// committed ones when it ends; `fit`: an identification, which runs through a slow host.
    Probing {
        stage: usize,
        candidate: bool,
        fit: bool,
    },
    /// Candidate `gains` fly until the stage's axis holds the settle on them, or the settle
    /// timeout.
    Holding {
        stage: usize,
        gains: Gains,
        level_s: f32,
        waited_s: f32,
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
            | Phase::Probing { stage, .. }
            | Phase::Holding { stage, .. } => Some(stage),
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
    /// A probe or a hold on candidate gains that took the vehicle past the envelope.
    Breached { reason: AutotuneAbort },
    /// A hold ended, `settled` on its candidate or timed out.
    Held { settled: bool },
}

/// A probe a procedure asked for, and the gains to fly it on (`None`: the committed ones).
#[derive(Clone, Copy, Debug, PartialEq)]
struct Request {
    probe: ProbeConfig,
    gains: Option<Gains>,
}

impl Request {
    /// Whether the probe starts at the settle timeout rather than fail the stage: a fit only
    /// gains excitation from the ringing.
    fn starts_anyway(&self) -> bool {
        matches!(self.probe, ProbeConfig::Identify(_))
    }

    /// Whether the probe is a fit, which runs through a slow host: a fit at 223 Hz on the sim
    /// (the physics 12% slower than the ticks claim) designs rate P 5–13% lower, and the
    /// search climbs from the design (#524).
    fn is_fit(&self) -> bool {
        matches!(self.probe, ProbeConfig::Identify(_))
    }
}

/// Runs a copter autotune between ticks: each [`StagePlan`]'s procedure decides
/// which probes to run on its loop ([`Tunable::start_probe`]) and what gains follow from
/// them; the supervisor executes it.
///
/// - **Start** ([`start`](Self::start)) when armed and hovering; the caller owns the trigger.
///   The config and altitude at start are the run's start, what a disarm reverts to.
/// - **Settle:** before each probe, the vehicle must hold the method's [`Settle`]
///   ([`Method::settle`]), or the [`Envelope`]'s. The settle and the envelope read tilt and
///   rates in the frame the probed loops fly ([`Tunable::probe_frame`]), so a tailsitter
///   hovering nose up in its view reads level.
/// - **Probe and decide:** a finished probe's [`Measurement`] goes to the stage's procedure,
///   which asks for another probe or finishes with gains for its loop, committed through
///   [`set_stabilizer_config`](FlightLoop::set_stabilizer_config); the next stage flies them.
///   With [`Method::StepTest`], alternating steps and the gains unchanged; with
///   [`Method::Search`], a gain search verified against a baseline
///   ([`AutotuneEvent::Tried`] per judgment, the [`Outcome`] in `StageDone`); with
///   [`Method::Design`], one fit, the gains designed from it, and a hold on them.
/// - **Candidates:** a procedure may ask for a probe on other gains; they go in once the
///   vehicle has settled on the committed ones and come out when the probe ends, so only the
///   probe flies them. An envelope breach while they fly fails the candidate, not the stage;
///   the committed gains then have [`Envelope::recovery_s`] to bring the vehicle back. A
///   design stage holds its design instead: it goes in at once and stays only if the vehicle
///   settles about the loop's axis on it.
/// - **Stalls:** [`note_stall`](Self::note_stall) (a freeze) marks the running probe's
///   measurement as untimely, and breaches for [`Envelope::recovery_s`] after it fail nothing:
///   the vehicle moved while the host saw nothing. A candidate or a held design still comes
///   out on one, as a stall. A step test runs that step again. While the
///   host is slow ([`set_host_slow`](Self::set_host_slow)), the running probe stops at once (a
///   step held for its ticks at half the tick rate would tilt twice as far) and counts as
///   stalled, and settling waits: it gains no level time and doesn't time out. A design
///   stage's fit and hold run through a slow host: it only lowers the design. While the
///   vehicle's state is stale ([`set_state_stale`](Self::set_state_stale): the host holds
///   the last one it got while the vehicle flies on unseen), every tick is a freeze, so the
///   breach excuse runs until [`Envelope::recovery_s`] after the state clears; any running
///   probe, fit included, stops as stalled, and settling and holds wait. Stale past
///   [`Envelope::max_stale_s`], the stage fails [`HostFault`](AutotuneAbort::HostFault).
/// - **Fail:** an envelope breach, a probe abort the procedure can't use (a search counts a
///   step's abort against its gain), an interrupt, or a rejected config stops the stage, puts
///   back the config as of the last kept stage (the earlier stages' gains stay, but a design
///   counts only once its loop's search finishes), and ends the run.
/// - **Disarm** before completion reverts every stage to the run's start.
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
/// let mut autotune = Autotune::new(&StagePlan::copter_design_search(), Envelope::default())
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
    run_start: Option<CopterStabilizerConfig>,
    /// The config as of the last kept stage: what a failure reverts to. A design isn't kept
    /// until its loop's search finishes.
    last_good: Option<CopterStabilizerConfig>,
    /// The config flown between probes: `last_good` plus designs awaiting their search. A
    /// candidate goes back to it.
    committed: Option<CopterStabilizerConfig>,
    start_altitude_m: f32,
    /// The host stalled during the running probe ([`note_stall`](Self::note_stall)).
    stalled: bool,
    /// The host runs slow ([`set_host_slow`](Self::set_host_slow)).
    host_slow: bool,
    /// The vehicle's state is stale ([`set_state_stale`](Self::set_state_stale)).
    state_stale: bool,
    /// How long, in s, the state has been stale.
    stale_s: f32,
    /// Time left, in s, for the vehicle to come back inside the envelope after a candidate
    /// ([`Envelope::recovery_s`]); breaches meanwhile don't fail the stage.
    recovering_s: f32,
    /// Time left, in s, after a freeze ([`note_stall`](Self::note_stall)) during which
    /// breaches don't count at all ([`Envelope::recovery_s`]).
    frozen_s: f32,
    /// The body rates (roll, pitch, yaw) through the stabilizer's gyro low-pass, for the
    /// settle ([`Motion`] turns them into the probed loops' frame): a copy of the stabilizer's
    /// filter, at its cutoff, since the stabilizer's own sits behind every
    /// [`View`](crate::view::View) and VTOL host. Filtering before the rotation is the same
    /// as after: every axis has the same cutoff.
    rate_filters: [LowPass; 3],
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
    /// (`plan.<method>`, e.g. `plan.search`; [`Method::validate`]), or the envelope is
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
            run_start: None,
            last_good: None,
            committed: None,
            start_altitude_m: 0.0,
            stalled: false,
            host_slow: false,
            state_stale: false,
            stale_s: 0.0,
            recovering_s: 0.0,
            frozen_s: 0.0,
            rate_filters: Default::default(),
        })
    }

    /// Starts a run: records the stabilizer's config and the altitude as the run's start; the
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
        self.run_start = Some(config);
        self.last_good = Some(config);
        self.committed = Some(config);
        self.start_altitude_m = flight_loop.state().position_m.z;
        self.recovering_s = 0.0;
        self.frozen_s = 0.0;
        self.rate_filters = Default::default();
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
            if let Some(run_start) = self.run_start {
                restore(flight_loop, run_start);
            }
            self.phase = Phase::Reverted;
            return Some(AutotuneEvent::Reverted);
        }
        if self.state_stale {
            self.stale_s += dt;
            self.note_stall();
            if self.stale_s > self.envelope.max_stale_s
                && let Some(stage) = self.phase.active_stage()
            {
                return Some(self.fail(flight_loop, stage, AutotuneAbort::HostFault));
            }
        } else {
            self.stale_s = 0.0;
        }
        let rates_radps = self.filter_rates(flight_loop, dt);
        let frame = flight_loop.stabilizer().probe_frame();
        let motion = Motion::of(flight_loop.state(), &frame, rates_radps);
        self.recovering_s = (self.recovering_s - dt).max(0.0);
        self.frozen_s = (self.frozen_s - dt).max(0.0);
        if let Some(stage) = self.phase.active_stage()
            && let Some(reason) = self.envelope.breach(&motion, self.start_altitude_m)
        {
            // A breach after a freeze is the loop catching up with what the vehicle did
            // unseen: on QuadCopterX in RealFlight, 0.47 s of the same state came back at −19°
            // pitch, and the recovery peaked at 4.08 rad/s (#524). It fails nothing, but a
            // candidate or a held design still comes out, as a stall: unjudged, run again.
            let frozen = self.frozen_s > 0.0;
            let on = if frozen {
                Decide::Stalled
            } else {
                Decide::Breached { reason }
            };
            let stalled = frozen.then(|| AutotuneEvent::Stalled {
                target: self.target(stage),
                measurement: None,
            });
            // A candidate's breach fails the candidate; the committed gains get time to
            // recover from it before a breach fails the stage.
            if let Phase::Probing {
                candidate: true, ..
            } = self.phase
            {
                self.end_probe(flight_loop, stage, true, on);
                return stalled;
            }
            if let Phase::Holding { .. } = self.phase {
                self.end_hold(flight_loop, stage, on);
                return stalled;
            }
            if !frozen && self.recovering_s <= 0.0 {
                return Some(self.fail(flight_loop, stage, reason));
            }
        }
        match self.phase {
            Phase::Starting { stage } => self.begin(stage),
            Phase::Deciding { stage, on } => self.decide(flight_loop, stage, on),
            // A slow host holds a probe's settle where it is, with no level time: the vehicle
            // settles again once the host keeps time. A fit's goes on, but not on a stale
            // state, which shows a vehicle that isn't there.
            Phase::Settling {
                stage,
                request,
                waited_s,
                ..
            } if (self.host_slow && !request.is_fit()) || self.state_stale => {
                self.phase = Phase::Settling {
                    stage,
                    request,
                    level_s: 0.0,
                    waited_s,
                };
                None
            }
            Phase::Probing {
                stage,
                candidate,
                fit,
            } if (self.host_slow && !fit) || self.state_stale => {
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
            Phase::Probing {
                stage, candidate, ..
            } => self.poll_probe(flight_loop, stage, candidate),
            Phase::Holding {
                stage,
                gains,
                waited_s,
                ..
            } if self.state_stale => {
                self.phase = Phase::Holding {
                    stage,
                    gains,
                    level_s: 0.0,
                    waited_s,
                };
                None
            }
            Phase::Holding {
                stage,
                gains,
                level_s,
                waited_s,
            } => self.hold(
                flight_loop,
                &motion,
                (stage, gains),
                level_s + dt,
                waited_s + dt,
            ),
            Phase::Idle
            | Phase::Finishing
            | Phase::Complete
            | Phase::Failed { .. }
            | Phase::Reverted => None,
        }
    }

    /// The state's body rates through the stabilizer's gyro low-pass at its configured cutoff,
    /// stepped `dt` on.
    fn filter_rates<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &FlightLoop<C, V, G, S, M, T>,
        dt: f32,
    ) -> Vec3 {
        let cutoff_hz = flight_loop.stabilizer().config().gyro_cutoff_hz;
        let rates_radps = flight_loop.state().angular_velocity_radps;
        Vec3::from_fn(|axis, _| self.rate_filters[axis].apply(rates_radps[axis], dt, cutoff_hz))
    }

    /// Tells the supervisor whether the host runs slow: its ticks come further apart than
    /// their `dt`, for a stretch (e.g. RealFlight at half its rate, sitl's `TickRate`). While
    /// it does, a running probe stops and runs again later, and settling waits (see the
    /// type's **Stalls**). Takes effect on the next [`update`](Self::update).
    pub fn set_host_slow(&mut self, slow: bool) {
        self.host_slow = slow;
    }

    /// Tells the supervisor whether the vehicle's state is stale: the host holds the last
    /// state it got (e.g. a simulator link down awaiting a reconnect) while the vehicle flies
    /// on unseen, so the first fresh state can jump. While it is, the run waits and every
    /// tick is a freeze ([`note_stall`](Self::note_stall)); past
    /// [`Envelope::max_stale_s`] the stage fails [`HostFault`](AutotuneAbort::HostFault) (see
    /// the type's **Stalls**). Takes effect on the next [`update`](Self::update).
    pub fn set_state_stale(&mut self, stale: bool) {
        self.state_stale = stale;
    }

    /// Tells the supervisor the host froze (e.g. a stalled simulator exchange, a tick
    /// much later than the last): the running probe's timing can't be trusted,
    /// so its procedure treats the measurement as invalid, and for [`Envelope::recovery_s`]
    /// breaches don't count, as the loop catches up with what the vehicle did meanwhile.
    pub fn note_stall(&mut self) {
        if matches!(self.phase, Phase::Probing { .. }) {
            self.stalled = true;
        }
        if self.is_running() {
            self.frozen_s = self.envelope.recovery_s;
        }
    }

    /// Ends the run from outside for `reason` (e.g. [`AutotuneAbort::HostFault`]), as a
    /// breach would: stops the running probe, puts back the config as of the last kept stage,
    /// and reports `StageFailed`. `None`, changing nothing, when no stage is active (idle,
    /// finishing, or already ended).
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use stable_flight::FlightLoop;
    /// # use stable_flight::stabilizers::copter::CopterStabilizer;
    /// # use stable_flight::guidance::copter::CopterGuidance;
    /// # use stable_flight::mixers::TableMixer;
    /// # use stable_flight::testing::{ConstantCommander, NullVehicle};
    /// use stable_flight::tuning::autotune::{Autotune, AutotuneAbort};
    ///
    /// # fn fly(flight_loop: &mut FlightLoop<ConstantCommander, NullVehicle, CopterGuidance, CopterStabilizer, TableMixer<4>, ()>, autotune: &mut Autotune, link_down_s: f32) {
    /// // The host's link to the vehicle has been down for seconds: end the run.
    /// if link_down_s > 2.0 {
    ///     if let Some(event) = autotune.abort(flight_loop, AutotuneAbort::HostFault) {
    ///         println!("{event:?}");
    ///     }
    /// }
    /// # }
    /// ```
    pub fn abort<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        reason: AutotuneAbort,
    ) -> Option<AutotuneEvent> {
        let stage = self.phase.active_stage()?;
        Some(self.fail(flight_loop, stage, reason))
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
            | Phase::Holding { stage, .. }
            | Phase::Deciding {
                stage,
                on:
                    Decide::Measured { .. }
                    | Decide::Stalled
                    | Decide::Aborted { .. }
                    | Decide::Breached { .. }
                    | Decide::Held { .. },
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
        let config = flight_loop.stabilizer().config();
        let target = self.target(stage);
        let gains = target.gains(config);
        let input = match on {
            Decide::Start => Input::Start {
                gains,
                inner: target.inner().map(|inner| inner.gains(config)),
                run_start: self
                    .run_start
                    .map_or(gains, |run_start| target.gains(&run_start)),
                gyro_cutoff_hz: config.gyro_cutoff_hz,
            },
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
            Decide::Held { settled } => Input::Held { settled, gains },
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
            Action::Hold { gains } => {
                if self.set_gains(flight_loop, stage, gains).is_none() {
                    return Some(self.fail(flight_loop, stage, AutotuneAbort::Rejected));
                }
                self.phase = Phase::Holding {
                    stage,
                    gains,
                    level_s: 0.0,
                    waited_s: 0.0,
                };
                None
            }
            Action::Fail(reason) => Some(self.fail(flight_loop, stage, reason)),
        }
    }

    /// Waits for the vehicle to hold the stage's [`Settle`] (`level_s` counts it, restarting
    /// when it moves), then sets the request's gains, if any, and starts its probe for
    /// `stage`; fails after `settle_timeout_s`, except an identification, which starts then
    /// instead: a fit only gains excitation from the ringing.
    fn settle<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        motion: &Motion,
        (stage, request): (usize, Request),
        level_s: f32,
        waited_s: f32,
    ) -> Option<AutotuneEvent> {
        let settle = self.settle_for(stage);
        let holds = settle.holds_for(motion, self.target(stage));
        let level_s = if holds { level_s } else { 0.0 };
        let timed_out = waited_s >= self.envelope.settle_timeout_s;
        if level_s >= settle.hold_s || (request.starts_anyway() && timed_out) {
            if let Some(gains) = request.gains
                && self.set_gains(flight_loop, stage, gains).is_none()
            {
                return Some(self.fail(flight_loop, stage, AutotuneAbort::Rejected));
            }
            let probed = probed_loop(self.target(stage), &request.probe);
            flight_loop.start_probe(probed, request.probe);
            self.phase = Phase::Probing {
                stage,
                candidate: request.gains.is_some(),
                fit: request.is_fit(),
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
    /// update (a stalled probe's abort as a stall), after a candidate's gains go back to the
    /// committed ones; a stopped probe fails the run.
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
            // A stalled probe's abort is the host's, not the loop's: it runs again.
            Some((target, ProbeStatus::Aborted(_))) if self.stalled => (
                Decide::Stalled,
                Some(AutotuneEvent::Stalled {
                    target,
                    measurement: None,
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
        if candidate && let Some(committed) = self.committed {
            restore(flight_loop, committed);
            self.recovering_s = self.envelope.recovery_s;
        }
        self.phase = Phase::Deciding { stage, on };
    }

    /// Waits for the vehicle to hold the stage's [`Settle`] about its loop's axis on the
    /// candidate `gains` (`level_s` counts it): then they stay for the procedure to commit.
    /// After `settle_timeout_s` the committed gains go back ([`AutotuneEvent::Unsettled`]).
    /// For a roll or pitch loop only its axis counts: the other may still ring on its start
    /// gains (pitch on `racer`'s high start with 25 ms of delay, #518). A yaw rate or heading
    /// loop needs both level and yaw still.
    fn hold<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        motion: &Motion,
        (stage, gains): (usize, Gains),
        level_s: f32,
        waited_s: f32,
    ) -> Option<AutotuneEvent> {
        let settle = self.settle_for(stage);
        let target = self.target(stage);
        let holds = tilt_axis(target).map_or_else(
            || settle.holds_yaw(motion),
            |axis| settle.holds_axis(motion, axis),
        );
        let level_s = if holds { level_s } else { 0.0 };
        if level_s >= settle.hold_s {
            self.phase = Phase::Deciding {
                stage,
                on: Decide::Held { settled: true },
            };
            return None;
        }
        if waited_s >= self.envelope.settle_timeout_s {
            self.end_hold(flight_loop, stage, Decide::Held { settled: false });
            return Some(AutotuneEvent::Unsettled { target, gains });
        }
        self.phase = Phase::Holding {
            stage,
            gains,
            level_s,
            waited_s,
        };
        None
    }

    /// Ends `stage`'s hold: the committed gains go back, with [`Envelope::recovery_s`] to
    /// bring the vehicle inside the envelope, and `on` goes to the procedure.
    fn end_hold<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
        on: Decide,
    ) {
        if let Some(committed) = self.committed {
            restore(flight_loop, committed);
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
    /// settles on them, and a failure restores them if the stage started on the loop's kept
    /// gains. A lowered design is still a design: a failure puts the loop's kept gains back.
    /// `StageFailed` if the stabilizer rejected them, else `Lowered`.
    fn lower<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
        gains: Gains,
    ) -> AutotuneEvent {
        let target = self.target(stage);
        let designed = self
            .committed
            .zip(self.last_good)
            .is_some_and(|(flown, kept)| target.gains(&flown) != target.gains(&kept));
        let Some(config) = self.set_gains(flight_loop, stage, gains) else {
            return self.fail(flight_loop, stage, AutotuneAbort::Rejected);
        };
        self.committed = Some(config);
        if !designed {
            self.keep(target, gains);
        }
        AutotuneEvent::Lowered {
            target: self.target(stage),
            gains,
        }
    }

    /// Commits a finished stage's gains and moves on. A design is flown but not yet kept: a
    /// failure before its loop's search puts the loop's earlier gains back. Any other stage
    /// keeps its loop's gains, and only those: other loops' designs still wait on their own.
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
        self.committed = Some(config);
        if outcome != Outcome::Designed {
            self.keep(self.target(stage), gains);
        }
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

    /// Keeps `target`'s `gains` in what a failure reverts to, leaving the other loops as kept.
    fn keep(&mut self, target: TunedLoop, gains: Gains) {
        if let Some(kept) = &mut self.last_good {
            *target.gains_mut(kept) = gains;
        }
    }

    /// Ends the run at `stage`: stops its probe and puts back the config as of the last kept
    /// stage.
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
        self.committed = self.last_good;
        self.phase = Phase::Failed { stage, reason };
        AutotuneEvent::StageFailed {
            target: self.target(stage),
            reason,
        }
    }
}

/// The loop `probe` runs on for a stage on `target`: an identification excites an angle
/// loop's axis on the rate loop under it ([`DesignPlan`]); every other probe runs on `target`
/// itself.
fn probed_loop(target: TunedLoop, probe: &ProbeConfig) -> TunedLoop {
    match probe {
        ProbeConfig::Identify(_) => target.inner().unwrap_or(target),
        ProbeConfig::Step(_) => target,
    }
}

/// The tilt axis `target` holds (0 roll, 1 pitch); `None` for yaw and heading.
fn tilt_axis(target: TunedLoop) -> Option<usize> {
    match target {
        TunedLoop::RollRate | TunedLoop::Roll => Some(0),
        TunedLoop::PitchRate | TunedLoop::Pitch => Some(1),
        TunedLoop::YawRate | TunedLoop::Heading => None,
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
