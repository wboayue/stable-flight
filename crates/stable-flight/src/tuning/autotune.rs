//! The copter autotune supervisor: see [`Autotune`].

use core::fmt;

use crate::controllers::{Gains, InvalidConfig, check, check_positive};
use crate::math::euler_angles;
use crate::stabilizers::copter::{CopterStabilizerConfig, TunedLoop};
use crate::{Configurable, FlightLoop, VehicleState};

use super::{
    ControllerForm, Measurement, ProbeAbort, ProbeConfig, ProbeStatus, RelayConfig, RelayResult,
    StepConfig, Tunable, TuningRule, gains_from_ultimate,
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

/// The loops [`StagePlan::copter_sequence`] tunes: roll and pitch rate, the loops a bad start
/// breaks. Flights on QuadCopterX (#295) left the rest out:
///
/// - Yaw rate: the relay can't measure it within the yaw authority the mixer leaves at hover
///   (#296).
/// - Roll and pitch angle: the relay measures a high ultimate gain (about 60: an integrator
///   around a fast rate loop), and the rule's P oscillates once a few degrees of error saturate
///   the rate setpoint and the moments. Step-response tuning (phase 2) sees the saturation.
/// - Heading: both of the above.
///
/// Their stages still run when planned explicitly (`StagePlan::copter`).
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
}

/// How a stage tunes its loop: which probes it runs and what gains follow from them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Method {
    /// One relay measurement, turned into gains by a tuning rule.
    Relay(RelayPlan),
    /// Alternating step responses, measured and reported; the gains don't change.
    StepTest(StepTestPlan),
}

impl Method {
    /// Checks the method's own fields and probe configs (its settle is checked against the
    /// envelope by [`Autotune::new`]).
    ///
    /// # Errors
    ///
    /// When the relay is invalid ([`RelayConfig::validate`]), or the step test is
    /// ([`StepTestPlan::validate`]).
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        match self {
            Self::Relay(plan) => plan.relay.validate(),
            Self::StepTest(plan) => plan.validate(),
        }
    }

    /// The settle this method asks for before each probe; `None` uses the envelope's.
    pub fn settle(&self) -> Option<Settle> {
        match self {
            Self::Relay(_) => None,
            Self::StepTest(plan) => Some(plan.settle),
        }
    }

    /// The section [`Autotune::new`] reports this method's errors in.
    fn plan_section(&self) -> &'static str {
        match self {
            Self::Relay(_) => "plan.relay",
            Self::StepTest(_) => "plan.step_test",
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

/// Step response of a copter angle loop: 10° held for 0.8 s, filtered at 20 Hz.
pub const COPTER_ANGLE_STEP: StepConfig = StepConfig {
    amplitude: 0.175,
    duration_s: 0.8,
    settle_band: 0.1,
    filter_hz: 20.0,
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
/// `max_saturated_s` (a clipped step measures the limit, not the loop), or flagged stalled
/// ([`Autotune::note_stall`]), is invalid and runs again; `max_invalid` in a row fail the
/// stage.
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
    /// Invalid steps in a row that fail the stage. At least 1.
    pub max_invalid: usize,
}

impl StepTestPlan {
    /// The copter defaults for `target`: three pairs of [`COPTER_RATE_STEP`] (rate loops) or
    /// [`COPTER_ANGLE_STEP`] (angle and heading loops) from [`COPTER_STEP_SETTLE`], allowing
    /// 20 ms (5 ticks) of clipping and 3 invalid steps in a row.
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
        let step = match target {
            TunedLoop::RollRate | TunedLoop::PitchRate | TunedLoop::YawRate => COPTER_RATE_STEP,
            TunedLoop::Roll | TunedLoop::Pitch | TunedLoop::Heading => COPTER_ANGLE_STEP,
        };
        Self {
            step,
            pairs: 3,
            settle: COPTER_STEP_SETTLE,
            max_saturated_s: 0.02,
            max_invalid: 3,
        }
    }

    /// Checks the step ([`StepConfig::validate`]) and the counts and clip allowance.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        self.step.validate()?;
        check("pairs", self.pairs >= 1, "at least 1")?;
        check(
            "max_saturated_s",
            self.max_saturated_s >= 0.0 && self.max_saturated_s.is_finite(),
            "non-negative and finite",
        )?;
        check("max_invalid", self.max_invalid >= 1, "at least 1")
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
        let relay = match target {
            TunedLoop::RollRate | TunedLoop::PitchRate | TunedLoop::YawRate => COPTER_RATE_RELAY,
            TunedLoop::Roll | TunedLoop::Pitch | TunedLoop::Heading => COPTER_ANGLE_RELAY,
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
    /// How long to wait for a settle before failing, in s.
    pub settle_timeout_s: f32,
}

impl Default for Envelope {
    /// Copter limits: 35° tilt, 4 rad/s, 3 m sink; settle within 6° and 0.3 rad/s for 1 s,
    /// waiting up to 10 s.
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
    /// Too many invalid probes in a row (clipped or stalled steps:
    /// [`StepTestPlan::max_invalid`]).
    Invalid,
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
    /// A stage finished and committed its loop's gains.
    StageDone {
        /// The tuned loop.
        target: TunedLoop,
        /// The gains committed for it.
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
    /// Waiting for the vehicle to settle before running `probe`.
    Settling {
        stage: usize,
        probe: ProbeConfig,
        level_s: f32,
        waited_s: f32,
    },
    Probing {
        stage: usize,
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
            | Phase::Probing { stage } => Some(stage),
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
}

/// Runs a copter autotune between ticks (#149, #298): each [`StagePlan`]'s procedure decides
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
///   [`Method::StepTest`], alternating steps and the gains unchanged.
/// - **Stalls:** [`note_stall`](Self::note_stall) marks the running probe's measurement as
///   untimely; a step test runs that step again.
/// - **Fail:** an envelope breach, a probe abort, an interrupt, or a rejected config stops the
///   stage, puts back the last committed config (the earlier stages' gains stay), and ends the
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
/// # use stable_flight::mixers::QuadCopterMixer;
/// # use stable_flight::testing::{ConstantCommander, NullVehicle};
/// use stable_flight::tuning::autotune::{Autotune, AutotuneEvent, Envelope, StagePlan};
///
/// # fn fly(flight_loop: &mut FlightLoop<ConstantCommander, NullVehicle, CopterGuidance, CopterStabilizer, QuadCopterMixer, ()>) {
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
        if let Some(stage) = self.phase.active_stage()
            && let Some(reason) = self.envelope.breach(&motion, self.start_altitude_m)
        {
            return Some(self.fail(flight_loop, stage, reason));
        }
        match self.phase {
            Phase::Starting { stage } => self.begin(stage),
            Phase::Deciding { stage, on } => self.decide(flight_loop, stage, on),
            Phase::Settling {
                stage,
                probe,
                level_s,
                waited_s,
            } => self.settle(
                flight_loop,
                &motion,
                (stage, probe),
                level_s + dt,
                waited_s + dt,
            ),
            Phase::Probing { stage } => self.poll_probe(flight_loop, stage),
            Phase::Idle
            | Phase::Finishing
            | Phase::Complete
            | Phase::Failed { .. }
            | Phase::Reverted => None,
        }
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
            Phase::Probing { stage }
            | Phase::Deciding {
                stage,
                on: Decide::Measured { .. },
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

    /// Hands the procedure the stage's start or its last measurement, with the loop's current
    /// gains, and carries out what it asks for.
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
        };
        match self.procedure.next(input) {
            Action::Probe(probe) => {
                self.phase = Phase::Settling {
                    stage,
                    probe,
                    level_s: 0.0,
                    waited_s: 0.0,
                };
                None
            }
            Action::Finish(gains) => Some(self.commit(flight_loop, stage, gains)),
            Action::Fail(reason) => Some(self.fail(flight_loop, stage, reason)),
        }
    }

    /// Waits for the vehicle to hold the stage's [`Settle`] (`level_s` counts it, restarting
    /// when it moves), then starts `probe` for `stage`; fails after `settle_timeout_s`.
    fn settle<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        motion: &Motion,
        (stage, probe): (usize, ProbeConfig),
        level_s: f32,
        waited_s: f32,
    ) -> Option<AutotuneEvent> {
        let settle = self.settle_for(stage);
        let level_s = if settle.holds(motion) { level_s } else { 0.0 };
        if level_s >= settle.hold_s {
            flight_loop.start_probe(self.target(stage), probe);
            self.phase = Phase::Probing { stage };
            self.stalled = false;
            return None;
        }
        if waited_s >= self.envelope.settle_timeout_s {
            return Some(self.fail(flight_loop, stage, AutotuneAbort::NotSettled));
        }
        self.phase = Phase::Settling {
            stage,
            probe,
            level_s,
            waited_s,
        };
        None
    }

    /// Checks the running probe: a measurement goes to the procedure on the next update; a
    /// stopped probe fails the run.
    fn poll_probe<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
    ) -> Option<AutotuneEvent> {
        let reason = match flight_loop.probe() {
            Some((_, ProbeStatus::Running(_))) => return None,
            Some((target, ProbeStatus::Done(measurement))) => {
                flight_loop.stop_probe();
                self.phase = Phase::Deciding {
                    stage,
                    on: Decide::Measured {
                        measurement,
                        stalled: self.stalled,
                    },
                };
                return Some(AutotuneEvent::Measured {
                    target,
                    measurement,
                });
            }
            Some((_, ProbeStatus::Aborted(abort))) => AutotuneAbort::Probe(abort),
            Some((_, ProbeStatus::Interrupted)) | None => AutotuneAbort::Interrupted,
        };
        Some(self.fail(flight_loop, stage, reason))
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

    /// Commits a finished stage's gains and moves on.
    fn commit<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
        gains: Gains,
    ) -> AutotuneEvent {
        let target = self.target(stage);
        let mut config = *flight_loop.stabilizer().config();
        *target.gains_mut(&mut config) = gains;
        if flight_loop.set_stabilizer_config(config).is_err() {
            return self.fail(flight_loop, stage, AutotuneAbort::Rejected);
        }
        self.last_good = Some(config);
        self.phase = if stage + 1 < self.stages {
            Phase::Starting { stage: stage + 1 }
        } else {
            Phase::Finishing
        };
        AutotuneEvent::StageDone { target, gains }
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
