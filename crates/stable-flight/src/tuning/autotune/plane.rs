//! The plane side of the supervisor: a plane's roll and pitch loops ([`PlaneSide`]), tuned by
//! a [`PlaneMethod`] within a [`PlaneEnvelope`], as a [`PlaneAutotune`].

use super::procedure::{Action, PlaneProcedure, Verdict};
use super::side::{Decide, Motion, RateFilters, Side, SideConfig, StageView, Waits, zero_if_nan};
use super::{AutotuneAbort, DEFAULT_SETTLE_SPEED_MPS, Settle, StagePlan, StepTestPlan, Supervisor};
use crate::controllers::{InvalidConfig, check, check_limit, check_positive};
use crate::math::DEG;
use crate::stabilizers::plane::{PlaneAxisGains, PlaneLoop, PlaneStabilizerConfig};
use crate::stabilizers::vtol::{QuadPlaneStabilizerConfig, TailsitterStabilizerConfig};
use crate::tuning::{IdentifyConfig, ProbeConfig, StepConfig, WingDesignConfig};
use crate::{Setpoint, Vec2, Vec3};

/// The plane autotune: a [`Supervisor`] of a plane's roll and pitch loops ([`PlaneSide`]), on
/// a [`PlaneStabilizer`](crate::stabilizers::plane::PlaneStabilizer) or a VTOL's plane side.
///
/// The probes ride on whatever the commander flies; a plane's need a steady attitude hold at
/// cruise thrust, the airspeed trimmed between probes (as ArduPilot's AUTOTUNE flies in FBWA
/// at the pilot's throttle), not a line held by the energy control, which answers a pitch step
/// with its own pitch (#533).
pub type PlaneAutotune = Supervisor<PlaneSide>;

/// A plane's roll and pitch loops ([`PlaneLoop`]), each stage on its loop's axis gains
/// ([`PlaneAxisGains`]), tuned by a [`PlaneMethod`] within a [`PlaneEnvelope`]: the
/// [`PlaneAutotune`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlaneSide;

/// How a plane stage tunes its loop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PlaneMethod {
    /// Alternating step responses, measured and reported; the gains don't change.
    StepTest(StepTestPlan),
    /// The axis fitted in flight, its gains designed on the fit, then checked on attitude
    /// steps.
    Design(PlaneDesignPlan),
}

/// The share of the attitude loop's rate limit a plane identification excites its rate loop
/// with: on the sim's planes it moved the angle of attack enough to fit the wing's stiffness
/// without leaving the envelope (#533).
pub const PLANE_IDENTIFY_SHARE: f32 = 0.3;

/// How long a plane identification excites, in s: the window the sweep fitted, against the
/// copter's 3 s (#533).
pub const PLANE_IDENTIFY_S: f32 = 6.0;

/// Largest overshoot a plane design allows, as a fraction of the step: ArduPilot's copter
/// target, `1 + AUTOTUNE_AGGR/2` at its default `AUTOTUNE_AGGR` 0.1 (`AC_AutoTune_Multi`);
/// neither ArduPlane's autotune nor PX4's has a step target.
pub const PLANE_MAX_OVERSHOOT: f32 = 0.05;

/// The 10–90% rise a plane design allows, in time constants: the attitude loop asks for the
/// error over its time constant, as PX4's does (`FW_R_TC`, `FW_P_TC`), so the response is
/// first order, which rises in 2.2.
pub const PLANE_RISE_PER_TIME_CONSTANT: f32 = 2.2;

/// The time constants a plane design picks from, in s: PX4's `FW_R_TC` and `FW_P_TC` range
/// from 0.2 up, and the Alia 250's roll needed up to 1.5 on the sim (#533).
pub const PLANE_TIME_CONSTANT_RANGE_S: [f32; 2] = [0.2, 1.5];

/// The share of the surfaces' throw (less the trim) a plane design may use: a fifth held back
/// for what the fit misses, since a step that clips measures the surfaces, not the loop (#533).
pub const PLANE_MOMENT_SHARE: f32 = 0.8;

/// A plane design stage: fit the stage's axis ([`ProbeConfig::IdentifyWing`] on its rate
/// loop), design its rate gains and time constant on the fit
/// ([`WingDesign`](crate::tuning::WingDesign)), then fly pairs of attitude steps on them until
/// a pair meets the design's targets. A pair that sags (unsettled, or slower than the rise
/// allowed) raises I by half, one that overshoots or clips raises the time constant by 15%:
/// a held attitude bleeds airspeed, which the fit at constant airspeed doesn't see (#533).
/// Tunes the whole axis ([`PlaneAxisGains`]); its stages are the angle loops.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlaneDesignPlan {
    /// The identification on the axis's rate loop.
    pub identify: IdentifyConfig,
    /// The design's targets and the limits its model flies under.
    pub design: WingDesignConfig,
    /// The attitude step a pair flies, each way; its amplitude should be the design's.
    pub verify: StepConfig,
    /// Most pairs before the stage fails ([`AutotuneAbort::Unverified`]). Positive.
    pub max_verify_pairs: usize,
    /// Design candidates stepped per update: about 4,000 multiplies each. Positive.
    pub candidates_per_update: usize,
    /// Steady enough to start each probe.
    pub settle: Settle,
}

impl PlaneDesignPlan {
    /// The plane defaults for `target`'s axis on a plane flying `config`: a 6 s identification
    /// at [`PLANE_IDENTIFY_SHARE`] of the axis's rate limit, a design to
    /// [`PLANE_MAX_OVERSHOOT`], [`PLANE_RISE_PER_TIME_CONSTANT`], and
    /// [`PLANE_MAX_SATURATED_S`] within [`PLANE_TIME_CONSTANT_RANGE_S`] and
    /// [`PLANE_MOMENT_SHARE`] of the throw, the step [`StepTestPlan::plane`]'s, up to 8
    /// pairs, 4 candidates per update, from [`PLANE_STEP_SETTLE`].
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::plane::{PlaneLoop, PlaneStabilizerConfig};
    /// use stable_flight::tuning::autotune::{PLANE_IDENTIFY_SHARE, PlaneDesignPlan};
    ///
    /// let config = PlaneStabilizerConfig::default();
    /// let plan = PlaneDesignPlan::for_plane(PlaneLoop::Pitch, &config);
    /// let max_rate = config.max_attitude_rate_radps.y;
    /// assert_eq!(plan.identify.amplitude, PLANE_IDENTIFY_SHARE * max_rate);
    /// assert_eq!(plan.design.amplitude_rad, plan.verify.amplitude);
    /// assert!(plan.validate().is_ok());
    /// ```
    pub fn for_plane(target: PlaneLoop, config: &PlaneStabilizerConfig) -> Self {
        let axis = target.axis();
        let max_rate_radps = config.max_attitude_rate_radps[axis];
        // The axis's attitude step, whichever loop names the stage.
        let angle = match target {
            PlaneLoop::RollRate | PlaneLoop::Roll => PlaneLoop::Roll,
            PlaneLoop::PitchRate | PlaneLoop::Pitch => PlaneLoop::Pitch,
        };
        let verify = StepTestPlan::plane(angle, config).step;
        Self {
            identify: IdentifyConfig {
                amplitude: PLANE_IDENTIFY_SHARE * max_rate_radps,
                duration_s: PLANE_IDENTIFY_S,
                washout_tc_s: 0.2,
            },
            design: WingDesignConfig {
                amplitude_rad: verify.amplitude,
                step_duration_s: verify.duration_s,
                max_rate_radps,
                max_moment: PLANE_MOMENT_SHARE * (1.0 - config.rate.trim[axis].abs()),
                max_integral: config.rate.max_integral,
                static_trim: config.rate.trim[axis],
                max_overshoot: PLANE_MAX_OVERSHOOT,
                max_saturated_s: PLANE_MAX_SATURATED_S,
                time_constant_range_s: PLANE_TIME_CONSTANT_RANGE_S,
                rise_per_time_constant: PLANE_RISE_PER_TIME_CONSTANT,
            },
            verify,
            max_verify_pairs: 8,
            candidates_per_update: 4,
            settle: PLANE_STEP_SETTLE,
        }
    }

    /// Checks the identification, the design, the step, and that the counts are positive.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        self.identify
            .validate()
            .map_err(|e| e.in_section("identify"))?;
        self.design.validate().map_err(|e| e.in_section("design"))?;
        self.verify.validate().map_err(|e| e.in_section("verify"))?;
        check("max_verify_pairs", self.max_verify_pairs > 0, "positive")?;
        check(
            "candidates_per_update",
            self.candidates_per_update > 0,
            "positive",
        )
    }
}

/// A plane rate step: 8°/s held for 1 s. A held rate turns the aircraft away from the attitude
/// its loops hold, so the step stays small: 8° of rotation by the end (#533).
pub const PLANE_RATE_STEP: StepConfig = StepConfig {
    amplitude: 8.0 * DEG,
    duration_s: 1.0,
    settle_band: 0.1,
    filter_hz: 0.0,
};

/// A plane roll step: 30°, a turn's bank, held 4 s: long enough to settle at the 0.4–0.6 s
/// time constants the sim's planes fly.
pub const PLANE_ROLL_STEP: StepConfig = StepConfig {
    amplitude: 30.0 * DEG,
    duration_s: 4.0,
    settle_band: 0.1,
    filter_hz: 0.0,
};

/// A plane pitch step: 10°, a climb or dive the airspeed survives at cruise thrust, held 4 s.
pub const PLANE_PITCH_STEP: StepConfig = StepConfig {
    amplitude: 10.0 * DEG,
    ..PLANE_ROLL_STEP
};

/// Level before each plane step: within 3° of the held attitude and 0.1 rad/s on each axis
/// for 1 s. A wing in an attitude hold rides its phugoid, so the band is wider than a
/// copter's (2.5°, [`COPTER_STEP_SETTLE`](super::COPTER_STEP_SETTLE)); the airspeed has a
/// settle of its own ([`AirspeedSettle`]). On the sim's four planes every step settled
/// inside [`PlaneEnvelope::for_plane`]'s 20 s (#534).
pub const PLANE_STEP_SETTLE: Settle = Settle {
    tilt_rad: 3.0 * DEG,
    rate_radps: 0.1,
    hold_s: 1.0,
    max_speed_mps: DEFAULT_SETTLE_SPEED_MPS,
};

/// How long a plane step may clip and still count, in s: 10 ticks at 250 Hz, as a copter's
/// ([`COPTER_MAX_SATURATED_S`](super::COPTER_MAX_SATURATED_S)). The sim's planes don't clip
/// on [`PLANE_ROLL_STEP`] or [`PLANE_PITCH_STEP`] (#310).
pub const PLANE_MAX_SATURATED_S: f32 = 0.042;

impl StepTestPlan {
    /// The plane defaults for `target` on a plane flying `config`: three pairs of
    /// [`PLANE_RATE_STEP`] (roll and pitch rate), [`PLANE_ROLL_STEP`] (roll), or
    /// [`PLANE_PITCH_STEP`] (pitch), each no larger than `config`'s bank or pitch limit, from
    /// [`PLANE_STEP_SETTLE`], each clipping for up to [`PLANE_MAX_SATURATED_S`], allowing 3
    /// clipped and 10 stalled steps in a row. A step replaces the saturated setpoint, so a step
    /// past the limit would fly an attitude the airframe's limit forbids: the Alia 250's 20°
    /// bank limit holds its roll steps to 20°, and at 30° its roll stage sank past the
    /// envelope's 40 m.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::plane::{PlaneLoop, PlaneStabilizerConfig};
    /// use stable_flight::tuning::autotune::{PLANE_PITCH_STEP, StepTestPlan};
    ///
    /// let config = PlaneStabilizerConfig { max_bank_rad: 0.2, ..Default::default() };
    /// assert_eq!(StepTestPlan::plane(PlaneLoop::Pitch, &config).step, PLANE_PITCH_STEP);
    /// assert_eq!(StepTestPlan::plane(PlaneLoop::Roll, &config).step.amplitude, 0.2);
    /// ```
    pub fn plane(target: PlaneLoop, config: &PlaneStabilizerConfig) -> Self {
        let capped = |step: StepConfig, limit_rad: f32| StepConfig {
            amplitude: step.amplitude.min(limit_rad),
            ..step
        };
        let step = match target {
            PlaneLoop::RollRate | PlaneLoop::PitchRate => PLANE_RATE_STEP,
            PlaneLoop::Roll => capped(PLANE_ROLL_STEP, config.max_bank_rad),
            PlaneLoop::Pitch => capped(PLANE_PITCH_STEP, config.max_pitch_rad),
        };
        Self {
            step,
            pairs: 3,
            settle: PLANE_STEP_SETTLE,
            max_saturated_s: PLANE_MAX_SATURATED_S,
            max_invalid: 3,
            max_stalled: 10,
        }
    }
}

impl StagePlan<PlaneSide> {
    /// A design of `target`'s axis with the plane defaults for `config`
    /// ([`PlaneDesignPlan::for_plane`]).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::plane::{PlaneLoop, PlaneStabilizerConfig};
    /// use stable_flight::tuning::autotune::{PlaneAutotune, PlaneEnvelope, StagePlan};
    ///
    /// let config = PlaneStabilizerConfig::default();
    /// let plan = [PlaneLoop::Roll, PlaneLoop::Pitch].map(|t| StagePlan::plane_design(t, &config));
    /// assert!(PlaneAutotune::new(&plan, PlaneEnvelope::for_plane(&config)).is_ok());
    /// ```
    pub fn plane_design(target: PlaneLoop, config: &PlaneStabilizerConfig) -> Self {
        Self {
            target,
            method: PlaneMethod::Design(PlaneDesignPlan::for_plane(target, config)),
        }
    }

    /// A step test of `target` with the plane defaults for `config`
    /// ([`StepTestPlan::plane`]).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::plane::{PlaneLoop, PlaneStabilizerConfig};
    /// use stable_flight::tuning::autotune::{PlaneAutotune, PlaneEnvelope, StagePlan};
    ///
    /// let config = PlaneStabilizerConfig::default();
    /// let plan = [PlaneLoop::Roll, PlaneLoop::Pitch].map(|t| StagePlan::plane_step_test(t, &config));
    /// assert!(PlaneAutotune::new(&plan, PlaneEnvelope::for_plane(&config)).is_ok());
    /// ```
    pub fn plane_step_test(target: PlaneLoop, config: &PlaneStabilizerConfig) -> Self {
        Self {
            target,
            method: PlaneMethod::StepTest(StepTestPlan::plane(target, config)),
        }
    }
}

/// Limits that end a plane stage, and how steady the wing must fly before a probe starts.
/// Angles are from level, in the airplane frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlaneEnvelope {
    /// Bank beyond this aborts the stage, in rad.
    pub max_bank_rad: f32,
    /// Pitch, up or down, beyond this aborts the stage, in rad.
    pub max_pitch_rad: f32,
    /// A roll, pitch, or yaw rate beyond this aborts the stage, in rad/s.
    pub max_rate_radps: f32,
    /// Sinking more than this below the altitude at the start aborts, in m.
    pub max_altitude_loss_m: f32,
    /// Airspeed below this aborts, in m/s: the wing nears its stall. A missing airspeed never
    /// counts.
    pub min_airspeed_mps: f32,
    /// Airspeed above this aborts, in m/s: a dive the loops weren't tuned for.
    pub max_airspeed_mps: f32,
    /// Steady enough to start a probe, unless the stage's method sets its own.
    pub settle: Settle,
    /// How long to wait for a settle, in s ([`Envelope`](super::Envelope)'s).
    pub settle_timeout_s: f32,
    /// How long the committed gains have to bring the wing back inside the limits after a
    /// candidate, or after a freeze, in s ([`Envelope`](super::Envelope)'s). Non-negative.
    pub recovery_s: f32,
    /// How long the host may go without a fresh vehicle state before the run fails, in s
    /// ([`Envelope`](super::Envelope)'s). Positive.
    pub max_stale_s: f32,
    /// The airspeed a probe starts at, besides the settle's attitude and rates.
    pub settle_airspeed: AirspeedSettle,
}

/// Steady enough in airspeed to start a plane probe: within `band_mps` of `trim_mps` and
/// changing by less than `max_change_mps2`, for the settle's hold. In an attitude hold a wing
/// rides its phugoid, trading airspeed for climb over several seconds with its attitude and
/// rates still, and a step started on that swing reads the loop wrong: on the sim, steps
/// started ±1.7 m/s off the airplane's trim read 0–2% overshoot where the same gains overshot
/// 4–5% from steady flight, so the stage passed gains the plane spec failed (#534). A missing
/// airspeed never holds a probe back.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AirspeedSettle {
    /// The airspeed the hold trims to, in m/s.
    pub trim_mps: f32,
    /// How far from `trim_mps` still counts, in m/s. Positive.
    pub band_mps: f32,
    /// The fastest the airspeed may change, in m/s². Positive.
    pub max_change_mps2: f32,
}

impl AirspeedSettle {
    /// Whether `motion`'s airspeed is within the band and steady (or missing).
    fn holds(&self, motion: &Motion) -> bool {
        motion.airspeed_mps.is_none_or(|airspeed_mps| {
            (airspeed_mps - self.trim_mps).abs() < self.band_mps
                && motion.airspeed_change_mps2.abs() < self.max_change_mps2
        })
    }
}

impl PlaneEnvelope {
    /// Limits for a plane flying `config`: 60° of bank and 30° of pitch (a 30° roll step and
    /// a 10° pitch step from the hold, with room to overshoot), 4 rad/s, 40 m of sink, and
    /// airspeed between 0.9 × `config`'s minimum and its maximum; settle as
    /// [`PLANE_STEP_SETTLE`] with the airspeed within 1 m/s of `config`'s trim and changing by
    /// under 0.2 m/s² ([`AirspeedSettle`]), waiting up to 20 s, 2 s to recover, 2 s without a
    /// fresh state.
    /// On the sim the airplane slowed past its minimum within a few steps unless the hold
    /// trimmed its pitch for airspeed; with the trim, these limits never tripped on a sound
    /// start (#533).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::plane::PlaneStabilizerConfig;
    /// use stable_flight::tuning::autotune::PlaneEnvelope;
    ///
    /// let config = PlaneStabilizerConfig::default();
    /// let envelope = PlaneEnvelope::for_plane(&config);
    /// assert_eq!(envelope.min_airspeed_mps, 0.9 * config.min_airspeed_mps);
    /// ```
    pub fn for_plane(config: &PlaneStabilizerConfig) -> Self {
        Self {
            max_bank_rad: 60.0 * DEG,
            max_pitch_rad: 30.0 * DEG,
            max_rate_radps: 4.0,
            max_altitude_loss_m: 40.0,
            min_airspeed_mps: 0.9 * config.min_airspeed_mps,
            max_airspeed_mps: config.max_airspeed_mps,
            settle: PLANE_STEP_SETTLE,
            settle_timeout_s: 20.0,
            recovery_s: 2.0,
            max_stale_s: 2.0,
            settle_airspeed: AirspeedSettle {
                trim_mps: config.trim_airspeed_mps,
                band_mps: 1.0,
                max_change_mps2: 0.2,
            },
        }
    }

    /// Checks that every limit and time is positive and finite, the airspeed band in order,
    /// and settling ([`check_settle`](Self::check_settle)) tighter than aborting.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_positive("max_bank_rad", self.max_bank_rad)?;
        check_positive("max_pitch_rad", self.max_pitch_rad)?;
        check_positive("max_rate_radps", self.max_rate_radps)?;
        check_positive("max_altitude_loss_m", self.max_altitude_loss_m)?;
        check_positive("min_airspeed_mps", self.min_airspeed_mps)?;
        check(
            "max_airspeed_mps",
            self.max_airspeed_mps > self.min_airspeed_mps && self.max_airspeed_mps.is_finite(),
            "finite and above min_airspeed_mps",
        )?;
        check_positive("settle_timeout_s", self.settle_timeout_s)?;
        check_limit("recovery_s", self.recovery_s)?;
        check_positive("max_stale_s", self.max_stale_s)?;
        let airspeed = &self.settle_airspeed;
        check_positive("settle_airspeed.trim_mps", airspeed.trim_mps)?;
        check_positive("settle_airspeed.band_mps", airspeed.band_mps)?;
        check_positive("settle_airspeed.max_change_mps2", airspeed.max_change_mps2)?;
        self.check_settle(&self.settle)
    }

    /// Checks that `settle` is positive, tighter than the bank, pitch, and rate limits, and
    /// fits in `settle_timeout_s`.
    pub fn check_settle(&self, settle: &Settle) -> Result<(), InvalidConfig> {
        check(
            "settle.tilt_rad",
            settle.tilt_rad > 0.0 && settle.tilt_rad < self.max_bank_rad.min(self.max_pitch_rad),
            "positive and below max_bank_rad and max_pitch_rad",
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
        let airspeed_mps = motion.airspeed_mps;
        if motion.bank_rad > self.max_bank_rad || motion.pitch_rad > self.max_pitch_rad {
            Some(AutotuneAbort::Tilt)
        } else if motion.rate_radps > self.max_rate_radps {
            Some(AutotuneAbort::Rate)
        } else if start_altitude_m - motion.altitude_m > self.max_altitude_loss_m {
            Some(AutotuneAbort::AltitudeLoss)
        } else if airspeed_mps.is_some_and(|mps| mps < self.min_airspeed_mps) {
            Some(AutotuneAbort::AirspeedLow)
        } else if airspeed_mps.is_some_and(|mps| mps > self.max_airspeed_mps) {
            Some(AutotuneAbort::AirspeedHigh)
        } else {
            None
        }
    }
}

impl SideConfig<PlaneSide> for PlaneStabilizerConfig {
    fn side(&self) -> &PlaneStabilizerConfig {
        self
    }

    fn side_mut(&mut self) -> &mut PlaneStabilizerConfig {
        self
    }
}

impl SideConfig<PlaneSide> for TailsitterStabilizerConfig {
    fn side(&self) -> &PlaneStabilizerConfig {
        &self.plane
    }

    fn side_mut(&mut self) -> &mut PlaneStabilizerConfig {
        &mut self.plane
    }
}

impl SideConfig<PlaneSide> for QuadPlaneStabilizerConfig {
    fn side(&self) -> &PlaneStabilizerConfig {
        &self.plane
    }

    fn side_mut(&mut self) -> &mut PlaneStabilizerConfig {
        &mut self.plane
    }
}

impl Side for PlaneSide {
    type Target = PlaneLoop;
    type Gains = PlaneAxisGains;
    type Config = PlaneStabilizerConfig;
    type Method = PlaneMethod;
    type Envelope = PlaneEnvelope;
    type Procedure = PlaneProcedure;

    fn gains(target: PlaneLoop, config: &PlaneStabilizerConfig) -> PlaneAxisGains {
        target.gains(config)
    }

    fn set_gains(target: PlaneLoop, config: &mut PlaneStabilizerConfig, gains: PlaneAxisGains) {
        target.set_gains(config, gains);
    }

    /// An identification excites an angle loop's axis on the rate loop under it; every other
    /// probe runs on `target` itself.
    fn probed_loop(target: PlaneLoop, probe: &ProbeConfig) -> PlaneLoop {
        match probe {
            ProbeConfig::Identify(_) | ProbeConfig::IdentifyWing(_) => {
                target.inner().unwrap_or(target)
            }
            ProbeConfig::Step(_) => target,
        }
    }

    fn validate_method(method: &PlaneMethod) -> Result<(), InvalidConfig> {
        match method {
            PlaneMethod::StepTest(plan) => {
                plan.validate().map_err(|e| e.in_section("plan.step_test"))
            }
            PlaneMethod::Design(plan) => plan.validate().map_err(|e| e.in_section("plan.design")),
        }
    }

    fn method_settle(method: &PlaneMethod) -> Option<Settle> {
        match method {
            PlaneMethod::StepTest(plan) => Some(plan.settle),
            PlaneMethod::Design(plan) => Some(plan.settle),
        }
    }

    fn procedure(method: PlaneMethod) -> PlaneProcedure {
        PlaneProcedure::new(method)
    }

    fn decide(
        procedure: &mut PlaneProcedure,
        on: Decide,
        stage: StageView<'_, Self>,
    ) -> Action<PlaneAxisGains> {
        procedure.next(on, stage.target.gains(stage.config))
    }

    fn take_verdict(_procedure: &mut PlaneProcedure) -> Option<Verdict<PlaneAxisGains>> {
        None
    }

    fn validate_envelope(envelope: &PlaneEnvelope) -> Result<(), InvalidConfig> {
        envelope.validate()
    }

    fn check_settle(envelope: &PlaneEnvelope, settle: &Settle) -> Result<(), InvalidConfig> {
        envelope.check_settle(settle)
    }

    fn waits(envelope: &PlaneEnvelope) -> Waits {
        Waits {
            settle: envelope.settle,
            settle_timeout_s: envelope.settle_timeout_s,
            recovery_s: envelope.recovery_s,
            max_stale_s: envelope.max_stale_s,
        }
    }

    fn breach(
        envelope: &PlaneEnvelope,
        motion: &Motion,
        start_altitude_m: f32,
    ) -> Option<AutotuneAbort> {
        envelope.breach(motion, start_altitude_m)
    }

    /// Raw: the plane's rate loop flies the unfiltered rates.
    fn filter_rates(
        _filters: &mut RateFilters,
        _config: &PlaneStabilizerConfig,
        rates_radps: Vec3,
        _dt: f32,
    ) -> Vec3 {
        rates_radps
    }

    /// An attitude setpoint's bank and pitch (a NaN axis is level, as the stabilizer reads
    /// it), a blend's plane side's, and level for anything else.
    fn attitude_target(setpoint: &Setpoint) -> Vec2 {
        let plane = match setpoint {
            Setpoint::Plane(plane) => plane,
            Setpoint::Blend(blend) => &blend.plane,
            _ => return Vec2::zeros(),
        };
        Vec2::new(zero_if_nan(plane.roll_rad), zero_if_nan(plane.pitch_rad))
    }

    /// Both axes near the held attitude and still, and the airspeed steady near trim
    /// ([`PlaneEnvelope::settle_airspeed`]): a wing moves over the ground, so its ground speed
    /// doesn't count.
    fn settles(
        envelope: &PlaneEnvelope,
        settle: &Settle,
        motion: &Motion,
        _target: PlaneLoop,
    ) -> bool {
        (0..2).all(|axis| steady_axis(settle, motion, axis))
            && envelope.settle_airspeed.holds(motion)
    }

    /// The loop's own axis near the held attitude and still.
    fn holds(settle: &Settle, motion: &Motion, target: PlaneLoop) -> bool {
        steady_axis(settle, motion, target.axis())
    }
}

/// Whether `motion` is within `settle` of the held attitude about `axis` (0 roll, 1 pitch).
fn steady_axis(settle: &Settle, motion: &Motion, axis: usize) -> bool {
    motion.axis_tilt_error_rad[axis] < settle.tilt_rad
        && motion.axis_rate_radps[axis] < settle.rate_radps
}

#[cfg(test)]
mod tests;
