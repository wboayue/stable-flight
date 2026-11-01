//! The plane side of the supervisor: a plane's roll and pitch loops ([`PlaneSide`]), tuned by
//! a [`PlaneMethod`] within a [`PlaneEnvelope`], as a [`PlaneAutotune`].

use super::procedure::{Action, PlaneProcedure, Verdict};
use super::side::{Decide, Motion, RateFilters, Side, SideConfig, StageView, Timing, zero_if_nan};
use super::{AutotuneAbort, Settle, StagePlan, StepTestPlan, Supervisor};
use crate::controllers::{InvalidConfig, check, check_limit, check_positive};
use crate::math::DEG;
use crate::stabilizers::plane::{PlaneAxisGains, PlaneLoop, PlaneStabilizerConfig};
use crate::stabilizers::vtol::{QuadPlaneStabilizerConfig, TailsitterStabilizerConfig};
use crate::tuning::{ProbeConfig, StepConfig};
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
}

/// A plane rate step: 8°/s held for 1 s. A held rate turns the aircraft away from the attitude
/// its loops hold, so the step stays small: 8° of rotation by the end (#533).
pub const PLANE_RATE_STEP: StepConfig = StepConfig {
    amplitude: 8.0 * DEG,
    duration_s: 1.0,
    settle_band: 0.1,
    filter_hz: 0.0,
};

/// A plane roll step: 30°, a turn's bank, held 4 s: the sim's plane spec flies the same
/// (PX4's `FW_R_TC` sets its rise).
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
/// copter's (2.5°, [`COPTER_STEP_SETTLE`](super::COPTER_STEP_SETTLE)).
pub const PLANE_STEP_SETTLE: Settle = Settle {
    tilt_rad: 3.0 * DEG,
    rate_radps: 0.1,
    hold_s: 1.0,
};

/// How long a plane step may clip and still count, in s: 10 ticks at 250 Hz, as a copter's
/// ([`COPTER_MAX_SATURATED_S`](super::COPTER_MAX_SATURATED_S)). The sim's planes don't clip
/// on [`PLANE_ROLL_STEP`] or [`PLANE_PITCH_STEP`] (#310).
pub const PLANE_MAX_SATURATED_S: f32 = 0.042;

impl StepTestPlan {
    /// The plane defaults for `target`: three pairs of [`PLANE_RATE_STEP`] (roll and pitch
    /// rate), [`PLANE_ROLL_STEP`] (roll), or [`PLANE_PITCH_STEP`] (pitch), from
    /// [`PLANE_STEP_SETTLE`], each clipping for up to [`PLANE_MAX_SATURATED_S`], allowing 3
    /// clipped and 10 stalled steps in a row.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::plane::PlaneLoop;
    /// use stable_flight::tuning::autotune::{PLANE_PITCH_STEP, StepTestPlan};
    ///
    /// assert_eq!(StepTestPlan::plane(PlaneLoop::Pitch).step, PLANE_PITCH_STEP);
    /// ```
    pub fn plane(target: PlaneLoop) -> Self {
        let step = match target {
            PlaneLoop::RollRate | PlaneLoop::PitchRate => PLANE_RATE_STEP,
            PlaneLoop::Roll => PLANE_ROLL_STEP,
            PlaneLoop::Pitch => PLANE_PITCH_STEP,
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
    /// A step test of `target` with the plane defaults ([`StepTestPlan::plane`]).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::plane::{PlaneLoop, PlaneStabilizerConfig};
    /// use stable_flight::tuning::autotune::{PlaneAutotune, PlaneEnvelope, StagePlan};
    ///
    /// let plan = [PlaneLoop::Roll, PlaneLoop::Pitch].map(StagePlan::plane_step_test);
    /// let envelope = PlaneEnvelope::for_plane(&PlaneStabilizerConfig::default());
    /// assert!(PlaneAutotune::new(&plan, envelope).is_ok());
    /// ```
    pub fn plane_step_test(target: PlaneLoop) -> Self {
        Self {
            target,
            method: PlaneMethod::StepTest(StepTestPlan::plane(target)),
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
}

impl PlaneEnvelope {
    /// Limits for a plane flying `config`: 60° of bank and 30° of pitch (a 30° roll step and
    /// a 10° pitch step from the hold, with room to overshoot), 4 rad/s, 40 m of sink, and
    /// airspeed between 0.9 × `config`'s minimum and its maximum; settle as
    /// [`PLANE_STEP_SETTLE`], waiting up to 20 s, 2 s to recover, 2 s without a fresh state.
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
            ProbeConfig::Identify(_) => target.inner().unwrap_or(target),
            ProbeConfig::Step(_) => target,
        }
    }

    fn validate_method(method: &PlaneMethod) -> Result<(), InvalidConfig> {
        match method {
            PlaneMethod::StepTest(plan) => {
                plan.validate().map_err(|e| e.in_section("plan.step_test"))
            }
        }
    }

    fn method_settle(method: &PlaneMethod) -> Option<Settle> {
        match method {
            PlaneMethod::StepTest(plan) => Some(plan.settle),
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

    fn timing(envelope: &PlaneEnvelope) -> Timing {
        Timing {
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

    /// Both axes near the held attitude and still: a wing moves over the ground, so its speed
    /// doesn't count.
    fn settles(settle: &Settle, motion: &Motion, _target: PlaneLoop) -> bool {
        (0..2).all(|axis| steady_axis(settle, motion, axis))
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
