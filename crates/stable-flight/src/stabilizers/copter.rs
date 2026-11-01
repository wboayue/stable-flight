//! Basic controller for a vehicle that does not transition between different flight modes such as copter and plane.
//!
//! A reference model for such a vehicle is the QuadCopter X.
//! Reference model: <https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX>
//!
//! # Control Architecture
//!
//! This implements a modular cascaded control system with configurable control loops:
//!
//! 1. **Inner Loop: Rate Control**
//!    - Controls angular rates (roll rate, pitch rate, yaw rate)
//!    - A PID per axis, no feedforward; D on the measured rate after the gyro low-pass
//!      (`STAB_GYRO_FLT`) and the optional notch
//!    - Directly commands motor mixing algorithm
//!    - Always active as the core control layer
//!
//! 2. **Middle Loop: Attitude Control (Optional)**
//!    - Controls orientation angles (roll angle, pitch angle, yaw angle)
//!    - Outputs desired angular rates to inner loop
//!    - Implements angle limiting for safety
//!    - Can be used without outer position loops for manual flight modes
//!
//! Both loops run every tick; each loop's form is in
//! [`controllers`](crate::controllers#loop-shapes).
//!
//! The outer position loops (altitude and lateral position) are guidance, not stabilization:
//! they live in [`CopterGuidance`](crate::guidance::copter::CopterGuidance), which turns
//! commands into the [`Setpoint`] this stabilizer tracks.
//!
//! # Setpoints
//!
//! - [`Setpoint::Rate`]: rate loop only (acro), at stick fractions of
//!   [`full_scale_rates_radps`](CopterStabilizerConfig::full_scale_rates_radps); the idle
//!   attitude and heading loops are cleared, so attitude flight resumes from fresh state.
//! - [`Setpoint::BodyRate`]: the same, at absolute rates saturated to the full-scale rates
//!   (maneuvers, offboard rates).
//! - [`Setpoint::Plane`], [`Setpoint::PlaneRate`]: a plane stabilizer's; [`Setpoint::Blend`]: a
//!   lift+cruise VTOL's; neutral output.
//! - [`Setpoint::Attitude`]: attitude and heading loops feeding the rate loop. They act on the
//!   body-frame error from the attitude quaternion: the shortest tilt of the thrust axis onto
//!   the setpoint's (roll and pitch relative to the current heading), then the heading error,
//!   weighted by the cosine of the tilt (PX4 `mc_att_control`, reduced attitude). Near level
//!   that is the per-axis Euler difference; far over (a tailsitter's view in transition) it
//!   stays defined and gives body rates. Roll and pitch saturate to
//!   [`max_angles_rad`](AttitudeControllerConfig::max_angles_rad), pitch to ±π/2 for a
//!   [`transition`](crate::AttitudeSetpoint::transition) ramp.
//!
//! An [`idle`](Setpoint::idle) setpoint (guidance's ground idle) clears every loop's integral
//! after the tick: on the ground there's nothing for the I terms to hold, and they'd wind up
//! against the resting attitude. Zero thrust alone doesn't: in the air it is the altitude loop
//! shedding a climb, and the I terms hold the trim.
//!
//! # Tuning
//!
//! An autotune stage ([`start_probe`](crate::tuning::Tunable::start_probe)) hands one loop to a
//! [`Probe`](crate::tuning::Probe). Every controller still runs each tick; before
//! the loop's controller runs, the probe sees its
//! [`LoopSignals`](crate::tuning::LoopSignals) and replaces the loop's setpoint (a step, or an
//! identification's square wave).
//!
//! | [`TunedLoop`] | Setpoint, error |
//! |---|---|
//! | `RollRate`, `PitchRate`, `YawRate` | desired body rate; desired − measured |
//! | `Roll`, `Pitch` | saturated desired angle; that − estimated angle |
//! | `Heading` | desired heading; wrapped heading error |
//!
//! The signals' `saturated` is the last tick's clip on the loop's path: the axis's moment, and
//! for an angle or heading loop also its rate setpoint.
//!
//! - **Rate stages** replace the rate setpoint the angle loop sets, so the angle loop around
//!   the tested axis has no say while a step runs, and its integral holds: the step tilts or
//!   turns the vehicle away from that loop's setpoint, an error the loop isn't flying.
//!   An identification adds its square wave to that setpoint instead, so the angle loop still
//!   holds the attitude and integrates as usual.
//! - **Handoff:** when the probe finishes, on [`stop_probe`](crate::tuning::Tunable::stop_probe),
//!   or when a [`Setpoint::Rate`] tick idles the angle or heading loop under test
//!   ([`ProbeStatus::Interrupted`](crate::tuning::ProbeStatus::Interrupted)), the loop
//!   restarts with no derivative history and keeps its I term, which a retune with the tuned
//!   gains rescales like any other (`architecture.md#config-ownership`). On an interrupt the
//!   rate tick then clears the idle angle loops, so their I terms are dropped. An angle
//!   stage's identification runs on the rate loop under it, so a rate tick doesn't interrupt
//!   it.
//! - The stage is runtime state, not config; [`prime`](crate::Stabilizer::prime) ends it.

use crate::{Vec2, Vec3};

use crate::{
    AttitudeSetpoint, Configurable, ControlOutput, Saturation, Setpoint, Stabilizer, VehicleState,
};

use crate::controllers::gyro::GyroFilter;
use crate::controllers::*;
use crate::controllers::{GyroFilters, NotchConfig};
use crate::controllers::{check_limit, check_positive};
use crate::math::{
    assert_dt, attitude_error, pitch_limit_rad, saturate_rates, saturate_unit, thrust_curve,
};

mod params;
#[cfg(test)]
mod tests;
mod tuning;

pub use tuning::TunedLoop;

/// Configuration for the [CopterStabilizer]: gains and limits for the inner control loops and
/// the thrust mapping.
///
/// Plain data: [`CopterStabilizer::new`] validates it, and the stabilizer passes it to its
/// controllers every tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CopterStabilizerConfig {
    /// Body-rate (inner) loop.
    pub body_rate: BodyRateControllerConfig,
    /// Roll/pitch attitude loop ([`Setpoint::Attitude`] only).
    pub attitude: AttitudeControllerConfig,
    /// Heading-hold loop ([`Setpoint::Attitude`] only).
    pub yaw: YawControllerConfig,
    /// Body rates (roll, pitch, yaw) in rad/s that a full-scale [`RateSetpoint`](crate::RateSetpoint) maps to. Each
    /// component must be non-negative and finite: an infinite or NaN scale makes even a neutral
    /// stick a non-finite rate, which zeroes that axis's moment.
    pub full_scale_rates_radps: Vec3,
    /// Exponent of the thrust curve applied to the linear thrust setpoint. Must be positive
    /// and finite: a negative exponent maps zero throttle to full thrust.
    pub thrust_curve_exponent: f32,
    /// Cutoff of the low-pass on the body rates the rate loop flies, in Hz; 0 turns it off,
    /// and so does one at or past Nyquist. Non-negative and finite. Lower: less gyro noise
    /// and vibration in the moments, more lag in the loop (about 5.6 ms at 40 Hz). The
    /// default is PX4's `IMU_GYRO_CUTOFF`; on the sim's 0.1 rad/s vibration at 90 Hz, 40 Hz
    /// passed the most robustness rows, 20 Hz (ArduPilot's `INS_GYRO_FILTER`) and 50 Hz and up
    /// lost clean rows (#373).
    pub gyro_cutoff_hz: f32,
    /// A notch on the roll and pitch rates before the low-pass, for one vibration tone (a
    /// motor's); off by default. The autotune's design models it with the low-pass. On the
    /// sim's 0.1 rad/s at 90 Hz under `racer`'s 75 Hz low-pass, a notch at the tone (90 Hz,
    /// 45 Hz wide, 40 dB) passed the robustness noise row, and flown on every row it changed
    /// none. The yaw rate stays unnotched, though ArduPilot and PX4 notch all three axes: with
    /// yaw notched too, `quad_x`'s yaw search overshot the step spec on one noise seed of five
    /// from both starts (peaks 1.052 and 1.066 against 1.05), which roll and pitch alone
    /// pass (#597).
    pub gyro_notch: NotchConfig,
}

/// [`CopterStabilizerConfig::gyro_cutoff_hz`]'s default, in Hz.
pub const DEFAULT_GYRO_CUTOFF_HZ: f32 = 40.0;

impl Default for CopterStabilizerConfig {
    /// Untuned starting values; airframes override them with flight-checked sets.
    fn default() -> Self {
        CopterStabilizerConfig {
            body_rate: BodyRateControllerConfig::default(),
            attitude: AttitudeControllerConfig::default(),
            yaw: YawControllerConfig::default(),
            full_scale_rates_radps: Vec3::new(3.0, 3.0, 3.0),
            thrust_curve_exponent: 2.0,
            gyro_cutoff_hz: DEFAULT_GYRO_CUTOFF_HZ,
            gyro_notch: NotchConfig::default(),
        }
    }
}

impl CopterStabilizerConfig {
    /// Checks every controller config ([`BodyRateControllerConfig::validate`],
    /// [`AttitudeControllerConfig::validate`], [`YawControllerConfig::validate`]; errors carry
    /// the loop as their `section`), that each `full_scale_rates_radps` component is
    /// non-negative and finite, that `thrust_curve_exponent` is positive and finite, and that
    /// `gyro_cutoff_hz` is non-negative and finite.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::CopterStabilizerConfig;
    ///
    /// let mut config = CopterStabilizerConfig::default();
    /// assert!(config.validate().is_ok());
    /// config.yaw.max_rate_radps = -1.0;
    /// assert!(config.validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        self.body_rate
            .validate()
            .map_err(|e| e.in_section("body_rate"))?;
        self.attitude
            .validate()
            .map_err(|e| e.in_section("attitude"))?;
        self.yaw.validate().map_err(|e| e.in_section("yaw"))?;
        let scales = &self.full_scale_rates_radps;
        check_limit("full_scale_rates_radps.x", scales.x)?;
        check_limit("full_scale_rates_radps.y", scales.y)?;
        check_limit("full_scale_rates_radps.z", scales.z)?;
        check_positive("thrust_curve_exponent", self.thrust_curve_exponent)?;
        check_limit("gyro_cutoff_hz", self.gyro_cutoff_hz)?;
        self.gyro_notch
            .validate()
            .map_err(|e| e.in_section("gyro_notch"))
    }

    /// The gyro filters body `axis` (0 roll, 1 pitch, 2 yaw) flies through: on roll and
    /// pitch, [`gyro_notch`](Self::gyro_notch) then the low-pass at
    /// [`gyro_cutoff_hz`](Self::gyro_cutoff_hz); on yaw, the low-pass alone.
    pub fn gyro_filters_on(&self, axis: usize) -> GyroFilters {
        let notch = if axis < 2 {
            self.gyro_notch
        } else {
            NotchConfig::default()
        };
        GyroFilters {
            cutoff_hz: self.gyro_cutoff_hz,
            notch,
        }
    }
}

/// The inner loop for vehicles like quadcopters, implementing a cascaded control system.
///
/// This controller supports rate and attitude control loops, with configurable gains for
/// each loop. It tracks a [`Setpoint`] ([`RateSetpoint`](crate::RateSetpoint) or [`AttitudeSetpoint`]) and
/// generates motor control outputs ([`ControlOutput`]).
///
/// # Examples
///
/// ```
/// use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
/// use stable_flight::{AttitudeSetpoint, Setpoint, Stabilizer, VehicleState};
///
/// let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
/// let hover = Setpoint::Attitude(AttitudeSetpoint { thrust: 0.5, ..Default::default() });
///
/// let output = stabilizer.stabilize(0.004, &hover, &VehicleState::default());
/// assert_eq!(output.thrust, 0.25); // 0.5 through the default quadratic thrust curve
/// ```
pub struct CopterStabilizer {
    /// Gains and limits, passed into each controller every tick.
    config: CopterStabilizerConfig,

    body_rate: BodyRateController,
    attitude: AttitudeController,
    yaw: YawController,

    /// What the last `stabilize` call clipped.
    saturation: Saturation,

    /// The autotune stage, if one was started and not yet stopped.
    stage: Option<tuning::Stage>,

    /// What the last `stabilize` call clipped per axis, for a probe's
    /// [`LoopSignals::saturated`](crate::tuning::LoopSignals::saturated).
    clipped: Clipped,

    /// The moments the last `stabilize` call commanded (roll, pitch, yaw), for a probe's
    /// [`LoopSignals::last_moment`](crate::tuning::LoopSignals::last_moment).
    last_moments: Vec3,

    /// The low-pass per body rate (roll, pitch, yaw) at
    /// [`gyro_cutoff_hz`](CopterStabilizerConfig::gyro_cutoff_hz).
    gyro_filters: [GyroFilter; 3],
}

/// Per-axis clips (roll, pitch, yaw) of one tick.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Clipped {
    /// The rate loop's moment hit ±1.
    moments: [bool; 3],
    /// The attitude or heading loop's rate setpoint hit its `max_rate_radps`.
    rates: [bool; 3],
}

impl CopterStabilizer {
    /// Creates a new [CopterStabilizer] with the specified configuration.
    ///
    /// # Arguments
    ///
    /// * `config` - Configuration for the controller, including PID gains and thrust curve exponent.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`CopterStabilizerConfig::validate`]).
    pub fn new(config: CopterStabilizerConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid CopterStabilizerConfig: {error}");
        }
        CopterStabilizer {
            config,
            body_rate: BodyRateController::default(),
            attitude: AttitudeController::default(),
            yaw: YawController::default(),
            saturation: Saturation::default(),
            stage: None,
            clipped: Clipped::default(),
            last_moments: Vec3::zeros(),
            gyro_filters: Default::default(),
        }
    }
}

impl Configurable for CopterStabilizer {
    type Config = CopterStabilizerConfig;

    fn config(&self) -> &CopterStabilizerConfig {
        &self.config
    }

    /// Validates with [`CopterStabilizerConfig::validate`].
    fn set_config(&mut self, config: CopterStabilizerConfig) -> Result<(), InvalidConfig> {
        config.validate()?;
        let old = &self.config;
        self.attitude
            .rescale_integral(&old.attitude, &config.attitude);
        self.yaw.rescale_integral(&old.yaw, &config.yaw);
        self.config = config;
        Ok(())
    }
}

impl Stabilizer for CopterStabilizer {
    /// Tracks a setpoint and generates motor control outputs based on the current vehicle state.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update in seconds (must be positive and finite).
    /// * `setpoint` - The attitude or rate target.
    /// * `estimated_state` - The current state of the vehicle (attitude, angular rates, etc.).
    ///
    /// # Returns
    ///
    /// A [ControlOutput] containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.
    fn stabilize(
        &mut self,
        dt: f32,
        setpoint: &Setpoint,
        estimated_state: &VehicleState,
    ) -> ControlOutput {
        assert_dt(dt);
        self.saturation = Saturation::default();

        let output = match setpoint {
            Setpoint::Attitude(attitude) => self.track_attitude(dt, attitude, estimated_state),
            Setpoint::Rate(rates) => {
                // Per-component saturation: `clamp` alone passes NaN, which would zero that
                // axis's moment (no damping) instead of meaning "hold zero rate".
                let fractions = Vec3::new(
                    saturate_unit(rates.roll_rate),
                    saturate_unit(rates.pitch_rate),
                    saturate_unit(rates.yaw_rate),
                );
                let desired = fractions.component_mul(&self.config.full_scale_rates_radps);
                self.track_rates(dt, desired, rates.thrust, estimated_state)
            }
            Setpoint::BodyRate(rates) => {
                let desired = saturate_rates(
                    Vec3::new(rates.roll_radps, rates.pitch_radps, rates.yaw_radps),
                    self.config.full_scale_rates_radps,
                );
                self.track_rates(dt, desired, rates.thrust, estimated_state)
            }
            // A plane stabilizer's setpoints: neutral (the trait's rule for unsupported ones).
            Setpoint::Plane(_) | Setpoint::PlaneRate(_) | Setpoint::Blend(_) => {
                ControlOutput::default()
            }
        };

        // At ground idle the I terms can't act on anything and would wind up against a
        // resting attitude, to kick at liftoff; clear them every such tick. ArduPilot decays
        // its rate I terms at GROUND_IDLE instead (`ArduCopter/mode_stabilize.cpp:36`); which
        // to follow is #397.
        if setpoint.idle() {
            self.body_rate.clear_integral();
            self.attitude.clear_integral();
            self.yaw.clear_integral();
        }

        // The rate loop clamps each moment to [-1, 1]; landing on a bound means it clipped.
        self.saturation.moments = output.moments.map(|m| m.abs() >= 1.0).into();
        self.clipped.moments = self.saturation.moments;
        self.last_moments = output.moments;
        self.finish_stage();
        output
    }

    /// Clears the body-rate, attitude, and yaw controller state (integrals and derivative
    /// history) and the gyro filters, so control after arming starts from zero, and ends any
    /// tuning stage.
    fn prime(&mut self, _state: &VehicleState) {
        self.stage = None;
        self.clipped = Clipped::default();
        self.last_moments = Vec3::zeros();
        self.gyro_filters = Default::default();
        self.body_rate = BodyRateController::default();
        self.attitude = AttitudeController::default();
        self.yaw = YawController::default();
        self.saturation = Saturation::default();
    }

    /// `tilt` when an attitude setpoint's roll or pitch was beyond
    /// [`max_angles_rad`](AttitudeControllerConfig::max_angles_rad), `thrust` when the
    /// setpoint's thrust was outside `[0, 1]`, and `moments` per axis when the rate loop's
    /// output hit ±1.
    fn saturation(&self) -> Saturation {
        self.saturation
    }
}

impl CopterStabilizer {
    /// Tracks an [AttitudeSetpoint].
    ///
    /// Converts the desired roll, pitch, and heading into angular rates using the attitude and
    /// yaw controllers, then passes the rates to the rate controller to generate moments.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update in seconds (must be positive and finite).
    /// * `setpoint` - The desired roll, pitch, and heading (radians), and linear thrust.
    /// * `estimated_state` - The current state of the vehicle (attitude, angular rates, etc.).
    ///
    /// # Returns
    ///
    /// A `ControlOutput` containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.
    fn track_attitude(
        &mut self,
        dt: f32,
        setpoint: &AttitudeSetpoint,
        estimated_state: &VehicleState,
    ) -> ControlOutput {
        // Roll/pitch saturate inside the attitude controller, thrust in the thrust curve (see
        // `crate::math`).
        let max_angles_rad = self.max_angles_rad(setpoint);
        // `>` is false for NaN, which saturates to level.
        self.saturation.tilt = setpoint.roll_rad.abs() > max_angles_rad.x
            || setpoint.pitch_rad.abs() > max_angles_rad.y;
        let attitude = &estimated_state.attitude;
        let saturate_tilt = |roll_rad: f32, pitch_rad: f32, heading_rad: f32| {
            let tilt = AttitudeController::saturate_setpoint(
                max_angles_rad,
                Vec2::new(roll_rad, pitch_rad),
            );
            Vec3::new(tilt.x, tilt.y, heading_rad)
        };

        // Roll, pitch, and heading setpoints; a probe on one of these loops may replace one.
        // A NaN heading gives a NaN yaw error, which the yaw loop holds on (no yaw rate).
        let mut desired =
            saturate_tilt(setpoint.roll_rad, setpoint.pitch_rad, setpoint.heading_rad);
        let error_for = |angles: &Vec3| attitude_error(attitude, angles.x, angles.y, angles.z);
        let mut error = error_for(&desired);
        let probed = desired;
        // The raw rates: the identify fit models the airframe, without the gyro filter's lag.
        self.probe_attitude(
            dt,
            &mut desired,
            &error,
            &estimated_state.angular_velocity_radps,
        );
        // `!=` is true for a NaN heading too; recomputing is harmless.
        if desired != probed {
            desired = saturate_tilt(desired.x, desired.y, desired.z);
            error = error_for(&desired);
        }

        // The loops act on the body-frame rotation to the setpoint, so their outputs are body
        // rates at any attitude (a tailsitter's view pitches far over in transition).
        let [hold_roll, hold_pitch, hold_heading] = self.held_integrals();
        let config = &self.config;
        let desired_rates =
            self.attitude
                .control_error(&config.attitude, dt, error.xy(), [hold_roll, hold_pitch]);
        let desired_yaw_rate = self
            .yaw
            .control_error(&config.yaw, dt, error.z, hold_heading);
        // The controllers clamp their rate outputs; landing on a bound means they clipped.
        let max_rates = config.attitude.max_rate_radps;
        self.clipped.rates = [
            desired_rates.x.abs() >= max_rates.x,
            desired_rates.y.abs() >= max_rates.y,
            desired_yaw_rate.abs() >= config.yaw.max_rate_radps,
        ];

        let desired_rates = Vec3::new(desired_rates.x, desired_rates.y, desired_yaw_rate);
        let moments = self.track_body_rates(dt, desired_rates, estimated_state);

        ControlOutput {
            thrust: self.fit_to_thrust_curve(setpoint.thrust),
            moments,
            cruise: None,
        }
    }

    /// The roll and pitch limits for `setpoint`: `max_angles_rad`, with pitch lifted to ±π/2
    /// for a transition ramp.
    fn max_angles_rad(&self, setpoint: &AttitudeSetpoint) -> Vec2 {
        let limits_rad = self.config.attitude.max_angles_rad;
        Vec2::new(
            limits_rad.x,
            pitch_limit_rad(limits_rad.y, setpoint.transition),
        )
    }

    /// Tracks body rates in rad/s with linear `thrust`, bypassing the attitude controller.
    fn track_rates(
        &mut self,
        dt: f32,
        desired_rates_radps: Vec3,
        thrust: f32,
        estimated_state: &VehicleState,
    ) -> ControlOutput {
        // The attitude and heading loops are idle: a stage on one of them ends, and they are
        // cleared so a switch back to attitude flight carries no stale integral or derivative
        // history.
        self.interrupt_attitude_stage();
        self.attitude = AttitudeController::default();
        self.yaw = YawController::default();
        self.clipped.rates = [false; 3];

        let moments = self.track_body_rates(dt, desired_rates_radps, estimated_state);
        ControlOutput {
            thrust: self.fit_to_thrust_curve(thrust),
            moments,
            cruise: None,
        }
    }

    /// Runs the body-rate loop toward `desired_rates_radps` on the gyro-filtered rates; a stage
    /// on a rate loop replaces that axis's rate setpoint.
    fn track_body_rates(
        &mut self,
        dt: f32,
        desired_rates_radps: Vec3,
        estimated_state: &VehicleState,
    ) -> Vec3 {
        let current_rates = self.filter_rates(dt, estimated_state.angular_velocity_radps);
        let mut desired_rates_radps = desired_rates_radps;
        self.probe_rates(
            dt,
            &mut desired_rates_radps,
            &current_rates,
            &estimated_state.angular_velocity_radps,
        );
        self.body_rate.control(
            &self.config.body_rate,
            dt,
            desired_rates_radps,
            current_rates,
        )
    }

    /// `rates_radps` (body frame, roll, pitch, yaw) through each axis's gyro filters
    /// ([`CopterStabilizerConfig::gyro_filters_on`]).
    fn filter_rates(&mut self, dt: f32, rates_radps: Vec3) -> Vec3 {
        let config = self.config;
        Vec3::from_fn(|axis, _| {
            let filters = config.gyro_filters_on(axis);
            self.gyro_filters[axis].apply(rates_radps[axis], dt, &filters)
        })
    }

    /// Applies a nonlinear thrust curve to a linear thrust command.
    ///
    /// Maps a linear thrust value (0.0 to 1.0) to a nonlinear output using the configured thrust curve exponent.
    /// This compensates for nonlinear motor responses. Out-of-range input saturates (flagging
    /// `thrust`) and NaN becomes 0 (motors off), per the input policy.
    ///
    /// # Arguments
    ///
    /// * `linear_thrust` - Linear thrust command (0.0 to 1.0).
    ///
    /// # Returns
    ///
    /// Nonlinear thrust command (0.0 to 1.0).
    fn fit_to_thrust_curve(&mut self, linear_thrust: f32) -> f32 {
        self.saturation.thrust = !(0.0..=1.0).contains(&linear_thrust) && !linear_thrust.is_nan();
        thrust_curve(linear_thrust, self.config.thrust_curve_exponent)
    }
}
