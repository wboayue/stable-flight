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
//!    - Fast update frequency (typically 400-1000Hz)
//!    - PID controllers with feed-forward terms for each axis
//!    - Directly commands motor mixing algorithm
//!    - Always active as the core control layer
//!
//! 2. **Middle Loop: Attitude Control (Optional)**
//!    - Controls orientation angles (roll angle, pitch angle, yaw angle)
//!    - Moderate update frequency (typically 100-250Hz)
//!    - Outputs desired angular rates to inner loop
//!    - Implements angle limiting for safety
//!    - Can be used without outer position loops for manual flight modes
//!
//! The outer position loops (altitude and lateral position) are guidance, not stabilization:
//! they live in [`CopterGuidance`](crate::guidance::copter::CopterGuidance), which turns
//! commands into the [`Setpoint`] this stabilizer tracks.
//!
//! # Setpoints
//!
//! - [`Setpoint::Rate`]: rate loop only (acro); the idle attitude and heading loops are
//!   cleared, so attitude flight resumes from fresh state.
//! - [`Setpoint::Attitude`]: attitude and heading loops feeding the rate loop.
//!
//! # Tuning
//!
//! An autotune stage ([`start_probe`](crate::tuning::Tunable::start_probe)) hands one loop to a
//! [`Probe`](crate::tuning::Probe) (#149), today a relay. Every controller still runs each
//! tick; the stage replaces one value:
//!
//! | [`TunedLoop`] | Probe error | Replaces |
//! |---|---|---|
//! | `RollRate`, `PitchRate`, `YawRate` | desired − measured body rate | that axis's moment (±1) |
//! | `Roll`, `Pitch` | saturated desired − estimated angle | that axis's rate setpoint (`max_rate_radps`) |
//! | `Heading` | wrapped heading error | the yaw rate setpoint (`yaw.max_rate_radps`) |
//!
//! - **Rate stages** keep the angle loop around the tested axis, which keeps it near level, so
//!   the relay measures the rate loop with the angle loop's `(s + Kp)/s` wrapped around it: a
//!   few degrees of phase at the rate loop's ultimate frequency.
//! - **Handoff:** when the probe finishes, on [`stop_probe`](crate::tuning::Tunable::stop_probe),
//!   or when a [`Setpoint::Rate`] tick idles the angle or heading loop under test
//!   ([`ProbeStatus::Interrupted`](crate::tuning::ProbeStatus::Interrupted)), the loop
//!   restarts with the probe's trim (the relay's bias) as its I term and no derivative
//!   history: the measured trim carries over, and a retune with the tuned gains
//!   rescales it like any other (`architecture.md#config-ownership`). On an interrupt the rate
//!   tick then clears the idle angle loops (#70), so that trim is dropped.
//! - The stage is runtime state, not config; [`prime`](crate::Stabilizer::prime) ends it.

use crate::{Vec2, Vec3};

use crate::{
    AttitudeSetpoint, Configurable, ControlOutput, RateSetpoint, Saturation, Setpoint, Stabilizer,
    VehicleState,
};

use crate::controllers::yaw::angle_diff;
use crate::controllers::*;
use crate::controllers::{check_limit, check_positive};
use crate::math::{assert_dt, euler_angles, saturate_symmetric, saturate_throttle, saturate_unit};

mod params;
#[cfg(test)]
mod tests;
mod tuning;

use tuning::Location;
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
    /// Body rates (roll, pitch, yaw) in rad/s that a full-scale [`RateSetpoint`] maps to. Each
    /// component must be non-negative and finite: an infinite or NaN scale makes even a neutral
    /// stick a non-finite rate, which zeroes that axis's moment.
    pub full_scale_rates_radps: Vec3,
    /// Exponent of the thrust curve applied to the linear thrust setpoint. Must be positive
    /// and finite: a negative exponent maps zero throttle to full thrust.
    pub thrust_curve_exponent: f32,
}

impl Default for CopterStabilizerConfig {
    /// Untuned starting values; airframes override them with flight-checked sets.
    fn default() -> Self {
        CopterStabilizerConfig {
            body_rate: BodyRateControllerConfig::default(),
            attitude: AttitudeControllerConfig::default(),
            yaw: YawControllerConfig::default(),
            full_scale_rates_radps: Vec3::new(3.0, 3.0, 3.0),
            thrust_curve_exponent: 2.0,
        }
    }
}

impl CopterStabilizerConfig {
    /// Checks every controller config ([`BodyRateControllerConfig::validate`],
    /// [`AttitudeControllerConfig::validate`], [`YawControllerConfig::validate`]; errors carry
    /// the loop as their `section`), that each `full_scale_rates_radps` component is
    /// non-negative and finite, and that `thrust_curve_exponent` is positive and finite.
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
        check_positive("thrust_curve_exponent", self.thrust_curve_exponent)
    }
}

/// The inner loop for vehicles like quadcopters, implementing a cascaded control system.
///
/// This controller supports rate and attitude control loops, with configurable gains for
/// each loop. It tracks a [`Setpoint`] ([`RateSetpoint`] or [`AttitudeSetpoint`]) and
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
        self.body_rate
            .rescale_integral(&old.body_rate, &config.body_rate);
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
                // The attitude and heading loops are idle: a stage on one of them ends, and
                // they are cleared so a switch back to attitude flight carries no stale
                // integral or derivative history (#70).
                self.interrupt_attitude_stage();
                self.attitude = AttitudeController::default();
                self.yaw = YawController::default();
                self.track_rates(dt, rates, estimated_state)
            }
        };

        // The rate loop clamps each moment to [-1, 1]; landing on a bound means it clipped.
        self.saturation.moments = output.moments.map(|m| m.abs() >= 1.0).into();
        output
    }

    /// Clears the body-rate, attitude, and yaw controller state (integrals and derivative
    /// history), so control after arming starts from zero, and ends any tuning stage.
    fn prime(&mut self, _state: &VehicleState) {
        self.stage = None;
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
        let desired_angles_rad = Vec2::new(setpoint.roll_rad, setpoint.pitch_rad);
        let max_angles_rad = self.config.attitude.max_angles_rad;
        // `>` is false for NaN, which the attitude controller treats as level.
        self.saturation.tilt = desired_angles_rad.x.abs() > max_angles_rad.x
            || desired_angles_rad.y.abs() > max_angles_rad.y;
        // The quaternion is the canonical attitude; `euler_angles` is NaN-safe.
        let (roll, pitch, current_heading) = euler_angles(&estimated_state.attitude);
        let current_angles = Vec2::new(roll, pitch);

        let config = &self.config;
        let mut desired_rates =
            self.attitude
                .control(&config.attitude, dt, desired_angles_rad, current_angles);

        let mut desired_yaw_rate =
            self.yaw
                .control(&config.yaw, dt, setpoint.heading_rad, current_heading);

        // A stage on an angle or heading loop replaces that loop's rate setpoint.
        match self.running_stage() {
            Some(Location::Angle(axis)) => {
                let attitude = config.attitude;
                let error =
                    AttitudeController::angle_error(&attitude, desired_angles_rad, current_angles);
                let rate = self.step_stage(dt, error[axis]);
                desired_rates[axis] = saturate_symmetric(rate, attitude.max_rate_radps[axis]);
            }
            Some(Location::Heading) => {
                let max_rate_radps = config.yaw.max_rate_radps;
                let error = angle_diff(setpoint.heading_rad, current_heading);
                desired_yaw_rate = saturate_symmetric(self.step_stage(dt, error), max_rate_radps);
            }
            Some(Location::Rate(_)) | None => {}
        }

        let desired_rates = Vec3::new(desired_rates.x, desired_rates.y, desired_yaw_rate);
        let moments = self.track_body_rates(dt, desired_rates, estimated_state);

        ControlOutput {
            thrust: self.fit_to_thrust_curve(setpoint.thrust),
            moments,
        }
    }

    /// Tracks a [RateSetpoint], bypassing the attitude controller.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update in seconds (must be positive and finite).
    /// * `setpoint` - The desired roll, pitch, yaw rates (normalized) and linear thrust.
    /// * `estimated_state` - The current state of the vehicle (angular rates, etc.).
    ///
    /// # Returns
    ///
    /// A `ControlOutput` containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.
    fn track_rates(
        &mut self,
        dt: f32,
        setpoint: &RateSetpoint,
        estimated_state: &VehicleState,
    ) -> ControlOutput {
        // Per-component saturation: `clamp` alone passes NaN, which would zero that axis's
        // moment (no damping) instead of meaning "hold zero rate".
        let desired_rates = Vec3::new(
            saturate_unit(setpoint.roll_rate),
            saturate_unit(setpoint.pitch_rate),
            saturate_unit(setpoint.yaw_rate),
        )
        .component_mul(&self.config.full_scale_rates_radps);

        let moments = self.track_body_rates(dt, desired_rates, estimated_state);

        ControlOutput {
            thrust: self.fit_to_thrust_curve(setpoint.thrust),
            moments,
        }
    }

    /// Runs the body-rate loop toward `desired_rates_radps`; a stage on a rate loop replaces
    /// that axis's moment.
    fn track_body_rates(
        &mut self,
        dt: f32,
        desired_rates_radps: Vec3,
        estimated_state: &VehicleState,
    ) -> Vec3 {
        let current_rates = estimated_state.angular_velocity_radps;
        let mut moments = self.body_rate.control(
            &self.config.body_rate,
            dt,
            desired_rates_radps,
            current_rates,
        );
        if let Some(Location::Rate(axis)) = self.running_stage() {
            let error = desired_rates_radps[axis] - current_rates[axis];
            moments[axis] = saturate_unit(self.step_stage(dt, error));
        }
        moments
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
        let nonlinear_thrust = libm::powf(
            saturate_throttle(linear_thrust),
            self.config.thrust_curve_exponent,
        );
        nonlinear_thrust.clamp(0.0, 1.0)
    }
}
