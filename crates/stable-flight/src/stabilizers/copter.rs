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
//! - [`Setpoint::Rate`]: rate loop only (acro).
//! - [`Setpoint::Attitude`]: attitude and heading loops feeding the rate loop.

use crate::{Vec2, Vec3};

use crate::{
    AttitudeSetpoint, Configurable, ControlOutput, RateSetpoint, Setpoint, Stabilizer, VehicleState,
};

use crate::controllers::*;
use crate::controllers::{check_limit, check_positive};
use crate::math::{euler_angles, saturate_throttle, saturate_unit};

#[cfg(test)]
mod tests;

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
    /// stick a non-finite rate, which zeroes every moment.
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
        self.config = config;
        Ok(())
    }
}

impl Stabilizer for CopterStabilizer {
    /// Tracks a setpoint and generates motor control outputs based on the current vehicle state.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update in seconds (must be positive).
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
        assert!(dt > 0.0, "Time step must be positive");

        match setpoint {
            Setpoint::Attitude(attitude) => self.track_attitude(dt, attitude, estimated_state),
            Setpoint::Rate(rates) => self.track_rates(dt, rates, estimated_state),
        }
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
    /// * `dt` - Time step since the last update in seconds (must be positive).
    /// * `setpoint` - The desired roll, pitch (normalized), heading (radians), and linear thrust.
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
        let desired_angles = Vec2::new(setpoint.roll, setpoint.pitch);
        // The quaternion is the canonical attitude; `euler_angles` is NaN-safe.
        let (roll, pitch, current_heading) = euler_angles(&estimated_state.attitude);
        let current_angles = Vec2::new(roll, pitch);

        let config = &self.config;
        let desired_rates =
            self.attitude
                .control(&config.attitude, dt, desired_angles, current_angles);

        let desired_yaw_rate =
            self.yaw
                .control(&config.yaw, dt, setpoint.heading_rad, current_heading);

        let current_rates = Vec3::new(
            estimated_state.angular_velocity_radps.x,
            estimated_state.angular_velocity_radps.y,
            estimated_state.angular_velocity_radps.z,
        );

        let desired_rates = Vec3::new(desired_rates.x, desired_rates.y, desired_yaw_rate);

        let moments = self
            .body_rate
            .control(&config.body_rate, dt, desired_rates, current_rates);

        ControlOutput {
            thrust: self.fit_to_thrust_curve(setpoint.thrust),
            moments,
        }
    }

    /// Tracks a [RateSetpoint], bypassing the attitude controller.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update in seconds (must be positive).
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
        // Per-component saturation: `clamp` alone passes NaN, which would zero every moment
        // (no damping) instead of meaning "hold zero rate".
        let desired_rates = Vec3::new(
            saturate_unit(setpoint.roll_rate),
            saturate_unit(setpoint.pitch_rate),
            saturate_unit(setpoint.yaw_rate),
        )
        .component_mul(&self.config.full_scale_rates_radps);

        let current_rates = Vec3::new(
            estimated_state.angular_velocity_radps.x,
            estimated_state.angular_velocity_radps.y,
            estimated_state.angular_velocity_radps.z,
        );

        let moments =
            self.body_rate
                .control(&self.config.body_rate, dt, desired_rates, current_rates);

        ControlOutput {
            thrust: self.fit_to_thrust_curve(setpoint.thrust),
            moments,
        }
    }

    /// Applies a nonlinear thrust curve to a linear thrust command.
    ///
    /// Maps a linear thrust value (0.0 to 1.0) to a nonlinear output using the configured thrust curve exponent.
    /// This compensates for nonlinear motor responses. Out-of-range input saturates and NaN
    /// becomes 0 (motors off), per the input policy.
    ///
    /// # Arguments
    ///
    /// * `linear_thrust` - Linear thrust command (0.0 to 1.0).
    ///
    /// # Returns
    ///
    /// Nonlinear thrust command (0.0 to 1.0).
    fn fit_to_thrust_curve(&self, linear_thrust: f32) -> f32 {
        let nonlinear_thrust = libm::powf(
            saturate_throttle(linear_thrust),
            self.config.thrust_curve_exponent,
        );
        nonlinear_thrust.clamp(0.0, 1.0)
    }
}
