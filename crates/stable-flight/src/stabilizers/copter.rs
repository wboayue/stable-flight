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
    AttitudeReference, AttitudeSetpoint, ControlOutput, RateSetpoint, Setpoint, Stabilizer,
    VehicleState,
};

use crate::controllers::*;
use crate::math::{euler_angles, saturate_throttle, saturate_unit};

#[cfg(test)]
mod tests;

/// Configuration for the [CopterStabilizer], including gains for the inner control loops and
/// thrust mapping.
#[derive(Clone, Debug, PartialEq)]
pub struct CopterStabilizerConfig {
    pub rates_controller: BodyRateController,
    pub attitude_controller: AttitudeController,

    pub yaw_gains: Gains,

    pub thrust_curve_exponent: f32,
    // Maximum angular rates (roll, pitch, yaw) in rad/s.
    //    pub max_rates: Vec3<f32>,
}

impl Default for CopterStabilizerConfig {
    /// Creates a new `CopterStabilizerConfig` with default PID gains and thrust curve exponent.
    fn default() -> Self {
        CopterStabilizerConfig {
            rates_controller: BodyRateController::default(),
            attitude_controller: AttitudeController::default(),
            yaw_gains: Gains {
                p: 2.5,
                i: 0.0,
                d: 0.06,
            },
            thrust_curve_exponent: 2.0,
            // max_rates: Vec3 {
            //     x: 3.0, // max roll rate in rad/s (~172 deg/s)
            //     y: 3.0, // max pitch rate in rad/s (~172 deg/s)
            //     z: 3.0, // max yaw rate in rad/s (~86 deg/s)
            // },
        }
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
    pub(crate) attitude_controller: AttitudeController,
    pub(crate) rates_controller: BodyRateController,
    pub(crate) yaw_controller: YawController,

    /// Maximum angular rates (roll, pitch, yaw) in rad/s that a full-scale rate setpoint maps to.
    pub(crate) max_rates: Vec3<f32>,

    thrust_curve_exponent: f32,
}

impl CopterStabilizer {
    /// Creates a new [CopterStabilizer] with the specified configuration.
    ///
    /// # Arguments
    ///
    /// * `config` - Configuration for the controller, including PID gains and thrust curve exponent.
    pub fn new(config: CopterStabilizerConfig) -> Self {
        CopterStabilizer {
            rates_controller: config.rates_controller,
            attitude_controller: config.attitude_controller,

            yaw_controller: YawController::with_config(&YawControllerConfig {
                gains: config.yaw_gains,
                max_integral: 100.0,
                max_rate: 2.0,
                //                max_rate: config.max_rates.z,
            }),

            max_rates: Vec3::new(3.0, 3.0, 3.0), // default max rates
            thrust_curve_exponent: config.thrust_curve_exponent,
        }
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
        let (current_angles, current_heading) = match setpoint.reference {
            AttitudeReference::Euler => (
                Vec2::new(estimated_state.roll, estimated_state.pitch),
                estimated_state.heading,
            ),
            AttitudeReference::Quaternion => {
                let (roll, pitch, yaw) = euler_angles(&estimated_state.attitude);
                (Vec2::new(roll, pitch), yaw)
            }
        };

        let desired_rates = self
            .attitude_controller
            .control(dt, desired_angles, current_angles);

        let desired_yaw_rate = self
            .yaw_controller
            .control(dt, setpoint.heading, current_heading);

        let current_rates = Vec3::new(
            estimated_state.angular_velocity.x,
            estimated_state.angular_velocity.y,
            estimated_state.angular_velocity.z,
        );

        let desired_rates = Vec3::new(desired_rates.x, desired_rates.y, desired_yaw_rate);

        let moments = self
            .rates_controller
            .control(dt, desired_rates, current_rates);

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
        .component_mul(&self.max_rates);

        let current_rates = Vec3::new(
            estimated_state.angular_velocity.x,
            estimated_state.angular_velocity.y,
            estimated_state.angular_velocity.z,
        );

        let moments = self
            .rates_controller
            .control(dt, desired_rates, current_rates);

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
        let nonlinear_thrust =
            libm::powf(saturate_throttle(linear_thrust), self.thrust_curve_exponent);
        nonlinear_thrust.clamp(0.0, 1.0)
    }
}
