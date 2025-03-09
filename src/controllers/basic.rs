//! Quadcopter Controller - X Frame Configuration
//!
//! Standard controller for a quadcopter with an X frame configuration.
//! Reference model: <https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX>
//!
//! Control Architecture:
//! --------------------
//! This implements a modular cascaded control system with configurable control loops:
//!
//! 1. Inner Loop: Rate Control
//!    - Controls angular rates (roll rate, pitch rate, yaw rate)
//!    - Fast update frequency (typically 400-1000Hz)
//!    - PID controllers with feed-forward terms for each axis
//!    - Directly commands motor mixing algorithm
//!    - Always active as the core control layer
//!
//! 2. Middle Loop: Attitude Control (Optional)
//!    - Controls orientation angles (roll angle, pitch angle, yaw angle)
//!    - Moderate update frequency (typically 100-250Hz)
//!    - Outputs desired angular rates to inner loop
//!    - Implements angle limiting for safety
//!    - Can be used without outer position loops for manual flight modes
//!
//! 3. Outer Loop: Position Control (Optional)
//!    - Altitude Control: Maintains desired height
//!    - Lateral Position Control: Maintains X,Y position
//!    - Slow update frequency (typically 10-50Hz)
//!    - Outputs desired roll/pitch angles to attitude controller
//!    - Can be independently enabled/disabled based on flight mode
//!
//! Configuration Options:
//! ---------------------
//! - Rate Control Only: Direct manual control of angular rates
//! - Rate + Attitude Control: Manual position control with automatic stabilization
//! - Full Control Stack: Automatic position and altitude holding
//! - Selective Enabling: Can enable altitude control while keeping manual lateral control

use core::error;
use std::time::Duration;

use uom::si::velocity;

use crate::{
    commanders::Command,
    math::{Quaternion, Vec3},
};

use super::{ControlOutput, Controller, VehicleState};

const G: f32 = 9.81; // acceleration due to gravity (m/s²)

#[derive(Default)]
pub struct Gains {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

#[derive(Default)]
pub struct QuadConfig {
    rate_gains: Gains,
    rate_ff: [f32; 3],
    angle_gains: Gains,
    angle_ff: [f32; 2],
    altitude_gains: Gains,
    lateral_gains: Gains,
    yaw_gains: Gains,
}

#[derive(Default)]
pub struct BasicController {
    config: QuadConfig,
    previous_rate_error: Vec3<f32>,
    integrated_rate_error: Vec3<f32>,
    integrated_altitude_error: f32,
}

impl Controller for BasicController {
    fn control(&self, _desired_state: &Command, _state: &VehicleState) -> ControlOutput {
        ControlOutput::default()
    }
}

impl BasicController {
    /// Controls the angular velocity (body rates) of the quadcopter.
    ///
    /// This is the innermost control loop that directly affects motor commands.
    /// It takes desired angular rates and current measured angular rates as inputs,
    /// then uses PID control to calculate motor commands that will achieve the
    /// desired rates around all three axes.
    ///
    /// # Arguments
    ///
    /// * `desired_rates` - Vector3 containing desired angular velocities [p, q, r] in rad/s
    /// * `current_rates` - Vector3 containing current measured angular velocities in rad/s
    ///
    /// # Returns
    ///
    /// * `Vector3` - Moment commands [roll_moment, pitch_moment, yaw_moment] in N·m
    pub fn rate_controller(
        &mut self,
        dt: Duration,
        desired_rates: Vec3<f32>,
        current_rates: Vec3<f32>,
    ) -> Vec3<f32> {
        let gains = &self.config.rate_gains;

        let error = desired_rates - current_rates;
        let error_derivative = (error - self.previous_rate_error) / dt.as_secs_f32();

        self.integrated_rate_error = self.integrated_rate_error + dt.as_secs_f32() * error;

        let moment =
            gains.p * error + gains.i * self.integrated_rate_error + gains.d * error_derivative;

        self.previous_rate_error = error;

        moment
    }

    /// Controls the roll and pitch angles of the quadcopter.
    ///
    /// This middle control loop takes desired roll and pitch angles and converts
    /// them to desired angular rates for the body rate controller.
    /// It implements angle limiting for safety and provides smooth transitions.
    ///
    /// # Arguments
    ///
    /// * `desired_angles` - Vector2 containing desired [roll, pitch] angles in radians
    /// * `current_angles` - Vector2 containing current [roll, pitch] angles in radians
    /// * `current_acceleration` - Vector3 of current acceleration in body frame m/s²
    ///
    /// # Returns
    ///
    /// * `Vector2` - Desired angular rates [p_command, q_command] in rad/s
    pub fn roll_pitch_controller() {
        println!("Angle controller");
    }

    /// Controls the altitude of the quadcopter.
    ///
    /// This outer loop maintains the desired altitude by calculating an appropriate
    /// thrust command. It can operate in position, velocity, or acceleration control modes.
    /// Enables altitude hold.
    ///
    /// # Arguments
    ///
    /// * `desired_altitude` - Desired altitude in meters
    /// * `current_altitude` - Current measured altitude in meters
    /// * `current_velocity` - Current vertical velocity in m/s
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    ///
    /// * `f32` - Collective thrust command in Newtons
    pub fn altitude_controller(
        &mut self,
        altitude_cmd: f32,
        velocity_cmd: f32,
        altitude: f32,
        velocity: f32,
        attitude: Quaternion,
        dt: Duration,
    ) -> f32 {
        let R = attitude.rotation_matrix_i_wrt_b();
        let thrust = 0.0;
        let gains = &self.config.altitude_gains;

        // Calculate position error (desired - actual)
        // In NED frame, positive Z is downward, so posZCmd > posZ means we're too high
        let altitude_error = altitude_cmd - altitude;

        // Update the integrated error term for the PID controller
        // This accumulates position error over time to eliminate steady-state errors
        self.integrated_altitude_error += altitude_error * dt.as_secs_f32();

        // Implement a PID controller for altitude
        let proportional_term = gains.p * altitude_error; // P term: proportional to position error
        let derivative_term = gains.d * (velocity_cmd - velocity); // D term: helps with damping
        let integral_term = gains.i * self.integrated_altitude_error; // I term: eliminates steady-state error

        // Combine all terms to get the desired acceleration
        let mut acceleration_cmd = proportional_term + derivative_term + integral_term;

        // Adjust for gravity and drone tilt
        // bodyZAlignment represents how much of the thrust vector is aligned with the Z axis
        // When the drone is perfectly level, bodyZAlignment = 1
        // When tilted, bodyZAlignment < 1, so we need more thrust to achieve the same vertical acceleration
        let body_z_alignment = R[[2, 2]]; // R(2,2) is the z-component of the body's z-axis in inertial frame

        // We subtract gravity because in NED, gravity acts in the positive Z direction
        // Then divide by the alignment factor to compensate for tilt
        acceleration_cmd = (acceleration_cmd - G) / body_z_alignment;

        // Constrain the commanded acceleration to respect maximum ascent/descent rates
        // Note: In NED, negative acceleration is upward (ascent), positive is downward (descent)
        //   accelZCmd = CONSTRAIN(accelZCmd, -maxAscentRate / dt, maxDescentRate / dt);

        // Convert acceleration to force (F = m*a)
        // Negative sign because in NED, an upward force (negative Z direction)
        // is needed to counteract gravity and achieve a downward acceleration reduction
        -acceleration_cmd
    }

    /// Controls the lateral (X-Y) position of the quadcopter.
    ///
    /// This outer loop maintains the desired horizontal position by calculating
    /// appropriate roll and pitch angle commands that will move the quadcopter
    /// to the target position.
    ///
    /// Enables position hold and navigation.
    ///
    /// # Arguments
    ///
    /// * `desired_position` - Vector2 containing target [x, y] position in meters
    /// * `current_position` - Vector2 containing current [x, y] position in meters
    /// * `current_velocity` - Vector2 containing current [vx, vy] velocity in m/s
    /// * `yaw` - Current yaw angle in radians
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    ///
    /// * `Vector2` - Desired [roll, pitch] angles in radians
    pub fn lateral_controller() {
        println!("Lateral controller");
    }

    /// Controls the yaw (heading) of the quadcopter.
    ///
    /// This controller maintains the desired heading by calculating an appropriate
    /// yaw rate command. It handles both absolute heading targets and relative
    /// yaw rate commands.
    ///
    /// # Arguments
    ///
    /// * `desired_yaw` - Target yaw angle in radians
    /// * `current_yaw` - Current measured yaw angle in radians
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    ///
    /// * `f32` - Desired yaw rate (r) in rad/s
    pub fn yaw_controller() {
        println!("Yaw controller");
    }
}
