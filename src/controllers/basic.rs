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

use std::f32::consts::PI;
use std::time::Duration;

use crate::{
    commanders::{AttitudeCommand, Command, RateCommand, WaypointCommand},
    math::{vec2, vec3, Quaternion, Vec2, Vec3},
};

use super::{ControlOutput, Controller, VehicleState};

#[cfg(test)]
mod tests;

const G: f32 = 9.81; // acceleration due to gravity (m/s²)

#[derive(Default)]
pub struct Gains {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

#[derive(Default)]
pub struct BasicControllerConfig {
    pub rate_gains: Gains,
    pub angle_gains: Gains,
    pub altitude_gains: Gains,
    pub lateral_gains: Gains,
    pub yaw_gains: Gains,
}

#[derive(Default)]
pub struct BasicController {
    config: BasicControllerConfig,
    previous_rate_error: Vec3<f32>,
    integrated_rate_error: Vec3<f32>,
    integrated_altitude_error: f32,
    previous_yaw_error: f32,
    integrated_yaw_error: f32,
    previous_pos_error: Vec2<f32>,
    integrated_pos_error: Vec2<f32>,
    previous_angle_error: Vec2<f32>,
    integrated_angle_error: Vec2<f32>,
}

impl Controller for BasicController {
    fn control(
        &mut self,
        dt: Duration,
        command: &Command,
        current_state: &VehicleState,
    ) -> ControlOutput {
        match command {
            Command::Attitude(attitude_command) => {
                self.attitude_control(dt, attitude_command, current_state)
            }
            Command::Rate(rate_command) => self.rate_control(dt, rate_command, current_state),
            Command::Waypoint(waypoint_command) => {
                self.waypoint_control(dt, waypoint_command, current_state)
            }
        }
    }
}

impl BasicController {
    fn attitude_control(
        &mut self,
        dt: Duration,
        desired_state: &AttitudeCommand,
        state: &VehicleState,
    ) -> ControlOutput {
        ControlOutput::default()
    }

    fn rate_control(
        &mut self,
        dt: Duration,
        rate_command: &RateCommand,
        current_state: &VehicleState,
    ) -> ControlOutput {
        let desired_rates = vec3(
            rate_command.roll_rate,
            rate_command.pitch_rate,
            rate_command.yaw_rate,
        );

        let current_rates = vec3(
            current_state.angular_velocity.x.value,
            current_state.angular_velocity.y.value,
            current_state.angular_velocity.z.value,
        );

        let cmd = self.rate_controller(dt, desired_rates, current_rates);

        ControlOutput {
            thrust: rate_command.throttle, // fit to curve
            moment: cmd,
        }
    }

    fn waypoint_control(
        &mut self,
        dt: Duration,
        desired_state: &WaypointCommand,
        state: &VehicleState,
    ) -> ControlOutput {
        ControlOutput::default()
    }

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
    pub fn roll_pitch_controller(
        &mut self,
        dt: Duration,
        desired_angles: Vec2<f32>,
        current_angles: Vec2<f32>,
    ) -> Vec2<f32> {
        let gains = &self.config.angle_gains;
        let error = desired_angles - current_angles;
        let error_derivative = (error - self.previous_angle_error) / dt.as_secs_f32();
        self.integrated_angle_error = self.integrated_angle_error + dt.as_secs_f32() * error;
        let rates =
            gains.p * error + gains.i * self.integrated_angle_error + gains.d * error_derivative;
        self.previous_angle_error = error;
        rates
    }

    /// Controls the altitude of a vehicle by computing a relative thrust command.
    ///
    /// This outer loop maintains the desired altitude by calculating an appropriate
    /// thrust command, normalized between 0 and 1, where:
    /// - 0.0: No thrust (vehicle falls).
    /// - 0.5: Nominal hover thrust (approximately balances gravity).
    /// - 1.0: Maximum thrust (maximum upward acceleration).
    /// It can operate in position, velocity, or acceleration control modes and enables altitude hold.
    ///
    /// # Arguments
    ///
    /// * `altitude_cmd` - Desired altitude in meters
    /// * `velocity_cmd` - Desired vertical velocity in m/s
    /// * `altitude` - Current measured altitude in meters
    /// * `velocity` - Current vertical velocity in m/s
    /// * `attitude` - Current attitude as a Quaternion
    /// * `dt` - Time step as a Duration
    ///
    /// # Returns
    ///
    /// * `f32` - Relative thrust command (0.0 to 1.0)
    pub fn altitude_controller(
        &mut self,
        altitude_cmd: f32,
        velocity_cmd: f32,
        altitude: f32,
        velocity: f32,
        attitude: Quaternion,
        dt: Duration,
    ) -> f32 {
        const GRAVITY: f32 = 9.81;
        const MAX_ACCEL: f32 = 20.0;

        assert!(dt.as_secs_f32() > 0.0, "Time step must be positive");

        let r = attitude.rotation_matrix_i_wrt_b();
        let gains = &self.config.altitude_gains;
        //        let thrust_curve_exponent = self.config.thrust_curve_exponent; // e.g., 2.0 for quadratic
        let thrust_curve_exponent = 2.0; // e.g., 2.0 for quadratic

        let altitude_error = altitude_cmd - altitude;
        self.integrated_altitude_error += altitude_error * dt.as_secs_f32();
        self.integrated_altitude_error = self.integrated_altitude_error.clamp(-100.0, 100.0);

        let proportional_term = gains.p * altitude_error;
        let derivative_term = gains.d * (velocity_cmd - velocity);
        let integral_term = gains.i * self.integrated_altitude_error;
        let mut acceleration_cmd = proportional_term + derivative_term + integral_term;

        let body_z_alignment = r[[2, 2]].max(0.01);
        acceleration_cmd = (acceleration_cmd - GRAVITY) / body_z_alignment;
        acceleration_cmd = acceleration_cmd.clamp(-MAX_ACCEL, MAX_ACCEL);

        // Map acceleration_cmd such that:
        // - acceleration_cmd = -GRAVITY (free fall) -> linear_thrust = 0
        // - acceleration_cmd = 0 (hover) -> linear_thrust = 0.5
        // - acceleration_cmd = MAX_ACCEL (max upward) -> linear_thrust = 1
        let accel_range_lower = GRAVITY; // From -GRAVITY to 0
        let accel_range_upper = MAX_ACCEL; // From 0 to MAX_ACCEL
        let linear_thrust = if acceleration_cmd < 0.0 {
            // Map [-GRAVITY, 0] to [0, 0.5]
            0.5 * (acceleration_cmd + GRAVITY) / GRAVITY
        } else {
            // Map [0, MAX_ACCEL] to [0.5, 1]
            0.5 + 0.5 * acceleration_cmd / MAX_ACCEL
        };
        let linear_thrust = linear_thrust.clamp(0.0, 1.0);

        // Apply nonlinear thrust curve with tunable exponent
        let nonlinear_thrust = linear_thrust.powf(thrust_curve_exponent);
        nonlinear_thrust.clamp(0.0, 1.0)
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
    pub fn lateral_controller(
        &mut self,
        dt: Duration,
        desired_position: Vec2<f32>,
        current_position: Vec2<f32>,
        current_velocity: Vec2<f32>,
    ) -> Vec2<f32> {
        let gains = &self.config.lateral_gains;
        let error = desired_position - current_position;
        let error_derivative = (error - self.previous_pos_error) / dt.as_secs_f32();
        self.integrated_pos_error = self.integrated_pos_error + dt.as_secs_f32() * error;
        let accel_cmd =
            gains.p * error + gains.i * self.integrated_pos_error + gains.d * (-current_velocity);
        self.previous_pos_error = error;
        // Convert acceleration to roll/pitch angles (simplified)
        let roll_cmd = accel_cmd.y / G;
        let pitch_cmd = -accel_cmd.x / G;
        vec2(roll_cmd, pitch_cmd)
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
    pub fn yaw_controller(&mut self, dt: Duration, desired_yaw: f32, current_yaw: f32) -> f32 {
        let gains = &self.config.yaw_gains;
        let error = angle_diff(desired_yaw, current_yaw); // Use a helper to handle angle wrapping
        let error_derivative = (error - self.previous_yaw_error) / dt.as_secs_f32();
        self.integrated_yaw_error += dt.as_secs_f32() * error;
        let yaw_rate =
            gains.p * error + gains.i * self.integrated_yaw_error + gains.d * error_derivative;
        self.previous_yaw_error = error;
        yaw_rate
    }
}

// Normalize the yaw error to the range [-π, π]
// This handles the circular nature of angles, ensuring we always rotate the shortest direction
// For example, if yawCmd = 350° and yaw = 10°, the error should be -20° (not 340°)
//
// How the normalization works:
// 1. Add π to shift the range to [-π, π] → [0, 2π]
// 2. Use fmodf to wrap to [0, 2π] range
// 3. Subtract π to shift back to [-π, π] range
fn angle_diff(desired_yaw: f32, current_yaw: f32) -> f32 {
    let mut yaw_error = desired_yaw - current_yaw;
    yaw_error += PI;
    yaw_error = yaw_error.rem_euclid(2.0 * PI);
    yaw_error -= PI;
    yaw_error
}
