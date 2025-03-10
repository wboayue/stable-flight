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
//! 3. **Outer Loop: Position Control (Optional)**
//!    - Altitude Control: Maintains desired height
//!    - Lateral Position Control: Maintains X,Y position
//!    - Slow update frequency (typically 10-50Hz)
//!    - Outputs desired roll/pitch angles to attitude controller
//!    - Can be independently enabled/disabled based on flight mode
//!
//! # Configuration Options
//!
//! - **Rate Control Only**: Direct manual control of angular rates
//! - **Rate + Attitude Control**: Manual position control with automatic stabilization
//! - **Full Control Stack**: Automatic position and altitude holding
//! - **Selective Enabling**: Can enable altitude control while keeping manual lateral control

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

/// PID gains for a single control loop.
#[derive(Default)]
pub struct Gains {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

/// Configuration for the [BasicController], including gains for all control loops and thrust mapping.
#[derive(Default)]
pub struct BasicControllerConfig {
    pub rate_gains: Gains,
    pub angle_gains: Gains,
    pub altitude_gains: Gains,
    pub lateral_gains: Gains,
    pub yaw_gains: Gains,
    pub thrust_curve_exponent: f32,
}

/// A basic flight controller for vehicles like quadcopters, implementing a cascaded control system.
///
/// This controller supports rate, attitude, and position control loops, with configurable gains
/// for each loop. It processes commands (`RateCommand`, `AttitudeCommand`, `WaypointCommand`)
/// and generates motor control outputs (`ControlOutput`).
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
    /// Processes a control command and generates motor control outputs based on the current vehicle state.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive).
    /// * `command` - The desired control command (`RateCommand`, `AttitudeCommand`, or `WaypointCommand`).
    /// * `current_state` - The current state of the vehicle (attitude, angular rates, position, etc.).
    ///
    /// # Returns
    ///
    /// A [ControlOutput] containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.    
    fn control(
        &mut self,
        dt: Duration,
        command: &Command,
        current_state: &VehicleState,
    ) -> ControlOutput {
        assert!(dt.as_secs_f32() > 0.0, "Time step must be positive");

        match command {
            Command::Attitude(attitude_command) => {
                self.process_attitude_command(dt, attitude_command, current_state)
            }
            Command::Rate(rate_command) => {
                self.process_rate_command(dt, rate_command, current_state)
            }
            Command::Waypoint(waypoint_command) => {
                self.process_waypoint_command(dt, waypoint_command, current_state)
            }
        }
    }
}

impl BasicController {
    /// Processes an `AttitudeCommand` to control the vehicle's orientation.
    ///
    /// This method converts desired roll, pitch, and yaw angles into angular rates using the attitude controller,
    /// then passes the rates to the rate controller to generate motor commands.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive).
    /// * `attitude_command` - The desired roll, pitch, yaw angles (in radians) and throttle (0.0 to 1.0).
    /// * `current_state` - The current state of the vehicle (attitude, angular rates, etc.).
    ///
    /// # Returns
    ///
    /// A `ControlOutput` containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.    
    fn process_attitude_command(
        &mut self,
        dt: Duration,
        attitude_command: &AttitudeCommand,
        current_state: &VehicleState,
    ) -> ControlOutput {
        let desired_angles = vec2(attitude_command.roll, attitude_command.pitch);
        let current_angles = vec2(
            current_state.attitude.roll(),
            current_state.attitude.pitch(),
        );

        let desired_rp_rates = self.control_roll_pitch(dt, desired_angles, current_angles);
        let desired_yaw_rate =
            self.control_yaw(dt, attitude_command.yaw, current_state.attitude.yaw());

        let desired_rates = vec3(desired_rp_rates.x, desired_rp_rates.y, desired_yaw_rate);

        let current_rates = vec3(
            current_state.angular_velocity.x.value,
            current_state.angular_velocity.y.value,
            current_state.angular_velocity.z.value,
        );

        let moments = self.control_rates(dt, desired_rates, current_rates);

        ControlOutput {
            thrust: self.fit_to_thrust_curve(attitude_command.throttle),
            moments,
        }
    }

    /// Processes a `RateCommand` to directly control the vehicle's angular rates.
    ///
    /// This method bypasses the attitude controller and directly commands angular rates using the rate controller.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive).
    /// * `rate_command` - The desired roll, pitch, yaw rates (in rad/s) and throttle (0.0 to 1.0).
    /// * `current_state` - The current state of the vehicle (angular rates, etc.).
    ///
    /// # Returns
    ///
    /// A `ControlOutput` containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.    
    fn process_rate_command(
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

        let moments = self.control_rates(dt, desired_rates, current_rates);

        ControlOutput {
            thrust: self.fit_to_thrust_curve(rate_command.throttle),
            moments,
        }
    }

    /// Processes a `WaypointCommand` to control the vehicle's position and altitude.
    ///
    /// This method uses the position controller to generate desired roll and pitch angles, the altitude controller
    /// to compute thrust, and the attitude and rate controllers to stabilize the vehicle.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive).
    /// * `waypoint_command` - The desired position (x, y in meters) and altitude (in meters).
    /// * `current_state` - The current state of the vehicle (position, velocity, attitude, etc.).
    ///
    /// # Returns
    ///
    /// A `ControlOutput` containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.    
    fn process_waypoint_command(
        &mut self,
        dt: Duration,
        waypoint_command: &WaypointCommand,
        current_state: &VehicleState,
    ) -> ControlOutput {
        let desired_position = vec2(waypoint_command.position.x, waypoint_command.position.y);
        let current_position = vec2(
            current_state.position.x.value,
            current_state.position.y.value,
        );

        let desired_angles = self.lateral_controller(
            dt,
            desired_position,
            current_position,
            vec2(
                current_state.velocity.x.value,
                current_state.velocity.y.value,
            ),
        );

        let current_angles = vec2(
            current_state.attitude.roll(),
            current_state.attitude.pitch(),
        );

        let desired_rp_rates = self.control_roll_pitch(dt, desired_angles, current_angles);
        let desired_yaw_rate = self.control_yaw(dt, 0.0, current_state.attitude.yaw());

        let desired_rates = vec3(desired_rp_rates.x, desired_rp_rates.y, desired_yaw_rate);

        let current_rates = vec3(
            current_state.angular_velocity.x.value,
            current_state.angular_velocity.y.value,
            current_state.angular_velocity.z.value,
        );

        let moments = self.control_rates(dt, desired_rates, current_rates);

        let desired_altitude = waypoint_command.altitude;
        let current_altitude = current_state.position.z.value;

        let linear_thrust = self.control_altitude(
            desired_altitude,
            0.0,
            current_altitude,
            current_state.velocity.z.value,
            current_state.attitude,
            dt,
        );

        ControlOutput {
            thrust: self.fit_to_thrust_curve(linear_thrust),
            moments,
        }
    }

    /// Controls the angular velocity (body rates) of the vehicle using a PID controller.
    ///
    /// This is the innermost control loop that directly affects motor commands. It takes desired angular rates
    /// and current measured angular rates as inputs, then uses PID control to calculate motor moments.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive).
    /// * `desired_rates` - Desired angular rates (roll, pitch, yaw) in rad/s.
    /// * `current_rates` - Current measured angular rates (roll, pitch, yaw) in rad/s.
    ///
    /// # Returns
    ///
    /// Moments (roll, pitch, yaw) in N·m to achieve the desired rates.
    fn control_rates(
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

    /// Controls the roll and pitch angles of the vehicle using a PID controller.
    ///
    /// This middle control loop converts desired roll and pitch angles into desired angular rates for the rate controller.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive).
    /// * `desired_angles` - Desired roll and pitch angles in radians.
    /// * `current_angles` - Current roll and pitch angles in radians.
    ///
    /// # Returns
    ///
    /// Desired angular rates (roll rate, pitch rate) in rad/s.
    fn control_roll_pitch(
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

    /// Controls the altitude of the vehicle by computing a normalized thrust command using a PID controller.
    ///
    /// This outer loop maintains the desired altitude by calculating a thrust command (0.0 to 1.0), where:
    /// - 0.0: No thrust (vehicle falls).
    /// - 0.5: Nominal hover thrust (approximately balances gravity).
    /// - 1.0: Maximum thrust (maximum upward acceleration).
    ///
    /// # Arguments
    ///
    /// * `altitude_cmd` - Desired altitude in meters.
    /// * `velocity_cmd` - Desired vertical velocity in m/s (currently unused, set to 0.0).
    /// * `altitude` - Current measured altitude in meters.
    /// * `velocity` - Current vertical velocity in m/s.
    /// * `attitude` - Current attitude as a quaternion.
    /// * `dt` - Time step since the last update (must be positive).
    ///
    /// # Returns
    ///
    /// A normalized thrust command (0.0 to 1.0).
    fn control_altitude(
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

        let r = attitude.rotation_matrix_i_wrt_b();
        let gains = &self.config.altitude_gains;

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

        linear_thrust.clamp(0.0, 1.0)
    }

    /// Controls the lateral (X-Y) position of the vehicle using a PID controller.
    ///
    /// This outer loop maintains the desired horizontal position by calculating desired roll and pitch angles.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive).
    /// * `desired_position` - Target position (x, y) in meters.
    /// * `current_position` - Current position (x, y) in meters.
    /// * `current_velocity` - Current velocity (vx, vy) in m/s.
    ///
    /// # Returns
    ///
    /// Desired roll and pitch angles in radians.
    fn lateral_controller(
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

    /// Controls the yaw (heading) of the vehicle using a PID controller.
    ///
    /// This controller maintains the desired heading by calculating a yaw rate command.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive).
    /// * `desired_yaw` - Target yaw angle in radians.
    /// * `current_yaw` - Current measured yaw angle in radians.
    ///
    /// # Returns
    ///
    /// Desired yaw rate in rad/s.
    fn control_yaw(&mut self, dt: Duration, desired_yaw: f32, current_yaw: f32) -> f32 {
        let gains = &self.config.yaw_gains;

        let error = angle_diff(desired_yaw, current_yaw); // Use a helper to handle angle wrapping
        let error_derivative = (error - self.previous_yaw_error) / dt.as_secs_f32();

        self.integrated_yaw_error += dt.as_secs_f32() * error;

        let yaw_rate =
            gains.p * error + gains.i * self.integrated_yaw_error + gains.d * error_derivative;

        self.previous_yaw_error = error;

        yaw_rate
    }

    /// Applies a nonlinear thrust curve to a linear thrust command.
    ///
    /// Maps a linear thrust value (0.0 to 1.0) to a nonlinear output using the configured thrust curve exponent.
    /// This compensates for nonlinear motor responses.
    ///
    /// # Arguments
    ///
    /// * `linear_thrust` - Linear thrust command (0.0 to 1.0).
    ///
    /// # Returns
    ///
    /// Nonlinear thrust command (0.0 to 1.0).    
    fn fit_to_thrust_curve(&self, linear_thrust: f32) -> f32 {
        let thrust_curve_exponent = self.config.thrust_curve_exponent;
        // Apply nonlinear thrust curve with tunable exponent
        let nonlinear_thrust = linear_thrust.powf(thrust_curve_exponent);
        nonlinear_thrust.clamp(0.0, 1.0)
    }
}

/// Normalizes the yaw error to the range [-π, π] to handle angle wrapping.
///
/// This ensures the shortest rotation path is chosen (e.g., if desired yaw is 350° and current yaw is 10°,
/// the error is -20°, not 340°).
///
/// # Arguments
///
/// * `desired_yaw` - Desired yaw angle in radians.
/// * `current_yaw` - Current yaw angle in radians.
///
/// # Returns
///
/// The normalized yaw error in radians, in the range [-π, π].
fn angle_diff(desired_yaw: f32, current_yaw: f32) -> f32 {
    let mut yaw_error = desired_yaw - current_yaw;
    yaw_error += PI;
    yaw_error = yaw_error.rem_euclid(2.0 * PI);
    yaw_error -= PI;
    yaw_error
}
