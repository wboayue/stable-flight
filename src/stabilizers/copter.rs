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

use robomath::{to_radians, vec2, vec3, Quaternion, Vec2, Vec3};
use std::f32::consts::PI;
use std::time::Duration;

use crate::{
    AttitudeCommand, Command, ControlOutput, RateCommand, Stabilizer, VehicleState, WaypointCommand,
};

use super::controllers::{AttitudeConfiguration, AttitudeController, BodyRateController};

use super::Gains;

#[cfg(test)]
mod tests;

const G: f32 = 9.81; // acceleration due to gravity (m/s²)

/// Configuration for the [CopterStabilizer], including gains for all control loops and thrust mapping.
#[derive(Clone, Copy)]
pub struct CopterStabilizerConfig {
    pub rate_gains: Gains,
    pub angle_gains: Gains,
    pub altitude_gains: Gains,
    pub lateral_gains: Gains,
    pub yaw_gains: Gains,
    pub thrust_curve_exponent: f32,

    /// Maximum angular rates (roll, pitch, yaw) in rad/s.
    pub max_rates: Vec3<f32>,
}

impl Default for CopterStabilizerConfig {
    /// Creates a new `BasicControllerConfig` with default PID gains and thrust curve exponent.
    fn default() -> Self {
        CopterStabilizerConfig {
            rate_gains: Gains {
                p: 1.0,
                i: 0.0,
                d: 0.0,
            },
            angle_gains: Gains {
                p: 2.0,
                i: 0.2,
                d: 0.1,
            },
            altitude_gains: Gains {
                p: 3.0,
                i: 0.3,
                d: 0.15,
            },
            lateral_gains: Gains {
                p: 1.5,
                i: 0.15,
                d: 0.075,
            },
            yaw_gains: Gains {
                p: 2.5,
                i: 0.0,
                d: 0.06,
            },
            thrust_curve_exponent: 2.0,
            max_rates: Vec3 {
                x: 3.0, // max roll rate in rad/s (~172 deg/s)
                y: 3.0, // max pitch rate in rad/s (~172 deg/s)
                z: 3.0, // max yaw rate in rad/s (~86 deg/s)
            },
        }
    }
}

/// A basic flight controller for vehicles like quadcopters, implementing a cascaded control system.
///
/// This controller supports rate, attitude, and position control loops, with configurable gains
/// for each loop. It processes commands (`RateCommand`, `AttitudeCommand`, `WaypointCommand`)
/// and generates motor control outputs (`ControlOutput`).
pub struct CopterStabilizer {
    config: CopterStabilizerConfig,

    attitude_controller: AttitudeController,
    rates_controller: BodyRateController,

    integrated_altitude_error: f32,
    previous_yaw_error: f32,
    integrated_yaw_error: f32,
    previous_pos_error: Vec2<f32>,
    integrated_pos_error: Vec2<f32>,
    previous_angle_error: Vec2<f32>,
    integrated_angle_error: Vec2<f32>,

    /// Maximum angular rates (roll, pitch, yaw) in rad/s.
    pub max_rates: Vec3<f32>,
}

impl CopterStabilizer {
    /// Creates a new `BasicController` with the specified configuration.
    ///
    /// # Arguments
    ///
    /// * `config` - Configuration for the controller, including PID gains and thrust curve exponent.
    pub fn new(config: CopterStabilizerConfig) -> Self {
        CopterStabilizer {
            config: config.clone(),

            rates_controller: BodyRateController {
                ..Default::default()
            },
            attitude_controller: AttitudeController::new(&AttitudeConfiguration::default()),

            integrated_altitude_error: 0.0,
            previous_yaw_error: 0.0,
            integrated_yaw_error: 0.0,
            previous_pos_error: vec2(0.0, 0.0),
            integrated_pos_error: vec2(0.0, 0.0),
            previous_angle_error: vec2(0.0, 0.0),
            integrated_angle_error: vec2(0.0, 0.0),
            max_rates: vec3(3.0, 3.0, 3.0), // default max rates
        }
    }
}
impl Stabilizer for CopterStabilizer {
    /// Processes a control command and generates motor control outputs based on the current vehicle state.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive).
    /// * `command` - The desired control command (`RateCommand`, `AttitudeCommand`, or `WaypointCommand`).
    /// * `estimated_state` - The current state of the vehicle (attitude, angular rates, position, etc.).
    ///
    /// # Returns
    ///
    /// A [ControlOutput] containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.
    fn stabilize(
        &mut self,
        dt: Duration,
        command: &Command,
        estimated_state: &VehicleState,
    ) -> ControlOutput {
        assert!(dt.as_secs_f32() > 0.0, "Time step must be positive");

        match command {
            Command::Attitude(attitude_command) => {
                self.process_attitude_command(dt, attitude_command, estimated_state)
            }
            Command::Rate(rate_command) => {
                self.process_rate_command(dt, rate_command, estimated_state)
            }
            Command::Waypoint(waypoint_command) => {
                self.process_waypoint_command(dt, waypoint_command, estimated_state)
            }
        }
    }
}

impl CopterStabilizer {
    /// Processes an `AttitudeCommand` to control the vehicle's orientation.
    ///
    /// This method converts desired roll, pitch, and yaw angles into angular rates using the attitude controller,
    /// then passes the rates to the rate controller to generate motor commands.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive).
    /// * `attitude_command` - The desired roll, pitch, yaw angles (in radians) and throttle (0.0 to 1.0).
    /// * `estimated_state` - The current state of the vehicle (attitude, angular rates, etc.).
    ///
    /// # Returns
    ///
    /// A `ControlOutput` containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.
    fn process_attitude_command(
        &mut self,
        dt: Duration,
        attitude_command: &AttitudeCommand,
        estimated_state: &VehicleState,
    ) -> ControlOutput {
        assert!(dt.as_secs_f32() > 0.0, "Time step must be positive");
        assert!(
            attitude_command.throttle >= 0.0 && attitude_command.throttle <= 1.0,
            "Throttle ({}) must be in the range [0, 1]",
            attitude_command.throttle
        );
        assert!(
            attitude_command.roll >= -1.0 && attitude_command.roll <= 1.0,
            "Desired roll ({}) must be in the range [-1, 1]",
            attitude_command.roll
        );
        assert!(
            attitude_command.pitch >= -1.0 && attitude_command.pitch <= 1.0,
            "Desired pitch ({}) must be in the range [-1, 1]",
            attitude_command.pitch
        );
        assert!(
            attitude_command.yaw >= -1.0 && attitude_command.yaw <= 1.0,
            "Desired yaw ({}) must be in the range [-1, 1]",
            attitude_command.yaw
        );
        let desired_angles = vec2(attitude_command.roll, attitude_command.pitch);
        let current_angles = vec2(estimated_state.roll, estimated_state.pitch);

        println!(
            "desired angles: {:?}, current angles: {:?}",
            desired_angles, current_angles
        );

        let desired_rates = self
            .attitude_controller
            .control(dt, desired_angles, current_angles);

        let desired_yaw_rate = self.control_yaw(dt, 0.0, estimated_state.heading);

        let current_rates = vec3(
            estimated_state.angular_velocity.x,
            estimated_state.angular_velocity.y,
            estimated_state.angular_velocity.z,
        );

        let desired_rates = vec3(desired_rates.x, desired_rates.y, desired_yaw_rate);

        let moments = self
            .rates_controller
            .control(dt, desired_rates, current_rates);

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
    /// * `estimated_state` - The current state of the vehicle (angular rates, etc.).
    ///
    /// # Returns
    ///
    /// A `ControlOutput` containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.
    fn process_rate_command(
        &mut self,
        dt: Duration,
        rate_command: &RateCommand,
        estimated_state: &VehicleState,
    ) -> ControlOutput {
        let desired_rates = vec3(
            rate_command.roll_rate,
            rate_command.pitch_rate,
            rate_command.yaw_rate,
        );

        let desired_rates = desired_rates.clamp(-1.0, 1.0) * self.max_rates;

        let current_rates = vec3(
            estimated_state.angular_velocity.x,
            estimated_state.angular_velocity.y,
            estimated_state.angular_velocity.z,
        );

        let moments = self
            .rates_controller
            .control(dt, desired_rates, current_rates);

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
    /// * `estimated_state` - The current state of the vehicle (position, velocity, attitude, etc.).
    ///
    /// # Returns
    ///
    /// A `ControlOutput` containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.
    fn process_waypoint_command(
        &mut self,
        dt: Duration,
        waypoint_command: &WaypointCommand,
        estimated_state: &VehicleState,
    ) -> ControlOutput {
        let desired_position = vec2(waypoint_command.position.x, waypoint_command.position.y);
        let current_position = vec2(
            estimated_state.position.x.value,
            estimated_state.position.y.value,
        );

        let desired_angles = self.lateral_controller(
            dt,
            desired_position,
            current_position,
            vec2(
                estimated_state.velocity.x.value,
                estimated_state.velocity.y.value,
            ),
        );

        let current_angles = vec2(
            estimated_state.attitude.roll(),
            estimated_state.attitude.pitch(),
        );

        let desired_rp_rates = self
            .attitude_controller
            .control(dt, desired_angles, current_angles);
        let desired_yaw_rate = self.control_yaw(dt, 0.0, estimated_state.attitude.yaw());

        let desired_rates = vec3(desired_rp_rates.x, desired_rp_rates.y, desired_yaw_rate);

        let current_rates = vec3(
            estimated_state.angular_velocity.x,
            estimated_state.angular_velocity.y,
            estimated_state.angular_velocity.z,
        );

        let moments = self
            .rates_controller
            .control(dt, desired_rates, current_rates);

        let desired_altitude = waypoint_command.altitude;
        let current_altitude = estimated_state.position.z.value;

        let linear_thrust = self.control_altitude(
            dt,
            desired_altitude,
            0.0,
            current_altitude,
            estimated_state.velocity.z.value,
            estimated_state.attitude,
        );

        ControlOutput {
            thrust: self.fit_to_thrust_curve(linear_thrust),
            moments,
        }
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
    /// * `dt` - Time step since the last update (must be positive).
    /// * `desired_altitude` - Desired altitude in meters.
    /// * `desired_velocity` - Desired vertical velocity in m/s (currently unused, set to 0.0).
    /// * `estimated_altitude` - Current measured altitude in meters.
    /// * `estimated_velocity` - Current vertical velocity in m/s.
    /// * `estimated_attitude` - Current attitude as a quaternion.
    ///
    /// # Returns
    ///
    /// A normalized thrust command (0.0 to 1.0).
    fn control_altitude(
        &mut self,
        dt: Duration,
        desired_altitude: f32,
        desired_velocity: f32,
        estimated_altitude: f32,
        estimated_velocity: f32,
        estimated_attitude: Quaternion,
    ) -> f32 {
        const GRAVITY: f32 = 9.81;
        const MAX_ACCEL: f32 = 20.0;

        let r = estimated_attitude.rotation_matrix_i_wrt_b();
        let gains = &self.config.altitude_gains;

        let altitude_error = desired_altitude - estimated_altitude;
        self.integrated_altitude_error += altitude_error * dt.as_secs_f32();
        self.integrated_altitude_error = self.integrated_altitude_error.clamp(-100.0, 100.0);

        let proportional_term = gains.p * altitude_error;
        let derivative_term = gains.d * (desired_velocity - estimated_velocity);
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
    /// * `estimated_position` - Current position (x, y) in meters.
    /// * `estimated_velocity` - Current velocity (vx, vy) in m/s.
    ///
    /// # Returns
    ///
    /// Desired roll and pitch angles in radians.
    fn lateral_controller(
        &mut self,
        dt: Duration,
        desired_position: Vec2<f32>,
        estimated_position: Vec2<f32>,
        estimated_velocity: Vec2<f32>,
    ) -> Vec2<f32> {
        let gains = &self.config.lateral_gains;

        let error = desired_position - estimated_position;
        let error_derivative = (error - self.previous_pos_error) / dt.as_secs_f32();

        self.integrated_pos_error = self.integrated_pos_error + dt.as_secs_f32() * error;
        let accel_cmd =
            gains.p * error + gains.i * self.integrated_pos_error + gains.d * (-estimated_velocity);

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
    /// * `estimated_yaw` - Current measured yaw angle in radians.
    ///
    /// # Returns
    ///
    /// Desired yaw rate in rad/s.
    fn control_yaw(&mut self, dt: Duration, desired_yaw: f32, estimated_yaw: f32) -> f32 {
        assert!(dt.as_secs_f32() > 0.0, "Time step must be positive");
        assert!(
            desired_yaw >= -2.0 / PI && desired_yaw <= 2.0 * PI,
            "Desired yaw rate ({}) must be in the range [-2PI, 2PI]",
            desired_yaw
        );

        let gains = &self.config.yaw_gains;

        let error = angle_diff(desired_yaw, estimated_yaw); // Use a helper to handle angle wrapping
        let error_derivative = (error - self.previous_yaw_error) / dt.as_secs_f32();

        self.integrated_yaw_error += dt.as_secs_f32() * error;
        self.integrated_yaw_error = self.integrated_yaw_error.clamp(-100.0, 100.0);

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

// alpha 0.1 is a good starting point for most applications
fn low_pass_filter_vec3(prev: Vec3<f32>, new: Vec3<f32>, alpha: f32) -> Vec3<f32> {
    vec3(
        alpha * new.x + (1.0 - alpha) * prev.x,
        alpha * new.y + (1.0 - alpha) * prev.y,
        alpha * new.z + (1.0 - alpha) * prev.z,
    )
}
