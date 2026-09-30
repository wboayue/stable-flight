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

use robomath::{Vec3, vec2, vec3};

use crate::{
    AttitudeCommand, Command, ControlOutput, RateCommand, Stabilizer, VehicleState, WaypointCommand,
};

use super::Gains;
use super::controllers::*;
use crate::math::{saturate_throttle, saturate_unit, wrap_angle};

#[cfg(test)]
mod tests;

const G: f32 = 9.81; // acceleration due to gravity (m/s²)

/// Heading-hold target change per tick at full yaw stick (radians). At 250 Hz this is
/// 0.025 rad/s (~1.4°/s).
const HEADING_STEP_PER_TICK: f32 = 0.0001;

/// Configuration for the [CopterStabilizer], including gains for all control loops and thrust mapping.
#[derive(Clone, Debug, PartialEq)]
pub struct CopterStabilizerConfig {
    pub rates_controller: BodyRateController,
    pub attitude_controller: AttitudeController,

    pub altitude_gains: Gains,
    pub lateral_gains: Gains,
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
            // max_rates: Vec3 {
            //     x: 3.0, // max roll rate in rad/s (~172 deg/s)
            //     y: 3.0, // max pitch rate in rad/s (~172 deg/s)
            //     z: 3.0, // max yaw rate in rad/s (~86 deg/s)
            // },
        }
    }
}

/// A basic flight controller for vehicles like quadcopters, implementing a cascaded control system.
///
/// This controller supports rate, attitude, and position control loops, with configurable gains
/// for each loop. It processes commands (`RateCommand`, `AttitudeCommand`, `WaypointCommand`)
/// and generates motor control outputs (`ControlOutput`).
pub struct CopterStabilizer {
    pub config: CopterStabilizerConfig,

    pub altitude_controller: AltitudeController,
    pub attitude_controller: AttitudeController,
    pub rates_controller: BodyRateController,
    pub yaw_controller: YawController,
    pub lateral_controller: LateralController,

    // Maximum angular rates (roll, pitch, yaw) in rad/s.
    pub max_rates: Vec3<f32>,

    heading: f32,
}

impl CopterStabilizer {
    /// Creates a new [CopterStabilizer] with the specified configuration.
    ///
    /// # Arguments
    ///
    /// * `config` - Configuration for the controller, including PID gains and thrust curve exponent.
    pub fn new(config: CopterStabilizerConfig) -> Self {
        CopterStabilizer {
            config: config.clone(),

            altitude_controller: AltitudeController::with_config(&AltitudeControllerConfig {
                gains: config.altitude_gains,
                ..Default::default()
            }),

            rates_controller: config.rates_controller,
            attitude_controller: config.attitude_controller,

            yaw_controller: YawController::with_config(&YawControllerConfig {
                gains: config.yaw_gains,
                max_integral: 100.0,
                max_rate: 2.0,
                //                max_rate: config.max_rates.z,
            }),

            lateral_controller: LateralController::with_config(&LateralControllerConfig {
                gains: config.lateral_gains,
                max_integral: 100.0,
                max_rate: vec2(2.0, 2.0), // 2 rad/s (~114°/s) for roll and pitch
            }),

            max_rates: vec3(3.0, 3.0, 3.0), // default max rates
            heading: 0.0,
        }
    }
}
impl Stabilizer for CopterStabilizer {
    /// Processes a control command and generates motor control outputs based on the current vehicle state.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update in seconds (must be positive).
    /// * `command` - The desired control command (`RateCommand`, `AttitudeCommand`, or `WaypointCommand`).
    /// * `estimated_state` - The current state of the vehicle (attitude, angular rates, position, etc.).
    ///
    /// # Returns
    ///
    /// A [ControlOutput] containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.
    fn stabilize(
        &mut self,
        dt: f32,
        command: &Command,
        estimated_state: &VehicleState,
    ) -> ControlOutput {
        assert!(dt > 0.0, "Time step must be positive");

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

    /// Latches the current heading as the yaw-hold target.
    fn prime(&mut self, state: &VehicleState) {
        self.heading = state.heading;
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
    /// * `dt` - Time step since the last update in seconds (must be positive).
    /// * `attitude_command` - The desired roll, pitch, yaw angles (in radians) and throttle (0.0 to 1.0).
    /// * `estimated_state` - The current state of the vehicle (attitude, angular rates, etc.).
    ///
    /// # Returns
    ///
    /// A `ControlOutput` containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.
    fn process_attitude_command(
        &mut self,
        dt: f32,
        attitude_command: &AttitudeCommand,
        estimated_state: &VehicleState,
    ) -> ControlOutput {
        assert!(dt > 0.0, "Time step must be positive");
        // Commands from input devices can be out of range or NaN; saturate rather than panic
        // (roll/pitch saturate inside the attitude controller; see `crate::math`).
        let throttle = saturate_throttle(attitude_command.throttle);
        let yaw = saturate_unit(attitude_command.yaw);
        let desired_angles = vec2(attitude_command.roll, attitude_command.pitch);
        let current_angles = vec2(estimated_state.roll, estimated_state.pitch);

        let desired_rates = self
            .attitude_controller
            .control(dt, desired_angles, current_angles);

        // Wrapped so sustained yaw input can't grow the target without bound; the yaw
        // controller works on the wrapped error either way.
        let target = wrap_angle(self.heading + yaw * HEADING_STEP_PER_TICK);
        if target.is_finite() {
            self.heading = target;
        } else if estimated_state.heading.is_finite() {
            // A NaN target (e.g. primed from a NaN estimate) would otherwise disable heading
            // hold for good; re-latch from the current estimate once it's valid.
            self.heading = wrap_angle(estimated_state.heading);
        }

        let desired_yaw_rate =
            self.yaw_controller
                .control(dt, self.heading, estimated_state.heading);

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
            thrust: self.fit_to_thrust_curve(throttle),
            moments,
        }
    }

    /// Processes a `RateCommand` to directly control the vehicle's angular rates.
    ///
    /// This method bypasses the attitude controller and directly commands angular rates using the rate controller.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update in seconds (must be positive).
    /// * `rate_command` - The desired roll, pitch, yaw rates (in rad/s) and throttle (0.0 to 1.0).
    /// * `estimated_state` - The current state of the vehicle (angular rates, etc.).
    ///
    /// # Returns
    ///
    /// A `ControlOutput` containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.
    fn process_rate_command(
        &mut self,
        dt: f32,
        rate_command: &RateCommand,
        estimated_state: &VehicleState,
    ) -> ControlOutput {
        let desired_rates = vec3(
            rate_command.roll_rate,
            rate_command.pitch_rate,
            rate_command.yaw_rate,
        );

        // Per-component saturation: `clamp` alone passes NaN, which would zero every moment
        // (no damping) instead of meaning "hold zero rate".
        let desired_rates = vec3(
            saturate_unit(desired_rates.x),
            saturate_unit(desired_rates.y),
            saturate_unit(desired_rates.z),
        ) * self.max_rates;

        let current_rates = vec3(
            estimated_state.angular_velocity.x,
            estimated_state.angular_velocity.y,
            estimated_state.angular_velocity.z,
        );

        let moments = self
            .rates_controller
            .control(dt, desired_rates, current_rates);

        ControlOutput {
            thrust: self.fit_to_thrust_curve(saturate_throttle(rate_command.throttle)),
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
    /// * `dt` - Time step since the last update in seconds (must be positive).
    /// * `waypoint_command` - The desired position (x, y in meters) and altitude (in meters).
    /// * `estimated_state` - The current state of the vehicle (position, velocity, attitude, etc.).
    ///
    /// # Returns
    ///
    /// A `ControlOutput` containing the normalized thrust (0.0 to 1.0) and moments (roll, pitch, yaw) in N·m.
    fn process_waypoint_command(
        &mut self,
        dt: f32,
        waypoint_command: &WaypointCommand,
        estimated_state: &VehicleState,
    ) -> ControlOutput {
        let desired_position = vec2(waypoint_command.position.x, waypoint_command.position.y);
        let current_position = vec2(
            estimated_state.position.x.value,
            estimated_state.position.y.value,
        );

        let desired_angles = self.lateral_controller.control(
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
        let desired_yaw_rate = self
            .yaw_controller
            .control(dt, 0.0, estimated_state.attitude.yaw());

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

        let linear_thrust = self.altitude_controller.control(
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
        let nonlinear_thrust = libm::powf(linear_thrust, thrust_curve_exponent);
        nonlinear_thrust.clamp(0.0, 1.0)
    }
}
