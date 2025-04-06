use std::time::Duration;

use robomath::{vec2, Vec2};

use crate::stabilizers::Gains;

#[cfg(test)]
mod tests;

const G: f32 = 9.81; // Gravitational acceleration in m/s²

#[derive(Clone, Debug, PartialEq)]
pub struct LateralControllerConfig {
    /// PID gains for roll and pitch control (Kp, Ki, Kd).
    pub gains: Gains,

    /// Maximum absolute value of the integral term.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,

    /// Maximum angular rates in radians/second for roll (x) and pitch (y).
    /// Limits the output rates sent to the rate controller.
    pub max_rate: Vec2<f32>,
}

impl Default for LateralControllerConfig {
    /// Provides a default configuration with reasonable starting values.
    /// Note: PID gains (P=7.0, I=0.0, D=0.01) and max_rate (2 rad/s) may need tuning
    /// for specific vehicles.
    fn default() -> Self {
        Self {
            gains: Gains {
                p: 3.0,
                i: 0.3,
                d: 0.15,
            },
            max_integral: 100.0,      // Arbitrary limit
            max_rate: vec2(2.0, 2.0), // 2 rad/s (~114°/s) for roll and pitch
        }
    }
}

pub struct LateralController {
    config: LateralControllerConfig,
    integrated_error: Vec2<f32>, // Accumulated error (roll, pitch)
    max_integral: f32,
    previous_error: Vec2<f32>, // Previous error for derivative calculation
    max_rate: Vec2<f32>,       // Max angular rates in rad/s (roll, pitch)
}

impl LateralController {
    pub fn new() -> Self {
        LateralController::with_config(&LateralControllerConfig::default())
    }

    /// Initializes a new lateral controller with the given configuration.
    pub fn with_config(config: &LateralControllerConfig) -> Self {
        Self {
            config: config.clone(),
            integrated_error: vec2(0.0, 0.0),
            max_integral: config.max_integral,
            previous_error: vec2(0.0, 0.0),
            max_rate: config.max_rate,
        }
    }

    /// Initializes a new lateral controller with the given configuration.
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
    pub fn control(
        &mut self,
        dt: Duration,
        desired_position: Vec2<f32>,
        estimated_position: Vec2<f32>,
        estimated_velocity: Vec2<f32>,
    ) -> Vec2<f32> {
        let gains = &self.config.gains;

        let error = desired_position - estimated_position;
        let error_derivative = (error - self.previous_error) / dt.as_secs_f32();

        self.integrated_error = self.integrated_error + dt.as_secs_f32() * error;
        let accel_cmd =
            gains.p * error + gains.i * self.integrated_error + gains.d * (-estimated_velocity);

        self.previous_error = error;

        // Convert acceleration to roll/pitch angles (simplified)
        let roll_cmd = accel_cmd.y / G;
        let pitch_cmd = -accel_cmd.x / G;

        vec2(roll_cmd, pitch_cmd)
    }
}
