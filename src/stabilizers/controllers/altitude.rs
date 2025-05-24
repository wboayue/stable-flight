use std::time::Duration;

use robomath::Quaternion;

use crate::stabilizers::Gains;

#[derive(Clone, Debug, PartialEq)]
pub struct AltitudeControllerConfig {
    pub gains: Gains,
    /// Maximum absolute value of the integral term.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,

    /// Maximum climb rate in meters/second.
    /// This is used to limit the output thrust command.
    pub max_rate: f32,
}

impl Default for AltitudeControllerConfig {
    fn default() -> Self {
        Self {
            gains: Gains {
                p: 1.0,
                i: 0.1,
                d: 0.05,
            },
            max_integral: 100.0, // Arbitrary limit
            max_rate: 2.0,       // 2 m/s for altitude
        }
    }
}

pub struct AltitudeController {
    config: AltitudeControllerConfig,
    previous_error: f32,
    integrated_error: f32,
}

impl AltitudeController {
    pub fn new() -> Self {
        AltitudeController::with_config(&AltitudeControllerConfig::default())
    }

    pub fn with_config(config: &AltitudeControllerConfig) -> Self {
        Self {
            config: config.clone(),
            previous_error: 0.0,
            integrated_error: 0.0,
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
    pub fn control(
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
        let gains = &self.config.gains;

        let altitude_error = desired_altitude - estimated_altitude;
        self.integrated_error += altitude_error * dt.as_secs_f32();
        self.integrated_error = self.integrated_error.clamp(-100.0, 100.0);

        let proportional_term = gains.p * altitude_error;
        let derivative_term = gains.d * (desired_velocity - estimated_velocity);
        let integral_term = gains.i * self.integrated_error;
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
}

#[cfg(test)]
mod tests {}
