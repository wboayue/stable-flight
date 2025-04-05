use std::{f32::consts::PI, time::Duration};

use crate::stabilizers::Gains;

#[cfg(test)]
mod tests;

/// Configures the yaw controller.
/// Defines PID gains and max yaw rates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct YawControllerConfig {
    /// PID gains for roll and pitch control (Kp, Ki, Kd).
    pub gains: Gains,

    /// Maximum absolute value of the integral term.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,

    /// Maximum angular rates in radians/second for yaw.
    /// Limits the output rates sent to the rate controller.
    pub max_rate: f32,
}

impl Default for YawControllerConfig {
    fn default() -> Self {
        Self {
            gains: Default::default(),
            max_integral: 100.0, // Arbitrary limit
            max_rate: 2.0,    // 2 rad/s (~114°/s) for yaw
        }
    }
}

pub struct YawController {
    config: YawControllerConfig,

    previous_error: f32,
    integrated_error: f32,
}

impl YawController {
    /// Initializes a new yaw controller with the given configuration.
    pub fn new() -> Self {
        Self::with_config(&YawControllerConfig::default())
    }

    pub fn with_config(config: &YawControllerConfig) -> Self {
        Self {
            config: config.clone(),
            previous_error: 0.0,
            integrated_error: 0.0,
        }
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
    pub fn control(&mut self, dt: Duration, desired_yaw: f32, estimated_yaw: f32) -> f32 {
        let dt = dt.as_secs_f32();
        assert!(dt.is_finite(), "Time step must be finite");
        assert!(dt > 0.0, "Time step must be positive");
        assert!(
            desired_yaw >= -2.0 * PI && desired_yaw <= 2.0 * PI,
            "Desired yaw rate ({}) must be in the range [-2PI, 2PI]",
            desired_yaw
        );

        let gains = &self.config.gains;

        let error = angle_diff(desired_yaw, estimated_yaw); // Use a helper to handle angle wrapping
        let error_derivative = (error - self.previous_error) / dt;

        self.integrated_error += dt * error;
        self.integrated_error = self
            .integrated_error
            .clamp(-self.config.max_integral, self.config.max_integral);

        let yaw_rate =
            gains.p * error + gains.i * self.integrated_error + gains.d * error_derivative;

        self.previous_error = error;

        yaw_rate.clamp(-self.config.max_rate, self.config.max_rate)
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
