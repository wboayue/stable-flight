use std::{f32::consts::PI, time::Duration};

use crate::stabilizers::Gains;

#[cfg(test)]
mod tests;

/// Configures the yaw controller with PID gains and operational limits.
///
/// This structure defines the control parameters that determine how the yaw controller
/// responds to heading errors, including responsiveness, stability, and maximum rates.
#[derive(Clone, Debug, PartialEq)]
pub struct YawControllerConfig {
    /// PID gains that determine controller responsiveness.
    ///
    /// * `p` - Proportional gain: controls response strength relative to current error.
    /// * `i` - Integral gain: eliminates steady-state errors over time.
    /// * `d` - Derivative gain: provides damping to reduce overshoot and oscillation.
    pub gains: Gains,

    /// Maximum absolute value of the integral term in rad.
    ///
    /// Prevents integral windup by limiting accumulated error, which
    /// could otherwise cause significant overshoot and control instability.
    pub max_integral: f32,

    /// Maximum angular rate in radians/second for yaw.
    ///
    /// Limits the output commands to physically achievable rates and
    /// prevents excessive control demands that could destabilize the system.
    pub max_rate: f32,
}

impl Default for YawControllerConfig {
    fn default() -> Self {
        Self {
            gains: Default::default(),
            max_integral: 100.0, // Arbitrary limit
            max_rate: 2.0,       // 2 rad/s (~114°/s) for yaw
        }
    }
}

/// Yaw controller that maintains a desired heading using PID control with noise filtering.
///
/// This controller:
/// * Takes desired and actual heading measurements
/// * Calculates the shortest path error accounting for angle wrapping
/// * Applies filtered PID control to generate appropriate yaw rate commands
/// * Incorporates anti-windup protection and output limiting
pub struct YawController {
    config: YawControllerConfig,

    previous_error: Option<f32>,
    integrated_error: f32,
}

impl YawController {
    /// Creates a new yaw controller with default configuration.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight_sitl::stabilizers::controllers::{YawController, YawControllerConfig};
    ///
    /// let controller = YawController::new();
    /// ```
    pub fn new() -> Self {
        Self::with_config(&YawControllerConfig::default())
    }

    /// Creates a new yaw controller with the specified configuration.
    ///
    /// # Arguments
    ///
    /// * `config` - Custom controller configuration settings
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight_sitl::stabilizers::controllers::{YawController, YawControllerConfig};
    /// use stable_flight_sitl::stabilizers::Gains;
    ///
    /// let config = YawControllerConfig {
    ///     gains: Gains { p: 2.0, i: 0.1, d: 0.5 },
    ///     max_integral: 50.0,
    ///     max_rate: 1.5,
    /// };
    ///
    /// let controller = YawController::with_config(&config);
    /// ```
    pub fn with_config(config: &YawControllerConfig) -> Self {
        Self {
            config: config.clone(),
            previous_error: None,
            integrated_error: 0.0,
        }
    }

    /// Calculates desired yaw rate based on current and target headings.
    ///
    /// This implementation:
    /// 1. Calculates the shortest-path heading error
    /// 2. Updates the integral term with anti-windup protection
    /// 3. Applies the PID formula
    /// 5. Limits the output rate to configured maximum values
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive).
    /// * `desired_yaw` - Target yaw angle in radians [-2π, 2π].
    /// * `estimated_yaw` - Current measured yaw angle in radians.
    ///
    /// # Returns
    ///
    /// Desired yaw rate in rad/s, limited to configured maximum rates.
    ///
    /// # Panics
    ///
    /// Panics if:
    /// * Time step is not finite or not positive
    /// * Desired yaw is outside the range [-2π, 2π]
    ///
    /// # Examples
    ///
    /// ```
    /// use std::{f32::consts::PI, time::Duration};
    /// use stable_flight_sitl::stabilizers::controllers::{YawController, YawControllerConfig};
    ///
    /// let mut controller = YawController::new();
    /// let dt = Duration::from_millis(20);
    /// let desired_yaw = PI / 4.0;  // 45 degrees
    /// let current_yaw = 0.0;       // 0 degrees
    ///
    /// let yaw_rate = controller.control(dt, desired_yaw, current_yaw);
    /// // Returns a rate command to turn toward the desired heading
    /// ```
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
        let error_derivative = if let Some(previous_error) = self.previous_error {
            (error - previous_error) / dt
        } else {
            0.0
        };

        self.integrated_error += dt * error;
        self.integrated_error = self
            .integrated_error
            .clamp(-self.config.max_integral, self.config.max_integral);

        let yaw_rate =
            gains.p * error + gains.i * self.integrated_error + gains.d * error_derivative;

        self.previous_error = Some(error);

        yaw_rate.clamp(-self.config.max_rate, self.config.max_rate)
    }

    /// Resets the controller state (integral and last error terms) to zero.
    pub fn reset(&mut self) {
        self.integrated_error = 0.0;
        self.previous_error = None;
    }
}

/// Normalizes the yaw error to the range [-π, π] to handle angle wrapping.
///
/// This function ensures the controller follows the shortest angular path by:
/// 1. Computing the raw angular difference
/// 2. Normalizing to a [-π, π] range using modulo arithmetic
///
/// # Arguments
///
/// * `desired_yaw` - Desired yaw angle in radians.
/// * `current_yaw` - Current yaw angle in radians.
///
/// # Returns
///
/// The shortest-path yaw error in radians, in the range [-π, π].
fn angle_diff(desired_yaw: f32, current_yaw: f32) -> f32 {
    let mut yaw_error = desired_yaw - current_yaw;
    yaw_error += PI;
    yaw_error = yaw_error.rem_euclid(2.0 * PI);
    yaw_error -= PI;
    yaw_error
}
