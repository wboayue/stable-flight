use std::{f32::consts::FRAC_PI_4, time::Duration};

use robomath::{vec2, Vec2};

use super::super::Gains;

#[cfg(test)]
mod tests;

/// Configures the attitude controller for a vehicle.
/// Defines PID gains, maximum angles, and maximum angular rates for roll and pitch control.
/// Yaw is typically handled separately by a dedicated yaw controller.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeControllerConfig {
    /// PID gains for roll and pitch control (Kp, Ki, Kd).
    pub gains: Gains,

    /// Maximum angles in radians for roll (x) and pitch (y) (e.g., π/4 = 45°).
    /// Limits the vehicle's tilt in each axis.
    pub max_angles: Vec2<f32>,

    /// Maximum absolute value of the integral term.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,

    /// Maximum angular rates in radians/second for roll (x) and pitch (y).
    /// Limits the output rates sent to the rate controller.
    pub max_rate: Vec2<f32>,
}

impl Default for AttitudeControllerConfig {
    /// Provides a default configuration with reasonable starting values.
    /// Note: PID gains (P=7.0, I=0.0, D=0.01) and max_rate (2 rad/s) may need tuning
    /// for specific vehicles.
    fn default() -> Self {
        Self {
            gains: Gains {
                p: 3.0,   // Proportional gain
                i: 0.001, // Integral gain (disabled by default)
                d: 1.0,   // Derivative gain
            },
            max_angles: vec2(FRAC_PI_4, FRAC_PI_4), // 45° for roll and pitch
            max_integral: 100.0,                    // Arbitrary limit
            max_rate: vec2(2.0, 2.0),               // 2 rad/s (~114°/s) for roll and pitch
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct AttitudeController {
    gains: Gains,
    integral_term: Vec2<f32>, // Accumulated error (roll, pitch)
    max_integral: f32,
    last_error: Vec2<f32>, // Previous error for derivative calculation
    max_angles: Vec2<f32>, // Max angles in radians (roll, pitch)
    max_rate: Vec2<f32>,   // Max angular rates in rad/s (roll, pitch)
}

impl Default for AttitudeController {
    /// Provides a default attitude controller with default configuration.
    /// Note: PID gains (P=7.0, I=0.0, D=0.01) and max_rate (2 rad/s) may need tuning
    /// for specific vehicles.
    fn default() -> Self {
        Self::with_config(&AttitudeControllerConfig::default())
    }
}

impl AttitudeController {
    /// Initializes a new attitude controller with the given configuration.
    pub fn new() -> Self {
        Self::default()
    }

    /// Initializes a new attitude controller with the given configuration.
    pub fn with_config(config: &AttitudeControllerConfig) -> Self {
        assert!(
            config.max_angles.x >= 0.0 && config.max_angles.y >= 0.0,
            "Maximum angles must be non-negative"
        );
        assert!(
            config.max_rate.x >= 0.0 && config.max_rate.y >= 0.0,
            "Maximum rates must be non-negative"
        );
        Self {
            gains: config.gains,
            integral_term: Vec2::default(),
            max_integral: config.max_integral,
            last_error: Vec2::default(),
            max_angles: config.max_angles,
            max_rate: config.max_rate,
        }
    }

    /// Computes desired angular rates to achieve target roll and pitch angles.
    ///
    /// Uses a PID controller to convert angle errors into angular rates (rad/s)
    /// for the rate control loop. Input angles are scaled from [-1,1] to
    /// [-max_angles.x, max_angles.x] for roll and [-max_angles.y, max_angles.y] for pitch.
    /// Output rates are clamped to [-max_rate.x, max_rate.x] for roll and
    /// [-max_rate.y, max_rate.y] for pitch.
    ///
    /// # Arguments
    /// * `dt` - Time step since last update (seconds, must be positive)
    /// * `desired_angles` - Target roll/pitch as normalized values in [-1,1]
    /// * `estimated_angles` - Current roll/pitch angles in radians
    ///
    /// # Returns
    /// Desired angular rates (roll rate, pitch rate) in radians/second,
    /// clamped to configured max_rate limits
    ///
    /// # Panics
    /// * If `dt` is not positive
    /// * If `desired_angles` components are not in [-1,1]
    pub fn control(
        &mut self,
        dt: Duration,
        desired_angles: Vec2<f32>,
        estimated_angles: Vec2<f32>,
    ) -> Vec2<f32> {
        assert!(dt.as_secs_f32() > 0.0, "Time step must be positive");
        assert!(
            desired_angles.x >= -1.0 && desired_angles.x <= 1.0,
            "Desired roll angle ({}) must be in [-1, 1]",
            desired_angles.x
        );
        assert!(
            desired_angles.y >= -1.0 && desired_angles.y <= 1.0,
            "Desired pitch angle ({}) must be in [-1, 1]",
            desired_angles.y
        );

        // Scale normalized inputs to actual angle setpoints in radians
        let desired_angles = desired_angles * self.max_angles;

        // Calculate error (setpoint - actual)
        let error = desired_angles - estimated_angles;

        // Derivative term: rate of change of error
        let error_derivative = (error - self.last_error) / dt.as_secs_f32();

        // Integral term with anti-windup
        self.integral_term = (self.integral_term + dt.as_secs_f32() * error)
            .clamp(-self.max_integral, self.max_integral);

        // PID calculation: Kp×error + Ki×∫error + Kd×d(error)/dt
        let Gains { p, i, d } = self.gains;
        let rates = p * error + i * self.integral_term + d * error_derivative;

        self.last_error = error;

        // Scale output to configured maximum rates
        vec2(
            rates.x.clamp(-self.max_rate.x, self.max_rate.x),
            rates.y.clamp(-self.max_rate.y, self.max_rate.y),
        )
    }
}
