use std::time::Duration;

use robomath::{vec3, Vec3};

use super::super::Gains;

#[cfg(test)]
mod tests;

/// Configures the [BodyRateController] with PID gains and operational limits.
///
/// This structure defines the control parameters that determine how the
/// [BodyRateController] responds to body rate errors, including responsiveness, stability, and maximum rates.
#[derive(Clone, Debug, PartialEq)]
pub struct BodyRateControllerConfig {
    /// PID gains for controlling the angular rates (roll, pitch, yaw).
    pub gains: Gains,

    /// Maximum absolute value of the integral term.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,

    /// Maximum angular rates in radians/second for roll (x), pitch (y), and yaw (z).
    pub max_angular_rates: Vec3<f32>,
}

impl Default for BodyRateControllerConfig {
    fn default() -> Self {
        Self {
            gains: Gains {
                p: 7.0,
                i: 0.00,
                d: 0.01,
            },
            max_integral: 100.0,                    // Arbitrary limit
            max_angular_rates: vec3(3.0, 3.0, 3.0), // Max roll, pitch, yaw rates in rad/s
        }
    }
}

// x: 3.0, // max roll rate in rad/s (~172 deg/s)
// y: 3.0, // max pitch rate in rad/s (~172 deg/s)
// z: 3.0, // max yaw rate in rad/s (~86 deg/s)

/// A PID-based controller for managing angular rates (roll, pitch, yaw) of a vehicle.
///
/// This controller uses a PID algorithm to compute corrective moments based on desired and estimated
/// angular rates. It is designed as the innermost control loop, directly affecting motor commands.
///
/// # Units
/// - Time (`dt`) is in seconds.
/// - Desired and estimated rates are expected in rad/s.
/// - Maximum rates (`max_angular_rates`) are in rad/s.
/// - Output moments are normalized to the range `[-1, 1]`, intended to be scaled to physical units (e.g., N·m) downstream.
pub struct BodyRateController {
    /// PID gains for controlling the angular rates (roll, pitch, yaw).
    pub gains: Gains,

    /// Integrated error term for the PID controller.
    pub integral_term: Vec3<f32>,

    /// Maximum integral term for the PID controller.
    pub max_integral: f32,

    /// Last error value for computing the derivative term.
    pub last_error: Vec3<f32>,

    /// Maximum angular rates (roll, pitch, yaw) in rad/s.
    pub max_angular_rates: Vec3<f32>,
}

impl BodyRateController {
    pub fn new() -> Self {
        BodyRateController::with_config(&BodyRateControllerConfig::default())
    }

    pub fn with_config(config: &BodyRateControllerConfig) -> Self {
        assert!(
            config.max_angular_rates.is_finite(),
            "Maximum angular rates must be finite"
        );
        assert!(
            config.max_integral.is_finite(),
            "Maximum integral term must be finite"
        );
        Self {
            gains: config.gains,
            max_angular_rates: config.max_angular_rates,
            max_integral: config.max_integral,
            integral_term: Vec3::default(),
            last_error: Vec3::default(),
        }
    }

    /// Controls the angular velocity (body rates) of the vehicle using a PID controller.
    ///
    /// This method takes normalized desired rates (e.g., from a joystick), scales them to rad/s using
    /// `max_angular_rates`, and computes the error against the actual estimated rates (in rad/s). It then
    /// applies PID control to calculate corrective moments.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive and finite).
    /// * `desired_rates` - Desired angular rates (roll, pitch, yaw) as normalized values in `[-1, 1]`.
    /// * `estimated_rates` - Current measured angular rates (roll, pitch, yaw) in rad/s.
    ///
    /// # Returns
    ///
    /// Moments (roll, pitch, yaw) normalized to the range `[-1, 1]`. These can be scaled to physical units
    /// (e.g., N·m) based on the vehicle's dynamics.
    ///
    /// # Panics
    ///
    /// Panics if:
    /// - `dt` is not positive or finite.
    /// - Any component of `max_angular_rates` is zero or non-finite.
    /// - Any component of `desired_rates` or `estimated_rates` is non-finite.
    pub fn control(
        &mut self,
        dt: Duration,
        desired_rates: Vec3<f32>,
        estimated_rates: Vec3<f32>,
    ) -> Vec3<f32> {
        let dt = dt.as_secs_f32();

        // Input validation
        assert!(
            dt > 0.0 && dt.is_finite(),
            "Time step (dt) must be positive and finite"
        );
        assert!(
            desired_rates.is_finite() && estimated_rates.is_finite(),
            "Desired and estimated rates must be finite"
        );

        // println!(
        //     "{:?},{:?},{:?},{:?},{:?},{:?}",
        //     scaled_desired_rates.x,
        //     estimated_rates.x,
        //     scaled_desired_rates.y,
        //     estimated_rates.y,
        //     scaled_desired_rates.z,
        //     estimated_rates.z
        // );

        println!(
            "{:?},{:?},{:?},{:?},{:?}",
            0.0, estimated_rates.x, estimated_rates.y, estimated_rates.z, desired_rates.z,
        );

        let error = desired_rates - estimated_rates;

        let error_derivative = (error - self.last_error) / dt;
        self.last_error = error;

        self.integral_term = self.integral_term + dt * error;
        self.integral_term = self
            .integral_term
            .clamp(-self.max_integral, self.max_integral);

        let moment = self.gains.p * error
            + self.gains.d * error_derivative
            + self.gains.i * self.integral_term;

        let moment_magnitude = moment.magnitude();
        let max_moment_magnitude = self.max_moment(dt).magnitude();

        let scaled_moment = if moment_magnitude > 0.0 && max_moment_magnitude > 0.0 {
            let scale = (moment_magnitude / max_moment_magnitude).clamp(-1.0, 1.0);
            moment * (scale / moment_magnitude)
        } else {
            moment // Zero vector or zero max_moment_magnitude
        };

        (scaled_moment * 1000.0).clamp(-1.0, 1.0) // Scale to physical units (e.g., N·m)
    }

    /// Calculates maximum moments (roll, pitch, yaw) based on gains and maximum angular rates.
    fn max_moment(&self, dt: f32) -> Vec3<f32> {
        let max_error = 2.0 * self.max_angular_rates;
        let max_derivative = 4.0 * self.max_angular_rates / dt;

        let max_moment_x = (self.gains.p * max_error.x)
            + (self.gains.i * self.max_integral)
            + (self.gains.d * max_derivative.x);

        let max_moment_y = (self.gains.p * max_error.y)
            + (self.gains.i * self.max_integral)
            + (self.gains.d * max_derivative.y);

        let max_moment_z = (self.gains.p * max_error.z)
            + (self.gains.i * self.max_integral)
            + (self.gains.d * max_derivative.z);

        vec3(max_moment_x, max_moment_y, max_moment_z)
    }

    /// Resets the controller state (integral and last error terms) to zero.
    pub fn reset(&mut self) {
        self.integral_term = Vec3::default();
        self.last_error = Vec3::default();
    }
}
