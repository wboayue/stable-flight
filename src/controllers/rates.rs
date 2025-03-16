use std::time::Duration;

use crate::math::{vec3, Vec3};

use super::basic::Gains;

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
pub struct RatesController {
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

impl Default for RatesController {
    /// Creates a new `RatesController` with default PID gains and maximum angular rates.
    fn default() -> Self {
        Self {
            gains: Gains {
                p: 1000.0,
                i: 50.0,
                d: 10.0,
            },
            max_angular_rates: Vec3 {
                x: 172.0, // max roll rate in deg/s)
                y: 172.0, // max pitch rate in deg/s)
                z: 86.0,  // max yaw rate in deg/s)
            },
            // max_angular_rates: Vec3 {
            //     x: 3.0, // max roll rate in rad/s (~172 deg/s)
            //     y: 3.0, // max pitch rate in rad/s (~172 deg/s)
            //     z: 1.5, // max yaw rate in rad/s (~86 deg/s)
            // },
            integral_term: Vec3::default(),
            max_integral: 100.0,
            last_error: Vec3::default(),
        }
    }
}

impl RatesController {
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
            self.max_angular_rates.x != 0.0 && self.max_angular_rates.x.is_finite(),
            "max_angular_rates.x must be non-zero and finite"
        );
        assert!(
            self.max_angular_rates.y != 0.0 && self.max_angular_rates.y.is_finite(),
            "max_angular_rates.y must be non-zero and finite"
        );
        assert!(
            self.max_angular_rates.z != 0.0 && self.max_angular_rates.z.is_finite(),
            "max_angular_rates.z must be non-zero and finite"
        );
        assert!(
            desired_rates.is_finite() && estimated_rates.is_finite(),
            "Desired and estimated rates must be finite"
        );

        let desired_rates = desired_rates.clamp(-1.0, 1.0);
        let scaled_desired_rates = desired_rates * self.max_angular_rates;

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
            "{:?},{:?},{:?},{:?}",
            0.0,
            estimated_rates.x,
            estimated_rates.y,
            estimated_rates.z
        );

        let error = scaled_desired_rates - estimated_rates;

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
