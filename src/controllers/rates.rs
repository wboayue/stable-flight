use std::time::Duration;

use crate::math::Vec3;

use super::basic::Gains;

pub struct RatesController {
    /// PID gains for controlling the angular rates (roll, pitch, yaw).
    pub gains: Gains,

    /// Integrated error for the rates controller.
    pub integrated_error: Vec3<f32>,

    /// Previous error for the rates controller.
    pub previous_error: Vec3<f32>,

    pub max_rates: Vec3<f32>,
}

impl Default for RatesController {
    /// Creates a new `RatesController` with default PID gains and max rates.
    fn default() -> Self {
        Self {
            gains: Gains {
                p: 2.0,
                i: 0.0,
                d: 0.0,
            },
            max_rates: Vec3 {
                x: 3.0, // max roll rate in rad/s ~172 deg/s
                y: 3.0, // max pitch rate in rad/s ~172 deg/s
                z: 1.5, // max yaw rate in rad/s ~86 deg/s
            },
            integrated_error: Vec3::default(),
            previous_error: Vec3::default(),
        }
    }
}

impl RatesController {
    /// Controls the angular velocity (body rates) of the vehicle using a PID controller.
    ///
    /// This is the innermost control loop that directly affects motor commands. It takes desired angular rates
    /// and current measured angular rates as inputs, then uses PID control to calculate motor moments.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive).
    /// * `desired_rates` - Desired angular rates (roll, pitch, yaw) in rad/s.
    /// * `estimated_rates` - Current measured angular rates (roll, pitch, yaw) in rad/s.
    ///
    /// # Returns
    ///
    /// Moments (roll, pitch, yaw) in N·m to achieve the desired rates.
    pub fn control(
        &mut self,
        dt: Duration,
        desired_rates: Vec3<f32>,
        estimated_rates: Vec3<f32>,
    ) -> Vec3<f32> {
        let dt = dt.as_secs_f32();

        // Ensure dt is positive and desired rates are within valid range
        assert!(dt > 0.0, "Time step (dt) must be positive");
        assert!(
            desired_rates.x >= -1.0 && desired_rates.x <= 1.0,
            "Desired roll rate ({}) must be in the range [-1, 1]",
            desired_rates.x
        );
        assert!(
            desired_rates.y >= -1.0 && desired_rates.y <= 1.0,
            "Desired pitch rate ({}) must be in the range [-1, 1]",
            desired_rates.y
        );
        assert!(
            desired_rates.z >= -1.0 && desired_rates.z <= 1.0,
            "Desired yaw rate ({}) must be in the range [-1, 1]",
            desired_rates.z
        );

        let mut estimated_rates = estimated_rates / self.max_rates;
        estimated_rates = estimated_rates.clamp(-1.0, 1.0);

        let error = desired_rates - estimated_rates;
        let error_derivative = (error - self.previous_error) / dt;

        self.integrated_error = self.integrated_error + dt * error;
        self.integrated_error = self.integrated_error.clamp(-100.0, 100.0);

        let moment = self.gains.p * error
            + self.gains.i * self.integrated_error
            + self.gains.d * error_derivative;

        self.previous_error = error;

        moment.clamp(-1.0, 1.0)
    }
}
