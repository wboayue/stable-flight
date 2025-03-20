use std::time::Duration;

use robomath::Vec2;

use super::basic::Gains;

/// Configures the attitude controller for a vehicle.
/// This includes the maximum roll and pitch angles that the vehicle can achieve.
/// The yaw angle is managed by the yaw controller.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeConfiguration {
    /// PID gains for controlling roll and pitch.
    pub gains: Gains,

    /// Maximum roll angle in radians.
    /// This limits how far the vehicle can tilt to the side.
    pub max_roll: f32,

    /// Maximum pitch angle in radians.
    /// This limits how far the vehicle can tilt forward or backward.
    pub max_pitch: f32,

    /// Maximum integral term for the PID controller.
    /// This prevents excessive accumulation of the integral term, which can lead to instability.
    pub max_integral: f32,
}

impl Default for AttitudeConfiguration {
    /// Creates a default configuration for the attitude controller.
    fn default() -> Self {
        Self {
            gains: Gains {
                p: 7.0,
                i: 0.0,
                d: 0.01,
            },
            max_roll: std::f32::consts::FRAC_PI_4, // 45 degrees
            max_pitch: std::f32::consts::FRAC_PI_4, // 45 degrees
            max_integral: 100.0,
        }
    }
}

pub struct AttitudeController {
    /// PID gains for controlling the roll and pitch.
    gains: Gains,

    /// Integrated error term for the PID controller.
    integral_term: Vec2<f32>,

    /// Maximum integral term for the PID controller.
    max_integral: f32,

    /// Last error value for computing the derivative term.
    last_error: Vec2<f32>,

    /// Maximum roll and pitch in radians.
    max_roll_pitch: Vec2<f32>,
}

impl AttitudeController {

    /// Creates a new `AttitudeController` with default configuration.
    pub fn new(configuration: AttitudeConfiguration) -> Self {
        Self {
            gains: configuration.gains,
            integral_term: Vec2::default(),
            max_integral: configuration.max_integral,
            last_error: Vec2::default(),
            max_roll_pitch: Vec2 {
                x: configuration.max_roll,
                y: configuration.max_pitch,
            },
        }
    }

    /// Controls the roll and pitch angles of the vehicle using a PID controller.
    ///
    /// This middle control loop converts desired roll and pitch angles into desired angular rates for the rate controller.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive).
    /// * `desired_angles` - Scaled roll/pitch angles in range [-1, 1]. indicating min/max configured roll/pitch
    /// * `current_angles` - Current roll and pitch angles in radians.
    ///
    /// # Returns
    ///
    /// Desired angular rates (roll rate, pitch rate) in rad/s.
    fn control(
        &mut self,
        dt: Duration,
        desired_angles: Vec2<f32>,
        estimated_angles: Vec2<f32>,
    ) -> Vec2<f32> {
        assert!(dt.as_secs_f32() > 0.0, "Time step must be positive");
        assert!(
            desired_angles.x >= -1.0 && desired_angles.x <= 1.0,
            "Desired roll angle ({}) must be in the range [-1, 1]",
            desired_angles.x
        );
        assert!(
            desired_angles.y >= -1.0 && desired_angles.y <= 1.0,
            "Desired pitch angle ({}) must be in the range [-1, 1]",
            desired_angles.y
        );

        let desired_angles = desired_angles * self.max_roll_pitch; // Scale to radians

        let error = desired_angles - estimated_angles;
        let error_derivative = (error - self.last_error) / dt.as_secs_f32();

        self.integral_term = self.integral_term + dt.as_secs_f32() * error;
        self.integral_term = self.integral_term.clamp(-self.max_integral, self.max_integral);

        let Gains { p, i, d } = self.gains;
        let rates =
            p * error + i * self.integral_term + d * error_derivative;

        self.last_error = error;

        rates.clamp(-1.0, 1.0)
    }

}

