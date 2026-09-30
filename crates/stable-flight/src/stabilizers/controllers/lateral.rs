use robomath::{Vec2, vec2};

use crate::stabilizers::{Gains, saturate_unit};

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

impl Default for LateralController {
    fn default() -> Self {
        Self::new()
    }
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
    /// Desired roll and pitch as normalized values in [-1, 1] (the attitude controller scales
    /// them by its `max_angles`). The acceleration demand saturates, so far from the target the
    /// vehicle commands full tilt toward it rather than an out-of-range angle. The integral term
    /// is clamped to `max_integral` to limit windup.
    pub fn control(
        &mut self,
        dt: f32,
        desired_position: Vec2<f32>,
        estimated_position: Vec2<f32>,
        estimated_velocity: Vec2<f32>,
    ) -> Vec2<f32> {
        let gains = &self.config.gains;

        let error = desired_position - estimated_position;
        let error_derivative = (error - self.previous_error) / dt;

        self.integrated_error =
            (self.integrated_error + dt * error).clamp(-self.max_integral, self.max_integral);
        let accel_cmd =
            gains.p * error + gains.i * self.integrated_error + gains.d * (-estimated_velocity);

        self.previous_error = error;

        // Convert acceleration to roll/pitch angles (simplified)
        let roll_cmd = accel_cmd.y / G;
        let pitch_cmd = -accel_cmd.x / G;

        vec2(saturate_unit(roll_cmd), saturate_unit(pitch_cmd))
    }
}

#[cfg(test)]
mod tests {
    use robomath::vec2;

    use super::*;

    #[test]
    fn output_is_normalized_and_saturates_far_from_target() {
        // (target position, expected (roll, pitch)) with the vehicle at rest at the origin
        let cases = [
            (vec2(0.0, 0.0), vec2(0.0, 0.0)),
            (vec2(1000.0, 0.0), vec2(0.0, -1.0)), // far +x: full nose-down pitch
            (vec2(-1000.0, 0.0), vec2(0.0, 1.0)),
            (vec2(0.0, 1000.0), vec2(1.0, 0.0)), // far +y: full right roll
            (vec2(0.0, f32::NAN), vec2(0.0, 0.0)),
        ];
        for (target, expected) in cases {
            let mut controller = LateralController::new();
            let angles = controller.control(0.004, target, vec2(0.0, 0.0), vec2(0.0, 0.0));
            assert!(
                angles == expected,
                "target {target:?} gave {angles:?}, expected {expected:?}"
            );
        }
    }

    #[test]
    fn integral_term_is_clamped_to_max_integral() {
        let mut controller = LateralController::with_config(&LateralControllerConfig {
            max_integral: 5.0,
            ..Default::default()
        });

        // A large, persistent error for many ticks would wind the integral far past the limit.
        for _ in 0..10_000 {
            controller.control(0.01, vec2(100.0, -100.0), vec2(0.0, 0.0), vec2(0.0, 0.0));
        }

        assert!(controller.integrated_error == vec2(5.0, -5.0));
    }
}
