use crate::{GRAVITY, Vec2};

use crate::controllers::Gains;
use crate::math::{cap_magnitude, clamp_each};

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
            max_integral: 100.0,           // Arbitrary limit
            max_rate: Vec2::new(2.0, 2.0), // 2 rad/s (~114°/s) for roll and pitch
        }
    }
}

pub struct LateralController {
    config: LateralControllerConfig,
    integrated_error: Vec2<f32>, // Accumulated error (roll, pitch)
    max_integral: f32,
}

impl Default for LateralController {
    fn default() -> Self {
        Self::new()
    }
}

impl LateralController {
    /// Creates a controller with [`LateralControllerConfig::default`].
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::LateralController;
    ///
    /// let controller = LateralController::new();
    /// ```
    pub fn new() -> Self {
        LateralController::with_config(&LateralControllerConfig::default())
    }

    /// Creates a controller from `config`.
    ///
    /// # Panics
    ///
    /// If `config.max_integral` is negative or not finite (it bounds the integral term each
    /// tick; rejecting it here keeps that clamp from panicking inside the flight loop).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::{Gains, LateralController, LateralControllerConfig};
    /// use stable_flight::Vec2;
    ///
    /// let config = LateralControllerConfig {
    ///     gains: Gains { p: 3.0, i: 0.3, d: 0.15 },
    ///     max_integral: 100.0,
    ///     max_rate: Vec2::new(2.0, 2.0),
    /// };
    ///
    /// let controller = LateralController::with_config(&config);
    /// ```
    pub fn with_config(config: &LateralControllerConfig) -> Self {
        assert!(
            config.max_integral >= 0.0 && config.max_integral.is_finite(),
            "max_integral must be non-negative and finite, got {}",
            config.max_integral
        );
        Self {
            config: config.clone(),
            integrated_error: Vec2::zeros(),
            max_integral: config.max_integral,
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
    /// Desired roll and pitch as normalized values (the attitude controller scales them by its
    /// `max_angles`). The length of the (roll, pitch) request is capped at 1, so far from the
    /// target the vehicle commands full tilt straight toward it: a far diagonal target gives
    /// about 0.71 on each axis, not 1. A non-finite error or velocity gives level. The integral
    /// term is clamped to `max_integral` to limit windup.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::LateralController;
    /// use stable_flight::Vec2;
    ///
    /// let mut controller = LateralController::new();
    /// let dt = 0.004;
    /// let desired_position = Vec2::new(5.0, 2.0); // 5 m north, 2 m east
    /// let estimated_position = Vec2::new(4.8, 1.9);
    /// let estimated_velocity = Vec2::new(0.1, 0.05);
    ///
    /// // Target is ahead and to the right: roll right (+x), nose down (-y)
    /// let tilt = controller.control(dt, desired_position, estimated_position, estimated_velocity);
    /// assert!(tilt.x > 0.0 && tilt.y < 0.0);
    /// assert!(tilt.norm() <= 1.0);
    /// ```
    pub fn control(
        &mut self,
        dt: f32,
        desired_position: Vec2<f32>,
        estimated_position: Vec2<f32>,
        estimated_velocity: Vec2<f32>,
    ) -> Vec2<f32> {
        let gains = &self.config.gains;

        let error = desired_position - estimated_position;

        // A non-finite error (NaN/∞ target or position) yields level (no tilt) and never enters
        // the integral, where NaN would stick (it survives `clamp`).
        if !(error.x.is_finite() && error.y.is_finite()) {
            return Vec2::zeros();
        }

        self.integrated_error = clamp_each(
            self.integrated_error + dt * error,
            -self.max_integral,
            self.max_integral,
        );
        let accel_cmd =
            gains.p * error + gains.i * self.integrated_error + gains.d * (-estimated_velocity);

        // Convert acceleration to roll/pitch angles (simplified)
        let roll_cmd = accel_cmd.y / GRAVITY;
        let pitch_cmd = -accel_cmd.x / GRAVITY;

        // Saturate the length, not each axis, so a far diagonal target keeps its direction.
        cap_magnitude(Vec2::new(roll_cmd, pitch_cmd), 1.0)
    }
}

#[cfg(test)]
mod tests {
    use crate::Vec2;

    use super::*;

    #[test]
    fn output_is_normalized_and_saturates_far_from_target() {
        // (target position, expected (roll, pitch)) with the vehicle at rest at the origin
        let cases = [
            (Vec2::zeros(), Vec2::zeros()),
            (Vec2::new(1000.0, 0.0), Vec2::new(0.0, -1.0)), // far +x: full nose-down pitch
            (Vec2::new(-1000.0, 0.0), Vec2::new(0.0, 1.0)),
            (Vec2::new(0.0, 1000.0), Vec2::new(1.0, 0.0)), // far +y: full right roll
            (Vec2::new(0.0, f32::NAN), Vec2::zeros()),
            (Vec2::new(f32::INFINITY, 0.0), Vec2::zeros()), // non-finite error: level, not full tilt
        ];
        for (target, expected) in cases {
            let mut controller = LateralController::new();
            let angles = controller.control(0.004, target, Vec2::zeros(), Vec2::zeros());
            assert!(
                angles == expected,
                "target {target:?} gave {angles:?}, expected {expected:?}"
            );
        }
    }

    /// A NaN or infinite velocity estimate gives level, not a NaN tilt.
    #[test]
    fn non_finite_velocity_yields_level() {
        let nan = f32::NAN;
        let inf = f32::INFINITY;
        let velocities = [
            Vec2::new(nan, 0.0),
            Vec2::new(0.0, inf),
            Vec2::new(-inf, nan),
        ];
        for velocity in velocities {
            let mut controller = LateralController::new();
            let angles = controller.control(0.004, Vec2::new(10.0, 5.0), Vec2::zeros(), velocity);
            assert!(
                angles == Vec2::zeros(),
                "velocity {velocity:?} gave {angles:?}"
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
            controller.control(0.01, Vec2::new(100.0, -100.0), Vec2::zeros(), Vec2::zeros());
        }

        assert!(controller.integrated_error == Vec2::new(5.0, -5.0));
    }

    #[test]
    fn recovers_full_tilt_after_a_nan_tick() {
        let mut controller = LateralController::new();
        let at_rest = Vec2::zeros();

        // One tick with a NaN target must not poison later ticks.
        let during = controller.control(0.004, Vec2::new(f32::NAN, 0.0), at_rest, at_rest);
        let after = controller.control(0.004, Vec2::new(1000.0, 0.0), at_rest, at_rest);

        assert!(during == Vec2::zeros(), "NaN tick gave {during:?}");
        assert!(
            after == Vec2::new(0.0, -1.0),
            "valid far target after NaN gave {after:?}"
        );
    }

    #[test]
    fn rejects_invalid_max_integral_at_construction() {
        for max_integral in [-1.0, f32::NAN, f32::INFINITY] {
            let result = std::panic::catch_unwind(|| {
                LateralController::with_config(&LateralControllerConfig {
                    max_integral,
                    ..Default::default()
                })
            });
            assert!(result.is_err(), "max_integral {max_integral} was accepted");
        }
    }
}
