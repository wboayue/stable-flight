use crate::{GRAVITY, Vec2};

use crate::controllers::{Gains, InvalidConfig, check_limit};
use crate::math::{cap_magnitude, clamp_symmetric_each};

/// Configures the [`LateralController`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LateralControllerConfig {
    /// PID gains for roll and pitch control (Kp, Ki, Kd).
    pub gains: Gains,

    /// Maximum absolute value of the integrated position error, in m·s, per axis.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,

    /// Maximum roll (x) and pitch (y) rates in radians/second. Currently unused: the
    /// controller outputs normalized tilt
    /// ([#71](https://github.com/wboayue/stable-flight/issues/71)).
    pub max_rate_radps: Vec2,
}

impl Default for LateralControllerConfig {
    /// Untuned starting values; airframes override them with flight-checked sets.
    fn default() -> Self {
        Self {
            gains: Gains {
                p: 3.0,
                i: 0.3,
                d: 0.15,
            },
            max_integral: 100.0,                 // Arbitrary limit
            max_rate_radps: Vec2::new(2.0, 2.0), // 2 rad/s (~114°/s) for roll and pitch
        }
    }
}

impl LateralControllerConfig {
    /// Checks the config: `max_integral` must be non-negative and finite.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::LateralControllerConfig;
    ///
    /// assert!(LateralControllerConfig::default().validate().is_ok());
    /// let bad = LateralControllerConfig { max_integral: f32::NAN, ..Default::default() };
    /// assert!(bad.validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_limit("max_integral", self.max_integral)
    }
}

/// Horizontal position PID loop: turns a position target into normalized roll/pitch.
///
/// It holds only state (the integral); gains and limits come from the
/// [`LateralControllerConfig`] passed to each [`control`](Self::control) call, so they can
/// change between ticks without resetting the state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LateralController {
    integrated_error: Vec2, // Accumulated error (x, y)
}

impl LateralController {
    /// Controls the lateral (X-Y) position of the vehicle using a PID controller.
    ///
    /// This outer loop maintains the desired horizontal position by calculating desired roll and pitch angles.
    ///
    /// # Arguments
    ///
    /// * `config` - Gains and limits; assumed valid ([`LateralControllerConfig::validate`]).
    ///   An invalid limit does not panic: a negative or NaN `max_integral` acts as 0.
    /// * `dt` - Time step since the last update (must be positive).
    /// * `desired_position_m` - Target position (x, y) in meters.
    /// * `estimated_position_m` - Current position (x, y) in meters.
    /// * `estimated_velocity_mps` - Current velocity (vx, vy) in m/s.
    ///
    /// # Returns
    ///
    /// Desired roll and pitch as normalized values (the attitude controller scales them by its
    /// `max_angles_rad`). The length of the (roll, pitch) request is capped at 1, so far from the
    /// target the vehicle commands full tilt straight toward it: a far diagonal target gives
    /// about 0.71 on each axis, not 1. A non-finite error or velocity gives level. The integral
    /// term is clamped to `max_integral` to limit windup.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::{LateralController, LateralControllerConfig};
    /// use stable_flight::Vec2;
    ///
    /// let config = LateralControllerConfig::default();
    /// let mut controller = LateralController::default();
    /// let dt = 0.004;
    /// let desired_position_m = Vec2::new(5.0, 2.0); // 5 m north, 2 m east
    /// let estimated_position_m = Vec2::new(4.8, 1.9);
    /// let estimated_velocity_mps = Vec2::new(0.1, 0.05);
    ///
    /// // Target is ahead and to the right: roll right (+x), nose down (-y)
    /// let tilt = controller.control(&config, dt, desired_position_m, estimated_position_m, estimated_velocity_mps);
    /// assert!(tilt.x > 0.0 && tilt.y < 0.0);
    /// assert!(tilt.norm() <= 1.0);
    /// ```
    pub fn control(
        &mut self,
        config: &LateralControllerConfig,
        dt: f32,
        desired_position_m: Vec2,
        estimated_position_m: Vec2,
        estimated_velocity_mps: Vec2,
    ) -> Vec2 {
        let gains = &config.gains;

        let error = desired_position_m - estimated_position_m;

        // A non-finite error (NaN/∞ target or position) yields level (no tilt) and never enters
        // the integral, where NaN would stick (it survives clamping).
        if !(error.x.is_finite() && error.y.is_finite()) {
            return Vec2::zeros();
        }

        self.integrated_error =
            clamp_symmetric_each(self.integrated_error + dt * error, config.max_integral);
        let accel_cmd =
            gains.p * error + gains.i * self.integrated_error + gains.d * (-estimated_velocity_mps);

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
        let config = LateralControllerConfig::default();
        for (target, expected) in cases {
            let mut controller = LateralController::default();
            let angles = controller.control(&config, 0.004, target, Vec2::zeros(), Vec2::zeros());
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
        let config = LateralControllerConfig::default();
        for velocity in velocities {
            let mut controller = LateralController::default();
            let angles = controller.control(
                &config,
                0.004,
                Vec2::new(10.0, 5.0),
                Vec2::zeros(),
                velocity,
            );
            assert!(
                angles == Vec2::zeros(),
                "velocity {velocity:?} gave {angles:?}"
            );
        }
    }

    #[test]
    fn integral_term_is_clamped_to_max_integral() {
        let config = LateralControllerConfig {
            max_integral: 5.0,
            ..Default::default()
        };
        let mut controller = LateralController::default();

        // A large, persistent error for many ticks would wind the integral far past the limit.
        for _ in 0..10_000 {
            controller.control(
                &config,
                0.01,
                Vec2::new(100.0, -100.0),
                Vec2::zeros(),
                Vec2::zeros(),
            );
        }

        assert!(controller.integrated_error == Vec2::new(5.0, -5.0));
    }

    #[test]
    fn recovers_full_tilt_after_a_nan_tick() {
        let config = LateralControllerConfig::default();
        let mut controller = LateralController::default();
        let at_rest = Vec2::zeros();

        // One tick with a NaN target must not poison later ticks.
        let during = controller.control(&config, 0.004, Vec2::new(f32::NAN, 0.0), at_rest, at_rest);
        let after = controller.control(&config, 0.004, Vec2::new(1000.0, 0.0), at_rest, at_rest);

        assert!(during == Vec2::zeros(), "NaN tick gave {during:?}");
        assert!(
            after == Vec2::new(0.0, -1.0),
            "valid far target after NaN gave {after:?}"
        );
    }

    /// An invalid `max_integral` fails `validate` but `control` stays panic-free: it acts as 0.
    #[test]
    fn invalid_max_integral_is_rejected_and_does_not_panic_the_tick() {
        for max_integral in [-1.0, f32::NAN, f32::INFINITY] {
            let config = LateralControllerConfig {
                max_integral,
                ..Default::default()
            };
            assert!(config.validate().is_err(), "max_integral {max_integral}");
        }
        for max_integral in [-1.0, f32::NAN] {
            let config = LateralControllerConfig {
                max_integral,
                ..Default::default()
            };
            let mut controller = LateralController::default();
            controller.control(
                &config,
                0.01,
                Vec2::new(1.0, 1.0),
                Vec2::zeros(),
                Vec2::zeros(),
            );
            assert!(
                controller.integrated_error == Vec2::zeros(),
                "max_integral {max_integral}"
            );
        }
    }

    /// A gain change between ticks applies on the next tick and keeps the integral.
    #[test]
    fn gain_change_between_ticks_keeps_state() {
        let config = LateralControllerConfig::default();
        let mut controller = LateralController::default();
        controller.control(
            &config,
            1.0,
            Vec2::new(0.0, 1.0),
            Vec2::zeros(),
            Vec2::zeros(),
        );
        let integral = controller.integrated_error;

        let retuned = LateralControllerConfig {
            gains: Gains {
                p: 0.0,
                i: 2.0,
                d: 0.0,
            },
            ..config
        };
        let tilt = controller.control(&retuned, 0.01, Vec2::zeros(), Vec2::zeros(), Vec2::zeros());

        assert!(controller.integrated_error == integral);
        approx::assert_relative_eq!(tilt.x, 2.0 * integral.y / GRAVITY, epsilon = 1e-6);
    }
}
