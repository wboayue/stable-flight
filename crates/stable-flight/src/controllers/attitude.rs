use core::f32::consts::FRAC_PI_4;

use crate::Vec2;

use super::derivative::DerivativeState;
use super::{Gains, InvalidConfig, check_limit};
use crate::math::{
    assert_dt, clamp_symmetric, clamp_symmetric_each, limit_or_zero, saturate_unit_vec2,
};

/// Configures the attitude controller for a vehicle.
/// Defines PID gains, maximum angles, and maximum angular rates for roll and pitch control.
/// Yaw is typically handled separately by a dedicated yaw controller.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeControllerConfig {
    /// PID gains for roll and pitch control (Kp, Ki, Kd).
    pub gains: Gains,

    /// Maximum angles in radians for roll (x) and pitch (y) (e.g., π/4 = 45°).
    /// Limits the vehicle's tilt in each axis.
    pub max_angles_rad: Vec2,

    /// Maximum absolute value of the integrated angle error, in rad·s, per axis.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,

    /// Maximum angular rates in radians/second for roll (x) and pitch (y).
    /// Limits the output rates sent to the rate controller.
    pub max_rate_radps: Vec2,
}

impl Default for AttitudeControllerConfig {
    /// Untuned starting values (45° tilt, 2 rad/s rate cap); airframes override them with
    /// flight-checked sets.
    fn default() -> Self {
        Self {
            gains: Gains {
                p: 3.0,
                i: 0.001,
                d: 1.0,
            },
            max_angles_rad: Vec2::new(FRAC_PI_4, FRAC_PI_4), // 45° for roll and pitch
            max_integral: 100.0,
            max_rate_radps: Vec2::new(2.0, 2.0), // 2 rad/s (~114°/s) for roll and pitch
        }
    }
}

impl AttitudeControllerConfig {
    /// Checks the config: `max_angles_rad`, `max_rate_radps`, and `max_integral` must be
    /// non-negative and finite.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::AttitudeControllerConfig;
    /// use stable_flight::Vec2;
    ///
    /// assert!(AttitudeControllerConfig::default().validate().is_ok());
    /// let bad = AttitudeControllerConfig {
    ///     max_angles_rad: Vec2::new(-1.0, 0.5),
    ///     ..Default::default()
    /// };
    /// assert!(bad.validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_limit("max_angles_rad.x", self.max_angles_rad.x)?;
        check_limit("max_angles_rad.y", self.max_angles_rad.y)?;
        check_limit("max_rate_radps.x", self.max_rate_radps.x)?;
        check_limit("max_rate_radps.y", self.max_rate_radps.y)?;
        check_limit("max_integral", self.max_integral)
    }
}

/// Roll/pitch angle PID loop: turns normalized tilt setpoints into body-rate setpoints for the
/// [`BodyRateController`](super::BodyRateController).
///
/// It holds only state (integral and derivative history); gains and limits come from the
/// [`AttitudeControllerConfig`] passed to each [`control`](Self::control) call, so they can
/// change between ticks without resetting the state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct AttitudeController {
    integrated_error: Vec2, // Accumulated error (roll, pitch)
    /// Previous error for the derivative term; empty on the first tick and after a gap.
    derivative: DerivativeState<Vec2>,
}

impl AttitudeController {
    /// Computes desired angular rates to achieve target roll and pitch angles.
    ///
    /// Uses a PID controller to convert angle errors into angular rates (rad/s)
    /// for the rate control loop. Input angles are scaled from [-1,1] to
    /// [-max_angles_rad.x, max_angles_rad.x] for roll and [-max_angles_rad.y, max_angles_rad.y] for pitch.
    /// Output rates are clamped to [-max_rate_radps.x, max_rate_radps.x] for roll and
    /// [-max_rate_radps.y, max_rate_radps.y] for pitch.
    ///
    /// # Arguments
    /// * `config` - Gains and limits; assumed valid ([`AttitudeControllerConfig::validate`]).
    ///   An invalid limit does not panic: a negative or NaN limit acts as 0.
    /// * `dt` - Time step since the last update in seconds (must be positive and finite).
    /// * `desired_angles` - Target roll/pitch as normalized values in [-1,1]. Values outside
    ///   the range saturate to it (so the setpoint never exceeds `max_angles_rad`), and NaN is
    ///   treated as 0 (level).
    /// * `estimated_angles_rad` - Current roll/pitch angles in radians
    ///
    /// # Returns
    /// Desired angular rates (roll rate, pitch rate) in radians/second,
    /// clamped to configured max_rate_radps limits
    ///
    /// # Panics
    ///
    /// If `dt` is not positive and finite.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::{AttitudeController, AttitudeControllerConfig};
    /// use stable_flight::Vec2;
    ///
    /// let config = AttitudeControllerConfig::default();
    /// let mut controller = AttitudeController::default();
    /// let dt = 0.004;
    /// let desired_angles = Vec2::new(0.5, -0.2); // 50% roll right, 20% pitch down
    /// let estimated_angles_rad = Vec2::new(0.1, 0.0); // current roll/pitch in radians
    ///
    /// let angular_rates = controller.control(&config, dt, desired_angles, estimated_angles_rad);
    /// assert!(angular_rates.x > 0.0 && angular_rates.y < 0.0);
    /// ```
    pub fn control(
        &mut self,
        config: &AttitudeControllerConfig,
        dt: f32,
        desired_angles: Vec2,
        estimated_angles_rad: Vec2,
    ) -> Vec2 {
        assert_dt(dt);

        // Saturate normalized inputs, then scale to angle setpoints in radians. Outer loops
        // (e.g. lateral position control far from the target) can ask for more than full tilt;
        // that must not panic the flight loop.
        let max_angles_rad = config.max_angles_rad.map(limit_or_zero);
        let desired_angles = saturate_unit_vec2(desired_angles).component_mul(&max_angles_rad);

        // Calculate error (setpoint - actual)
        let error = desired_angles - estimated_angles_rad;

        // A non-finite error (NaN/∞ estimate) yields no rate command and never enters the
        // integral or derivative history (see `crate::math` for the policy).
        if !(error.x.is_finite() && error.y.is_finite()) {
            self.derivative.forget();
            return Vec2::zeros();
        }

        // Derivative term: rate of change of error (skipped on the first tick after a gap)
        let error_derivative = self.derivative.step(error, dt);

        // Integral term with anti-windup
        self.integrated_error =
            clamp_symmetric_each(self.integrated_error + dt * error, config.max_integral);

        // PID calculation: Kp×error + Ki×∫error + Kd×d(error)/dt
        let Gains { p, i, d } = config.gains;
        let rates = p * error + i * self.integrated_error + d * error_derivative;

        // Scale output to configured maximum rates
        Vec2::new(
            clamp_symmetric(rates.x, config.max_rate_radps.x),
            clamp_symmetric(rates.y, config.max_rate_radps.y),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gains(p: f32, i: f32, d: f32) -> AttitudeControllerConfig {
        AttitudeControllerConfig {
            gains: Gains { p, i, d },
            ..AttitudeControllerConfig::default()
        }
    }

    #[test]
    fn default_is_zeroed_state() {
        let controller = AttitudeController::default();

        assert_eq!(controller.integrated_error, Vec2::zeros());
        assert_eq!(controller.derivative, DerivativeState::default());
    }

    /// A fresh controller has no previous error, so D alone gives nothing on the first tick.
    #[test]
    fn first_tick_has_no_derivative_kick() {
        let mut controller = AttitudeController::default();
        let rates = controller.control(
            &gains(0.0, 0.0, 0.5),
            0.1,
            Vec2::new(0.3, -0.2),
            Vec2::zeros(),
        );
        assert_eq!(rates, Vec2::zeros());
    }

    #[test]
    fn validate_rejects_negative_or_non_finite_limits() {
        let default = AttitudeControllerConfig::default();
        let (angles, rates) = (default.max_angles_rad, default.max_rate_radps);
        // (max_angles_rad, max_rate_radps, max_integral, offending field)
        let cases = [
            (Vec2::new(-1.0, angles.y), rates, 1.0, "max_angles_rad.x"),
            (Vec2::new(angles.x, -1.0), rates, 1.0, "max_angles_rad.y"),
            (angles, Vec2::new(-1.0, rates.y), 1.0, "max_rate_radps.x"),
            (angles, Vec2::new(rates.x, -1.0), 1.0, "max_rate_radps.y"),
            (angles, rates, -1.0, "max_integral"),
        ];
        assert!(default.validate().is_ok());
        for (max_angles_rad, max_rate_radps, max_integral, field) in cases {
            let config = AttitudeControllerConfig {
                max_angles_rad,
                max_rate_radps,
                max_integral,
                ..default
            };
            assert_eq!(config.validate().map_err(|e| e.field), Err(field));
        }
    }

    /// An invalid limit fails `validate` but `control` stays panic-free: the limit acts as 0.
    #[test]
    fn invalid_limits_give_neutral_output_without_panicking() {
        for bad in [-1.0, f32::NAN] {
            let config = AttitudeControllerConfig {
                max_angles_rad: Vec2::new(bad, bad),
                max_rate_radps: Vec2::new(bad, bad),
                max_integral: bad,
                ..AttitudeControllerConfig::default()
            };
            let mut controller = AttitudeController::default();
            let rates = controller.control(&config, 0.1, Vec2::new(1.0, -1.0), Vec2::new(0.2, 0.0));

            assert_eq!(rates, Vec2::zeros(), "limit {bad}");
            assert_eq!(controller.integrated_error, Vec2::zeros(), "limit {bad}");
        }
    }

    #[test]
    fn test_control_zero_error() {
        let mut controller = AttitudeController::default();

        let rates = controller.control(
            &AttitudeControllerConfig::default(),
            0.1,
            Vec2::zeros(),
            Vec2::zeros(),
        );

        assert_eq!(rates, Vec2::zeros()); // No error, no rate output
        assert_eq!(controller.integrated_error, Vec2::zeros()); // No integral buildup
    }

    #[test]
    fn test_control_proportional_response() {
        let config = gains(2.0, 0.0, 0.0);
        let mut controller = AttitudeController::default();
        let dt = 0.1;
        let desired_angles = Vec2::new(1.0, -1.0); // Max roll right, max pitch down

        let rates = controller.control(&config, dt, desired_angles, Vec2::zeros());

        let expected_error = Vec2::new(FRAC_PI_4, -FRAC_PI_4); // 45° in radians
        let expected_rates = config.gains.p * expected_error;
        assert!((rates.x - expected_rates.x).abs() < 1e-6);
        assert!((rates.y - expected_rates.y).abs() < 1e-6);
        assert_eq!(controller.integrated_error, dt * expected_error);
    }

    #[test]
    fn test_control_rate_clamping() {
        let config = AttitudeControllerConfig {
            max_rate_radps: Vec2::new(2.0, 2.0),
            ..gains(100.0, 0.0, 0.0) // High P gain to exceed max_rate_radps
        };

        let rates =
            AttitudeController::default().control(&config, 0.1, Vec2::new(1.0, 1.0), Vec2::zeros());

        assert_eq!(rates, Vec2::new(2.0, 2.0)); // Clamped to max_rate_radps
    }

    #[test]
    fn test_control_integral_buildup() {
        let config = gains(1.0, 1.0, 0.0);
        let mut controller = AttitudeController::default();
        let dt = 0.1;
        let desired_angles = Vec2::new(0.5, 0.5);

        controller.control(&config, dt, desired_angles, Vec2::zeros());
        let rates2 = controller.control(&config, dt, desired_angles, Vec2::zeros());

        let error = Vec2::new(0.5 * FRAC_PI_4, 0.5 * FRAC_PI_4);
        let expected_integral = dt * 2.0 * error; // Two steps
        assert_eq!(controller.integrated_error, expected_integral);
        let expected_rates = 1.0 * error + 1.0 * expected_integral; // P + I terms
        assert!((rates2.x - expected_rates.x).abs() < 1e-6);
        assert!((rates2.y - expected_rates.y).abs() < 1e-6);
    }

    #[test]
    fn test_control_integral_clamping() {
        let config = AttitudeControllerConfig {
            max_integral: 1.0,         // Low limit for testing
            ..gains(0.0, 8000.0, 0.0)  // High I to hit max_integral
        };
        let mut controller = AttitudeController::default();
        let dt = 2.0; // Large dt to quickly reach max_integral

        let rates = controller.control(&config, dt, Vec2::new(1.0, 1.0), Vec2::zeros());

        assert_eq!(controller.integrated_error, Vec2::new(1.0, 1.0)); // Clamped to max_integral
        assert_eq!(rates, Vec2::new(2.0, 2.0));
    }

    /// A gain change between ticks applies on the next tick and keeps the integral; a shrunk
    /// `max_integral` clamps the existing integral.
    #[test]
    fn config_change_between_ticks_keeps_state() {
        let mut controller = AttitudeController::default();
        controller.control(
            &gains(0.0, 1.0, 0.0),
            1.0,
            Vec2::new(1.0, 0.0),
            Vec2::zeros(),
        );
        let integral = controller.integrated_error;

        let rates = controller.control(&gains(0.0, 2.0, 0.0), 0.1, Vec2::zeros(), Vec2::zeros());
        assert_eq!(controller.integrated_error, integral);
        approx::assert_relative_eq!(rates.x, 2.0 * integral.x);

        let shrunk = AttitudeControllerConfig {
            max_integral: 0.1,
            ..gains(0.0, 1.0, 0.0)
        };
        controller.control(&shrunk, 0.1, Vec2::zeros(), Vec2::zeros());
        assert_eq!(controller.integrated_error.x, 0.1);
    }

    #[test]
    fn test_control_derivative_response() {
        let config = gains(0.0, 0.0, 0.5);
        let mut controller = AttitudeController::default();
        let dt = 0.1;
        let desired_angles = Vec2::new(1.0, 1.0);
        let estimated_angles2 = Vec2::new(0.5 * FRAC_PI_4, 0.5 * FRAC_PI_4);

        controller.control(&config, dt, desired_angles, Vec2::zeros());
        let rates2 = controller.control(&config, dt, desired_angles, estimated_angles2);

        let error1 = Vec2::new(FRAC_PI_4, FRAC_PI_4);
        let error2 = error1 - estimated_angles2;
        let derivative = (error2 - error1) / dt;
        let expected_rates = config.gains.d * derivative;
        assert!((rates2.x - expected_rates.x).abs() < 1e-6);
        assert!((rates2.y - expected_rates.y).abs() < 1e-6);
    }

    #[test]
    #[should_panic(expected = "dt must be positive and finite")]
    fn zero_dt_panics() {
        AttitudeController::default().control(
            &AttitudeControllerConfig::default(),
            0.0,
            Vec2::zeros(),
            Vec2::zeros(),
        );
    }

    #[test]
    fn out_of_range_or_nan_desired_angles_saturate_instead_of_panicking() {
        let config = AttitudeControllerConfig::default();
        // (requested normalized angles, equivalent in-range request)
        let cases = [
            (Vec2::new(2.0, 0.0), Vec2::new(1.0, 0.0)),
            (Vec2::new(0.0, -4.6), Vec2::new(0.0, -1.0)), // lateral loop far from its waypoint
            (Vec2::new(-1.5, 3.0), Vec2::new(-1.0, 1.0)),
            (Vec2::new(f32::NAN, 0.5), Vec2::new(0.0, 0.5)),
            (
                Vec2::new(f32::INFINITY, f32::NEG_INFINITY),
                Vec2::new(1.0, -1.0),
            ),
        ];
        for (requested, equivalent) in cases {
            let rates =
                AttitudeController::default().control(&config, 0.1, requested, Vec2::zeros());
            let expected =
                AttitudeController::default().control(&config, 0.1, equivalent, Vec2::zeros());
            assert!(
                rates.x.is_finite() && rates.y.is_finite(),
                "{requested:?} gave non-finite rates {rates:?}"
            );
            approx::assert_relative_eq!(rates.x, expected.x);
            approx::assert_relative_eq!(rates.y, expected.y);
        }
    }

    #[test]
    fn nan_estimate_does_not_poison_later_ticks() {
        let config = AttitudeControllerConfig::default();
        let mut controller = AttitudeController::default();
        let desired = Vec2::new(0.5, -0.5);

        let during = controller.control(&config, 0.1, desired, Vec2::new(f32::NAN, 0.0));
        assert!(during == Vec2::zeros(), "NaN estimate gave {during:?}");
        assert!(controller.integrated_error == Vec2::zeros());

        // First finite tick after the gap: P + I only (no derivative from the pre-gap error).
        let after_nan = controller.control(&config, 0.1, desired, Vec2::zeros());
        let error = desired.component_mul(&config.max_angles_rad);
        let Gains { p, i, .. } = config.gains;
        let expected = p * error + i * (0.1 * error);
        approx::assert_relative_eq!(after_nan.x, expected.x.clamp(-2.0, 2.0));
        approx::assert_relative_eq!(after_nan.y, expected.y.clamp(-2.0, 2.0));
    }
}
