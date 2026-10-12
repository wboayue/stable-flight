use crate::controllers::derivative::DerivativeState;
use crate::controllers::{Gains, InvalidConfig, check_limit, rescale_integral, seed_integral};
use crate::math::{assert_dt, clamp_symmetric, wrap_angle};

/// Configures the yaw controller with PID gains and operational limits.
///
/// This structure defines the control parameters that determine how the yaw controller
/// responds to heading errors, including responsiveness, stability, and maximum rates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct YawControllerConfig {
    /// PID gains that determine controller responsiveness.
    ///
    /// * `p` - Proportional gain: controls response strength relative to current error.
    /// * `i` - Integral gain: eliminates steady-state errors over time.
    /// * `d` - Derivative gain: provides damping to reduce overshoot and oscillation.
    pub gains: Gains,

    /// Maximum absolute value of the integrated heading error, in rad·s.
    ///
    /// Prevents integral windup by limiting accumulated error, which
    /// could otherwise cause significant overshoot and control instability.
    pub max_integral: f32,

    /// Maximum angular rate in radians/second for yaw.
    ///
    /// Limits the output commands to physically achievable rates and
    /// prevents excessive control demands that could destabilize the system.
    pub max_rate_radps: f32,
}

impl Default for YawControllerConfig {
    fn default() -> Self {
        Self {
            // Heading hold, flight-checked on RealFlight's QuadCopterX (#62).
            gains: Gains {
                p: 2.5,
                i: 0.0,
                d: 0.06,
            },
            max_integral: 100.0, // Arbitrary limit
            max_rate_radps: 2.0, // 2 rad/s (~114°/s) for yaw
        }
    }
}

impl YawControllerConfig {
    /// Checks the config: `max_integral` and `max_rate_radps` must be non-negative and finite.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::YawControllerConfig;
    ///
    /// assert!(YawControllerConfig::default().validate().is_ok());
    /// let bad = YawControllerConfig { max_rate_radps: f32::NAN, ..Default::default() };
    /// assert!(bad.validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_limit("max_integral", self.max_integral)?;
        check_limit("max_rate_radps", self.max_rate_radps)
    }
}

/// Yaw controller that maintains a desired heading using PID control.
///
/// This controller:
/// * Takes desired and actual heading measurements
/// * Calculates the shortest path error accounting for angle wrapping
/// * Applies PID control to generate appropriate yaw rate commands
/// * Incorporates anti-windup protection and output limiting
///
/// It holds only state (integral and derivative history); gains and limits come from the
/// [`YawControllerConfig`] passed to each [`control`](Self::control) call, so they can change
/// between ticks without resetting the state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct YawController {
    /// Previous error for the derivative term; empty on the first tick and after a gap.
    derivative: DerivativeState<f32>,
    integrated_error: f32,
}

impl YawController {
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
    /// * `config` - Gains and limits; assumed valid ([`YawControllerConfig::validate`]). An
    ///   invalid limit does not panic: a negative or NaN limit acts as 0.
    /// * `dt` - Time step since the last update in seconds (must be positive and finite).
    /// * `desired_yaw_rad` - Target yaw angle in radians. Any finite value is accepted; the
    ///   error is wrapped to the shortest path, so `desired_yaw_rad` and `desired_yaw_rad + 2πk` are
    ///   equivalent. A non-finite target (or estimate) yields a zero rate and leaves the
    ///   controller state untouched.
    /// * `estimated_yaw_rad` - Current measured yaw angle in radians.
    ///
    /// # Returns
    ///
    /// Desired yaw rate in rad/s, limited to configured maximum rates.
    ///
    /// # Panics
    ///
    /// If `dt` is not positive and finite.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f32::consts::PI;
    /// use stable_flight::controllers::{YawController, YawControllerConfig};
    ///
    /// let config = YawControllerConfig::default();
    /// let mut controller = YawController::default();
    /// let dt = 0.02;
    /// let desired_yaw_rad = PI / 4.0;  // 45 degrees
    /// let current_yaw_rad = 0.0;       // 0 degrees
    ///
    /// let yaw_rate = controller.control(&config, dt, desired_yaw_rad, current_yaw_rad);
    /// // Returns a rate command to turn toward the desired heading
    /// ```
    pub fn control(
        &mut self,
        config: &YawControllerConfig,
        dt: f32,
        desired_yaw_rad: f32,
        estimated_yaw_rad: f32,
    ) -> f32 {
        assert_dt(dt);

        let gains = &config.gains;

        let error = angle_diff(desired_yaw_rad, estimated_yaw_rad); // Use a helper to handle angle wrapping
        if !error.is_finite() {
            // Hold (no yaw rate) rather than letting NaN into the integral and derivative
            // history, where it would stick; clamping passes NaN through. Forget the previous
            // error too, so the first valid tick after the gap has no derivative kick.
            self.derivative.forget();
            return 0.0;
        }
        let error_derivative = self.derivative.step(error, dt);

        self.integrated_error =
            clamp_symmetric(self.integrated_error + dt * error, config.max_integral);

        let yaw_rate =
            gains.p * error + gains.i * self.integrated_error + gains.d * error_derivative;

        clamp_symmetric(yaw_rate, config.max_rate_radps)
    }

    /// Zeroes the integral, keeping the derivative history.
    pub(crate) fn clear_integral(&mut self) {
        self.integrated_error = 0.0;
    }

    /// Rescales the integral for a change of integral gain from `old` to `new`, so the next
    /// tick's I term picks up where the last left off (`rescale_integral`).
    pub(crate) fn rescale_integral(
        &mut self,
        old: &YawControllerConfig,
        new: &YawControllerConfig,
    ) {
        self.integrated_error = rescale_integral(self.integrated_error, old.gains.i, new.gains.i);
    }

    /// The I term (`i · integral`): the yaw rate the loop holds as trim.
    pub(crate) fn integral_term(&self, config: &YawControllerConfig) -> f32 {
        config.gains.i * self.integrated_error
    }

    /// Restarts from a trim: the I term becomes `i_term` (`seed_integral`) and the derivative
    /// history is cleared.
    pub(crate) fn hand_back(&mut self, config: &YawControllerConfig, i_term: f32) {
        *self = Self {
            derivative: DerivativeState::default(),
            integrated_error: seed_integral(i_term, config.gains.i, config.max_integral),
        };
    }
}

/// The shortest-path yaw error from `current_yaw` to `desired_yaw`, in [-π, π).
///
/// # Arguments
///
/// * `desired_yaw` - Desired yaw angle in radians.
/// * `current_yaw` - Current yaw angle in radians.
///
/// # Returns
///
/// The shortest-path yaw error in radians, in [-π, π) (see [`wrap_angle`]).
pub(crate) fn angle_diff(desired_yaw: f32, current_yaw: f32) -> f32 {
    wrap_angle(desired_yaw - current_yaw)
}

#[cfg(test)]
mod tests {

    use std::f32::consts::PI;

    use approx::assert_relative_eq;

    use crate::controllers::Gains;

    use super::*;

    const EPSILON: f32 = 1e-5;

    // Helper functions for test setup

    /// A config with only proportional gain.
    fn p_only_config(p_gain: f32, max_rate_radps: f32) -> YawControllerConfig {
        YawControllerConfig {
            gains: Gains {
                p: p_gain,
                i: 0.0,
                d: 0.0,
            },
            max_integral: 10.0,
            max_rate_radps,
        }
    }

    /// A config with only integral gain.
    fn i_only_config(i_gain: f32, max_integral: f32, max_rate_radps: f32) -> YawControllerConfig {
        YawControllerConfig {
            gains: Gains {
                p: 0.0,
                i: i_gain,
                d: 0.0,
            },
            max_integral,
            max_rate_radps,
        }
    }

    /// A config with only derivative gain.
    fn d_only_config(d_gain: f32, max_rate_radps: f32) -> YawControllerConfig {
        YawControllerConfig {
            gains: Gains {
                p: 0.0,
                i: 0.0,
                d: d_gain,
            },
            max_integral: 10.0,
            max_rate_radps,
        }
    }

    /// Tests that the default controller configuration has expected values.
    #[test]
    fn test_default_config() {
        let config = YawControllerConfig::default();

        assert_relative_eq!(config.gains.p, 2.5, epsilon = EPSILON, max_relative = 1e-3);
        assert_relative_eq!(config.gains.i, 0.0, epsilon = EPSILON, max_relative = 1e-3);
        assert_relative_eq!(config.gains.d, 0.06, epsilon = EPSILON, max_relative = 1e-3);
        assert_relative_eq!(
            config.max_integral,
            100.0,
            epsilon = EPSILON,
            max_relative = 1e-3
        );
        assert_relative_eq!(
            config.max_rate_radps,
            2.0,
            epsilon = EPSILON,
            max_relative = 1e-3
        );
    }

    #[test]
    fn default_is_zeroed_state() {
        assert_eq!(
            YawController::default(),
            YawController {
                derivative: DerivativeState::default(),
                integrated_error: 0.0,
            }
        );
    }

    /// Tests that the proportional control term works correctly in isolation.
    #[test]
    fn test_proportional_control() {
        // Test that the proportional term works correctly
        let config = p_only_config(2.0, 5.0);
        let mut controller = YawController::default();

        // Simple case: 0.5 rad error should result in rate = 2.0 * 0.5 = 1.0 rad/s
        let dt = 0.01;
        let desired_yaw = 0.5;
        let estimated_yaw = 0.0;

        let rate = controller.control(&config, dt, desired_yaw, estimated_yaw);
        assert_relative_eq!(
            rate,
            1.0,
            epsilon = EPSILON,
            max_relative = 1e-3,
            max_relative = 1e-3
        );

        // Test with negative error
        let rate = controller.control(&config, dt, 0.0, 0.5);
        assert_relative_eq!(rate, -1.0, epsilon = EPSILON, max_relative = 1e-3);
    }

    /// Tests that the integral control term accumulates correctly and respects limits.
    #[test]
    fn test_integral_control() {
        // Test that the integral term accumulates and affects output
        let config = i_only_config(1.0, 10.0, 5.0);
        let mut controller = YawController::default();

        // Apply constant error for several iterations
        let dt = 1.0; // 1 second for easy calculation
        let desired_yaw = 0.5;
        let estimated_yaw = 0.0;

        // First iteration: integral = 0.5 * 1.0 = 0.5
        let rate = controller.control(&config, dt, desired_yaw, estimated_yaw);
        assert_relative_eq!(rate, 0.5, epsilon = EPSILON, max_relative = 1e-3);

        // Second iteration: integral = 0.5 + 0.5 * 1.0 = 1.0
        let rate = controller.control(&config, dt, desired_yaw, estimated_yaw);
        assert_relative_eq!(rate, 1.0, epsilon = EPSILON, max_relative = 1e-3);

        // Test integral clamping
        let config = i_only_config(1.0, 1.5, 5.0);
        let mut controller = YawController::default();

        // Accumulate for 3 seconds to exceed max_integral
        controller.control(&config, dt, desired_yaw, estimated_yaw); // 0.5
        controller.control(&config, dt, desired_yaw, estimated_yaw); // 1.0
        controller.control(&config, dt, desired_yaw, estimated_yaw); // 1.5
        let rate = controller.control(&config, dt, desired_yaw, estimated_yaw);

        // Should be clamped at max_integral * i_gain = 1.5
        assert_relative_eq!(rate, 1.5, epsilon = EPSILON, max_relative = 1e-3);
    }

    /// Tests that the derivative control term responds correctly to error changes.
    #[test]
    fn test_derivative_control() {
        // Test that the derivative term works correctly
        let config = d_only_config(2.0, 5.0);
        let mut controller = YawController::default();

        let dt = 1.0; // 1 second for easy calculation

        // First call - establishes baseline error
        let rate = controller.control(&config, dt, 0.5, 0.0);
        assert_relative_eq!(
            rate,
            0.0,
            epsilon = EPSILON,
            max_relative = 1e-3,
            max_relative = 1e-3
        );

        // Second call - error unchanged, derivative should be zero
        let rate = controller.control(&config, dt, 0.5, 0.0);
        assert_relative_eq!(rate, 0.0, epsilon = EPSILON, max_relative = 1e-3);

        // Error increasing by 0.1 rad/s
        let rate = controller.control(&config, dt, 0.6, 0.0);
        assert_relative_eq!(rate, 0.2, epsilon = EPSILON, max_relative = 1e-3);

        // Error decreasing by 0.2 rad/s
        let rate = controller.control(&config, dt, 0.4, 0.0);
        assert_relative_eq!(rate, -0.4, epsilon = EPSILON, max_relative = 1e-3);
    }

    /// Tests all PID components working together with detailed calculation.
    #[test]
    fn test_pid_combined() {
        // Test all PID components working together
        let config = YawControllerConfig {
            gains: Gains {
                p: 1.0,
                i: 0.5,
                d: 0.2,
            },
            max_integral: 10.0,
            max_rate_radps: 5.0,
        };

        let mut controller = YawController::default();

        let dt = 1.0;

        // First call:
        // - P term = 1.0 * 0.5 = 0.5
        // - I term = 0.5 * (0.5 * 1.0) = 0.25
        // - D term = 0 (no previous error)
        // Total = 0.5 + 0.25 + 0 = 0.75
        let rate = controller.control(&config, dt, 0.5, 0.0);
        assert_relative_eq!(rate, 0.75, epsilon = EPSILON, max_relative = 1e-3);

        // Second call with same error:
        // - P term = 1.0 * 0.5 = 0.5
        // - I term = 0.5 * (0.5 + 0.5) = 0.5
        // - D term = 0.2 * (0.5 - 0.5) / 1.0 = 0
        // Total = 0.5 + 0.5 + 0 = 1.0
        let rate = controller.control(&config, dt, 0.5, 0.0);
        assert_relative_eq!(rate, 1.0, epsilon = EPSILON, max_relative = 1e-3);

        // Third call with increased error:
        // - P term = 1.0 * 0.6 = 0.6
        // - I term = 0.5 * (0.5 + 0.5 + 0.6) = 0.8
        // - D term = 0.2 * (0.6 - 0.5) / 1.0 = 0.02
        // Total = 0.6 + 0.8 + 0.02 = 1.42
        let rate = controller.control(&config, dt, 0.6, 0.0);
        assert_relative_eq!(rate, 1.42, epsilon = EPSILON, max_relative = 1e-3);
    }

    /// Tests that the controller handles varying time steps correctly.
    #[test]
    fn test_varying_time_steps() {
        let config = YawControllerConfig {
            gains: Gains {
                p: 0.0,
                i: 1.0,
                d: 0.0,
            },
            max_integral: 10.0,
            max_rate_radps: 5.0,
        };

        let mut controller = YawController::default();

        // Control with a 1-second time step
        let rate_1s = controller.control(&config, 1.0, 0.5, 0.0);
        assert_relative_eq!(rate_1s, 0.5, epsilon = EPSILON, max_relative = 1e-3);

        // Next control with a 0.5-second time step should accumulate half as much
        let rate_05s = controller.control(&config, 0.5, 0.5, 0.0);
        assert_relative_eq!(
            rate_05s,
            0.5 + 0.25, // Previous 0.5 plus new 0.5*0.5
            epsilon = EPSILON,
            max_relative = 1e-3
        );

        // Next control with a 0.25-second time step
        let rate_025s = controller.control(&config, 0.25, 0.5, 0.0);
        assert_relative_eq!(
            rate_025s,
            0.5 + 0.25 + 0.125, // Plus new 0.5*0.25
            epsilon = EPSILON,
            max_relative = 1e-3
        );
    }

    /// Tests controller behavior with very small time steps.
    #[test]
    fn test_small_time_steps() {
        let config = YawControllerConfig {
            gains: Gains {
                p: 1.0,
                i: 1.0,
                d: 1.0,
            },
            max_integral: 10.0,
            max_rate_radps: 5.0,
        };

        let mut controller = YawController::default();

        // Use a very small time step
        let dt = 0.000001; // 1 microsecond
        let error = 0.5;
        let rate = controller.control(&config, dt, error, 0.0);

        // With a tiny dt:
        // - P term = 1.0 * 0.5 = 0.5
        // - I term = 1.0 * 0.5 * 1e-6 ≈ 0.0000005 (effectively 0)
        // - D term = undefined (first call)
        assert_relative_eq!(
            rate,
            0.5, // Only P term is significant
            epsilon = EPSILON,
            max_relative = 1e-3
        );

        // Second call with same error should still have negligible I term
        let rate = controller.control(&config, dt, error, 0.0);
        assert_relative_eq!(
            rate,
            0.5, // P term only, I term still tiny
            epsilon = EPSILON,
            max_relative = 1e-3
        );
    }

    /// Tests that angle wrapping is handled correctly at ±π boundaries.
    #[test]
    fn test_angle_wrapping() {
        // Test angle wrapping when crossing the ±π boundary
        let config = p_only_config(1.0, 5.0);
        let mut controller = YawController::default();

        let dt = 0.01;

        // Test case: desired = 175°, current = -175°
        // Shortest path is -10° (not +350°)
        let desired_yaw = 175.0 * PI / 180.0;
        let estimated_yaw = -175.0 * PI / 180.0;

        let rate = controller.control(&config, dt, desired_yaw, estimated_yaw);
        let expected_rate = -10.0 * PI / 180.0; // -10 degrees converted to radians
        assert_relative_eq!(rate, expected_rate, epsilon = EPSILON, max_relative = 1e-3);

        // Test case: desired = -175°, current = 175°
        // Shortest path is +10° (not -350°)
        let desired_yaw = -175.0 * PI / 180.0;
        let estimated_yaw = 175.0 * PI / 180.0;

        let rate = controller.control(&config, dt, desired_yaw, estimated_yaw);
        let expected_rate = 10.0 * PI / 180.0; // 10 degrees converted to radians
        assert_relative_eq!(rate, expected_rate, epsilon = EPSILON, max_relative = 1e-3);
    }

    /// Directly tests the angle_diff helper function with various inputs.
    #[test]
    fn angle_diff_is_the_shortest_signed_path() {
        let deg = PI / 180.0;
        // (desired, current, expected error)
        let cases = [
            (0.5, 0.0, 0.5),                        // no wrapping needed
            (PI + 0.1, -PI - 0.1, 0.2),             // wraps across ±π
            (350.0 * deg, 10.0 * deg, -20.0 * deg), // turn left through north
            (10.0 * deg, 350.0 * deg, 20.0 * deg),  // turn right through north
            (PI, -PI, 0.0),                         // same heading
            (0.0, 2.0 * PI, 0.0),                   // same heading, one turn apart
            (5.0, 5.0 - 2.0 * PI, 0.0),             // [0, 2π) heading vs its [-π, π) form
        ];
        for (desired, current, expected) in cases {
            assert_relative_eq!(
                angle_diff(desired, current),
                expected,
                epsilon = EPSILON,
                max_relative = 1e-3
            );
        }
    }

    #[test]
    fn test_valid_inputs() {
        let config = YawControllerConfig::default();
        let mut controller = YawController::default();
        let dt = 0.01;

        // Valid inputs should not panic
        let _ = controller.control(&config, dt, 0.0, 0.0);
        let _ = controller.control(&config, dt, PI, -PI);
        let _ = controller.control(&config, dt, 2.0 * PI, 0.0);
    }

    #[test]
    #[should_panic(expected = "dt must be positive and finite")]
    fn zero_dt_panics() {
        let config = YawControllerConfig::default();
        let mut controller = YawController::default();
        let dt = 0.0;
        let _ = controller.control(&config, dt, 0.0, 0.0);
    }

    #[test]
    fn any_finite_target_is_equivalent_to_its_wrapped_heading() {
        // (target, equivalent target within [-π, π))
        let cases = [
            (3.0 * PI, -PI),
            (10.0 * PI + 0.5, 0.5),
            (-7.0 * PI / 2.0, PI / 2.0),
            (1000.0, wrap_angle(1000.0)),
        ];
        let config = YawControllerConfig::default();
        for (target, equivalent) in cases {
            let rate = YawController::default().control(&config, 0.01, target, 0.2);
            let expected = YawController::default().control(&config, 0.01, equivalent, 0.2);
            assert_relative_eq!(rate, expected, epsilon = 1e-3);
        }
    }

    #[test]
    fn non_finite_target_holds_and_does_not_poison_later_ticks() {
        let config = YawControllerConfig::default();
        let mut controller = YawController::default();
        for target in [f32::NAN, f32::INFINITY] {
            assert!(controller.control(&config, 0.01, target, 0.2) == 0.0);
        }
        let after = controller.control(&config, 0.01, 1.0, 0.2);
        let fresh = YawController::default().control(&config, 0.01, 1.0, 0.2);
        assert_relative_eq!(after, fresh);
    }

    #[test]
    fn nan_gap_does_not_cause_a_derivative_kick() {
        // P+D only, so the only difference a stale previous error could make is the D term.
        let config = YawControllerConfig {
            gains: Gains {
                p: 1.0,
                i: 0.0,
                d: 0.5,
            },
            max_integral: 1.0,
            max_rate_radps: 100.0,
        };
        let mut controller = YawController::default();

        controller.control(&config, 0.01, 1.0, 0.2); // error 0.8 becomes the previous error
        controller.control(&config, 0.01, f32::NAN, 0.2); // gap
        let after_gap = controller.control(&config, 0.01, 0.5, 0.2); // error 0.3

        // Without the reset, D would add 0.5 * (0.3 - 0.8) / 0.01 = -25 rad/s.
        let fresh = YawController::default().control(&config, 0.01, 0.5, 0.2);
        assert_relative_eq!(after_gap, fresh);
    }

    /// Tests that output rate is properly limited to `max_rate_radps`.
    #[test]
    fn test_max_rate_limiting() {
        // Test that output is clamped to max_rate_radps
        let config = p_only_config(10.0, 2.0);
        let mut controller = YawController::default();

        let dt = 0.01;

        // Error of 0.5 rad with P=10 would give 5.0 rad/s
        // Should be limited to 2.0 rad/s
        let rate = controller.control(&config, dt, 0.5, 0.0);

        assert_relative_eq!(rate, 2.0, epsilon = EPSILON, max_relative = 1e-3);

        // Test negative limiting too
        let rate = controller.control(&config, dt, -0.5, 0.0);
        assert_relative_eq!(rate, -2.0, epsilon = EPSILON, max_relative = 1e-3);
    }

    /// Tests a realistic maneuver to verify controller convergence and stability.
    #[test]
    fn test_realistic_maneuvers() {
        // Test a realistic 90-degree turn maneuver
        let config = YawControllerConfig {
            gains: Gains {
                p: 2.0,
                i: 0.3,
                d: 0.5,
            },
            max_integral: 5.0,
            max_rate_radps: 3.0,
        };

        let mut controller = YawController::default();

        let dt = 0.1; // 0.1 seconds

        // Simulate a 90-degree turn
        let desired_yaw = 90.0 * PI / 180.0;
        let mut current_yaw = 0.0;
        let mut rates = Vec::new();

        // Simulate control loop for 2 seconds (20 iterations)
        for _ in 0..20 {
            let rate = controller.control(&config, dt, desired_yaw, current_yaw);
            rates.push(rate);

            // Simple integration to update current yaw for next iteration
            current_yaw += rate * dt;

            // Normalize to [-PI, PI] for realism
            if current_yaw > PI {
                current_yaw -= 2.0 * PI;
            } else if current_yaw < -PI {
                current_yaw += 2.0 * PI;
            }
        }

        // Verify the controller converges to the desired yaw
        assert!(
            (desired_yaw - current_yaw).abs() < 0.1,
            "Controller should converge to desired yaw (within 0.1 rad)"
        );

        // Verify that the final rate approaches zero as error decreases
        assert!(
            rates.last().unwrap().abs() < 0.2,
            "Rate should approach zero as error decreases"
        );

        // Additional checks for controller behavior:

        // Initial rates should be high (turning toward target)
        assert!(
            rates[0] > 1.0,
            "Initial rate should be high when error is large"
        );

        // Rates should gradually decrease as we approach target
        assert!(
            rates[0] > rates[rates.len() / 2] && rates[rates.len() / 2] > rates[rates.len() - 1],
            "Rates should decrease as controller approaches target"
        );
    }

    #[test]
    fn validate_rejects_negative_or_non_finite_limits() {
        // (max_integral, max_rate_radps, offending field)
        let cases = [(-1.0, 1.0, "max_integral"), (1.0, -1.0, "max_rate_radps")];
        for (max_integral, max_rate_radps, field) in cases {
            let config = YawControllerConfig {
                max_integral,
                max_rate_radps,
                ..Default::default()
            };
            assert_eq!(config.validate().map_err(|e| e.field), Err(field));
        }
    }

    /// An invalid limit fails `validate` but `control` stays panic-free: the limit acts as 0.
    #[test]
    fn invalid_limits_give_neutral_output_without_panicking() {
        for bad in [-1.0, f32::NAN] {
            let config = YawControllerConfig {
                gains: Gains {
                    p: 1.0,
                    i: 1.0,
                    d: 0.0,
                },
                max_integral: bad,
                max_rate_radps: bad,
            };
            let mut controller = YawController::default();
            assert_eq!(
                controller.control(&config, 0.1, 1.0, 0.0),
                0.0,
                "limit {bad}"
            );
            assert_eq!(controller.integrated_error, 0.0, "limit {bad}");
        }
    }

    /// A gain change between ticks applies on the next tick and keeps the integral.
    #[test]
    fn gain_change_between_ticks_keeps_state() {
        let mut config = i_only_config(1.0, 10.0, 10.0);
        let mut controller = YawController::default();
        controller.control(&config, 1.0, 0.5, 0.0);
        let integral = controller.integrated_error;

        config.gains.i = 2.0;
        let rate = controller.control(&config, 0.01, 0.0, 0.0);

        assert_eq!(controller.integrated_error, integral);
        assert_relative_eq!(rate, 2.0 * integral, epsilon = EPSILON);
    }
}
