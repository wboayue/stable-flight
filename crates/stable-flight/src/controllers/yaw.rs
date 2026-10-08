use crate::controllers::Gains;
use crate::math::wrap_angle;

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
            gains: Default::default(),
            max_integral: 100.0, // Arbitrary limit
            max_rate_radps: 2.0, // 2 rad/s (~114°/s) for yaw
        }
    }
}

/// Yaw controller that maintains a desired heading using PID control with noise filtering.
///
/// This controller:
/// * Takes desired and actual heading measurements
/// * Calculates the shortest path error accounting for angle wrapping
/// * Applies filtered PID control to generate appropriate yaw rate commands
/// * Incorporates anti-windup protection and output limiting
pub struct YawController {
    pub config: YawControllerConfig,

    previous_error: Option<f32>,
    integrated_error: f32,
}

impl Default for YawController {
    fn default() -> Self {
        Self::new()
    }
}

impl YawController {
    /// Creates a new yaw controller with default configuration.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::{YawController, YawControllerConfig};
    ///
    /// let controller = YawController::new();
    /// ```
    pub fn new() -> Self {
        Self::with_config(&YawControllerConfig::default())
    }

    /// Creates a new yaw controller with the specified configuration.
    ///
    /// # Arguments
    ///
    /// * `config` - Custom controller configuration settings
    ///
    /// # Panics
    ///
    /// If `max_integral` or `max_rate_radps` is negative or not finite: they bound per-tick clamps,
    /// which would otherwise panic inside the flight loop.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::{YawController, YawControllerConfig};
    /// use stable_flight::controllers::Gains;
    ///
    /// let config = YawControllerConfig {
    ///     gains: Gains { p: 2.0, i: 0.1, d: 0.5 },
    ///     max_integral: 50.0,
    ///     max_rate_radps: 1.5,
    /// };
    ///
    /// let controller = YawController::with_config(&config);
    /// ```
    pub fn with_config(config: &YawControllerConfig) -> Self {
        assert!(
            config.max_integral >= 0.0 && config.max_integral.is_finite(),
            "max_integral must be non-negative and finite, got {}",
            config.max_integral
        );
        assert!(
            config.max_rate_radps >= 0.0 && config.max_rate_radps.is_finite(),
            "max_rate_radps must be non-negative and finite, got {}",
            config.max_rate_radps
        );
        Self {
            config: *config,
            previous_error: None,
            integrated_error: 0.0,
        }
    }

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
    /// * `dt` - Time step since the last update (must be positive).
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
    /// Panics if the time step is not finite or not positive.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f32::consts::PI;
    /// use stable_flight::controllers::{YawController, YawControllerConfig};
    ///
    /// let mut controller = YawController::new();
    /// let dt = 0.02;
    /// let desired_yaw_rad = PI / 4.0;  // 45 degrees
    /// let current_yaw_rad = 0.0;       // 0 degrees
    ///
    /// let yaw_rate = controller.control(dt, desired_yaw_rad, current_yaw_rad);
    /// // Returns a rate command to turn toward the desired heading
    /// ```
    pub fn control(&mut self, dt: f32, desired_yaw_rad: f32, estimated_yaw_rad: f32) -> f32 {
        assert!(dt.is_finite(), "Time step must be finite");
        assert!(dt > 0.0, "Time step must be positive");

        let gains = &self.config.gains;

        let error = angle_diff(desired_yaw_rad, estimated_yaw_rad); // Use a helper to handle angle wrapping
        if !error.is_finite() {
            // Hold (no yaw rate) rather than letting NaN into the integral and derivative
            // history, where it would stick; `clamp` passes NaN through. Forget the previous
            // error too, so the first valid tick after the gap has no derivative kick.
            self.previous_error = None;
            return 0.0;
        }
        let error_derivative = if let Some(previous_error) = self.previous_error {
            (error - previous_error) / dt
        } else {
            0.0
        };

        self.integrated_error += dt * error;
        self.integrated_error = self
            .integrated_error
            .clamp(-self.config.max_integral, self.config.max_integral);

        let yaw_rate =
            gains.p * error + gains.i * self.integrated_error + gains.d * error_derivative;

        self.previous_error = Some(error);

        yaw_rate.clamp(-self.config.max_rate_radps, self.config.max_rate_radps)
    }

    /// Resets the controller state (integral and last error terms) to zero.
    pub fn reset(&mut self) {
        self.integrated_error = 0.0;
        self.previous_error = None;
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
fn angle_diff(desired_yaw: f32, current_yaw: f32) -> f32 {
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

    /// Creates a controller with only proportional gain for testing
    fn create_p_only_controller(p_gain: f32, max_rate_radps: f32) -> YawController {
        let config = YawControllerConfig {
            gains: Gains {
                p: p_gain,
                i: 0.0,
                d: 0.0,
            },
            max_integral: 10.0,
            max_rate_radps,
        };

        YawController::with_config(&config)
    }

    /// Creates a controller with only integral gain for testing
    fn create_i_only_controller(
        i_gain: f32,
        max_integral: f32,
        max_rate_radps: f32,
    ) -> YawController {
        let config = YawControllerConfig {
            gains: Gains {
                p: 0.0,
                i: i_gain,
                d: 0.0,
            },
            max_integral,
            max_rate_radps,
        };

        YawController::with_config(&config)
    }

    /// Creates a controller with only derivative gain for testing
    fn create_d_only_controller(d_gain: f32, max_rate_radps: f32) -> YawController {
        let config = YawControllerConfig {
            gains: Gains {
                p: 0.0,
                i: 0.0,
                d: d_gain,
            },
            max_integral: 10.0,
            max_rate_radps,
        };

        YawController::with_config(&config)
    }

    /// Tests that the default controller configuration has expected values.
    #[test]
    fn test_default_config() {
        let config = YawControllerConfig::default();

        assert_relative_eq!(config.gains.p, 0.0, epsilon = EPSILON, max_relative = 1e-3);
        assert_relative_eq!(config.gains.i, 0.0, epsilon = EPSILON, max_relative = 1e-3);
        assert_relative_eq!(config.gains.d, 0.0, epsilon = EPSILON, max_relative = 1e-3);
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

    /// Tests that the controller initializes correctly with both default and custom configs.
    #[test]
    fn test_new_controller_initialization() {
        // Test that the controller initializes with default values
        let controller = YawController::new();

        assert_eq!(
            controller.config,
            YawControllerConfig::default(),
            "Controller should initialize with default config"
        );

        // Create controller with custom config
        let config = YawControllerConfig {
            gains: Gains {
                p: 1.5,
                i: 0.5,
                d: 0.2,
            },
            max_integral: 10.0,
            max_rate_radps: 2.0,
        };

        let controller = YawController::with_config(&config);
        assert_eq!(
            controller.config, config,
            "Controller should initialize with configured values"
        );
    }

    /// Tests that the proportional control term works correctly in isolation.
    #[test]
    fn test_proportional_control() {
        // Test that the proportional term works correctly
        let mut controller = create_p_only_controller(2.0, 5.0);

        // Simple case: 0.5 rad error should result in rate = 2.0 * 0.5 = 1.0 rad/s
        let dt = 0.01;
        let desired_yaw = 0.5;
        let estimated_yaw = 0.0;

        let rate = controller.control(dt, desired_yaw, estimated_yaw);
        assert_relative_eq!(
            rate,
            1.0,
            epsilon = EPSILON,
            max_relative = 1e-3,
            max_relative = 1e-3
        );

        // Test with negative error
        let rate = controller.control(dt, 0.0, 0.5);
        assert_relative_eq!(rate, -1.0, epsilon = EPSILON, max_relative = 1e-3);
    }

    /// Tests that the integral control term accumulates correctly and respects limits.
    #[test]
    fn test_integral_control() {
        // Test that the integral term accumulates and affects output
        let mut controller = create_i_only_controller(1.0, 10.0, 5.0);

        // Apply constant error for several iterations
        let dt = 1.0; // 1 second for easy calculation
        let desired_yaw = 0.5;
        let estimated_yaw = 0.0;

        // First iteration: integral = 0.5 * 1.0 = 0.5
        let rate = controller.control(dt, desired_yaw, estimated_yaw);
        assert_relative_eq!(rate, 0.5, epsilon = EPSILON, max_relative = 1e-3);

        // Second iteration: integral = 0.5 + 0.5 * 1.0 = 1.0
        let rate = controller.control(dt, desired_yaw, estimated_yaw);
        assert_relative_eq!(rate, 1.0, epsilon = EPSILON, max_relative = 1e-3);

        // Test integral clamping
        let mut controller = create_i_only_controller(1.0, 1.5, 5.0);

        // Accumulate for 3 seconds to exceed max_integral
        controller.control(dt, desired_yaw, estimated_yaw); // 0.5
        controller.control(dt, desired_yaw, estimated_yaw); // 1.0
        controller.control(dt, desired_yaw, estimated_yaw); // 1.5
        let rate = controller.control(dt, desired_yaw, estimated_yaw);

        // Should be clamped at max_integral * i_gain = 1.5
        assert_relative_eq!(rate, 1.5, epsilon = EPSILON, max_relative = 1e-3);
    }

    /// Tests that the derivative control term responds correctly to error changes.
    #[test]
    fn test_derivative_control() {
        // Test that the derivative term works correctly
        let mut controller = create_d_only_controller(2.0, 5.0);

        let dt = 1.0; // 1 second for easy calculation

        // First call - establishes baseline error
        let rate = controller.control(dt, 0.5, 0.0);
        assert_relative_eq!(
            rate,
            0.0,
            epsilon = EPSILON,
            max_relative = 1e-3,
            max_relative = 1e-3
        );

        // Second call - error unchanged, derivative should be zero
        let rate = controller.control(dt, 0.5, 0.0);
        assert_relative_eq!(rate, 0.0, epsilon = EPSILON, max_relative = 1e-3);

        // Error increasing by 0.1 rad/s
        let rate = controller.control(dt, 0.6, 0.0);
        assert_relative_eq!(rate, 0.2, epsilon = EPSILON, max_relative = 1e-3);

        // Error decreasing by 0.2 rad/s
        let rate = controller.control(dt, 0.4, 0.0);
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

        let mut controller = YawController::with_config(&config);

        let dt = 1.0;

        // First call:
        // - P term = 1.0 * 0.5 = 0.5
        // - I term = 0.5 * (0.5 * 1.0) = 0.25
        // - D term = 0 (no previous error)
        // Total = 0.5 + 0.25 + 0 = 0.75
        let rate = controller.control(dt, 0.5, 0.0);
        assert_relative_eq!(rate, 0.75, epsilon = EPSILON, max_relative = 1e-3);

        // Second call with same error:
        // - P term = 1.0 * 0.5 = 0.5
        // - I term = 0.5 * (0.5 + 0.5) = 0.5
        // - D term = 0.2 * (0.5 - 0.5) / 1.0 = 0
        // Total = 0.5 + 0.5 + 0 = 1.0
        let rate = controller.control(dt, 0.5, 0.0);
        assert_relative_eq!(rate, 1.0, epsilon = EPSILON, max_relative = 1e-3);

        // Third call with increased error:
        // - P term = 1.0 * 0.6 = 0.6
        // - I term = 0.5 * (0.5 + 0.5 + 0.6) = 0.8
        // - D term = 0.2 * (0.6 - 0.5) / 1.0 = 0.02
        // Total = 0.6 + 0.8 + 0.02 = 1.42
        let rate = controller.control(dt, 0.6, 0.0);
        assert_relative_eq!(rate, 1.42, epsilon = EPSILON, max_relative = 1e-3);
    }

    /// Tests that the reset method properly clears controller state.
    #[test]
    fn test_reset_method() {
        let config = YawControllerConfig {
            gains: Gains {
                p: 0.0,
                i: 1.0,
                d: 0.0,
            },
            max_integral: 10.0,
            max_rate_radps: 5.0,
        };

        let mut controller = YawController::with_config(&config);
        let dt = 1.0;

        // Accumulate some error
        controller.control(dt, 0.5, 0.0); // First call
        controller.control(dt, 0.5, 0.0); // Second call

        // Verify we have accumulated error (integrated_error should be around 1.0)
        let rate_before_reset = controller.control(dt, 0.5, 0.0);
        assert!(
            rate_before_reset > 1.0,
            "Controller should have accumulated error"
        );

        // Reset the controller
        controller.reset();

        // First call after reset should behave like initial call
        let rate_after_reset = controller.control(dt, 0.5, 0.0);
        assert_relative_eq!(
            rate_after_reset,
            0.5, // I gain * error * dt
            epsilon = EPSILON,
            max_relative = 1e-3,
            max_relative = 1e-3
        );
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

        let mut controller = YawController::with_config(&config);

        // Control with a 1-second time step
        let rate_1s = controller.control(1.0, 0.5, 0.0);
        assert_relative_eq!(rate_1s, 0.5, epsilon = EPSILON, max_relative = 1e-3);

        // Next control with a 0.5-second time step should accumulate half as much
        let rate_05s = controller.control(0.5, 0.5, 0.0);
        assert_relative_eq!(
            rate_05s,
            0.5 + 0.25, // Previous 0.5 plus new 0.5*0.5
            epsilon = EPSILON,
            max_relative = 1e-3
        );

        // Next control with a 0.25-second time step
        let rate_025s = controller.control(0.25, 0.5, 0.0);
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

        let mut controller = YawController::with_config(&config);

        // Use a very small time step
        let dt = 0.000001; // 1 microsecond
        let error = 0.5;
        let rate = controller.control(dt, error, 0.0);

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
        let rate = controller.control(dt, error, 0.0);
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
        let mut controller = create_p_only_controller(1.0, 5.0);

        let dt = 0.01;

        // Test case: desired = 175°, current = -175°
        // Shortest path is -10° (not +350°)
        let desired_yaw = 175.0 * PI / 180.0;
        let estimated_yaw = -175.0 * PI / 180.0;

        let rate = controller.control(dt, desired_yaw, estimated_yaw);
        let expected_rate = -10.0 * PI / 180.0; // -10 degrees converted to radians
        assert_relative_eq!(rate, expected_rate, epsilon = EPSILON, max_relative = 1e-3);

        // Test case: desired = -175°, current = 175°
        // Shortest path is +10° (not -350°)
        let desired_yaw = -175.0 * PI / 180.0;
        let estimated_yaw = 175.0 * PI / 180.0;

        let rate = controller.control(dt, desired_yaw, estimated_yaw);
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
        let mut controller = YawController::new();
        let dt = 0.01;

        // Valid inputs should not panic
        let _ = controller.control(dt, 0.0, 0.0);
        let _ = controller.control(dt, PI, -PI);
        let _ = controller.control(dt, 2.0 * PI, 0.0);
    }

    #[test]
    #[should_panic(expected = "Time step must be positive")]
    fn test_zero_time_step() {
        let mut controller = YawController::new();
        let dt = 0.0;
        let _ = controller.control(dt, 0.0, 0.0);
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
        for (target, equivalent) in cases {
            let rate = YawController::new().control(0.01, target, 0.2);
            let expected = YawController::new().control(0.01, equivalent, 0.2);
            assert_relative_eq!(rate, expected, epsilon = 1e-3);
        }
    }

    #[test]
    fn non_finite_target_holds_and_does_not_poison_later_ticks() {
        let mut controller = YawController::new();
        for target in [f32::NAN, f32::INFINITY] {
            assert!(controller.control(0.01, target, 0.2) == 0.0);
        }
        let after = controller.control(0.01, 1.0, 0.2);
        let fresh = YawController::new().control(0.01, 1.0, 0.2);
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
        let mut controller = YawController::with_config(&config);

        controller.control(0.01, 1.0, 0.2); // error 0.8 becomes the previous error
        controller.control(0.01, f32::NAN, 0.2); // gap
        let after_gap = controller.control(0.01, 0.5, 0.2); // error 0.3

        // Without the reset, D would add 0.5 * (0.3 - 0.8) / 0.01 = -25 rad/s.
        let fresh = YawController::with_config(&config).control(0.01, 0.5, 0.2);
        assert_relative_eq!(after_gap, fresh);
    }

    /// Tests that output rate is properly limited to `max_rate_radps`.
    #[test]
    fn test_max_rate_limiting() {
        // Test that output is clamped to max_rate_radps
        let mut controller = create_p_only_controller(10.0, 2.0);

        let dt = 0.01;

        // Error of 0.5 rad with P=10 would give 5.0 rad/s
        // Should be limited to 2.0 rad/s
        let rate = controller.control(dt, 0.5, 0.0);

        assert_relative_eq!(rate, 2.0, epsilon = EPSILON, max_relative = 1e-3);

        // Test negative limiting too
        let rate = controller.control(dt, -0.5, 0.0);
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

        let mut controller = YawController::with_config(&config);

        let dt = 0.1; // 0.1 seconds

        // Simulate a 90-degree turn
        let desired_yaw = 90.0 * PI / 180.0;
        let mut current_yaw = 0.0;
        let mut rates = Vec::new();

        // Simulate control loop for 2 seconds (20 iterations)
        for _ in 0..20 {
            let rate = controller.control(dt, desired_yaw, current_yaw);
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
    fn rejects_invalid_limits_at_construction() {
        // (max_integral, max_rate_radps)
        let cases = [
            (-1.0, 1.0),
            (f32::NAN, 1.0),
            (1.0, -1.0),
            (1.0, f32::INFINITY),
        ];
        for (max_integral, max_rate_radps) in cases {
            let result = std::panic::catch_unwind(|| {
                YawController::with_config(&YawControllerConfig {
                    max_integral,
                    max_rate_radps,
                    ..Default::default()
                })
            });
            assert!(
                result.is_err(),
                "({max_integral}, {max_rate_radps}) was accepted"
            );
        }
    }
}
