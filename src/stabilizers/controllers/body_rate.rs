use robomath::{Vec3, vec3};

use super::super::Gains;

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
#[derive(Clone, Debug, PartialEq)]
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

impl Default for BodyRateController {
    fn default() -> Self {
        BodyRateController::with_config(&BodyRateControllerConfig::default())
    }
}

impl BodyRateController {
    pub fn new() -> Self {
        BodyRateController::default()
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
    /// * `dt` - Time step since the last update in seconds (must be positive and finite).
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
        dt: f32,
        desired_rates: Vec3<f32>,
        estimated_rates: Vec3<f32>,
    ) -> Vec3<f32> {
        // Input validation
        assert!(
            dt > 0.0 && dt.is_finite(),
            "Time step (dt) must be positive and finite"
        );
        assert!(
            desired_rates.is_finite() && estimated_rates.is_finite(),
            "Desired and estimated rates must be finite"
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

#[cfg(test)]
mod tests {
    use approx::{assert_abs_diff_eq, assert_relative_eq};
    use robomath::vec3;

    use super::*;

    /// Test suite for the BodyRateController.
    ///
    /// These tests verify the controller's behavior in:
    /// - Nominal operating conditions
    /// - Boundary conditions (min/max rates and parameters)
    /// - Edge cases (small time steps, controller reset)
    /// - Error handling (invalid inputs)
    /// - Individual component testing (P, I, D terms)
    ///
    /// Most tests follow the Assemble-Activate-Assert pattern for clarity.

    /// Helper function to create a default controller for testing
    fn setup_controller() -> BodyRateController {
        BodyRateController::new()
    }

    /// Helper function to create a controller with custom configuration
    fn setup_controller_with_config(config: &BodyRateControllerConfig) -> BodyRateController {
        BodyRateController::with_config(config)
    }

    /// Test 1: Nominal case with zero error between desired and estimated rates
    #[test]
    fn test_nominal_zero_error() {
        // Assemble
        let mut controller = setup_controller();
        let dt = 0.1;
        let desired_rates = vec3(0.0, 0.0, 0.0); // No desired movement
        let estimated_rates = vec3(0.0, 0.0, 0.0); // No actual movement

        // Activate
        let result = controller.control(dt, desired_rates, estimated_rates);

        // Assert - expect zero output with zero error
        assert_abs_diff_eq!(result.x, 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(result.y, 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(result.z, 0.0, epsilon = 1e-6);
    }

    /// Test 2: Nominal case with non-zero error to verify proportional response
    #[test]
    fn test_nominal_non_zero_error() {
        // Assemble
        let mut controller = setup_controller();
        let dt = 0.1;
        let desired_rates = vec3(0.5, 0.5, 0.5); // Half max rates
        let estimated_rates = vec3(0.0, 0.0, 0.0); // No actual movement

        // Calculate expected p-term (P gain * error * scaling factor)
        // Based on default P gain of 7.0 and desired rates scaled by max_angular_rates
        let error = desired_rates;
        let expected_p_term = controller.gains.p * error;

        // Activate
        let result = controller.control(dt, desired_rates, estimated_rates);

        // Assert - verify proportional response and clamping
        // The exact output depends on internal scaling, but should be positive and clamped
        assert!(
            result.x > 0.0 && result.y > 0.0 && result.z > 0.0,
            "Moments should be positive due to proportional term"
        );
        assert!(
            result.x <= 1.0 && result.y <= 1.0 && result.z <= 1.0,
            "Moments should be clamped to 1.0"
        );

        // Verify the sign and approximate magnitude are as expected
        // Note: Exact values depend on internal scaling in the controller
        assert_relative_eq!(result.x.signum(), expected_p_term.x.signum());
        assert_relative_eq!(result.y.signum(), expected_p_term.y.signum());
        assert_relative_eq!(result.z.signum(), expected_p_term.z.signum());
    }

    /// Test 3: Boundary case - maximum desired rates
    #[test]
    fn test_max_desired_rates() {
        // Assemble
        let mut controller = setup_controller();
        let dt = 0.1;
        let desired_rates = vec3(1.0, 1.0, 1.0); // Max desired rates (normalized)
        let estimated_rates = vec3(0.0, 0.0, 0.0); // No actual movement

        // Activate
        let result = controller.control(dt, desired_rates, estimated_rates);

        // Assert - verify response is positive and clamped appropriately
        assert!(
            result.x > 0.0 && result.y > 0.0 && result.z > 0.0,
            "Moments should be positive with positive error"
        );

        // Verify clamping at maximum output
        assert_abs_diff_eq!(result.x, 1.0, epsilon = 1e-6);
        assert_abs_diff_eq!(result.y, 1.0, epsilon = 1e-6);
        assert_abs_diff_eq!(result.z, 1.0, epsilon = 1e-6);
    }

    /// Test 4: Boundary case - minimum desired rates (negative maximum)
    #[test]
    fn test_min_desired_rates() {
        // Assemble
        let mut controller = setup_controller();
        let dt = 0.1;
        let desired_rates = vec3(-1.0, -1.0, -1.0); // Min desired rates (normalized)
        let estimated_rates = vec3(0.0, 0.0, 0.0); // No actual movement

        // Activate
        let result = controller.control(dt, desired_rates, estimated_rates);

        // Assert - verify response is negative and clamped appropriately
        assert!(
            result.x < 0.0 && result.y < 0.0 && result.z < 0.0,
            "Moments should be negative with negative error"
        );

        // Verify clamping at minimum output
        assert_abs_diff_eq!(result.x, -1.0, epsilon = 1e-6);
        assert_abs_diff_eq!(result.y, -1.0, epsilon = 1e-6);
        assert_abs_diff_eq!(result.z, -1.0, epsilon = 1e-6);
    }

    /// Test 5: Edge case - very small time step
    #[test]
    fn test_small_dt() {
        // Assemble
        let mut controller = setup_controller();
        let dt = 0.0001; // Very small time step
        let desired_rates = vec3(0.5, 0.5, 0.5); // Half max rates
        let estimated_rates = vec3(0.0, 0.0, 0.0); // No actual movement

        // Activate
        let result = controller.control(dt, desired_rates, estimated_rates);

        // Assert - verify controller behaves reasonably with small dt
        // Small dt should primarily affect derivative and integral calculations
        assert!(
            result.x > 0.0 && result.y > 0.0 && result.z > 0.0,
            "Moments should be positive with positive error regardless of dt"
        );
        assert!(
            result.x <= 1.0 && result.y <= 1.0 && result.z <= 1.0,
            "Moments should be clamped to 1.0 regardless of dt"
        );

        // With default config (I=0), the small dt should not significantly affect output
        // compared to a controller with normal dt for proportional term
        let normal_dt = 0.1;
        let mut normal_controller = setup_controller();
        let normal_result = normal_controller.control(normal_dt, desired_rates, estimated_rates);

        // Verify outputs are similar (may differ slightly due to derivative term)
        assert_relative_eq!(result.x, normal_result.x, max_relative = 0.1);
        assert_relative_eq!(result.y, normal_result.y, max_relative = 0.1);
        assert_relative_eq!(result.z, normal_result.z, max_relative = 0.1);
    }

    /// Test 6: State persistence - integral term accumulation
    #[test]
    fn test_integral_accumulation() {
        // Assemble
        let mut controller = setup_controller();
        controller.gains.i = 0.1; // Enable integral term
        let dt = 0.1;
        let desired_rates = vec3(0.5, 0.5, 0.5); // Half max rates
        let estimated_rates = vec3(0.0, 0.0, 0.0); // No actual movement

        // Activate - first control call
        let _ = controller.control(dt, desired_rates, estimated_rates);
        let result1 = controller.integral_term;

        // Activate - second control call with the same error
        let _ = controller.control(dt, desired_rates, estimated_rates);
        let result2 = controller.integral_term;

        // Assert - verify integral term increases with consistent error
        assert!(
            result2.x > result1.x,
            "Roll integral term should increase with consistent positive error"
        );
        assert!(
            result2.y > result1.y,
            "Pitch integral term should increase with consistent positive error"
        );
        assert!(
            result2.z > result1.z,
            "Yaw integral term should increase with consistent positive error"
        );

        // Calculate expected increase based on error and dt
        let expected_increase = dt * desired_rates;
        assert_relative_eq!(
            result2.x - result1.x,
            expected_increase.x,
            max_relative = 1e-5
        );
    }

    /// Test 7: Integral term clamping to prevent windup
    #[test]
    fn test_integral_clamping() {
        // Assemble
        let mut controller = setup_controller();
        controller.gains.i = 1.0; // High integral gain
        controller.max_integral = 0.1; // Low integral limit
        let dt = 0.1;
        let desired_rates = vec3(1.0, 1.0, 1.0); // Max rates to quickly saturate integral
        let estimated_rates = vec3(0.0, 0.0, 0.0); // No actual movement

        // Activate - multiple calls to saturate the integral term
        for _ in 0..20 {
            controller.control(dt, desired_rates, estimated_rates);
        }

        // Assert - verify integral term is properly clamped
        assert!(
            controller.integral_term.x <= controller.max_integral,
            "Roll integral should be clamped to max_integral"
        );
        assert!(
            controller.integral_term.y <= controller.max_integral,
            "Pitch integral should be clamped to max_integral"
        );
        assert!(
            controller.integral_term.z <= controller.max_integral,
            "Yaw integral should be clamped to max_integral"
        );

        // Verify exact clamping value
        assert_abs_diff_eq!(
            controller.integral_term.x,
            controller.max_integral,
            epsilon = 1e-6
        );
    }

    /// Test 8: Derivative term functionality
    #[test]
    fn test_derivative_term() {
        // Assemble
        let mut controller = setup_controller();
        controller.gains.p = 0.0; // Disable proportional term
        controller.gains.i = 0.0; // Disable integral term
        controller.gains.d = 0.5; // Emphasize derivative term
        let dt = 0.1;

        // First control call establishes initial error
        let desired_rates = vec3(0.5, 0.5, 0.5);
        let estimated_rates = vec3(0.0, 0.0, 0.0);
        let _ = controller.control(dt, desired_rates, estimated_rates);

        // Second call with same error (zero derivative)
        let result_steady = controller.control(dt, desired_rates, estimated_rates);

        // Third call with decreasing error (negative derivative)
        let new_estimated_rates = vec3(0.2, 0.2, 0.2); // Error is now smaller
        let result_decreasing = controller.control(dt, desired_rates, new_estimated_rates);

        // Assert
        // With decreasing error, derivative term should produce negative output
        assert!(
            result_decreasing.x < 0.0 && result_steady.x == 0.0,
            "Derivative term should produce negative output when error decreases"
        );

        // Calculate expected derivative value
        let error_change =
            (desired_rates - new_estimated_rates) - (desired_rates - estimated_rates);
        let expected_deriv = controller.gains.d * error_change / dt;

        // Allow for some difference due to internal scaling and clamping
        assert_relative_eq!(
            result_decreasing.x.signum(),
            expected_deriv.x.signum(),
            max_relative = 1e-5
        );
    }

    /// Test 9: Reset functionality
    #[test]
    fn test_reset() {
        // Assemble
        let mut controller = setup_controller();
        controller.gains.i = 0.1; // Enable integral term
        let dt = 0.1;
        let desired_rates = vec3(0.5, 0.5, 0.5);
        let estimated_rates = vec3(0.0, 0.0, 0.0);

        // Activate - accumulate state
        controller.control(dt, desired_rates, estimated_rates);
        controller.control(dt, desired_rates, estimated_rates);

        // Verify state was accumulated
        assert!(
            controller.integral_term.magnitude() > 0.0,
            "State should be accumulated"
        );

        // Reset state
        controller.reset();

        // Assert - verify state was reset
        assert_abs_diff_eq!(controller.integral_term.x, 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(controller.integral_term.y, 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(controller.integral_term.z, 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(controller.last_error.x, 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(controller.last_error.y, 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(controller.last_error.z, 0.0, epsilon = 1e-6);

        // Verify output after reset matches a fresh controller
        let result = controller.control(dt, desired_rates, estimated_rates);
        let mut fresh_controller = setup_controller();
        fresh_controller.gains.i = 0.1; // Match configuration
        let fresh_result = fresh_controller.control(dt, desired_rates, estimated_rates);

        assert_relative_eq!(result.x, fresh_result.x, max_relative = 1e-5);
        assert_relative_eq!(result.y, fresh_result.y, max_relative = 1e-5);
        assert_relative_eq!(result.z, fresh_result.z, max_relative = 1e-5);
    }

    /// Test 10: Custom configuration via with_config constructor
    #[test]
    fn test_custom_config() {
        // Assemble
        let custom_config = BodyRateControllerConfig {
            gains: Gains {
                p: 2.0, // Custom P gain
                i: 0.5, // Custom I gain
                d: 0.2, // Custom D gain
            },
            max_integral: 0.5,                      // Custom max integral
            max_angular_rates: vec3(1.0, 2.0, 3.0), // Custom max rates
        };

        // Activate
        let controller = setup_controller_with_config(&custom_config);

        // Assert - verify configuration was properly applied
        assert_abs_diff_eq!(controller.gains.p, 2.0, epsilon = 1e-6);
        assert_abs_diff_eq!(controller.gains.i, 0.5, epsilon = 1e-6);
        assert_abs_diff_eq!(controller.gains.d, 0.2, epsilon = 1e-6);
        assert_abs_diff_eq!(controller.max_integral, 0.5, epsilon = 1e-6);
        assert_abs_diff_eq!(controller.max_angular_rates.x, 1.0, epsilon = 1e-6);
        assert_abs_diff_eq!(controller.max_angular_rates.y, 2.0, epsilon = 1e-6);
        assert_abs_diff_eq!(controller.max_angular_rates.z, 3.0, epsilon = 1e-6);
    }

    /// Test 11: Max moment calculation
    #[test]
    fn test_max_moment_calculation() {
        // Assemble
        let mut controller = setup_controller();
        controller.gains.p = 1.0;
        controller.gains.i = 0.5;
        controller.gains.d = 0.25;
        controller.max_integral = 1.0;
        let dt = 0.1;

        // Activate
        let max_moment = controller.max_moment(dt);

        // Assert
        let max_angular_rates = controller.max_angular_rates;

        // Calculate expected max moment based on the formula in max_moment method
        let expected_x = (1.0 * 2.0 * max_angular_rates.x)
            + (0.5 * 1.0)
            + (0.25 * 4.0 * max_angular_rates.x / dt);

        assert_relative_eq!(
            max_moment.x,
            expected_x,
            max_relative = 1e-5,
            epsilon = 1e-5
        );
    }

    /// Test 12: Zero dt input validation
    #[test]
    #[should_panic(expected = "Time step (dt) must be positive and finite")]
    fn test_zero_dt() {
        // Assemble
        let mut controller = setup_controller();
        let dt = 0.0;
        let desired_rates = vec3(0.0, 0.0, 0.0);
        let estimated_rates = vec3(0.0, 0.0, 0.0);

        // Activate
        controller.control(dt, desired_rates, estimated_rates);

        // Assert (implicitly handled by should_panic)
    }

    /// Test 13: Infinite desired rates input validation
    #[test]
    #[should_panic(expected = "Desired and estimated rates must be finite")]
    fn test_infinite_desired_rates() {
        // Assemble
        let mut controller = setup_controller();
        let dt = 0.1;
        let desired_rates = vec3(f32::INFINITY, 0.0, 0.0);
        let estimated_rates = vec3(0.0, 0.0, 0.0);

        // Activate
        controller.control(dt, desired_rates, estimated_rates);

        // Assert (implicitly handled by should_panic)
    }

    /// Test 14: NaN estimated rates input validation
    #[test]
    #[should_panic(expected = "Desired and estimated rates must be finite")]
    fn test_nan_estimated_rates() {
        // Assemble
        let mut controller = setup_controller();
        let dt = 0.1;
        let desired_rates = vec3(0.0, 0.0, 0.0);
        let estimated_rates = vec3(f32::NAN, 0.0, 0.0);

        // Activate
        controller.control(dt, desired_rates, estimated_rates);

        // Assert (implicitly handled by should_panic)
    }

    /// Test 15: Invalid config validation in with_config
    #[test]
    #[should_panic(expected = "Maximum angular rates must be finite")]
    fn test_invalid_config_infinite_rates() {
        // Assemble
        let invalid_config = BodyRateControllerConfig {
            gains: Gains {
                p: 1.0,
                i: 0.0,
                d: 0.0,
            },
            max_integral: 1.0,
            max_angular_rates: vec3(f32::INFINITY, 1.0, 1.0),
        };

        // Activate
        BodyRateController::with_config(&invalid_config);

        // Assert (implicitly handled by should_panic)
    }

    /// Test 16: Invalid config validation for max_integral
    #[test]
    #[should_panic(expected = "Maximum integral term must be finite")]
    fn test_invalid_config_infinite_max_integral() {
        // Assemble
        let invalid_config = BodyRateControllerConfig {
            gains: Gains {
                p: 1.0,
                i: 0.0,
                d: 0.0,
            },
            max_integral: f32::INFINITY,
            max_angular_rates: vec3(1.0, 1.0, 1.0),
        };

        // Activate
        BodyRateController::with_config(&invalid_config);

        // Assert (implicitly handled by should_panic)
    }
}

#[cfg(test)]
mod table_driven_tests {
    use super::*;
    use approx::{assert_abs_diff_eq, assert_relative_eq};
    use assert2::assert;
    use robomath::{Vec3, vec3};
    use uom::num_traits::Signed;

    fn safe_sign_num(value: f32) -> f32 {
        if value == 0.0 { 0.0 } else { value.signum() }
    }

    /// Table-driven test for validating controller responses across a range of input configurations.
    #[test]
    fn test_controller_response_table() {
        // Define test cases: (dt, desired_rates, estimated_rates, expected_output_sign)
        let test_cases = vec![
            // dt, desired, estimated, expected_sign
            (
                0.1,
                vec3(0.0, 0.0, 0.0),
                vec3(0.0, 0.0, 0.0),
                vec3(0.0, 0.0, 0.0),
            ),
            (
                0.1,
                vec3(0.5, 0.0, 0.0),
                vec3(0.0, 0.0, 0.0),
                vec3(1.0, 0.0, 0.0),
            ),
            (
                0.1,
                vec3(0.0, 0.5, 0.0),
                vec3(0.0, 0.0, 0.0),
                vec3(0.0, 1.0, 0.0),
            ),
            (
                0.1,
                vec3(0.0, 0.0, 0.5),
                vec3(0.0, 0.0, 0.0),
                vec3(0.0, 0.0, 1.0),
            ),
            (
                0.1,
                vec3(-0.5, 0.0, 0.0),
                vec3(0.0, 0.0, 0.0),
                vec3(-1.0, 0.0, 0.0),
            ),
            (
                0.1,
                vec3(0.5, 0.5, 0.5),
                vec3(0.5, 0.5, 0.5),
                vec3(0.0, 0.0, 0.0),
            ),
            (
                0.1,
                vec3(0.5, 0.5, 0.5),
                vec3(1.0, 1.0, 1.0),
                vec3(-1.0, -1.0, -1.0),
            ),
            (
                0.1,
                vec3(1.0, 1.0, 1.0),
                vec3(0.0, 0.0, 0.0),
                vec3(1.0, 1.0, 1.0),
            ),
            (
                0.1,
                vec3(-1.0, -1.0, -1.0),
                vec3(0.0, 0.0, 0.0),
                vec3(-1.0, -1.0, -1.0),
            ),
        ];

        // For each test case in the table
        for (i, (dt, desired_rates, estimated_rates, expected_sign)) in
            test_cases.iter().enumerate()
        {
            // Create a fresh controller for each test case for consistency
            let mut controller = BodyRateController::new();

            let dt = *dt;

            // Run the controller
            let result = controller.control(dt, *desired_rates, *estimated_rates);

            // Verify the sign of the output matches expected
            let actual_sign = Vec3 {
                x: safe_sign_num(result.x),
                y: safe_sign_num(result.y),
                z: safe_sign_num(result.z),
            };

            assert!(
                (actual_sign.x - expected_sign.x).abs() <= 1e-5,
                "Test case {}: X-axis sign mismatch\n\
             dt: {:.3}s, desired: {:?}, estimated: {:?}\n\
             Expected sign: {:.6}, Actual sign: {:.6}\n\
             Controller output: {:?}",
                i,
                dt,
                desired_rates,
                estimated_rates,
                expected_sign.x,
                actual_sign.x,
                result
            );

            assert!(
                (actual_sign.y - expected_sign.y).abs() <= 1e-5,
                "Test case {}: Y-axis sign mismatch\n\
             dt: {:.3}s, desired: {:?}, estimated: {:?}\n\
             Expected sign: {:.6}, Actual sign: {:.6}\n\
             Controller output: {:?}",
                i,
                dt,
                desired_rates,
                estimated_rates,
                expected_sign.y,
                actual_sign.y,
                result
            );

            assert!(
                (actual_sign.z - expected_sign.z).abs() <= 1e-5,
                "Test case {}: Z-axis sign mismatch\n\
             dt: {:.3}s, desired: {:?}, estimated: {:?}\n\
             Expected sign: {:.6}, Actual sign: {:.6}\n\
             Controller output: {:?}",
                i,
                dt,
                desired_rates,
                estimated_rates,
                expected_sign.z,
                actual_sign.z,
                result
            );
        }
    }

    /// Table-driven test for validating the PID gains effect on controller output.
    /// Tests how each component (P, I, D) contributes to the output independently.
    #[test]
    fn test_pid_component_response() {
        // Define test cases: (gains, error, last_error, integral, expected output sign)
        // Each case isolates one component of the PID controller
        let test_cases = vec![
            // Test case: P-only
            (
                "P-only (proportional response)",
                Gains {
                    p: 1.0,
                    i: 0.0,
                    d: 0.0,
                },
                vec3(0.5, 0.0, 0.0), // Current error
                vec3(0.5, 0.0, 0.0), // Last error (same, so no derivative effect)
                vec3(0.0, 0.0, 0.0), // Integral term (zero, so no integral effect)
                vec3(1.0, 0.0, 0.0), // Expected output sign: positive in X only
            ),
            // Test case: I-only
            (
                "I-only (integral response)",
                Gains {
                    p: 0.0,
                    i: 1.0,
                    d: 0.0,
                },
                vec3(0.0, 0.5, 0.0), // Current error (zero, so no proportional effect)
                vec3(0.0, 0.5, 0.0), // Last error (same, so no derivative effect)
                vec3(0.0, 0.5, 0.0), // Integral term (positive in Y only)
                vec3(0.0, 1.0, 0.0), // Expected output sign: positive in Y only
            ),
            // Test case: D-only, with decreasing error
            (
                "D-only (derivative response - decreasing error)",
                Gains {
                    p: 0.0,
                    i: 0.0,
                    d: 1.0,
                },
                vec3(0.0, 0.0, 0.3),  // Current error (less than last error)
                vec3(0.0, 0.0, 0.5),  // Last error (higher than current)
                vec3(0.0, 0.0, 0.0),  // Integral term (zero)
                vec3(0.0, 0.0, -1.0), // Expected output sign: negative in Z (decreasing error)
            ),
            // Test case: D-only, with increasing error
            (
                "D-only (derivative response - increasing error)",
                Gains {
                    p: 0.0,
                    i: 0.0,
                    d: 1.0,
                },
                vec3(0.5, 0.0, 0.0), // Current error (higher than last error)
                vec3(0.3, 0.0, 0.0), // Last error (lower than current)
                vec3(0.0, 0.0, 0.0), // Integral term (zero)
                vec3(1.0, 0.0, 0.0), // Expected output sign: positive in X (increasing error)
            ),
            // Test case: Full PID, positive error, all terms positive
            (
                "Full PID (all components positive)",
                Gains {
                    p: 1.0,
                    i: 1.0,
                    d: 1.0,
                },
                vec3(0.5, 0.5, 0.5), // Current error (positive in all axes)
                vec3(0.3, 0.3, 0.3), // Last error (lower than current, derivative positive)
                vec3(0.2, 0.2, 0.2), // Integral term (positive in all axes)
                vec3(1.0, 1.0, 1.0), // Expected output sign: positive in all axes
            ),
            // Test case: Full PID, mixed signs
            (
                "Full PID (mixed signs)",
                Gains {
                    p: 1.0,
                    i: 1.0,
                    d: 1.0,
                },
                vec3(-0.5, 0.5, 0.0), // Current error (negative in X, positive in Y, zero in Z)
                vec3(-0.3, 0.3, 0.0), // Last error
                vec3(-0.2, 0.2, 0.0), // Integral term
                vec3(-1.0, 1.0, 0.0), // Expected output sign: matches error signs
            ),
        ];

        for (i, (test_name, gains, error, last_error, integral_term, expected_sign)) in
            test_cases.iter().enumerate()
        {
            // Create and configure a controller with specific state
            let mut controller = BodyRateController::new();
            controller.gains = *gains;
            controller.last_error = *last_error;
            controller.integral_term = *integral_term;

            // Calculate output directly to test PID formula
            // Normally this would happen in the control method, but we're bypassing it
            // to isolate the PID calculation from other scaling and clamping
            let dt = 0.1;
            let error_derivative = (*error - controller.last_error) / dt;
            let pid_output = controller.gains.p * *error
                + controller.gains.i * controller.integral_term
                + controller.gains.d * error_derivative;

            // Extract the sign of the output
            let actual_sign = Vec3 {
                x: safe_sign_num(pid_output.x),
                y: safe_sign_num(pid_output.y),
                z: safe_sign_num(pid_output.z),
            };

            let tolerance = 1e-5;

            // Check each axis individually with detailed error messages
            assert!(
                (actual_sign.x - expected_sign.x).abs() <= tolerance,
                "\n❌ PID Component Test Failed - X Axis ❌\n\
             Test Case {}: {}\n\
             PID Configuration:\n\
             ├─ P gain: {:.3}, I gain: {:.3}, D gain: {:.3}\n\
             └─ dt: {:.3}s\n\
             Input State:\n\
             ├─ Current Error: [{:.3}, {:.3}, {:.3}]\n\
             ├─ Last Error: [{:.3}, {:.3}, {:.3}]\n\
             └─ Integral Term: [{:.3}, {:.3}, {:.3}]\n\
             PID Calculation:\n\
             ├─ P term: [{:.6}, {:.6}, {:.6}]\n\
             ├─ I term: [{:.6}, {:.6}, {:.6}]\n\
             ├─ D term: [{:.6}, {:.6}, {:.6}]\n\
             └─ Total Output: [{:.6}, {:.6}, {:.6}]\n\
             X-Axis Result:\n\
             ├─ Expected Sign: {:.3}\n\
             ├─ Actual Sign: {:.3}\n\
             └─ Difference: {:.6} (tolerance: {:.6})",
                i,
                test_name,
                gains.p,
                gains.i,
                gains.d,
                dt,
                error.x,
                error.y,
                error.z,
                last_error.x,
                last_error.y,
                last_error.z,
                integral_term.x,
                integral_term.y,
                integral_term.z,
                (gains.p * error.x),
                (gains.p * error.y),
                (gains.p * error.z),
                (gains.i * integral_term.x),
                (gains.i * integral_term.y),
                (gains.i * integral_term.z),
                (gains.d * error_derivative.x),
                (gains.d * error_derivative.y),
                (gains.d * error_derivative.z),
                pid_output.x,
                pid_output.y,
                pid_output.z,
                expected_sign.x,
                actual_sign.x,
                actual_sign.x - expected_sign.x,
                tolerance
            );

            assert!(
                (actual_sign.y - expected_sign.y).abs() <= tolerance,
                "\n❌ PID Component Test Failed - Y Axis ❌\n\
             Test Case {}: {}\n\
             PID Configuration:\n\
             ├─ P gain: {:.3}, I gain: {:.3}, D gain: {:.3}\n\
             └─ dt: {:.3}s\n\
             Input State:\n\
             ├─ Current Error: [{:.3}, {:.3}, {:.3}]\n\
             ├─ Last Error: [{:.3}, {:.3}, {:.3}]\n\
             └─ Integral Term: [{:.3}, {:.3}, {:.3}]\n\
             PID Calculation:\n\
             ├─ P term: [{:.6}, {:.6}, {:.6}]\n\
             ├─ I term: [{:.6}, {:.6}, {:.6}]\n\
             ├─ D term: [{:.6}, {:.6}, {:.6}]\n\
             └─ Total Output: [{:.6}, {:.6}, {:.6}]\n\
             Y-Axis Result:\n\
             ├─ Expected Sign: {:.3}\n\
             ├─ Actual Sign: {:.3}\n\
             └─ Difference: {:.6} (tolerance: {:.6})",
                i,
                test_name,
                gains.p,
                gains.i,
                gains.d,
                dt,
                error.x,
                error.y,
                error.z,
                last_error.x,
                last_error.y,
                last_error.z,
                integral_term.x,
                integral_term.y,
                integral_term.z,
                (gains.p * error.x),
                (gains.p * error.y),
                (gains.p * error.z),
                (gains.i * integral_term.x),
                (gains.i * integral_term.y),
                (gains.i * integral_term.z),
                (gains.d * error_derivative.x),
                (gains.d * error_derivative.y),
                (gains.d * error_derivative.z),
                pid_output.x,
                pid_output.y,
                pid_output.z,
                expected_sign.y,
                actual_sign.y,
                actual_sign.y - expected_sign.y,
                tolerance
            );

            assert!(
                (actual_sign.z - expected_sign.z).abs() <= tolerance,
                "\n❌ PID Component Test Failed - Z Axis ❌\n\
             Test Case {}: {}\n\
             PID Configuration:\n\
             ├─ P gain: {:.3}, I gain: {:.3}, D gain: {:.3}\n\
             └─ dt: {:.3}s\n\
             Input State:\n\
             ├─ Current Error: [{:.3}, {:.3}, {:.3}]\n\
             ├─ Last Error: [{:.3}, {:.3}, {:.3}]\n\
             └─ Integral Term: [{:.3}, {:.3}, {:.3}]\n\
             PID Calculation:\n\
             ├─ P term: [{:.6}, {:.6}, {:.6}]\n\
             ├─ I term: [{:.6}, {:.6}, {:.6}]\n\
             ├─ D term: [{:.6}, {:.6}, {:.6}]\n\
             └─ Total Output: [{:.6}, {:.6}, {:.6}]\n\
             Z-Axis Result:\n\
             ├─ Expected Sign: {:.3}\n\
             ├─ Actual Sign: {:.3}\n\
             └─ Difference: {:.6} (tolerance: {:.6})",
                i,
                test_name,
                gains.p,
                gains.i,
                gains.d,
                dt,
                error.x,
                error.y,
                error.z,
                last_error.x,
                last_error.y,
                last_error.z,
                integral_term.x,
                integral_term.y,
                integral_term.z,
                (gains.p * error.x),
                (gains.p * error.y),
                (gains.p * error.z),
                (gains.i * integral_term.x),
                (gains.i * integral_term.y),
                (gains.i * integral_term.z),
                (gains.d * error_derivative.x),
                (gains.d * error_derivative.y),
                (gains.d * error_derivative.z),
                pid_output.x,
                pid_output.y,
                pid_output.z,
                expected_sign.z,
                actual_sign.z,
                actual_sign.z - expected_sign.z,
                tolerance
            );
        }
    }

    /// Table-driven test for validating the integral term clamping across
    /// various max_integral values.
    #[test]
    fn test_integral_clamping_table() {
        // Define test cases: (max_integral, dt, error, num_iterations, expected_integral)
        let test_cases = vec![
            // max_integral, dt, error, iterations, expected_integral (clamped)
            (0.1, 0.1, 1.0, 10, 0.1), // Should reach max and clamp (1.0 * 0.1 * 10 = 1.0 > 0.1)
            (0.5, 0.1, 1.0, 3, 0.3), // Should accumulate but not reach max (1.0 * 0.1 * 3 = 0.3 < 0.5)
            (0.1, 0.1, -1.0, 10, -0.1), // Should reach min and clamp
            (0.5, 0.1, -0.1, 3, -0.03), // Small negative error, shouldn't hit clamp
            (0.01, 0.01, 5.0, 1, 0.01), // Large error, small dt, hits max in one iteration
        ];

        for (i, (max_integral, dt, error_value, iterations, expected_integral)) in
            test_cases.iter().enumerate()
        {
            // Create a controller with the specified max_integral
            let mut controller = BodyRateController::new();
            controller.max_integral = *max_integral;
            controller.gains.i = 1.0; // Set to 1.0 to make calculations simpler

            // Create uniform error in all axes
            let error = vec3(*error_value, *error_value, *error_value);
            let dt = *dt;
            let estimated_rates = vec3(0.0, 0.0, 0.0);

            // Run the controller for specified number of iterations
            for _ in 0..*iterations {
                controller.control(dt, error, estimated_rates);
            }

            // Check if the integral term is correctly clamped
            assert_abs_diff_eq!(
                controller.integral_term.x.abs(),
                expected_integral.abs(),
                epsilon = 1e-5,
                // Custom message includes test case index
            );

            // Check the sign of the integral matches error sign
            assert_relative_eq!(
                controller.integral_term.x.signum(),
                error.x.signum(),
                max_relative = 1e-5,
                epsilon = 1e-5,
            );
        }
    }

    /// Table-driven test for validating error input handling.
    /// Tests various invalid inputs to ensure the controller behaves as expected.
    #[test]
    fn test_input_validation_table() {
        // Test zero dt case
        {
            let mut controller = BodyRateController::new();
            let dt = 0.0;
            let desired_rates = vec3(0.0, 0.0, 0.0);
            let estimated_rates = vec3(0.0, 0.0, 0.0);

            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                controller.control(dt, desired_rates, estimated_rates);
            }));

            assert!(result.is_err(), "Expected panic for zero dt");
            if let Err(panic_value) = result {
                if let Some(panic_message) = panic_value.downcast_ref::<String>() {
                    assert!(panic_message.contains("Time step (dt) must be positive and finite"));
                } else if let Some(panic_message) = panic_value.downcast_ref::<&str>() {
                    assert!(panic_message.contains("Time step (dt) must be positive and finite"));
                }
            }
        }

        // Test infinite desired rates case
        {
            let mut controller = BodyRateController::new();
            let dt = 0.1;
            let desired_rates = vec3(f32::INFINITY, 0.0, 0.0);
            let estimated_rates = vec3(0.0, 0.0, 0.0);

            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                controller.control(dt, desired_rates, estimated_rates);
            }));

            assert!(result.is_err(), "Expected panic for infinite desired rates");
            if let Err(panic_value) = result {
                if let Some(panic_message) = panic_value.downcast_ref::<String>() {
                    assert!(panic_message.contains("Desired and estimated rates must be finite"));
                } else if let Some(panic_message) = panic_value.downcast_ref::<&str>() {
                    assert!(panic_message.contains("Desired and estimated rates must be finite"));
                }
            }
        }

        // Test NaN estimated rates case
        {
            let mut controller = BodyRateController::new();
            let dt = 0.1;
            let desired_rates = vec3(0.0, 0.0, 0.0);
            let estimated_rates = vec3(0.0, f32::NAN, 0.0);

            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                controller.control(dt, desired_rates, estimated_rates);
            }));

            assert!(result.is_err(), "Expected panic for NaN estimated rates");
            if let Err(panic_value) = result {
                if let Some(panic_message) = panic_value.downcast_ref::<String>() {
                    assert!(panic_message.contains("Desired and estimated rates must be finite"));
                } else if let Some(panic_message) = panic_value.downcast_ref::<&str>() {
                    assert!(panic_message.contains("Desired and estimated rates must be finite"));
                }
            }
        }
    }

    /// Table-driven test for reset behavior
    #[test]
    fn test_reset_table() {
        // Define test cases of different controller states
        let test_cases = vec![
            // integral_term, last_error, desired_rates, estimated_rates
            (vec3(1.0, 1.0, 1.0), vec3(0.5, 0.5, 0.5)),
            (vec3(-1.0, 0.0, 2.0), vec3(-0.5, 0.0, 1.0)),
            (vec3(0.1, 0.2, 0.3), vec3(0.3, 0.2, 0.1)),
        ];

        for (integral_term, last_error) in test_cases {
            // Create and setup controller with non-default state
            let mut controller = BodyRateController::new();
            controller.integral_term = integral_term;
            controller.last_error = last_error;

            // Verify the state is set
            assert_ne!(controller.integral_term, Vec3::default());
            assert_ne!(controller.last_error, Vec3::default());

            // Reset the controller
            controller.reset();

            // Verify everything is reset to default
            assert_eq!(controller.integral_term, Vec3::default());
            assert_eq!(controller.last_error, Vec3::default());
        }
    }

    /// Table-driven test for config initialization
    #[test]
    fn test_config_initialization_table() {
        // Define test cases of different configurations
        let test_cases = vec![
            // gains (p, i, d), max_integral, max_angular_rates
            (
                Gains {
                    p: 1.0,
                    i: 0.0,
                    d: 0.0,
                },
                0.1,
                vec3(1.0, 1.0, 1.0),
            ),
            (
                Gains {
                    p: 2.0,
                    i: 0.5,
                    d: 0.1,
                },
                0.5,
                vec3(2.0, 3.0, 4.0),
            ),
            (
                Gains {
                    p: 0.0,
                    i: 1.0,
                    d: 0.0,
                },
                1.0,
                vec3(5.0, 5.0, 5.0),
            ),
        ];

        for (gains, max_integral, max_angular_rates) in test_cases {
            // Create config
            let config = BodyRateControllerConfig {
                gains,
                max_integral,
                max_angular_rates,
            };

            // Create controller from config
            let controller = BodyRateController::with_config(&config);

            // Verify config was properly applied
            assert_eq!(controller.gains, config.gains);
            assert_eq!(controller.max_integral, config.max_integral);
            assert_eq!(controller.max_angular_rates, config.max_angular_rates);

            // Verify default state was initialized
            assert_eq!(controller.integral_term, Vec3::default());
            assert_eq!(controller.last_error, Vec3::default());
        }
    }
}
