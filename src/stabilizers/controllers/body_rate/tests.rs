use std::time::Duration;

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
    let dt = Duration::from_secs_f32(0.1);
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
    let dt = Duration::from_secs_f32(0.1);
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
    let dt = Duration::from_secs_f32(0.1);
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
    let dt = Duration::from_secs_f32(0.1);
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
    let dt = Duration::from_secs_f32(0.0001); // Very small time step
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
    let normal_dt = Duration::from_secs_f32(0.1);
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
    let dt = Duration::from_secs_f32(0.1);
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
    let expected_increase = dt.as_secs_f32() * desired_rates;
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
    let dt = Duration::from_secs_f32(0.1);
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
    let dt = Duration::from_secs_f32(0.1);

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
    let error_change = (desired_rates - new_estimated_rates) - (desired_rates - estimated_rates);
    let expected_deriv = controller.gains.d * error_change / dt.as_secs_f32();

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
    let dt = Duration::from_secs_f32(0.1);
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
    let expected_x =
        (1.0 * 2.0 * max_angular_rates.x) + (0.5 * 1.0) + (0.25 * 4.0 * max_angular_rates.x / dt);

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
    let dt = Duration::from_secs_f32(0.0);
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
    let dt = Duration::from_secs_f32(0.1);
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
    let dt = Duration::from_secs_f32(0.1);
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
