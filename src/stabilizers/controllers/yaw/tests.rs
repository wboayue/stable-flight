use std::time::Duration;

use approx::{assert_relative_eq, relative_eq};

use crate::stabilizers::Gains;

use super::*;

const EPSILON: f32 = 1e-5;

// Helper functions for test setup

/// Creates a controller with only proportional gain for testing
fn create_p_only_controller(p_gain: f32, max_rate: f32) -> YawController {
    let config = YawControllerConfig {
        gains: Gains {
            p: p_gain,
            i: 0.0,
            d: 0.0,
        },
        max_integral: 10.0,
        max_rate,
    };

    YawController::with_config(&config)
}

/// Creates a controller with only integral gain for testing
fn create_i_only_controller(i_gain: f32, max_integral: f32, max_rate: f32) -> YawController {
    let config = YawControllerConfig {
        gains: Gains {
            p: 0.0,
            i: i_gain,
            d: 0.0,
        },
        max_integral,
        max_rate,
    };

    YawController::with_config(&config)
}

/// Creates a controller with only derivative gain for testing
fn create_d_only_controller(d_gain: f32, max_rate: f32) -> YawController {
    let config = YawControllerConfig {
        gains: Gains {
            p: 0.0,
            i: 0.0,
            d: d_gain,
        },
        max_integral: 10.0,
        max_rate,
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
    assert_relative_eq!(config.max_rate, 2.0, epsilon = EPSILON, max_relative = 1e-3);
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
        max_rate: 2.0,
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
    let dt = Duration::from_millis(10);
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
    let dt = Duration::from_secs(1); // 1 second for easy calculation
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

    let dt = Duration::from_secs(1); // 1 second for easy calculation

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
        max_rate: 5.0,
    };

    let mut controller = YawController::with_config(&config);

    let dt = Duration::from_secs(1);

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
        max_rate: 5.0,
    };

    let mut controller = YawController::with_config(&config);
    let dt = Duration::from_secs(1);

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
        max_rate: 5.0,
    };

    let mut controller = YawController::with_config(&config);

    // Control with a 1-second time step
    let rate_1s = controller.control(Duration::from_secs(1), 0.5, 0.0);
    assert_relative_eq!(rate_1s, 0.5, epsilon = EPSILON, max_relative = 1e-3);

    // Next control with a 0.5-second time step should accumulate half as much
    let rate_05s = controller.control(Duration::from_millis(500), 0.5, 0.0);
    assert_relative_eq!(
        rate_05s,
        0.5 + 0.25, // Previous 0.5 plus new 0.5*0.5
        epsilon = EPSILON,
        max_relative = 1e-3
    );

    // Next control with a 0.25-second time step
    let rate_025s = controller.control(Duration::from_millis(250), 0.5, 0.0);
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
        max_rate: 5.0,
    };

    let mut controller = YawController::with_config(&config);

    // Use a very small time step
    let dt = Duration::from_micros(1); // 1 microsecond
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

    let dt = Duration::from_millis(10);

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
fn test_angle_diff_function() {
    // No wrapping needed
    assert_relative_eq!(
        angle_diff(0.5, 0.0),
        0.5,
        epsilon = EPSILON,
        max_relative = 1e-3
    );

    // Basic wrapping
    assert_relative_eq!(
        angle_diff(PI + 0.1, -PI - 0.1),
        0.2,
        epsilon = EPSILON,
        max_relative = 1e-3
    );

    // Common real-world cases (in degrees, converted to radians)
    assert_relative_eq!(
        angle_diff(350.0 * PI / 180.0, 10.0 * PI / 180.0),
        -20.0 * PI / 180.0,
        epsilon = EPSILON,
        max_relative = 1e-3
    );

    assert_relative_eq!(
        angle_diff(10.0 * PI / 180.0, 350.0 * PI / 180.0),
        20.0 * PI / 180.0,
        epsilon = EPSILON,
        max_relative = 1e-3
    );

    // Edge cases
    assert_relative_eq!(
        angle_diff(PI, -PI),
        0.0,
        epsilon = EPSILON,
        max_relative = 1e-3
    );

    assert_relative_eq!(
        angle_diff(0.0, 2.0 * PI),
        0.0,
        epsilon = EPSILON,
        max_relative = 1e-3
    );
}

/// Tests that input validation checks work as expected.
#[test]
fn test_valid_inputs() {
    let mut controller = YawController::new();
    let dt = Duration::from_millis(10);

    // Valid inputs should not panic
    let _ = controller.control(dt, 0.0, 0.0);
    let _ = controller.control(dt, PI, -PI);
    let _ = controller.control(dt, 2.0 * PI, 0.0);
}

#[test]
#[should_panic(expected = "Time step must be positive")]
fn test_zero_time_step() {
    let mut controller = YawController::new();
    let dt = Duration::from_millis(0);
    let _ = controller.control(dt, 0.0, 0.0);
}

#[test]
#[should_panic(expected = "must be in the range")]
fn test_invalid_yaw_range() {
    let mut controller = YawController::new();
    let dt = Duration::from_millis(10);
    let _ = controller.control(dt, 3.0 * PI, 0.0);
}

/// Tests that output rate is properly limited to max_rate.
#[test]
fn test_max_rate_limiting() {
    // Test that output is clamped to max_rate
    let mut controller = create_p_only_controller(10.0, 2.0);

    let dt = Duration::from_millis(10);

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
        max_rate: 3.0,
    };

    let mut controller = YawController::with_config(&config);

    let dt = Duration::from_millis(100); // 0.1 seconds

    // Simulate a 90-degree turn
    let desired_yaw = 90.0 * PI / 180.0;
    let mut current_yaw = 0.0;
    let mut rates = Vec::new();

    // Simulate control loop for 2 seconds (20 iterations)
    for _ in 0..20 {
        let rate = controller.control(dt, desired_yaw, current_yaw);
        rates.push(rate);

        // Simple integration to update current yaw for next iteration
        current_yaw += rate * dt.as_secs_f32();

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
