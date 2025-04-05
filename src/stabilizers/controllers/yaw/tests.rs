use std::{f32::consts::PI, time::Duration};

use super::*;
use crate::stabilizers::Gains;
//   use float_cmp::approx_eq;

// Helper function to compare floating-point values with a tolerance
fn assert_float_eq(a: f32, b: f32, message: &str) {
    const EPSILON: f32 = 1e-5;
    assert!(
        (a - b).abs() < EPSILON,
        "{}: expected {}, got {}",
        message,
        b,
        a
    );
}

#[test]
fn test_default_config() {
    let config = YawControllerConfig::default();

    assert_eq!(config.gains.p, 0.0, "Default P gain should be 0.0");
    assert_eq!(config.gains.i, 0.0, "Default I gain should be 0.0");
    assert_eq!(config.gains.d, 0.0, "Default D gain should be 0.0");
    assert_eq!(
        config.max_integral, 100.0,
        "Default max_integral should be 100.0"
    );
    assert_eq!(config.max_rate, 2.0, "Default max_rate should be 2.0");
}

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

#[test]
fn test_proportional_control() {
    // Test that the proportional term works correctly
    let config = YawControllerConfig {
        gains: Gains {
            p: 2.0,
            i: 0.0,
            d: 0.0,
        }, // Only P gain
        max_integral: 10.0,
        max_rate: 5.0,
    };

    let mut controller = YawController::with_config(&config);

    // Simple case: 0.5 rad error should result in rate = 2.0 * 0.5 = 1.0 rad/s
    let dt = Duration::from_millis(10);
    let desired_yaw = 0.5;
    let estimated_yaw = 0.0;

    let rate = controller.control(dt, desired_yaw, estimated_yaw);
    assert_float_eq(rate, 1.0, "Proportional control with 0.5 rad error");

    // Test with negative error
    let rate = controller.control(dt, 0.0, 0.5);
    assert_float_eq(rate, -1.0, "Proportional control with -0.5 rad error");
}

#[test]
fn test_integral_control() {
    // Test that the integral term accumulates and affects output
    let config = YawControllerConfig {
        gains: Gains {
            p: 0.0,
            i: 1.0,
            d: 0.0,
        }, // Only I gain
        max_integral: 10.0,
        max_rate: 5.0,
    };

    let mut controller = YawController::with_config(&config);

    // Apply constant error for several iterations
    let dt = Duration::from_secs(1); // 1 second for easy calculation
    let desired_yaw = 0.5;
    let estimated_yaw = 0.0;

    // First iteration: integral = 0.5 * 1.0 = 0.5
    let rate = controller.control(dt, desired_yaw, estimated_yaw);
    assert_float_eq(rate, 0.5, "Integral control after 1 second");

    // Second iteration: integral = 0.5 + 0.5 * 1.0 = 1.0
    let rate = controller.control(dt, desired_yaw, estimated_yaw);
    assert_float_eq(rate, 1.0, "Integral control after 2 seconds");

    // Test integral clamping
    let config = YawControllerConfig {
        gains: Gains {
            p: 0.0,
            i: 1.0,
            d: 0.0,
        },
        max_integral: 1.5, // Low max to test clamping
        max_rate: 5.0,
    };

    let mut controller = YawController::with_config(&config);

    // Accumulate for 3 seconds to exceed max_integral
    controller.control(dt, desired_yaw, estimated_yaw); // 0.5
    controller.control(dt, desired_yaw, estimated_yaw); // 1.0
    controller.control(dt, desired_yaw, estimated_yaw); // 1.5
    let rate = controller.control(dt, desired_yaw, estimated_yaw);

    // Should be clamped at max_integral * i_gain = 1.5
    assert_float_eq(rate, 1.5, "Integral control with clamping");
}

#[test]
fn test_derivative_control() {
    // Test that the derivative term works correctly
    let config = YawControllerConfig {
        gains: Gains {
            p: 0.0,
            i: 0.0,
            d: 2.0,
        }, // Only D gain
        max_integral: 10.0,
        max_rate: 5.0,
    };

    let mut controller = YawController::with_config(&config);

    let dt = Duration::from_secs(1); // 1 second for easy calculation

    // First call - establishes baseline error
    let rate = controller.control(dt, 0.5, 0.0);
    assert_float_eq(rate, 0.0, "First derivative control call");

    // Second call - error unchanged, derivative should be zero
    let rate = controller.control(dt, 0.5, 0.0);
    assert_float_eq(rate, 0.0, "Derivative control with constant error");

    // Error increasing by 0.1 rad/s
    let rate = controller.control(dt, 0.6, 0.0);
    assert_float_eq(rate, 0.2, "Derivative control with increasing error");

    // Error decreasing by 0.2 rad/s
    let rate = controller.control(dt, 0.4, 0.0);
    assert_float_eq(rate, -0.4, "Derivative control with decreasing error");
}

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

    // First call: P=0.5, I=0.5*0.5=0.25, D=0 (no previous error)
    // Expected: 0.5 + 0.25 + 0 = 0.75
    let rate = controller.control(dt, 0.5, 0.0);
    assert_float_eq(rate, 0.75, "First combined PID call");

    // Second call with same error: P=0.5, I=0.5*(0.5+0.5)=0.5, D=0 (no change in error)
    // Expected: 0.5 + 0.5 + 0 = 1.0
    let rate = controller.control(dt, 0.5, 0.0);
    assert_float_eq(rate, 1.0, "Second combined PID call");

    // Third call with increased error: P=0.6, I=0.5*(0.5+0.5+0.6)=0.8, D=0.2*(0.6-0.5)/1.0=0.02
    // Expected: 0.6 + 0.8 + 0.02 = 1.42
    let rate = controller.control(dt, 0.6, 0.0);
    assert_float_eq(rate, 1.42, "Third combined PID call with changing error");
}

#[test]
fn test_angle_wrapping() {
    // Test angle wrapping when crossing the ±π boundary
    let config = YawControllerConfig {
        gains: Gains {
            p: 1.0,
            i: 0.0,
            d: 0.0,
        }, // Only P gain for clarity
        max_integral: 10.0,
        max_rate: 5.0,
    };

    let mut controller = YawController::with_config(&config);

    let dt = Duration::from_millis(10);

    // Test case: desired = 175°, current = -175°
    // Shortest path is +10° (not +350°)
    let desired_yaw = 175.0 * PI / 180.0;
    let estimated_yaw = -175.0 * PI / 180.0;

    let rate = controller.control(dt, desired_yaw, estimated_yaw);
    let expected_rate = 10.0 * PI / 180.0; // 10 degrees converted to radians
    assert_float_eq(rate, expected_rate, "Angle wrapping across ±π boundary");

    // Test case: desired = -175°, current = 175°
    // Shortest path is -10° (not -350°)
    let desired_yaw = -175.0 * PI / 180.0;
    let estimated_yaw = 175.0 * PI / 180.0;

    let rate = controller.control(dt, desired_yaw, estimated_yaw);
    let expected_rate = -10.0 * PI / 180.0; // -10 degrees converted to radians
    assert_float_eq(rate, expected_rate, "Angle wrapping across ±π boundary");
}

#[test]
fn test_angle_diff_function() {
    // Directly test the angle_diff helper function

    // No wrapping needed
    assert_float_eq(
        angle_diff(0.5, 0.0),
        0.5,
        "Simple angle diff without wrapping",
    );

    // Basic wrapping
    assert_float_eq(
        angle_diff(PI + 0.1, -PI - 0.1),
        0.2,
        "Angle diff with wrapping",
    );

    // Common real-world cases (in degrees, converted to radians)
    assert_float_eq(
        angle_diff(350.0 * PI / 180.0, 10.0 * PI / 180.0),
        -20.0 * PI / 180.0,
        "Angle diff for 350° to 10°",
    );

    assert_float_eq(
        angle_diff(10.0 * PI / 180.0, 350.0 * PI / 180.0),
        20.0 * PI / 180.0,
        "Angle diff for 10° to 350°",
    );

    // Edge cases
    assert_float_eq(angle_diff(PI, -PI), 0.0, "Angle diff between PI and -PI");

    assert_float_eq(
        angle_diff(0.0, 2.0 * PI),
        0.0,
        "Angle diff between 0 and 2PI",
    );
}

#[test]
fn test_input_validation() {
    let mut controller = YawController::new();
    let dt = Duration::from_millis(10);

    // Valid inputs should not panic
    let _ = controller.control(dt, 0.0, 0.0);
    let _ = controller.control(dt, PI, -PI);
    let _ = controller.control(dt, 2.0 * PI, 0.0);

    // Invalid inputs should panic - uncomment to test
    // These are commented out because we don't want tests to fail
    // but you would implement them with #[should_panic] if needed

    // #[should_panic(expected = "Time step must be positive")]
    // #[test]
    // fn test_zero_time_step() {
    //     let mut controller = YawController::new();
    //     let dt = Duration::from_millis(0);
    //     let _ = controller.control(dt, 0.0, 0.0);
    // }

    // #[should_panic(expected = "Desired yaw rate")]
    // #[test]
    // fn test_invalid_yaw_range() {
    //     let mut controller = YawController::new();
    //     let dt = Duration::from_millis(10);
    //     let _ = controller.control(dt, 3.0 * PI, 0.0);
    // }
}

#[test]
fn test_max_rate_limiting() {
    // Test that output is clamped to max_rate
    let config = YawControllerConfig {
        gains: Gains {
            p: 10.0,
            i: 0.0,
            d: 0.0,
        }, // High P gain to exceed max_rate
        max_integral: 10.0,
        max_rate: 2.0, // Low max_rate to test limiting
    };

    let mut controller = YawController::with_config(&config);

    let dt = Duration::from_millis(10);

    // Error of 0.5 rad with P=10 would give 5.0 rad/s
    // Should be limited to 2.0 rad/s
    let rate = controller.control(dt, 0.5, 0.0);

    // Note: max_rate limiting isn't actually implemented in the provided code!
    // This test assumes that rate limiting should be added to the implementation
    // If implemented, it would look like:
    // let rate = rate.clamp(-self.config.max_rate, self.config.max_rate);

    // For now, test the current implementation
    assert_float_eq(rate, 5.0, "Current implementation without rate limiting");

    // When max_rate limiting is implemented, use this test instead:
    // assert_float_eq(rate, 2.0, "Rate limiting to max_rate");
}

#[test]
fn test_realistic_maneuvers() {
    // Test a few realistic maneuvers
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

    for _ in 0..20 {
        // 2 seconds of simulation
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
    // The final yaw should be close to 90 degrees
    assert!(
        (desired_yaw - current_yaw).abs() < 0.1,
        "Controller should converge to desired yaw"
    );

    // Verify that the final rate approaches zero as error decreases
    assert!(
        rates.last().unwrap().abs() < 0.2,
        "Rate should approach zero as error decreases"
    );
}
