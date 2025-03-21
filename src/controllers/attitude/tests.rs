use std::time::Duration;

use super::*;

// Helper function to create a default controller for tests
fn default_controller() -> AttitudeController {
    AttitudeController::new(&AttitudeConfiguration::default())
}

#[test]
fn test_new_with_default_config() {
    // Assemble
    let config = AttitudeConfiguration::default();

    // Activate
    let controller = AttitudeController::new(&config);

    // Assert
    assert_eq!(controller.gains.p, 3.0);
    // assert_eq!(controller.gains.i, 0.0);
    // assert_eq!(controller.gains.d, 0.01);
    assert_eq!(controller.max_angles, vec2(FRAC_PI_4, FRAC_PI_4));
    assert_eq!(controller.max_rate, vec2(2.0, 2.0));
    assert_eq!(controller.max_integral, 100.0);
    assert_eq!(controller.integral_term, vec2(0.0, 0.0));
    assert_eq!(controller.last_error, vec2(0.0, 0.0));
}

#[test]
#[should_panic(expected = "Maximum angles must be non-negative")]
fn test_new_panics_on_negative_max_angles() {
    // Assemble
    let config = AttitudeConfiguration {
        max_angles: vec2(-1.0, FRAC_PI_4),
        ..AttitudeConfiguration::default()
    };

    // Activate
    let _controller = AttitudeController::new(&config);

    // Assert (panic expected, no assertions needed)
}

#[test]
#[should_panic(expected = "Maximum rates must be non-negative")]
fn test_new_panics_on_negative_max_rate() {
    // Assemble
    let config = AttitudeConfiguration {
        max_rate: vec2(2.0, -2.0),
        ..AttitudeConfiguration::default()
    };

    // Activate
    let _controller = AttitudeController::new(&config);

    // Assert (panic expected, no assertions needed)
}

#[test]
fn test_control_zero_error() {
    // Assemble
    let mut controller = default_controller();
    let dt = Duration::from_secs_f32(0.1);
    let desired_angles = vec2(0.0, 0.0); // No tilt requested
    let estimated_angles = vec2(0.0, 0.0); // Already at target

    // Activate
    let rates = controller.control(dt, desired_angles, estimated_angles);

    // Assert
    assert_eq!(rates, vec2(0.0, 0.0)); // No error, no rate output
    assert_eq!(controller.integral_term, vec2(0.0, 0.0)); // No integral buildup
    assert_eq!(controller.last_error, vec2(0.0, 0.0)); // Error remains zero
}

#[test]
fn test_control_proportional_response() {
    // Assemble
    let mut controller = default_controller();
    controller.gains = Gains {
        p: 2.0, // Proportional gain
        i: 0.0,
        d: 0.0,
    }; // No integral or derivative initially
    let dt = Duration::from_secs_f32(0.1);
    let desired_angles = vec2(1.0, -1.0); // Max roll right, max pitch down
    let estimated_angles = vec2(0.0, 0.0); // Starting at zero

    // Activate
    let rates = controller.control(dt, desired_angles, estimated_angles);

    // Assert
    let expected_error = vec2(FRAC_PI_4, -FRAC_PI_4); // 45° in radians
    let expected_rates = controller.gains.p * expected_error; // P=7.0, I=0, D=0 initially

    assert!((rates.x - expected_rates.x).abs() < 1e-6);
    assert!((rates.y - expected_rates.y).abs() < 1e-6);
    assert_eq!(controller.last_error, expected_error);
    assert_eq!(controller.integral_term, dt.as_secs_f32() * expected_error); // Small integral buildup
}

#[test]
fn test_control_rate_clamping() {
    // Assemble
    let mut controller = AttitudeController::new(&AttitudeConfiguration {
        gains: Gains {
            p: 100.0,
            i: 0.0,
            d: 0.0,
        }, // High P gain to exceed max_rate
        max_rate: vec2(2.0, 2.0),
        ..AttitudeConfiguration::default()
    });
    let dt = Duration::from_secs_f32(0.1);
    let desired_angles = vec2(1.0, 1.0); // Max roll and pitch
    let estimated_angles = vec2(0.0, 0.0);

    // Activate
    let rates = controller.control(dt, desired_angles, estimated_angles);

    // Assert
    assert_eq!(rates, vec2(2.0, 2.0)); // Clamped to max_rate
}

#[test]
fn test_control_integral_buildup() {
    // Assemble
    let mut controller = AttitudeController::new(&AttitudeConfiguration {
        gains: Gains {
            p: 1.0,
            i: 1.0,
            d: 0.0,
        }, // Enable integral term
        ..AttitudeConfiguration::default()
    });
    let dt = Duration::from_secs_f32(0.1);
    let desired_angles = vec2(0.5, 0.5);
    let estimated_angles = vec2(0.0, 0.0);

    // Activate
    let _rates1 = controller.control(dt, desired_angles, estimated_angles);
    let rates2 = controller.control(dt, desired_angles, estimated_angles);

    // Assert
    let error = vec2(0.5 * FRAC_PI_4, 0.5 * FRAC_PI_4);
    let expected_integral = dt.as_secs_f32() * 2.0 * error; // Two steps
    assert_eq!(controller.integral_term, expected_integral);
    let expected_rates = 1.0 * error + 1.0 * expected_integral; // P + I terms
    assert!((rates2.x - expected_rates.x).abs() < 1e-6);
    assert!((rates2.y - expected_rates.y).abs() < 1e-6);
}

#[test]
fn test_control_integral_clamping() {
    // Assemble
    let mut controller = AttitudeController::new(&AttitudeConfiguration {
        gains: Gains {
            p: 0.0,
            i: 8000.0,
            d: 0.0,
        }, // High I to hit max_integral
        max_integral: 1.0, // Low limit for testing
        ..AttitudeConfiguration::default()
    });
    let dt = Duration::from_secs_f32(2.0); // Large dt to quickly reach max_integral
    let desired_angles = vec2(1.0, 1.0);
    let estimated_angles = vec2(0.0, 0.0);

    // Activate
    let rates = controller.control(dt, desired_angles, estimated_angles);

    // Assert
    assert_eq!(controller.integral_term, vec2(1.0, 1.0)); // Clamped to max_integral
    assert_eq!(rates, vec2(2.0, 2.0));
}

#[test]
fn test_control_derivative_response() {
    // Assemble
    let mut controller = AttitudeController::new(&AttitudeConfiguration {
        gains: Gains {
            p: 0.0,
            i: 0.0,
            d: 0.5,
        }, // Only D term
        ..AttitudeConfiguration::default()
    });
    let dt = Duration::from_secs_f32(0.1);
    let desired_angles = vec2(1.0, 1.0);
    let estimated_angles1 = vec2(0.0, 0.0);
    let estimated_angles2 = vec2(0.5 * FRAC_PI_4, 0.5 * FRAC_PI_4);

    // Activate
    let _rates1 = controller.control(dt, desired_angles, estimated_angles1);
    let rates2 = controller.control(dt, desired_angles, estimated_angles2);

    // Assert
    let error1 = vec2(FRAC_PI_4, FRAC_PI_4);
    let error2 = error1 - estimated_angles2;
    let derivative = (error2 - error1) / dt.as_secs_f32();
    let expected_rates = controller.gains.d * derivative; // D gain = 1.0
    assert!((rates2.x - expected_rates.x).abs() < 1e-6);
    assert!((rates2.y - expected_rates.y).abs() < 1e-6);
}

#[test]
#[should_panic(expected = "Time step must be positive")]
fn test_control_panics_on_zero_dt() {
    // Assemble
    let mut controller = default_controller();
    let dt = Duration::from_secs_f32(0.0);
    let desired_angles = vec2(0.0, 0.0);
    let estimated_angles = vec2(0.0, 0.0);

    // Activate
    let _rates = controller.control(dt, desired_angles, estimated_angles);

    // Assert (panic expected)
}

#[test]
#[should_panic(expected = "Desired roll angle (2) must be in [-1, 1]")]
fn test_control_panics_on_invalid_desired_angles() {
    // Assemble
    let mut controller = default_controller();
    let dt = Duration::from_secs_f32(0.1);
    let desired_angles = vec2(2.0, 0.0); // Out of range
    let estimated_angles = vec2(0.0, 0.0);

    // Activate
    let _rates = controller.control(dt, desired_angles, estimated_angles);

    // Assert (panic expected)
}
