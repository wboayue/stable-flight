use std::time::Duration;

use robomath::vec3;

use super::*;

// Helper function to create a default controller
fn setup_controller() -> BodyRateController {
    BodyRateController::default()
}

// Test 1: Nominal case with zero error
#[test]
fn test_nominal_zero_error() {
    let mut controller = setup_controller();

    let dt = Duration::from_secs_f32(0.1);
    let desired_rates = vec3(0.0, 0.0, 0.0); // No desired movement
    let estimated_rates = vec3(0.0, 0.0, 0.0); // No actual movement

    let result = controller.control(dt, desired_rates, estimated_rates);

    assert_eq!(result, vec3(0.0, 0.0, 0.0)); // Expect zero output
}

// Test 2: Nominal case with non-zero error
#[test]
fn test_nominal_non_zero_error() {
    let mut controller = setup_controller();
    
    let dt = Duration::from_secs_f32(0.1);
    let desired_rates = vec3(0.5, 0.5, 0.5); // Half max rates
    let estimated_rates = vec3(0.0, 0.0, 0.0); // No actual movement
    
    let result = controller.control(dt, desired_rates, estimated_rates);
    
    // Expect positive moments due to proportional term (P * error)
    assert!(result.x > 0.0 && result.y > 0.0 && result.z > 0.0);
    assert!(result.x <= 1.0 && result.y <= 1.0 && result.z <= 1.0);
}

// Test 3: Boundary case - max desired rates
#[test]
fn test_max_desired_rates() {
    let mut controller = setup_controller();

    let dt = Duration::from_secs_f32(0.1);
    let desired_rates = vec3(1.0, 1.0, 1.0); // Max desired rates
    let estimated_rates = vec3(0.0, 0.0, 0.0);
    
    let result = controller.control(dt, desired_rates, estimated_rates);
    
    // Expect clamped output at 1.0
    assert!(result.x > 0.0 && result.y > 0.0 && result.z > 0.0);
    assert!(result.x <= 1.0 && result.y <= 1.0 && result.z <= 1.0);
}

// Test 4: Boundary case - min desired rates
#[test]
fn test_min_desired_rates() {
    let mut controller = setup_controller();

    let dt = Duration::from_secs_f32(0.1);
    let desired_rates = vec3(-1.0, -1.0, -1.0); // Min desired rates
    let estimated_rates = vec3(0.0, 0.0, 0.0);
    
    let result = controller.control(dt, desired_rates, estimated_rates);
    
    // Expect clamped output at -1.0
    assert!(result.x < 0.0 && result.y < 0.0 && result.z < 0.0);
    assert!(result.x >= -1.0 && result.y >= -1.0 && result.z >= -1.0);
}

// Test 5: Edge case - very small dt
#[test]
fn test_small_dt() {
    let mut controller = setup_controller();

    let dt = Duration::from_secs_f32(0.0001); // Very small time step
    let desired_rates = vec3(0.5, 0.5, 0.5);
    let estimated_rates = vec3(0.0, 0.0, 0.0);

    let result = controller.control(dt, desired_rates, estimated_rates);
    
    assert!(result.x > 0.0 && result.y > 0.0 && result.z > 0.0);
    assert!(result.x <= 1.0 && result.y <= 1.0 && result.z <= 1.0);
}

// Test 6: State persistence - integral term accumulation
#[test]
fn test_integral_accumulation() {
    let mut controller = setup_controller();

    controller.gains.i = 0.1; // Enable integral term
    let dt = Duration::from_secs_f32(0.1);
    let desired_rates = vec3(0.5, 0.5, 0.5);
    let estimated_rates = vec3(0.0, 0.0, 0.0);

    let _ = controller.control(dt, desired_rates, estimated_rates);
    let result1 = controller.integral_term;
    let _ = controller.control(dt, desired_rates, estimated_rates);
    let result2 = controller.integral_term;

    // Integral term should increase over time
    assert!(result2.x > result1.x, "roll term should increase");
    assert!(result2.y > result1.y, "pitch term should increase");
    assert!(result2.z > result1.z, "yaw term should increase");
}

// Test 7: Reset functionality
#[test]
fn test_reset() {
    let mut controller = setup_controller();

    controller.gains.i = 0.1;
    let dt = Duration::from_secs_f32(0.1);
    let desired_rates = vec3(0.5, 0.5, 0.5);
    let estimated_rates = vec3(0.0, 0.0, 0.0);

    controller.control(dt, desired_rates, estimated_rates); // Accumulate state
    controller.reset(); // Reset state
    let result = controller.control(dt, desired_rates, estimated_rates);

    // After reset, expect only proportional term
    let expected = controller.gains.p * (desired_rates * controller.max_angular_rates);
    assert!(approx_eq(result.x, expected.x.clamp(-1.0, 1.0), 0.1));
    assert!(approx_eq(result.y, expected.y.clamp(-1.0, 1.0), 0.1));
    assert!(approx_eq(result.z, expected.z.clamp(-1.0, 1.0), 0.1));
}

// Test 8: Panic case - zero dt
#[test]
#[should_panic(expected = "Time step (dt) must be positive and finite")]
fn test_zero_dt() {
    let mut controller = setup_controller();

    let dt = Duration::from_secs_f32(0.0);
    let desired_rates = vec3(0.0, 0.0, 0.0);
    let estimated_rates = vec3(0.0, 0.0, 0.0);
   
    controller.control(dt, desired_rates, estimated_rates);
}

// Test 9: Panic case - infinite desired rates
#[test]
#[should_panic(expected = "Desired and estimated rates must be finite")]
fn test_infinite_desired_rates() {
    let mut controller = setup_controller();

    let dt = Duration::from_secs_f32(0.1);
    let desired_rates = vec3(f32::INFINITY, 0.0, 0.0);
    let estimated_rates = vec3(0.0, 0.0, 0.0);
    
    controller.control(dt, desired_rates, estimated_rates);
}

// Test 10: Panic case - NaN estimated rates
#[test]
#[should_panic(expected = "Desired and estimated rates must be finite")]
fn test_nan_estimated_rates() {
    let mut controller = setup_controller();

    let dt = Duration::from_secs_f32(0.1);
    let desired_rates = vec3(0.0, 0.0, 0.0);
    let estimated_rates = vec3(f32::NAN, 0.0, 0.0);
    
    controller.control(dt, desired_rates, estimated_rates);
}

// Test 11: Panic case - zero max_angular_rates
#[test]
#[should_panic(expected = "max_angular_rates.x must be non-zero and finite")]
fn test_zero_max_angular_rates() {
    let mut controller = setup_controller();

    controller.max_angular_rates.x = 0.0; // Invalid max rate
    let dt = Duration::from_secs_f32(0.1);
    let desired_rates = vec3(0.0, 0.0, 0.0);
    let estimated_rates = vec3(0.0, 0.0, 0.0);
    
    controller.control(dt, desired_rates, estimated_rates);
}

// Helper function for approximate equality
fn approx_eq(a: f32, b: f32, epsilon: f32) -> bool {
    (a - b).abs() < epsilon
}
