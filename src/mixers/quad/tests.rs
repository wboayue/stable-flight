
use crate::math::vec3;

use super::*;
use approx::assert_relative_eq;

// Helper function to create a mixer instance
fn create_mixer() -> QuadCopterMixer {
    QuadCopterMixer {}
}

// Test 1: Hover (no moments, thrust only)
#[test]
fn test_hover_no_moments() {
    let mixer = create_mixer();
    let input = ControlOutput::new(1.0, vec3(0.0, 0.0, 0.0)); // Full thrust, no moments
    let output = mixer.mix(&input);

    // Expected: all motors at 25% (1.0 / 4 * 100)
    let expected = 25.0;
    for motor in output.motors.iter() {
        assert_relative_eq!(motor, &expected, epsilon = 1e-4);
    }
}

// Test 2: Pure Roll (max roll, no thrust/pitch/yaw)
#[test]
fn test_pure_roll() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, vec3(1.0, 0.0, 0.0)); // No thrust, max positive roll
    let output = mixer.mix(&input);

    // Expected: left motors (0, 2) higher, right motors (1, 3) lower
    // Raw: roll = 0.25
    // Motor 0: 0 + 0.25 = 0.25
    // Motor 1: 0 - 0.25 = -0.25
    // Motor 2: 0 + 0.25 = 0.25
    // Motor 3: 0 - 0.25 = -0.25
    // After scaling to [0, 1]: [0.25, -0.25] -> [1, 0]
    assert_relative_eq!(output.motors[0], 100.0, epsilon = 1e-4); // Front left
    assert_relative_eq!(output.motors[1], 0.0, epsilon = 1e-4); // Front right
    assert_relative_eq!(output.motors[2], 100.0, epsilon = 1e-4); // Rear left
    assert_relative_eq!(output.motors[3], 0.0, epsilon = 1e-4); // Rear right
}

// Test 3: Pure Pitch (max pitch, no thrust/roll/yaw)
#[test]
fn test_pure_pitch() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, vec3(0.0, 1.0, 0.0)); // No thrust, max positive pitch
    let output = mixer.mix(&input);

    // Expected: front motors (0, 1) higher, rear motors (2, 3) lower
    // Raw: pitch = 0.25
    // Motor 0: 0 + 0.25 = 0.25
    // Motor 1: 0 + 0.25 = 0.25
    // Motor 2: 0 - 0.25 = -0.25
    // Motor 3: 0 - 0.25 = -0.25
    // After scaling to [0, 1]: [0.25, -0.25] -> [1, 0]
    assert_relative_eq!(output.motors[0], 100.0, epsilon = 1e-4); // Front left
    assert_relative_eq!(output.motors[1], 100.0, epsilon = 1e-4); // Front right
    assert_relative_eq!(output.motors[2], 0.0, epsilon = 1e-4); // Rear left
    assert_relative_eq!(output.motors[3], 0.0, epsilon = 1e-4); // Rear right
}

// Test 4: Pure Yaw (max yaw, no thrust/roll/pitch)
#[test]
fn test_pure_yaw() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, vec3(0.0, 0.0, 1.0)); // No thrust, max positive yaw
    let output = mixer.mix(&input);

    // Expected: CW motors (1, 2) higher, CCW motors (0, 3) lower
    // Raw: yaw = 0.125
    // Motor 0: 0 - 0.125 = -0.125
    // Motor 1: 0 + 0.125 = 0.125
    // Motor 2: 0 + 0.125 = 0.125
    // Motor 3: 0 - 0.125 = -0.125
    // After scaling to [0, 1]: [0.125, -0.125] -> [1, 0]
    assert_relative_eq!(output.motors[0], 0.0, epsilon = 1e-4); // Front left (CCW)
    assert_relative_eq!(output.motors[1], 100.0, epsilon = 1e-4); // Front right (CW)
    assert_relative_eq!(output.motors[2], 100.0, epsilon = 1e-4); // Rear left (CW)
    assert_relative_eq!(output.motors[3], 0.0, epsilon = 1e-4); // Rear right (CCW)
}

// Test 5: Max Inputs (max thrust, max moments)
#[test]
fn test_max_inputs() {
    let mixer = create_mixer();
    let input = ControlOutput::new(1.0, vec3(1.0, 1.0, 1.0)); // Max everything
    let output = mixer.mix(&input);

    // Raw values:
    // baseline = 0.25, roll = 0.25, pitch = 0.25, yaw = 0.125
    // Motor 0: 0.25 - 0.125 + 0.25 + 0.25 = 0.625
    // Motor 1: 0.25 + 0.125 + 0.25 - 0.25 = 0.375
    // Motor 2: 0.25 + 0.125 - 0.25 + 0.25 = 0.375
    // Motor 3: 0.25 - 0.125 - 0.25 - 0.25 = -0.375
    // After scaling to [0, 1]: [-0.375, 0.625] -> [0, 1]
    // Scaled range = 1.0 / (0.625 - (-0.375)) = 1.0 / 1.0 = 1.0
    // Shift min to 0: +0.375 -> [0, 1]
    // Final outputs: [1.0, 0.75, 0.75, 0.0] * 100
    assert_relative_eq!(output.motors[0], 100.0, epsilon = 1e-4); // Front left
    assert_relative_eq!(output.motors[1], 75.0, epsilon = 1e-4); // Front right
    assert_relative_eq!(output.motors[2], 75.0, epsilon = 1e-4); // Rear left
    assert_relative_eq!(output.motors[3], 0.0, epsilon = 1e-4); // Rear right
}

// Test 6: Zero Inputs (zero thrust, zero moments)
#[test]
fn test_zero_inputs() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, vec3(0.0, 0.0, 0.0)); // Zero everything
    let output = mixer.mix(&input);

    // Expected: all motors at 0
    let expected = 0.0;
    for motor in output.motors.iter() {
        assert_relative_eq!(motor, &expected, epsilon = 1e-4);
    }
}

// Test 7: Negative Outputs (inputs that would produce negative raw outputs)
#[test]
fn test_negative_outputs() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, vec3(-1.0, -1.0, -1.0)); // No thrust, max negative moments
    let output = mixer.mix(&input);

    // Raw values:
    // roll = -0.25, pitch = -0.25, yaw = -0.125
    // Motor 0: 0 - (-0.125) + (-0.25) + (-0.25) = -0.375
    // Motor 1: 0 + (-0.125) + (-0.25) - (-0.25) = -0.125
    // Motor 2: 0 + (-0.125) - (-0.25) + (-0.25) = -0.125
    // Motor 3: 0 - (-0.125) - (-0.25) - (-0.25) = 0.625
    // After scaling to [0, 1]: [-0.375, 0.625] -> [0, 1]
    // Scaled range = 1.0 / (0.625 - (-0.375)) = 1.0 / 1.0 = 1.0
    // Shift min to 0: +0.375 -> [0, 1]
    // Final outputs: [0.0, 0.25, 0.25, 1.0] * 100
    assert_relative_eq!(output.motors[0], 0.0, epsilon = 1e-4); // Front left
    assert_relative_eq!(output.motors[1], 25.0, epsilon = 1e-4); // Front right
    assert_relative_eq!(output.motors[2], 25.0, epsilon = 1e-4); // Rear left
    assert_relative_eq!(output.motors[3], 100.0, epsilon = 1e-4); // Rear right
}
