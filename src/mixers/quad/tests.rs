use robomath::vec3;

use super::*;

/// Helper function for approximate equality of f32 values.
fn approx_eq(a: f32, b: f32) -> bool {
    (a - b).abs() < 1e-5
}

/// Helper function to create a mixer instance.
fn create_mixer() -> QuadCopterMixer {
    QuadCopterMixer::default()
}

/// Test 1: Hover (no moments, thrust only)
///
/// With full thrust (1.0) and no moments, the baseline thrust is equally divided among
/// the four motors. Thus, each motor should receive:
///
///     baseline = 1.0 / 4.0 = 0.25 -> 0.25 * 100 = 25.0
#[test]
fn test_hover_no_moments() {
    let mixer = create_mixer();
    let input = ControlOutput::new(1.0, vec3(0.0, 0.0, 0.0));
    let output = mixer.mix(&input);

    let expected = 0.9;
    for i in 0..4 {
        assert!(
            approx_eq(output.motors[i], expected),
            "Motor {} expected {} but got {}",
            i,
            expected,
            output.motors[i]
        );
    }
}

/// Test 2: Pure Roll (max roll, no thrust/pitch/yaw)
///
/// With no thrust and maximum positive roll:
/// - roll contribution = 1.0 / 4.0 = 0.25
///
/// Motor outputs:
/// - Motor 0 (Front Left): 0 - 0 + 0 + 0.25 = 0.25  -> 25.0 after scaling
/// - Motor 1 (Front Right): 0 + 0 + 0 - 0.25 = -0.25 -> clamped to 0.0
/// - Motor 2 (Rear Left):   0 + 0 + 0 + 0.25 = 0.25  -> 25.0 after scaling
/// - Motor 3 (Rear Right):  0 - 0 + 0 - 0.25 = -0.25 -> clamped to 0.0
#[test]
fn test_pure_roll() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, vec3(1.0, 0.0, 0.0));
    let output = mixer.mix(&input);

    assert!(
        approx_eq(output.motors[0], 0.1),
        "Motor 0 expected 0.1 but got {}",
        output.motors[0]
    ); // Front Left
    assert!(
        approx_eq(output.motors[1], 0.0),
        "Motor 1 expected 0.0 but got {}",
        output.motors[1]
    ); // Front Right
    assert!(
        approx_eq(output.motors[2], 0.1),
        "Motor 2 expected 0.1 but got {}",
        output.motors[2]
    ); // Rear Left
    assert!(
        approx_eq(output.motors[3], 0.0),
        "Motor 3 expected 0.0 but got {}",
        output.motors[3]
    ); // Rear Right
}

/// Test 3: Pure Pitch (max pitch, no thrust/roll/yaw)
///
/// With no thrust and maximum positive pitch:
/// - pitch contribution = 1.0 / 4.0 = 0.25
///
/// Motor outputs:
/// - Motor 0 (Front Left): 0 + 0.25 = 0.25  -> 25.0 after scaling
/// - Motor 1 (Front Right): 0 + 0.25 = 0.25  -> 25.0 after scaling
/// - Motor 2 (Rear Left):   0 - 0.25 = -0.25 -> clamped to 0.0
/// - Motor 3 (Rear Right):  0 - 0.25 = -0.25 -> clamped to 0.0
#[test]
fn test_pure_pitch() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, vec3(0.0, 1.0, 0.0));
    let output = mixer.mix(&input);

    assert!(
        approx_eq(output.motors[0], 0.1),
        "Motor 0 expected 0.1 but got {}",
        output.motors[0]
    ); // Front Left
    assert!(
        approx_eq(output.motors[1], 0.1),
        "Motor 1 expected 0.1 but got {}",
        output.motors[1]
    ); // Front Right
    assert!(
        approx_eq(output.motors[2], 0.0),
        "Motor 2 expected 0.0 but got {}",
        output.motors[2]
    ); // Rear Left
    assert!(
        approx_eq(output.motors[3], 0.0),
        "Motor 3 expected 0.0 but got {}",
        output.motors[3]
    ); // Rear Right
}

/// Test 4: Pure Yaw (max yaw, no thrust/roll/pitch)
///
/// With no thrust and maximum positive yaw:
/// - yaw contribution = 1.0 / 8.0 = 0.125
///
/// Motor outputs:
/// - Motor 0 (Front Left, CCW): 0 - 0.125 = -0.125 -> clamped to 0.0
/// - Motor 1 (Front Right, CW):  0 + 0.125 = 0.125  -> 12.5 after scaling
/// - Motor 2 (Rear Left, CW):    0 + 0.125 = 0.125  -> 12.5 after scaling
/// - Motor 3 (Rear Right, CCW):  0 - 0.125 = -0.125 -> clamped to 0.0
#[test]
fn test_pure_yaw() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, vec3(0.0, 0.0, 1.0));
    let output = mixer.mix(&input);

    assert!(
        approx_eq(output.motors[0], 0.25),
        "Motor 0 expected 0.0 but got {}",
        output.motors[0]
    ); // Front Left (CCW)
    assert!(
        approx_eq(output.motors[1], 0.0),
        "Motor 1 expected 12.5 but got {}",
        output.motors[1]
    ); // Front Right (CW)
    assert!(
        approx_eq(output.motors[2], 0.0),
        "Motor 2 expected 12.5 but got {}",
        output.motors[2]
    ); // Rear Left (CW)
    assert!(
        approx_eq(output.motors[3], 0.25),
        "Motor 3 expected 0.0 but got {}",
        output.motors[3]
    ); // Rear Right (CCW)
}

/// Test 5: Max Inputs (max thrust and max moments)
///
/// For input thrust = 1.0 and moments = (1.0, 1.0, 1.0):
/// - Baseline thrust = 1.0 / 4.0 = 0.25
/// - Roll = 1.0 / 4.0 = 0.25
/// - Pitch = 1.0 / 4.0 = 0.25
/// - Yaw = 1.0 / 8.0 = 0.125
///
/// Motor outputs before clamping:
/// - Motor 0: 0.25 - 0.125 + 0.25 + 0.25 = 0.625  -> 62.5 after scaling
/// - Motor 1: 0.25 + 0.125 + 0.25 - 0.25 = 0.375  -> 37.5 after scaling
/// - Motor 2: 0.25 + 0.125 - 0.25 + 0.25 = 0.375  -> 37.5 after scaling
/// - Motor 3: 0.25 - 0.125 - 0.25 - 0.25 = -0.375 -> clamped to 0.0
#[test]
fn test_max_inputs() {
    let mixer = create_mixer();
    let input = ControlOutput::new(1.0, vec3(1.0, 1.0, 1.0));
    let output = mixer.mix(&input);

    assert!(
        approx_eq(output.motors[0], 1.0),
        "Motor 0 expected 1.0 but got {}",
        output.motors[0]
    ); // Front Left
    assert!(
        approx_eq(output.motors[1], 0.65),
        "Motor 1 expected 0.65 but got {}",
        output.motors[1]
    ); // Front Right
    assert!(
        approx_eq(output.motors[2], 0.65),
        "Motor 2 expected 0.65 but got {}",
        output.motors[2]
    ); // Rear Left
    assert!(
        approx_eq(output.motors[3], 0.95),
        "Motor 3 expected 0.95 but got {}",
        output.motors[3]
    ); // Rear Right
}

/// Test 6: Zero Inputs (zero thrust and zero moments)
///
/// With zero inputs, all motor outputs should be zero.
#[test]
fn test_zero_inputs() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, vec3(0.0, 0.0, 0.0));
    let output = mixer.mix(&input);

    let expected = 0.0;
    for motor in output.motors.iter() {
        assert!(
            approx_eq(*motor, expected),
            "Expected {} but got {}",
            expected,
            motor
        );
    }
}

/// Test 7: Negative Moments (inputs that would produce negative raw outputs)
///
/// With no thrust and maximum negative moments:
/// - Roll = -1.0 / 4.0 = -0.25
/// - Pitch = -1.0 / 4.0 = -0.25
/// - Yaw = -1.0 / 8.0 = -0.125
///
/// Motor outputs before clamping:
/// - Motor 0: 0 - (-0.125) + (-0.25) + (-0.25) = -0.375 -> clamped to 0.0
/// - Motor 1: 0 + (-0.125) + (-0.25) - (-0.25) = -0.125 -> clamped to 0.0
/// - Motor 2: 0 + (-0.125) - (-0.25) + (-0.25) = -0.125 -> clamped to 0.0
/// - Motor 3: 0 - (-0.125) - (-0.25) - (-0.25) = 0.625  -> 62.5 after scaling
#[test]
fn test_negative_outputs() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, vec3(-1.0, -1.0, -1.0));
    let output = mixer.mix(&input);

    assert!(
        approx_eq(output.motors[0], 0.0),
        "Motor 0 expected 0.0 but got {}",
        output.motors[0]
    ); // Front Left
    assert!(
        approx_eq(output.motors[1], 0.25),
        "Motor 1 expected 0.25 but got {}",
        output.motors[1]
    ); // Front Right
    assert!(
        approx_eq(output.motors[2], 0.25),
        "Motor 2 expected 0.25 but got {}",
        output.motors[2]
    ); // Rear Left
    assert!(
        approx_eq(output.motors[3], 0.0),
        "Motor 3 expected 0.0 but got {}",
        output.motors[3]
    ); // Rear Right
}

#[test]
#[should_panic(expected = "Thrust (1.5) must be in the range [0, 1]")]
fn test_out_of_range_thrust() {
    let mixer = create_mixer();
    // Thrust is out of range (> 1.0)
    let input = ControlOutput::new(1.5, vec3(0.0, 0.0, 0.0));
    let _ = mixer.mix(&input);
}

#[test]
#[should_panic(expected = "Roll moment (1.5) must be in the range [-1, 1]")]
fn test_out_of_range_moment() {
    let mixer = create_mixer();
    // Roll moment is out of range (> 1.0)
    let input = ControlOutput::new(0.5, vec3(1.5, 0.0, 0.0));
    let _ = mixer.mix(&input);
}
