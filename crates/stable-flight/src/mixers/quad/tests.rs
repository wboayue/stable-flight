use crate::Vec3;

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
/// With full thrust (1.0) and no moments, the baseline thrust is scaled by `THRUST_FACTOR` (0.9)
/// and equally distributed among the four motors. Thus, each motor should receive:
///
///     baseline = 1.0 * 0.9 = 0.9
#[test]
fn test_hover_no_moments() {
    let mixer = create_mixer();
    let input = ControlOutput::new(1.0, Vec3::zeros());
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
/// With no thrust and maximum positive roll (1.0):
/// - Roll contribution = 1.0 * `roll_pitch_scaling` = 1.0 * 0.1 = 0.1
///
/// Motor outputs:
/// - Motor 0 (Front Left): 0 + 0 + 0 + 0.1 = 0.1
/// - Motor 1 (Front Right): 0 + 0 + 0 - 0.1 = -0.1 -> clamped to 0.0
/// - Motor 2 (Rear Left): 0 + 0 + 0 + 0.1 = 0.1
/// - Motor 3 (Rear Right): 0 + 0 + 0 - 0.1 = -0.1 -> clamped to 0.0
#[test]
fn test_pure_roll() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, Vec3::new(1.0, 0.0, 0.0));
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
/// With no thrust and maximum positive pitch (1.0):
/// - Pitch contribution = 1.0 * `roll_pitch_scaling` = 1.0 * 0.1 = 0.1
///
/// Motor outputs:
/// - Motor 0 (Front Left): 0 + 0 + 0.1 + 0 = 0.1
/// - Motor 1 (Front Right): 0 + 0 + 0.1 + 0 = 0.1
/// - Motor 2 (Rear Left): 0 + 0 - 0.1 + 0 = -0.1 -> clamped to 0.0
/// - Motor 3 (Rear Right): 0 + 0 - 0.1 + 0 = -0.1 -> clamped to 0.0
#[test]
fn test_pure_pitch() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, Vec3::new(0.0, 1.0, 0.0));
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
/// With no thrust and maximum positive yaw (1.0), nose right:
/// - Yaw contribution = 1.0 * `yaw_scaling` = 1.0 * 0.25 = 0.25
///
/// Motor outputs (the CCW props speed up):
/// - Motor 0 (Front Left, CW): 0 - 0.25 = -0.25 -> clamped to 0.0
/// - Motor 1 (Front Right, CCW): 0 + 0.25 = 0.25
/// - Motor 2 (Rear Left, CCW): 0 + 0.25 = 0.25
/// - Motor 3 (Rear Right, CW): 0 - 0.25 = -0.25 -> clamped to 0.0
#[test]
fn test_pure_yaw() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, Vec3::new(0.0, 0.0, 1.0));
    let output = mixer.mix(&input);

    assert!(
        approx_eq(output.motors[0], 0.0),
        "Motor 0 expected 0.0 but got {}",
        output.motors[0]
    ); // Front Left (CW)
    assert!(
        approx_eq(output.motors[1], 0.25),
        "Motor 1 expected 0.25 but got {}",
        output.motors[1]
    ); // Front Right (CCW)
    assert!(
        approx_eq(output.motors[2], 0.25),
        "Motor 2 expected 0.25 but got {}",
        output.motors[2]
    ); // Rear Left (CCW)
    assert!(
        approx_eq(output.motors[3], 0.0),
        "Motor 3 expected 0.0 but got {}",
        output.motors[3]
    ); // Rear Right (CW)
}

/// Test 5: Max Inputs (max thrust and max moments)
///
/// With maximum thrust (1.0) and maximum moments (roll = 1.0, pitch = 1.0, yaw = 1.0):
/// - Baseline thrust = 1.0 * `THRUST_FACTOR` = 0.9
/// - Roll = 1.0 * `roll_pitch_scaling` = 0.1
/// - Pitch = 1.0 * `roll_pitch_scaling` = 0.1
/// - Yaw = 1.0 * `yaw_scaling` = 0.25
///
/// Motor outputs:
/// - Motor 0: 0.9 - 0.25 + 0.1 + 0.1 = 0.85
/// - Motor 1: 0.9 + 0.25 + 0.1 - 0.1 = 1.15 -> clamped to 1.0
/// - Motor 2: 0.9 + 0.25 - 0.1 + 0.1 = 1.15 -> clamped to 1.0
/// - Motor 3: 0.9 - 0.25 - 0.1 - 0.1 = 0.45
#[test]
fn test_max_inputs() {
    let mixer = create_mixer();
    let input = ControlOutput::new(1.0, Vec3::new(1.0, 1.0, 1.0));
    let output = mixer.mix(&input);

    assert!(
        approx_eq(output.motors[0], 0.85),
        "Motor 0 expected 0.85 but got {}",
        output.motors[0]
    ); // Front Left (CW)
    assert!(
        approx_eq(output.motors[1], 1.0),
        "Motor 1 expected 1.0 but got {}",
        output.motors[1]
    ); // Front Right (CCW)
    assert!(
        approx_eq(output.motors[2], 1.0),
        "Motor 2 expected 1.0 but got {}",
        output.motors[2]
    ); // Rear Left (CCW)
    assert!(
        approx_eq(output.motors[3], 0.45),
        "Motor 3 expected 0.45 but got {}",
        output.motors[3]
    ); // Rear Right (CW)
}

/// Test 6: Zero Inputs (zero thrust and zero moments)
///
/// With zero thrust and zero moments, all motor outputs should be zero.
#[test]
fn test_zero_inputs() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, Vec3::zeros());
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

/// Test 7: Negative Moments (max negative moments, no thrust)
///
/// With no thrust and maximum negative moments (roll = -1.0, pitch = -1.0, yaw = -1.0):
/// - Roll = -1.0 * `roll_pitch_scaling` = -0.1
/// - Pitch = -1.0 * `roll_pitch_scaling` = -0.1
/// - Yaw = -1.0 * `yaw_scaling` = -0.25
///
/// Motor outputs:
/// - Motor 0: 0 - (-0.25) + (-0.1) + (-0.1) = 0.05
/// - Motor 1: 0 + (-0.25) + (-0.1) - (-0.1) = -0.25 -> clamped to 0.0
/// - Motor 2: 0 + (-0.25) - (-0.1) + (-0.1) = -0.25 -> clamped to 0.0
/// - Motor 3: 0 - (-0.25) - (-0.1) - (-0.1) = 0.45
#[test]
fn test_negative_outputs() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, Vec3::new(-1.0, -1.0, -1.0));
    let output = mixer.mix(&input);

    assert!(
        approx_eq(output.motors[0], 0.05),
        "Motor 0 expected 0.05 but got {}",
        output.motors[0]
    ); // Front Left (CW)
    assert!(
        approx_eq(output.motors[1], 0.0),
        "Motor 1 expected 0.0 but got {}",
        output.motors[1]
    ); // Front Right (CCW)
    assert!(
        approx_eq(output.motors[2], 0.0),
        "Motor 2 expected 0.0 but got {}",
        output.motors[2]
    ); // Rear Left (CCW)
    assert!(
        approx_eq(output.motors[3], 0.45),
        "Motor 3 expected 0.45 but got {}",
        output.motors[3]
    ); // Rear Right (CW)
}

/// Test 8: Partial Thrust (intermediate thrust, no moments)
///
/// With thrust = 0.5 and no moments:
/// - Baseline thrust = 0.5 * `THRUST_FACTOR` = 0.5 * 0.9 = 0.45
///
/// Motor outputs:
/// - All motors: 0.45
#[test]
fn test_partial_thrust() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.5, Vec3::zeros());
    let output = mixer.mix(&input);

    let expected = 0.45;
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

/// Test 9: Mixed Moments (partial mixed moments, no thrust)
///
/// With no thrust and moments (roll = 0.5, pitch = -0.5, yaw = 0.0):
/// - Roll = 0.5 * `roll_pitch_scaling` = 0.5 * 0.1 = 0.05
/// - Pitch = -0.5 * `roll_pitch_scaling` = -0.5 * 0.1 = -0.05
/// - Yaw = 0.0 * `yaw_scaling` = 0.0
///
/// Motor outputs:
/// - Motor 0: 0 + 0 + (-0.05) + 0.05 = 0.0
/// - Motor 1: 0 + 0 + (-0.05) - 0.05 = -0.1 -> clamped to 0.0
/// - Motor 2: 0 + 0 - (-0.05) + 0.05 = 0.1
/// - Motor 3: 0 + 0 - (-0.05) - 0.05 = 0.0
#[test]
fn test_mixed_moments() {
    let mixer = create_mixer();
    let input = ControlOutput::new(0.0, Vec3::new(0.5, -0.5, 0.0));
    let output = mixer.mix(&input);

    assert!(
        approx_eq(output.motors[0], 0.0),
        "Motor 0 expected 0.0 but got {}",
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

/// Test 10: Boundary Condition (max thrust with mixed moments)
///
/// With thrust = 1.0 and moments (roll = -1.0, pitch = 1.0, yaw = 0.0):
/// - Baseline = 1.0 * `THRUST_FACTOR` = 0.9
/// - Roll = -1.0 * `roll_pitch_scaling` = -0.1
/// - Pitch = 1.0 * `roll_pitch_scaling` = 0.1
/// - Yaw = 0.0 * `yaw_scaling` = 0.0
///
/// Motor outputs:
/// - Motor 0: 0.9 + 0 + 0.1 + (-0.1) = 0.9
/// - Motor 1: 0.9 + 0 + 0.1 - (-0.1) = 1.1 -> clamped to 1.0
/// - Motor 2: 0.9 + 0 - 0.1 + (-0.1) = 0.7
/// - Motor 3: 0.9 + 0 - 0.1 - (-0.1) = 0.9
#[test]
fn test_boundary_condition() {
    let mixer = create_mixer();
    let input = ControlOutput::new(1.0, Vec3::new(-1.0, 1.0, 0.0));
    let output = mixer.mix(&input);

    assert!(
        approx_eq(output.motors[0], 0.9),
        "Motor 0 expected 0.9 but got {}",
        output.motors[0]
    ); // Front Left
    assert!(
        approx_eq(output.motors[1], 1.0),
        "Motor 1 expected 1.0 but got {}",
        output.motors[1]
    ); // Front Right
    assert!(
        approx_eq(output.motors[2], 0.7),
        "Motor 2 expected 0.7 but got {}",
        output.motors[2]
    ); // Rear Left
    assert!(
        approx_eq(output.motors[3], 0.9),
        "Motor 3 expected 0.9 but got {}",
        output.motors[3]
    ); // Rear Right
}

/// Out-of-range or NaN inputs saturate instead of panicking the loop: each row mixes the same
/// as its in-range equivalent.
#[test]
fn out_of_range_or_nan_inputs_saturate_instead_of_panicking() {
    // (input thrust, input moments, equivalent thrust, equivalent moments)
    let cases = [
        (-0.1, Vec3::zeros(), 0.0, Vec3::zeros()),
        (1.5, Vec3::zeros(), 1.0, Vec3::zeros()),
        (0.5, Vec3::new(1.5, 0.0, 0.0), 0.5, Vec3::new(1.0, 0.0, 0.0)),
        (
            0.5,
            Vec3::new(0.0, -3.0, 2.0),
            0.5,
            Vec3::new(0.0, -1.0, 1.0),
        ),
        (
            f32::NAN,
            Vec3::new(0.2, 0.0, 0.0),
            0.0,
            Vec3::new(0.2, 0.0, 0.0),
        ),
        (
            0.5,
            Vec3::new(f32::NAN, 0.1, f32::NAN),
            0.5,
            Vec3::new(0.0, 0.1, 0.0),
        ),
    ];
    let mixer = create_mixer();
    for (thrust, moments, eq_thrust, eq_moments) in cases {
        let mixed = mixer.mix(&ControlOutput::new(thrust, moments));
        let expected = mixer.mix(&ControlOutput::new(eq_thrust, eq_moments));
        assert!(
            mixed.motors.iter().all(|m| m.is_finite()),
            "({thrust}, {moments:?}) gave non-finite motors {:?}",
            mixed.motors
        );
        assert!(
            mixed.motors == expected.motors,
            "({thrust}, {moments:?}) gave {:?}, expected {:?}",
            mixed.motors,
            expected.motors
        );
    }
}
