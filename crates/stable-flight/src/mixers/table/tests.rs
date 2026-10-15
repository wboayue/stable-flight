use crate::{CruiseOutput, Vec3};

use super::*;

/// Helper function for approximate equality of f32 values.
fn approx_eq(a: f32, b: f32) -> bool {
    (a - b).abs() < 1e-5
}

/// The quad X preset: the rows below are the former `QuadCopterMixer`'s tests, unchanged, so
/// the preset is checked to mix the same.
fn create_mixer() -> TableMixer<4> {
    TableMixer::quad_x()
}

/// Test 1: Hover (no moments, thrust only)
///
/// With full thrust (1.0) and no moments, the baseline thrust is scaled by `THRUST_FACTOR` (0.9)
/// and equally distributed among the four motors. Thus, each motor should receive:
///
///     baseline = 1.0 * 0.9 = 0.9
#[test]
fn test_hover_no_moments() {
    let mut mixer = create_mixer();
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
    let mut mixer = create_mixer();
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
    let mut mixer = create_mixer();
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
    let mut mixer = create_mixer();
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
/// Raw outputs 0.85, 1.15, 1.15, 0.45 exceed full throttle by 0.15, so all four drop by 0.15,
/// keeping the moments and giving up thrust:
/// - Motor 0: 0.85 - 0.15 = 0.70
/// - Motor 1: 1.15 - 0.15 = 1.0
/// - Motor 2: 1.15 - 0.15 = 1.0
/// - Motor 3: 0.45 - 0.15 = 0.30
#[test]
fn test_max_inputs() {
    let mut mixer = create_mixer();
    let input = ControlOutput::new(1.0, Vec3::new(1.0, 1.0, 1.0));
    let output = mixer.mix(&input);

    assert!(
        approx_eq(output.motors[0], 0.7),
        "Motor 0 expected 0.7 but got {}",
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
        approx_eq(output.motors[3], 0.3),
        "Motor 3 expected 0.3 but got {}",
        output.motors[3]
    ); // Rear Right (CW)
}

/// Test 6: Zero Inputs (zero thrust and zero moments)
///
/// With zero thrust and zero moments, all motor outputs should be zero.
#[test]
fn test_zero_inputs() {
    let mut mixer = create_mixer();
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
    let mut mixer = create_mixer();
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
    let mut mixer = create_mixer();
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
    let mut mixer = create_mixer();
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
/// Raw outputs 0.9, 1.1, 0.7, 0.9 exceed full throttle by 0.1, so all four drop by 0.1:
/// - Motor 0: 0.8
/// - Motor 1: 1.0
/// - Motor 2: 0.6
/// - Motor 3: 0.8
#[test]
fn test_boundary_condition() {
    let mut mixer = create_mixer();
    let input = ControlOutput::new(1.0, Vec3::new(-1.0, 1.0, 0.0));
    let output = mixer.mix(&input);

    assert!(
        approx_eq(output.motors[0], 0.8),
        "Motor 0 expected 0.8 but got {}",
        output.motors[0]
    ); // Front Left
    assert!(
        approx_eq(output.motors[1], 1.0),
        "Motor 1 expected 1.0 but got {}",
        output.motors[1]
    ); // Front Right
    assert!(
        approx_eq(output.motors[2], 0.6),
        "Motor 2 expected 0.6 but got {}",
        output.motors[2]
    ); // Rear Left
    assert!(
        approx_eq(output.motors[3], 0.8),
        "Motor 3 expected 0.8 but got {}",
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
    let mut mixer = create_mixer();
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

/// At low throttle a large moment shifts all motors up rather than clipping the low side, so
/// the full differential survives; at zero thrust nothing is raised.
#[test]
fn low_throttle_moments_shift_up_instead_of_clipping() {
    let mut mixer = create_mixer();
    // (thrust, moments, expected motors)
    let cases = [
        // Roll 1 at thrust 0.1: raw [0.19, -0.01, 0.19, -0.01], up 0.01.
        (0.1, Vec3::new(1.0, 0.0, 0.0), [0.2, 0.0, 0.2, 0.0]),
        // Pitch -1 at thrust 0.05: raw [-0.055, -0.055, 0.145, 0.145], up 0.055.
        (0.05, Vec3::new(0.0, -1.0, 0.0), [0.0, 0.0, 0.2, 0.2]),
        // Yaw 1 at thrust 0.2: raw [-0.07, 0.43, 0.43, -0.07], up 0.07.
        (0.2, Vec3::new(0.0, 0.0, 1.0), [0.0, 0.5, 0.5, 0.0]),
        // Zero thrust: the low side clips, nothing rises.
        (0.0, Vec3::new(1.0, 0.0, 0.0), [0.1, 0.0, 0.1, 0.0]),
    ];
    for (thrust, moments, expected) in cases {
        let output = mixer.mix(&ControlOutput::new(thrust, moments));
        for (i, e) in expected.iter().enumerate() {
            assert!(
                approx_eq(output.motors[i], *e),
                "thrust {thrust}, moments {moments:?}: motor {i} {} vs {e}",
                output.motors[i]
            );
        }
    }
}

/// The mixer reports what it gave up on its last call: collective when it shifted down,
/// moments when it clipped (or scaled); a clean mix clears the flags.
#[test]
fn reports_what_the_last_mix_gave_up() {
    // (thrust, moments, mixer_thrust, mixer_moments)
    let cases = [
        (0.5, Vec3::zeros(), false, false),
        (0.5, Vec3::new(1.0, 1.0, 1.0), false, false), // fits as is
        (1.0, Vec3::new(1.0, 1.0, 1.0), true, false),  // collective gives way
        (0.1, Vec3::new(1.0, 0.0, 0.0), false, false), // shifted up: nothing lost
        (0.0, Vec3::new(1.0, 0.0, 0.0), false, true),  // zero thrust: clipped
    ];
    let mut mixer = create_mixer();
    for (thrust, moments, mixer_thrust, mixer_moments) in cases {
        mixer.mix(&ControlOutput::new(thrust, moments));
        let saturation = mixer.saturation();
        assert!(
            (saturation.mixer_thrust, saturation.mixer_moments) == (mixer_thrust, mixer_moments),
            "thrust {thrust}, moments {moments:?}: {saturation:?}"
        );
        assert!(!saturation.tilt && !saturation.thrust && saturation.moments == [false; 3]);
    }
}

/// Where each motor of a motor-only preset sits: azimuth from the nose (degrees, clockwise
/// seen from above) and spin (+1 counterclockwise, whose drag torque yaws the nose right).
type Geometry = &'static [(f32, f32)];

/// One mix through a fresh preset.
type MixFn = fn(&ControlOutput) -> ActuatorCommand;

const CW: f32 = -1.0;
const CCW: f32 = 1.0;

/// Motor-only presets with their geometry, written from the diagrams, not the tables.
const MOTOR_PRESETS: [(&str, MixFn, Geometry); 7] = [
    (
        "quad_x",
        |c| TableMixer::quad_x().mix(c),
        &[(-45.0, CW), (45.0, CCW), (-135.0, CCW), (135.0, CW)],
    ),
    (
        "quad_plus",
        |c| TableMixer::quad_plus().mix(c),
        &[(0.0, CW), (90.0, CCW), (180.0, CW), (-90.0, CCW)],
    ),
    (
        "hex_x",
        |c| TableMixer::hex_x().mix(c),
        &[
            (-30.0, CW),
            (30.0, CCW),
            (90.0, CW),
            (150.0, CCW),
            (-150.0, CW),
            (-90.0, CCW),
        ],
    ),
    (
        "hex_plus",
        |c| TableMixer::hex_plus().mix(c),
        &[
            (0.0, CW),
            (60.0, CCW),
            (120.0, CW),
            (180.0, CCW),
            (-120.0, CW),
            (-60.0, CCW),
        ],
    ),
    (
        "octo_x",
        |c| TableMixer::octo_x().mix(c),
        &[
            (-22.5, CW),
            (22.5, CCW),
            (67.5, CW),
            (112.5, CCW),
            (157.5, CW),
            (-157.5, CCW),
            (-112.5, CW),
            (-67.5, CCW),
        ],
    ),
    (
        "y6",
        |c| TableMixer::y6().mix(c),
        &[
            (-60.0, CW),
            (60.0, CW),
            (180.0, CW),
            (-60.0, CCW),
            (60.0, CCW),
            (180.0, CCW),
        ],
    ),
    (
        "x8",
        |c| TableMixer::x8().mix(c),
        &[
            (-45.0, CW),
            (45.0, CCW),
            (135.0, CW),
            (-135.0, CCW),
            (-45.0, CCW),
            (45.0, CW),
            (135.0, CCW),
            (-135.0, CW),
        ],
    ),
];

/// The body moments (roll, pitch, yaw) a change in motor outputs makes on `geometry`.
fn moments_from(geometry: Geometry, delta: &[f32]) -> Vec3 {
    geometry
        .iter()
        .zip(delta)
        .fold(Vec3::zeros(), |sum, (&(azimuth_deg, spin), &d)| {
            let azimuth_rad = azimuth_deg.to_radians();
            // A motor left of center (negative sine) rolls right as it speeds up; one ahead of
            // center (positive cosine) raises the nose.
            sum + Vec3::new(-azimuth_rad.sin(), azimuth_rad.cos(), spin) * d
        })
}

/// Each motor preset hovers with equal motors, and turns a pure roll, pitch, or yaw request
/// into that moment alone, with the frame-contract sign (roll right, nose up, nose right).
#[test]
fn motor_presets_make_the_requested_moment_only() {
    for (name, mix, geometry) in MOTOR_PRESETS {
        let n = geometry.len();
        let hover = mix(&ControlOutput::new(0.5, Vec3::zeros())).motors;
        assert!(
            hover[..n].iter().all(|&m| approx_eq(m, 0.45)) && hover[n..].iter().all(|&m| m == 0.0),
            "{name}: hover {hover:?}"
        );
        for axis in 0..3 {
            let mut request = Vec3::zeros();
            request[axis] = 0.5;
            let output = mix(&ControlOutput::new(0.5, request)).motors;
            let delta: [f32; 8] = core::array::from_fn(|i| output[i] - hover[i]);
            let made = moments_from(geometry, &delta[..n]);
            for other in 0..3 {
                if other == axis {
                    assert!(made[other] > 0.01, "{name}: axis {axis} made {made:?}");
                } else {
                    assert!(
                        made[other].abs() < 1e-5,
                        "{name}: axis {axis} made {made:?}"
                    );
                }
            }
        }
    }
}

/// Tricopter, bicopter, and tailsitter hover: servos carry the moments their rows name, motors the rest; servo
/// outputs don't depend on thrust.
#[test]
fn servo_presets_move_the_right_outputs() {
    // (preset, mix, thrust, moments, expected motors[..3], expected servos[..2])
    type Case = (&'static str, MixFn, f32, Vec3, [f32; 3], [f32; 2]);
    let tri: MixFn = |c| TableMixer::tri().mix(c);
    let bi: MixFn = |c| TableMixer::bi().mix(c);
    let tailsitter: MixFn = |c| TableMixer::tailsitter_2p().mix(c);
    let cases: [Case; 13] = [
        // Tail servo yaws; motors stay level.
        (
            "tri",
            tri,
            0.5,
            Vec3::new(0.0, 0.0, 0.3),
            [0.45, 0.45, 0.45],
            [0.3, 0.0],
        ),
        // Roll: front-left up, front-right down, tail motor unchanged.
        (
            "tri",
            tri,
            0.5,
            Vec3::new(0.5, 0.0, 0.0),
            [0.5, 0.4, 0.45],
            [0.0, 0.0],
        ),
        // Pitch up: front pair up half as much as the tail goes down.
        (
            "tri",
            tri,
            0.5,
            Vec3::new(0.0, 0.5, 0.0),
            [0.475, 0.475, 0.4],
            [0.0, 0.0],
        ),
        // Roll: differential thrust only.
        (
            "bi",
            bi,
            0.5,
            Vec3::new(0.5, 0.0, 0.0),
            [0.5, 0.4, 0.0],
            [0.0, 0.0],
        ),
        // Pitch up: both rotors tilt aft.
        (
            "bi",
            bi,
            0.5,
            Vec3::new(0.0, 0.4, 0.0),
            [0.45, 0.45, 0.0],
            [0.4, 0.4],
        ),
        // Nose right: left rotor forward, right rotor aft.
        (
            "bi",
            bi,
            0.5,
            Vec3::new(0.0, 0.0, 0.4),
            [0.45, 0.45, 0.0],
            [-0.4, 0.4],
        ),
        // Servos ignore thrust.
        (
            "bi",
            bi,
            0.0,
            Vec3::new(0.0, 0.4, 0.4),
            [0.0, 0.0, 0.0],
            [0.0, 0.8],
        ),
        (
            "bi",
            bi,
            1.0,
            Vec3::new(0.0, 0.4, 0.4),
            [0.9, 0.9, 0.0],
            [0.0, 0.8],
        ),
        // Tailsitter, airplane frame. Roll right: left trailing edge down, right up.
        (
            "tailsitter",
            tailsitter,
            0.5,
            Vec3::new(0.4, 0.0, 0.0),
            [0.45, 0.45, 0.0],
            [-0.4, 0.4],
        ),
        // Nose up: both trailing edges up.
        (
            "tailsitter",
            tailsitter,
            0.5,
            Vec3::new(0.0, 0.4, 0.0),
            [0.45, 0.45, 0.0],
            [0.4, 0.4],
        ),
        // Nose right: more left thrust (yaw ratio 2.5 on 0.1 headroom).
        (
            "tailsitter",
            tailsitter,
            0.5,
            Vec3::new(0.0, 0.0, 0.4),
            [0.55, 0.35, 0.0],
            [0.0, 0.0],
        ),
        // Elevons ignore thrust; at full thrust the motors give way to keep the yaw split.
        (
            "tailsitter",
            tailsitter,
            0.0,
            Vec3::new(0.2, 0.3, 0.0),
            [0.0, 0.0, 0.0],
            [0.1, 0.5],
        ),
        (
            "tailsitter",
            tailsitter,
            1.0,
            Vec3::new(0.0, 0.0, 1.0),
            [1.0, 0.5, 0.0],
            [0.0, 0.0],
        ),
    ];
    for (name, mix, thrust, moments, motors, servos) in cases {
        let output = mix(&ControlOutput::new(thrust, moments));
        for (i, &e) in motors.iter().enumerate() {
            assert!(
                approx_eq(output.motors[i], e),
                "{name} {thrust} {moments:?}: motors {:?}",
                output.motors
            );
        }
        for (i, &e) in servos.iter().enumerate() {
            assert!(
                approx_eq(output.servos[i], e),
                "{name} {thrust} {moments:?}: servos {:?}",
                output.servos
            );
        }
    }
}

/// Plane: each surface carries its one moment, the motor only thrust, up to `thrust_factor`.
#[test]
fn plane_preset_puts_each_moment_on_its_surface() {
    // (thrust, moments, expected motor 0, expected servos[..3])
    let cases = [
        (0.5, Vec3::zeros(), 0.45, [0.0, 0.0, 0.0]),
        (1.0, Vec3::zeros(), 0.9, [0.0, 0.0, 0.0]),
        (0.5, Vec3::new(0.3, 0.0, 0.0), 0.45, [0.3, 0.0, 0.0]),
        (0.5, Vec3::new(0.0, -0.4, 0.0), 0.45, [0.0, -0.4, 0.0]),
        (0.0, Vec3::new(0.0, 0.0, 0.2), 0.0, [0.0, 0.0, 0.2]),
        (0.7, Vec3::new(-0.1, 0.2, -0.3), 0.63, [-0.1, 0.2, -0.3]),
    ];
    for (thrust, moments, motor, servos) in cases {
        let mut mixer = TableMixer::plane();
        let output = mixer.mix(&ControlOutput::new(thrust, moments));
        let label = format!("{thrust} {moments:?}: {output:?}");
        assert!(approx_eq(output.motors[0], motor), "{label}");
        assert!(output.motors[1..] == [0.0; 7], "{label}");
        for (servo, expected) in output.servos.iter().zip(servos) {
            assert!(approx_eq(*servo, expected), "{label}");
        }
        assert!(mixer.saturation() == Saturation::default(), "{label}");
    }
}

/// The quadplane preset flies its lift rotors on `thrust` and `moments` exactly as `quad_x`
/// does, and its pusher and surfaces on `cruise`: lift rotors idle in cruise, surfaces neutral
/// and the pusher stopped in a hover, and the pusher kept out of the lift rotors'
/// desaturation.
#[test]
fn quadplane_preset_splits_lift_and_cruise() {
    let cruise = |thrust, moments| Some(CruiseOutput { thrust, moments });
    let saturation = |mixer_thrust, mixer_moments| Saturation {
        mixer_thrust,
        mixer_moments,
        ..Saturation::default()
    };
    // (case, control, expected pusher, expected servos[..3], saturation)
    let cases = [
        (
            "hover: surfaces neutral, pusher stopped",
            ControlOutput {
                cruise: cruise(0.0, Vec3::zeros()),
                ..ControlOutput::new(0.5, Vec3::new(0.2, -0.1, 0.1))
            },
            0.0,
            [0.0; 3],
            saturation(false, false),
        ),
        (
            "cruise: lift rotors idle",
            ControlOutput {
                cruise: cruise(0.6, Vec3::new(0.3, -0.2, 0.1)),
                ..ControlOutput::default()
            },
            0.54,
            [0.3, -0.2, 0.1],
            saturation(false, false),
        ),
        (
            "transition: both sides fly",
            ControlOutput {
                cruise: cruise(0.8, Vec3::new(0.0, 0.3, 0.0)),
                ..ControlOutput::new(0.5, Vec3::new(0.1, 0.0, 0.0))
            },
            0.72,
            [0.0, 0.3, 0.0],
            saturation(false, false),
        ),
        (
            "no cruise output: servos take the moments, pusher stopped",
            ControlOutput::new(0.5, Vec3::new(0.1, 0.0, -0.2)),
            0.0,
            [0.1, 0.0, -0.2],
            saturation(false, false),
        ),
        (
            "lift saturated: collective gives way, pusher doesn't",
            ControlOutput {
                cruise: cruise(1.0, Vec3::zeros()),
                ..ControlOutput::new(1.0, Vec3::new(1.0, 1.0, 0.0))
            },
            0.9,
            [0.0; 3],
            saturation(true, false),
        ),
        (
            "out-of-range or NaN cruise saturates",
            ControlOutput {
                cruise: cruise(f32::NAN, Vec3::new(2.0, f32::NAN, -3.0)),
                ..ControlOutput::default()
            },
            0.0,
            [1.0, 0.0, -1.0],
            saturation(false, false),
        ),
    ];
    for (case, control, pusher, servos, saturation) in cases {
        let mut mixer = TableMixer::quadplane();
        let output = mixer.mix(&control);
        let lift = TableMixer::quad_x()
            .mix(&ControlOutput::new(control.thrust, control.moments))
            .motors;
        assert!(output.motors[..4] == lift[..4], "{case}: {output:?}");
        assert!(approx_eq(output.motors[4], pusher), "{case}: {output:?}");
        assert!(output.motors[5..] == [0.0; 3], "{case}: {output:?}");
        for (servo, expected) in output.servos.iter().zip(servos) {
            assert!(approx_eq(*servo, expected), "{case}: {output:?}");
        }
        assert!(output.servos[3..] == [0.0; 5], "{case}: {output:?}");
        assert!(
            mixer.saturation() == saturation,
            "{case}: {:?}",
            mixer.saturation()
        );
    }
}

/// A servo asked past its range clamps on its own and reports the lost moment; the other
/// servo and the motors are untouched.
#[test]
fn servo_clamp_reports_lost_moments() {
    let mut mixer = TableMixer::bi();
    let output = mixer.mix(&ControlOutput::new(0.5, Vec3::new(0.0, 1.0, 1.0)));
    assert!(output.servos[..2] == [0.0, 1.0]);
    assert!(output.motors[..2] == [0.45, 0.45]);
    assert!(mixer.saturation().mixer_moments);
    assert!(!mixer.saturation().mixer_thrust);

    mixer.mix(&ControlOutput::new(0.5, Vec3::new(0.0, 0.5, 0.5)));
    assert!(mixer.saturation() == Saturation::default());
}

/// On six motors, too much thrust plus full roll and pitch still delivers the whole moment:
/// collective gives way, as on the quad.
#[test]
fn hex_desaturation_keeps_moments() {
    let mut mixer = TableMixer::hex_x();
    let moments = Vec3::new(1.0, 1.0, 0.0);
    let fits = mixer.mix(&ControlOutput::new(0.5, moments)).motors;
    assert!(mixer.saturation() == Saturation::default());
    let full = mixer.mix(&ControlOutput::new(1.0, moments)).motors;
    assert!(mixer.saturation().mixer_thrust && !mixer.saturation().mixer_moments);
    for i in 0..6 {
        assert!(
            approx_eq(full[i] - full[0], fits[i] - fits[0]),
            "motor {i}: {full:?} vs {fits:?}"
        );
    }
    assert!(full[..6].iter().all(|&m| (0.0..=1.0).contains(&m)));
}

/// A table of servos alone mixes with nothing to desaturate: motors stay off, servos follow
/// the moments, and only a clamped servo reports.
#[test]
fn servo_only_table_mixes_without_motors() {
    let rows = [
        MixRow::servo(0, 1.0, 0.0, 0.0),
        MixRow::servo(1, 0.0, 1.0, 1.0),
    ];
    let mut mixer = TableMixer::new(rows, MixerConfig::default()).expect("valid table");
    // (thrust, moments, expected servos[..2], mixer_moments)
    let cases = [
        (0.0, Vec3::new(0.3, -0.2, 0.1), [0.3, -0.1], false),
        (1.0, Vec3::new(-0.5, 0.0, 0.0), [-0.5, 0.0], false),
        (0.5, Vec3::new(0.0, 0.8, 0.8), [0.0, 1.0], true),
    ];
    for (thrust, moments, servos, mixer_moments) in cases {
        let output = mixer.mix(&ControlOutput::new(thrust, moments));
        assert!(output.motors == [0.0; ActuatorCommand::SLOTS]);
        assert!(
            output.servos[..2]
                .iter()
                .zip(servos)
                .all(|(&s, e)| approx_eq(s, e)),
            "{thrust} {moments:?}: {:?}",
            output.servos
        );
        assert!(
            mixer.saturation()
                == Saturation {
                    mixer_moments,
                    ..Saturation::default()
                }
        );
    }
}

/// Every preset passes the same validation a custom table gets.
#[test]
fn presets_are_valid_tables() {
    fn check<const N: usize>(name: &str, mixer: TableMixer<N>) {
        assert!(
            TableMixer::new(mixer.rows, mixer.config) == Ok(mixer),
            "{name}"
        );
    }
    check("quad_x", TableMixer::quad_x());
    check("quad_plus", TableMixer::quad_plus());
    check("tri", TableMixer::tri());
    check("bi", TableMixer::bi());
    check("tailsitter_2p", TableMixer::tailsitter_2p());
    check("plane", TableMixer::plane());
    check("hex_x", TableMixer::hex_x());
    check("hex_plus", TableMixer::hex_plus());
    check("y6", TableMixer::y6());
    check("octo_x", TableMixer::octo_x());
    check("x8", TableMixer::x8());
    check("quadplane", TableMixer::quadplane());
}

/// Invalid tables and configs are rejected at construction, naming the problem.
#[test]
fn rejects_invalid_tables_and_configs() {
    let motor = |i| MixRow::motor(i, 1.0, 0.0, 0.0);
    let good = MixerConfig::default();
    let cases: [([MixRow; 2], MixerConfig, MixerError); 10] = [
        (
            [motor(0), motor(2)],
            good,
            MixerError::MotorGap(Slot::Motor(2)),
        ),
        (
            [motor(0), motor(8)],
            good,
            MixerError::SlotOutOfRange(Slot::Motor(8)),
        ),
        (
            [motor(0), MixRow::pusher(2)],
            good,
            MixerError::MotorGap(Slot::Motor(2)),
        ),
        (
            [MixRow::pusher(0), motor(1)],
            good,
            MixerError::PusherBeforeMotor(Slot::Motor(0)),
        ),
        (
            [motor(0), MixRow::servo(8, 0.0, 0.0, 1.0)],
            good,
            MixerError::SlotOutOfRange(Slot::Servo(8)),
        ),
        (
            [motor(3), motor(3)],
            good,
            MixerError::DuplicateSlot(Slot::Motor(3)),
        ),
        (
            [motor(0), MixRow::motor(1, 0.0, f32::NAN, 0.0)],
            good,
            MixerError::NonFiniteFactor(Slot::Motor(1)),
        ),
        (
            [motor(0), motor(1)],
            MixerConfig {
                thrust_factor: 1.0,
                ..good
            },
            MixerError::ThrustFactor,
        ),
        (
            [motor(0), motor(1)],
            MixerConfig {
                thrust_factor: f32::NAN,
                ..good
            },
            MixerError::ThrustFactor,
        ),
        (
            [motor(0), motor(1)],
            MixerConfig {
                yaw_ratio: -0.5,
                ..good
            },
            MixerError::YawRatio,
        ),
    ];
    for (rows, config, error) in cases {
        assert!(
            TableMixer::new(rows, config) == Err(error),
            "{rows:?} {config:?}"
        );
    }
    assert!(TableMixer::<0>::new([], good) == Err(MixerError::Empty));
    // A pusher alone, or after the motors, is a valid table.
    assert!(TableMixer::new([MixRow::pusher(0)], good).is_ok());
    assert!(TableMixer::new([motor(0), MixRow::pusher(1)], good).is_ok());
    // A motor and a servo may share an index: they are different slots.
    assert!(TableMixer::new([motor(0), MixRow::servo(0, 0.0, 0.0, 1.0)], good).is_ok());
    assert!(
        TableMixer::quad_x().with_config(MixerConfig {
            yaw_ratio: f32::INFINITY,
            ..good
        }) == Err(MixerError::YawRatio)
    );
}

/// Config scales the motor split: a lower thrust factor leaves more headroom for moments.
#[test]
fn config_sets_the_thrust_and_moment_split() {
    let config = MixerConfig {
        thrust_factor: 0.8,
        yaw_ratio: 1.0,
    };
    let mut mixer = TableMixer::quad_x()
        .with_config(config)
        .expect("valid config");
    let output = mixer.mix(&ControlOutput::new(0.5, Vec3::new(0.0, 0.0, 1.0)));
    // Collective 0.4; yaw 1 · headroom 0.2 · ratio 1.
    assert!(
        output.motors[..4]
            .iter()
            .zip([0.2, 0.6, 0.6, 0.2])
            .all(|(&m, e)| approx_eq(m, e))
    );
}
