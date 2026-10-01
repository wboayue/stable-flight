use core::f32::consts::PI;

use approx::assert_relative_eq;

use super::*;
use crate::Vec3;

#[test]
fn saturation_clamps_to_range_and_maps_nan_to_neutral() {
    // (input, saturate_unit, saturate_throttle)
    let cases = [
        (0.0, 0.0, 0.0),
        (0.4, 0.4, 0.4),
        (-0.4, -0.4, 0.0),
        (1.7, 1.0, 1.0),
        (-4.6, -1.0, 0.0),
        (f32::INFINITY, 1.0, 1.0),
        (f32::NEG_INFINITY, -1.0, 0.0),
        (f32::NAN, 0.0, 0.0),
    ];
    for (input, unit, throttle) in cases {
        assert!(saturate_unit(input) == unit, "saturate_unit({input})");
        assert!(
            saturate_throttle(input) == throttle,
            "saturate_throttle({input})"
        );
    }
    assert!(saturate_unit_vec2(Vec2::new(2.0, f32::NAN)) == Vec2::new(1.0, 0.0));
}

#[test]
fn clamp_each_bounds_every_component_and_passes_nan() {
    // (input, expected) clamped to [-1, 2]
    let cases = [
        ([0.5, -0.5, 1.5], [0.5, -0.5, 1.5]),
        ([3.0, -4.0, 2.0], [2.0, -1.0, 2.0]),
        ([f32::INFINITY, f32::NEG_INFINITY, 0.0], [2.0, -1.0, 0.0]),
    ];
    for (input, expected) in cases {
        let actual = clamp_each(Vec3::from(input), -1.0, 2.0);
        assert!(actual == Vec3::from(expected), "clamp_each({input:?})");
    }
    let with_nan = clamp_each(Vec3::new(f32::NAN, 5.0, -5.0), -1.0, 2.0);
    assert!(with_nan.x.is_nan() && with_nan.y == 2.0 && with_nan.z == -1.0);
}

#[test]
fn euler_angles_match_nalgebra_and_are_nan_for_a_non_finite_attitude() {
    use nalgebra::{Quaternion, UnitQuaternion};

    // (roll, pitch, yaw): round-trips through nalgebra's constructor
    let cases = [
        (0.0, 0.0, 0.0),
        (0.3, -0.2, 1.1),
        (-1.2, 0.8, -2.9),
        (PI / 2.0, -1.4, PI - 0.01),
    ];
    for (roll, pitch, yaw) in cases {
        let attitude = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
        let (r, p, y) = euler_angles(&attitude);
        assert_relative_eq!(r, roll, epsilon = 1e-5);
        assert_relative_eq!(p, pitch, epsilon = 1e-5);
        assert_relative_eq!(y, yaw, epsilon = 1e-5);
    }

    // (w, x, y, z) of a corrupted estimate
    let cases = [
        [f32::NAN, f32::NAN, f32::NAN, f32::NAN],
        [1.0, f32::NAN, 0.0, 0.0],
        [f32::INFINITY, 0.0, 0.0, 0.0],
    ];
    for [w, i, j, k] in cases {
        let attitude = UnitQuaternion::new_unchecked(Quaternion::new(w, i, j, k));
        let (r, p, y) = euler_angles(&attitude);
        assert!(
            r.is_nan() && p.is_nan() && y.is_nan(),
            "({w}, {i}, {j}, {k}) gave ({r}, {p}, {y})"
        );
    }
}

/// Float rounding pushes the `asin` argument just past ±1 near vertical; a valid attitude must
/// still give a finite pitch, not NaN (which the controllers would treat as "no estimate").
#[test]
fn euler_angles_pitch_stays_finite_at_vertical() {
    use nalgebra::{Quaternion, UnitQuaternion};

    // (roll, yaw) paired with pitches swept up to ±π/2
    let cases = [(0.0, 0.0), (0.3, 1.0), (-2.0, -2.5)];
    for (roll, yaw) in cases {
        for step in 0..2000 {
            for sign in [1.0, -1.0] {
                let pitch = sign * (PI / 2.0 - step as f32 * 1e-7);
                let attitude = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
                let (_, p, _) = euler_angles(&attitude);
                assert!(
                    (p - pitch).abs() < 1e-3,
                    "pitch {pitch} (roll {roll}, yaw {yaw}) gave {p}"
                );
            }
        }
    }

    // Slightly non-unit, as a backend reporting unnormalized quaternions may send.
    let over_unit = UnitQuaternion::new_unchecked(Quaternion::new(0.7072, 0.0, 0.7072, 0.0));
    assert_relative_eq!(euler_angles(&over_unit).1, PI / 2.0);
}

#[test]
fn all_finite_rejects_any_nan_or_infinite_component() {
    // (input, all finite)
    let cases = [
        ([0.0, 1.0, -1.0], true),
        ([f32::NAN, 0.0, 0.0], false),
        ([0.0, f32::INFINITY, 0.0], false),
        ([0.0, 0.0, f32::NEG_INFINITY], false),
    ];
    for (input, expected) in cases {
        assert!(
            all_finite(&Vec3::from(input)) == expected,
            "all_finite({input:?})"
        );
    }
}

#[test]
fn wrap_angle_maps_into_minus_pi_to_pi() {
    // (angle, wrapped)
    let cases = [
        (0.0, 0.0),
        (PI / 2.0, PI / 2.0),
        (-0.0, 0.0),
        // Half-open: +π maps to -π, and -π stays.
        (PI, -PI),
        (-PI, -PI),
        (3.0 * PI / 2.0, -PI / 2.0),
        (-3.0 * PI / 2.0, PI / 2.0),
        (4.0 * PI + 0.25, 0.25),
        // Negative inputs: `%` alone would leave these below -π.
        (-PI - 0.25, PI - 0.25),
        (-2.0 * PI, 0.0),
        (-4.0 * PI - 0.25, -0.25),
        (-7.0 * PI / 2.0, PI / 2.0),
    ];
    for (angle, wrapped) in cases {
        assert_relative_eq!(wrap_angle(angle), wrapped, epsilon = 1e-5);
    }
    assert!(wrap_angle(f32::NAN).is_nan());
    assert!(wrap_angle(f32::INFINITY).is_nan());
    assert!(wrap_angle(f32::NEG_INFINITY).is_nan());
}
