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
}

#[test]
fn saturate_symmetric_bounds_value_and_maps_nan_to_neutral() {
    // (value, limit, expected); a negative or NaN limit acts as 0.
    let cases = [
        (0.3, 0.5, 0.3),
        (0.9, 0.5, 0.5),
        (-0.9, 0.5, -0.5),
        (f32::INFINITY, 0.5, 0.5),
        (f32::NEG_INFINITY, 0.5, -0.5),
        (f32::NAN, 0.5, 0.0),
        (0.3, -1.0, 0.0),
        (0.3, f32::NAN, 0.0),
    ];
    for (value, limit, expected) in cases {
        assert!(
            saturate_symmetric(value, limit) == expected,
            "saturate_symmetric({value}, {limit})"
        );
    }
}

#[test]
fn clamp_symmetric_bounds_value_and_never_panics_on_a_bad_limit() {
    // (value, limit, expected); a negative or NaN limit acts as 0.
    let cases = [
        (0.5, 1.0, 0.5),
        (3.0, 2.0, 2.0),
        (-4.0, 2.0, -2.0),
        (f32::INFINITY, 2.0, 2.0),
        (f32::NEG_INFINITY, 2.0, -2.0),
        (5.0, f32::INFINITY, 5.0),
        (5.0, 0.0, 0.0),
        (5.0, -1.0, 0.0),
        (5.0, f32::NAN, 0.0),
    ];
    for (value, limit, expected) in cases {
        assert!(
            clamp_symmetric(value, limit) == expected,
            "clamp_symmetric({value}, {limit})"
        );
    }
    assert!(clamp_symmetric(f32::NAN, 1.0).is_nan());

    let each = clamp_symmetric_each(Vec3::new(f32::NAN, 5.0, -5.0), 2.0);
    assert!(each.x.is_nan() && each.y == 2.0 && each.z == -2.0);
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

/// Outputs shift together into [0, 1], keeping their differences: down from above 1, up from
/// below 0 by at most twice the collective (none at zero). Spreads over 1 scale about the
/// collective first.
#[test]
fn desaturate_keeps_moments_by_giving_up_collective() {
    // (input, collective, expected)
    let cases: [([f32; 4], f32, [f32; 4]); 11] = [
        ([0.5, 0.5, 0.5, 0.5], 0.5, [0.5, 0.5, 0.5, 0.5]),
        ([1.0, 0.2, 0.0, 0.7], 0.5, [1.0, 0.2, 0.0, 0.7]), // already fits
        ([1.15, 0.85, 1.15, 0.45], 0.9, [1.0, 0.7, 1.0, 0.3]), // too high: down
        ([0.25, -0.15, 0.35, -0.05], 0.1, [0.4, 0.0, 0.5, 0.1]), // too low: up
        ([0.2, -0.2, 0.2, -0.2], 0.0, [0.2, 0.0, 0.2, 0.0]), // zero collective: clipped
        ([0.2, -0.2, 0.2, -0.2], -0.1, [0.2, 0.0, 0.2, 0.0]), // below zero: same
        // Near idle the raise is capped at 2 × collective: up 0.02, not the 0.03 needed.
        ([0.07, -0.03, 0.07, -0.03], 0.01, [0.09, 0.0, 0.09, 0.0]),
        // Spread 2 about 0.5: shares halve, fitting exactly.
        ([1.5, -0.5, 0.5, 0.5], 0.5, [1.0, 0.0, 0.5, 0.5]),
        // Spread 2 about 0.8: halved to [1.2, 0.2], then down 0.2.
        ([1.6, -0.4, 0.8, 0.8], 0.8, [1.0, 0.0, 0.6, 0.6]),
        // Spread 1.25 about 0.1: scaled by 0.8 to [0.9, -0.1], then up 0.1.
        ([1.1, -0.15, 0.1, 0.1], 0.1, [1.0, 0.0, 0.2, 0.2]),
        // Spread 2 at zero collective: halved to [0.5, -0.5], not raised, clipped.
        ([1.0, -1.0, 0.0, 0.0], 0.0, [0.5, 0.0, 0.0, 0.0]),
    ];
    for (input, collective, expected) in cases {
        let mut outputs = input;
        desaturate(&mut outputs, collective);
        for (o, e) in outputs.iter().zip(expected) {
            assert_relative_eq!(*o, e, epsilon = 1e-6);
        }
    }
}

/// Long vectors shrink to `max` along their direction; short ones pass; non-finite ones have
/// no direction and become zero.
#[test]
fn cap_magnitude_keeps_direction_and_neutralizes_non_finite() {
    let nan = f32::NAN;
    let inf = f32::INFINITY;
    // (input, max, expected)
    let cases = [
        (Vec2::new(0.3, -0.4), 1.0, Vec2::new(0.3, -0.4)),
        (Vec2::new(3.0, 4.0), 1.0, Vec2::new(0.6, 0.8)),
        (Vec2::new(-30.0, 40.0), 0.5, Vec2::new(-0.3, 0.4)),
        (Vec2::new(nan, 1.0), 1.0, Vec2::zeros()),
        (Vec2::new(inf, 0.0), 1.0, Vec2::zeros()),
        (Vec2::new(-inf, inf), 0.6, Vec2::zeros()),
    ];
    for (input, max, expected) in cases {
        let capped = cap_magnitude(input, max);
        assert_relative_eq!(capped.x, expected.x, epsilon = 1e-6);
        assert_relative_eq!(capped.y, expected.y, epsilon = 1e-6);
    }
}

#[test]
fn assert_dt_accepts_only_positive_finite_steps() {
    // (dt, ok)
    let cases = [
        (0.004, true),
        (1.0, true),
        (0.0, false),
        (-0.01, false),
        (f32::NAN, false),
        (f32::INFINITY, false),
    ];
    for (dt, ok) in cases {
        assert_eq!(
            std::panic::catch_unwind(|| assert_dt(dt)).is_ok(),
            ok,
            "{dt}"
        );
    }
}

/// `tilt_boost` is `1 / cos(tilt)` from the rotation matrix's `(2, 2)` entry, at most 2× past
/// 60°, fading linearly to 0 as the cosine falls from 0.1 to 0 (about 84°–90°) and 0 when
/// inverted; a non-finite attitude counts as level.
#[test]
fn tilt_boost_compensates_tilt_and_cuts_out_inverted() {
    let euler = |roll: f32, pitch: f32, yaw: f32| {
        UnitQuaternion::from_euler_angles(
            f32::to_radians(roll),
            f32::to_radians(pitch),
            f32::to_radians(yaw),
        )
    };
    // Below 60°: exactly 1 / cos(tilt).
    for (roll, pitch, yaw) in [
        (0.0, 0.0, 0.0),
        (30.0, 0.0, 1.0),
        (0.0, -30.0, -2.0),
        (20.0, 20.0, 0.5),
        (-45.0, 10.0, 3.0),
    ] {
        let attitude = euler(roll, pitch, yaw);
        let expected = 1.0 / attitude.to_rotation_matrix()[(2, 2)];
        assert_relative_eq!(tilt_boost(&attitude), expected, epsilon = 1e-5);
    }
    let roll_for_cosine = |cosine: f32| euler(cosine.acos().to_degrees(), 0.0, 0.0);
    let nan = UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(f32::NAN, 0.0, 0.0, 0.0));
    // (attitude, expected boost)
    let cases = [
        (euler(70.0, 0.0, 0.0), 2.0),  // floored
        (roll_for_cosine(0.1), 2.0),   // start of the fade: no jump
        (roll_for_cosine(0.05), 1.0),  // halfway
        (euler(90.0, 0.0, 0.0), 0.0),  // on its side
        (euler(120.0, 0.0, 0.0), 0.0), // past it
        (euler(180.0, 0.0, 0.0), 0.0), // upside down
        (
            euler(0.0, 89.0, 0.0),
            2.0 * (89.0_f32.to_radians().cos() / 0.1),
        ),
        (nan, 1.0),
    ];
    for (attitude, expected) in cases {
        assert_relative_eq!(tilt_boost(&attitude), expected, epsilon = 1e-4);
    }
}
