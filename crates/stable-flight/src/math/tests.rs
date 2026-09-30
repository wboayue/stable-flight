use core::f32::consts::PI;

use approx::assert_relative_eq;

use super::*;

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
    assert!(saturate_unit_vec2(vec2(2.0, f32::NAN)) == vec2(1.0, 0.0));
}

#[test]
fn wrap_angle_maps_into_minus_pi_to_pi() {
    // (angle, wrapped)
    let cases = [
        (0.0, 0.0),
        (PI / 2.0, PI / 2.0),
        (PI, -PI),
        (-PI, -PI),
        (3.0 * PI / 2.0, -PI / 2.0),
        (-3.0 * PI / 2.0, PI / 2.0),
        (4.0 * PI + 0.25, 0.25),
    ];
    for (angle, wrapped) in cases {
        assert_relative_eq!(wrap_angle(angle), wrapped, epsilon = 1e-5);
    }
    assert!(wrap_angle(f32::NAN).is_nan());
}
