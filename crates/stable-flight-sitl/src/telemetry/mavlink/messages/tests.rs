use assert2::check;

use super::*;

#[test]
fn maps_normalized_thrust_to_throttle_percent() {
    // (thrust, expected percent)
    let cases = [
        (0.0, 0),
        (0.5, 50),
        (0.734, 73),
        (1.0, 100),
        (-0.2, 0),
        (1.7, 100),
        (f32::NAN, 0),
    ];

    for (thrust, expected) in cases {
        check!(throttle_percent(thrust) == expected, "thrust {thrust}");
    }
}
