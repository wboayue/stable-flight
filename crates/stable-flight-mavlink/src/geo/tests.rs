use assert2::check;

use super::*;

#[test]
fn rejects_an_out_of_range_origin() {
    let origin = |latitude_deg, longitude_deg, altitude_m| GeodeticOrigin {
        latitude_deg,
        longitude_deg,
        altitude_m,
    };
    // (origin, valid)
    let cases = [
        (DEFAULT_ORIGIN, true),
        (origin(89.9, -180.0, -400.0), true),
        (origin(-89.9, 180.0, 8_000.0), true),
        (origin(90.0, 0.0, 0.0), false), // pole: longitude degenerates
        (origin(-90.0, 0.0, 0.0), false),
        (origin(f64::NAN, 0.0, 0.0), false),
        (origin(0.0, 180.5, 0.0), false),
        (origin(0.0, f64::NAN, 0.0), false),
        (origin(0.0, 0.0, f64::INFINITY), false),
    ];

    for (origin, valid) in cases {
        let result = FlatEarth::new(origin);
        if valid {
            check!(result.is_ok(), "{origin:?}");
        } else {
            assert2::assert!(let Err(InvalidOrigin(rejected)) = result);
            // Compared as text: NaN never equals itself.
            check!(format!("{rejected:?}") == format!("{origin:?}"));
        }
    }
}
