use assert2::check;

use super::*;

#[test]
fn parses_lat_lon_alt_and_rejects_malformed_or_out_of_range_origins() {
    // (argument, accepted)
    let cases = [
        ("34.884389,-120.4035,0", true),
        (" 47.6 , -122.3 , 56.5 ", true), // spaces around the parts
        ("-89.9,180,-400", true),
        ("34.8,-120.4", false),     // two parts
        ("34.8,-120.4,0,1", false), // four parts
        ("north,-120.4,0", false),
        ("34.8,,0", false),
        ("90,0,0", false),  // the pole: out of range
        ("0,181,0", false), // out of range
        ("0,0,inf", false), // not finite
        ("", false),
    ];
    for (arg, accepted) in cases {
        check!(parse_origin(arg).is_ok() == accepted, "{arg:?}");
    }
}

#[test]
fn accepts_only_positive_finite_speeds() {
    // (argument, expected)
    let cases = [
        ("5", Some(5.0)),
        (" 2.5 ", Some(2.5)),
        ("0", None),
        ("-1", None),
        ("NaN", None),
        ("inf", None),
        ("fast", None),
    ];
    for (arg, expected) in cases {
        check!(parse_speed(arg).ok() == expected, "{arg:?}");
    }
}
