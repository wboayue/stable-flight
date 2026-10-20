use assert2::check;

use super::*;

/// A name and a number around the first `=`, trimmed; no `=` or a non-number is an error.
#[test]
fn parses_name_equals_value() {
    // (argument, expected)
    let cases = [
        ("FWD_THR_GAIN=1.2", Ok(("FWD_THR_GAIN".to_owned(), 1.2))),
        (
            " GUID_THR_MIN = 0.75 ",
            Ok(("GUID_THR_MIN".to_owned(), 0.75)),
        ),
        (
            "STAB_ATT_AMAX_X=-40",
            Ok(("STAB_ATT_AMAX_X".to_owned(), -40.0)),
        ),
        ("GUID_TILT_MAX", Err(ParamArgError::MissingEquals)),
    ];
    for (arg, expected) in cases {
        check!(parse_param(arg) == expected, "{arg}");
    }
    check!(matches!(
        parse_param("GUID_TILT_MAX=steep"),
        Err(ParamArgError::InvalidValue(_))
    ));
}
