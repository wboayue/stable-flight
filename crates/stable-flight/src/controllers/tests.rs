use super::*;

#[test]
fn check_limit_accepts_only_non_negative_finite_values() {
    // (value, ok)
    let cases = [
        (0.0, true),
        (1.5, true),
        (-0.1, false),
        (f32::INFINITY, false),
        (f32::NAN, false),
    ];
    for (value, ok) in cases {
        assert_eq!(check_limit("max_integral", value).is_ok(), ok, "{value}");
    }
}

#[test]
fn invalid_config_names_the_field_and_requirement() {
    let error = check_limit("max_integral", -1.0).unwrap_err();
    assert_eq!(
        error.to_string(),
        "max_integral must be non-negative and finite"
    );
}

#[test]
fn check_positive_accepts_only_positive_finite_values() {
    // (value, ok)
    let cases = [
        (1e-6, true),
        (2.0, true),
        (0.0, false),
        (-1.0, false),
        (f32::INFINITY, false),
        (f32::NAN, false),
    ];
    for (value, ok) in cases {
        assert_eq!(check_positive("exponent", value).is_ok(), ok, "{value}");
    }
}

#[test]
fn check_unit_interval_accepts_only_values_in_zero_one() {
    // (value, ok)
    let cases = [
        (0.5, true),
        (1.0, true),
        (0.0, false),
        (-0.1, false),
        (1.5, false),
        (f32::INFINITY, false),
        (f32::NAN, false),
    ];
    for (value, ok) in cases {
        assert_eq!(
            check_unit_interval("hover_thrust", value).is_ok(),
            ok,
            "{value}"
        );
    }
}
