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
