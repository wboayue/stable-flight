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

#[test]
fn rescale_integral_keeps_the_i_term_or_resets() {
    // (integral, old_i, new_i, expected)
    let cases = [
        (0.4, 0.1, 0.1, 0.4),      // unchanged gain
        (0.4, 0.1, 0.2, 0.2),      // doubled gain halves the integral
        (0.4, 0.2, 0.1, 0.8),      // halved gain doubles it
        (-0.4, 0.1, -0.2, 0.2),    // sign flip keeps the term's sign
        (0.4, 0.1, 0.0, 0.0),      // gain removed
        (0.4, 0.0, 0.1, 0.0),      // never applied, so not carried in
        (0.4, 0.0, 0.0, 0.4),      // unchanged zero gain
        (0.4, 1e30, 1e-30, 0.0),   // ratio overflows
        (0.4, 0.1, f32::NAN, 0.0), // non-finite gain
    ];
    for (integral, old_i, new_i, expected) in cases {
        assert_eq!(
            rescale_integral(integral, old_i, new_i),
            expected,
            "{integral} {old_i} → {new_i}"
        );
    }
}
