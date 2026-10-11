use approx::assert_relative_eq;

use super::*;

/// Each rule × form against the published rows, for `Ku` 2 and `Tu` 0.5 s.
#[test]
fn gains_follow_the_rule_tables() {
    let point = UltimatePoint {
        ku: 2.0,
        tu_s: 0.5,
        relay: 0.1,
        cycle_amplitude: 0.06,
        bias: 0.0,
    };
    let gains = |p, i, d| Gains { p, i, d };
    let (tl_kp, tl_pi_kp, zn_kp) = (2.0 / 2.2, 2.0 / 3.2, 0.4);
    // (rule, form, expected)
    let cases = [
        (
            TuningRule::TyreusLuyben,
            ControllerForm::Pid,
            gains(tl_kp, tl_kp / 1.1, tl_kp * 0.5 / 6.3),
        ),
        (
            TuningRule::TyreusLuyben,
            ControllerForm::Pi,
            gains(tl_pi_kp, tl_pi_kp / 1.1, 0.0),
        ),
        (
            TuningRule::TyreusLuyben,
            ControllerForm::Pd,
            gains(tl_kp, 0.0, tl_kp * 0.5 / 6.3),
        ),
        (
            TuningRule::TyreusLuyben,
            ControllerForm::P,
            gains(tl_pi_kp, 0.0, 0.0),
        ),
        (
            TuningRule::NoOvershootZn,
            ControllerForm::Pid,
            gains(zn_kp, zn_kp / 0.25, zn_kp * 0.5 / 3.0),
        ),
        (
            TuningRule::NoOvershootZn,
            ControllerForm::Pi,
            gains(zn_kp, zn_kp / 0.25, 0.0),
        ),
        (
            TuningRule::NoOvershootZn,
            ControllerForm::Pd,
            gains(zn_kp, 0.0, zn_kp * 0.5 / 3.0),
        ),
        (
            TuningRule::NoOvershootZn,
            ControllerForm::P,
            gains(zn_kp, 0.0, 0.0),
        ),
    ];
    for (rule, form, expected) in cases {
        let actual = gains_from_ultimate(&point, rule, form);
        assert_relative_eq!(actual.p, expected.p, max_relative = 1e-6);
        assert_relative_eq!(actual.i, expected.i, max_relative = 1e-6);
        assert_relative_eq!(actual.d, expected.d, max_relative = 1e-6);
    }
}
