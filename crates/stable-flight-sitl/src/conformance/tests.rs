use std::f32::consts::TAU;

use super::*;

#[test]
fn every_scenario_accepts_its_expected_state() {
    for scenario in SCENARIOS {
        let result = scenario.check(&scenario.expected_state());
        assert!(result.is_ok(), "{result:?}");
    }
}

#[test]
fn scenarios_are_distinguishable() {
    for (i, a) in SCENARIOS.iter().enumerate() {
        for b in &SCENARIOS[i + 1..] {
            assert!(
                a.check(&b.expected_state()).is_err(),
                "`{}` accepts `{}`",
                a.name,
                b.name
            );
        }
    }
}

#[test]
fn heading_is_compared_modulo_a_full_turn() {
    let mut state = HEADING_EAST.expected_state();
    state.heading += TAU;

    assert!(HEADING_EAST.check(&state).is_ok());
}

#[test]
fn reports_each_field_off_contract() {
    // (scenario, corruption, fields reported)
    type Corrupt = fn(&mut VehicleState);
    let cases: [(Scenario, Corrupt, &[&str]); 4] = [
        (
            CLIMBING,
            |s| s.velocity.z = -s.velocity.z, // down-positive vertical velocity
            &["velocity.z"],
        ),
        (
            ROLL_RIGHT_30,
            |s| s.roll = s.roll.to_radians(), // converted to radians twice
            &["roll"],
        ),
        (
            YAW_RIGHT_RATE,
            |s| s.angular_velocity.z = f32::NAN,
            &["angular_velocity.z"],
        ),
        (
            LEVEL,
            |s| s.position.z = -s.position.z, // altitude as NED down
            &["position.z"],
        ),
    ];

    for (scenario, corrupt, fields) in cases {
        let mut state = scenario.expected_state();
        corrupt(&mut state);

        let error = scenario.check(&state).expect_err(scenario.name);
        let reported: Vec<_> = error.mismatches.iter().map(|m| m.field).collect();
        assert_eq!(reported, fields, "{}", scenario.name);
        assert!(error.to_string().contains(fields[0]), "{error}");
    }
}
