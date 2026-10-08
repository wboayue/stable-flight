use core::f32::consts::{PI, TAU};

use nalgebra::Quaternion;

use super::*;

fn expected(scenario: &Scenario) -> VehicleState {
    scenario.expected_state()
}

// The kit's own coverage: a perfect backend passes every scenario.
crate::conformance_tests! {
    expected;
    conforms_level: LEVEL;
    conforms_roll_right_30: ROLL_RIGHT_30;
    conforms_pitch_up_15: PITCH_UP_15;
    conforms_heading_east: HEADING_EAST;
    conforms_banked_climb_east: BANKED_CLIMB_EAST;
    conforms_climbing: CLIMBING;
    conforms_moving_northeast: MOVING_NORTHEAST;
    conforms_yaw_right_rate: YAW_RIGHT_RATE;
    conforms_combined_rates: COMBINED_RATES;
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
fn angles_are_compared_modulo_a_full_turn() {
    type Shift = fn(&mut VehicleState);
    // (scenario, shift, still conforms)
    let cases: [(Scenario, Shift, bool); 6] = [
        (HEADING_EAST, |s| s.heading += TAU, true),
        (HEADING_EAST, |s| s.heading -= TAU, true),
        (ROLL_RIGHT_30, |s| s.roll += TAU, true),
        (PITCH_UP_15, |s| s.pitch -= TAU, true),
        // Heading π reported as -π: the same direction.
        (
            Scenario {
                heading_deg: 180.0,
                ..LEVEL
            },
            |s| s.heading = -PI,
            true,
        ),
        // A real error hidden behind a full turn still fails.
        (HEADING_EAST, |s| s.heading += TAU + 0.01, false),
    ];

    for (scenario, shift, conforms) in cases {
        let mut state = scenario.expected_state();
        shift(&mut state);

        let result = scenario.check(&state);
        assert_eq!(result.is_ok(), conforms, "{}: {result:?}", scenario.name);
    }
}

#[test]
fn reports_each_field_off_contract() {
    type Corrupt = fn(&mut VehicleState);
    // (scenario, corruption, fields reported)
    let cases: [(Scenario, Corrupt, &[&str]); 6] = [
        (
            CLIMBING,
            |s| s.velocity_mps.z = -s.velocity_mps.z, // down-positive vertical velocity
            &["velocity_mps.z"],
        ),
        (
            CLIMBING,
            |s| s.acceleration_mps2.z = -s.acceleration_mps2.z, // down-positive vertical acceleration
            &["acceleration_mps2.z"],
        ),
        (
            ROLL_RIGHT_30,
            |s| s.roll = s.roll.to_radians(), // converted to radians twice
            &["roll"],
        ),
        (
            MOVING_NORTHEAST,
            |s| s.position_m.swap_rows(0, 1), // ENU vs NEU
            &["position_m.x", "position_m.y"],
        ),
        (
            COMBINED_RATES,
            |s| s.angular_velocity.z = f32::NAN,
            &["angular_velocity.z"],
        ),
        (
            LEVEL,
            |s| s.position_m.z = -s.position_m.z, // altitude as NED down
            &["position_m.z"],
        ),
    ];

    for (scenario, corrupt, fields) in cases {
        let mut state = scenario.expected_state();
        corrupt(&mut state);

        let error = scenario.check(&state).expect_err(scenario.name);
        let reported: Vec<_> = error.mismatches().iter().map(|m| m.field).collect();
        assert_eq!(reported, fields, "{}", scenario.name);
        assert!(error.to_string().contains(fields[0]), "{error}");
    }
}

#[test]
fn reports_every_field_when_all_are_nan() {
    // All NaN: every field is reported, and CHECKED_FIELDS holds them all.
    let all_nan = VehicleState {
        position_m: Vec3::new(f32::NAN, f32::NAN, f32::NAN),
        velocity_mps: Vec3::new(f32::NAN, f32::NAN, f32::NAN),
        acceleration_mps2: Vec3::new(f32::NAN, f32::NAN, f32::NAN),
        attitude: UnitQuaternion::new_unchecked(Quaternion::new(
            f32::NAN,
            f32::NAN,
            f32::NAN,
            f32::NAN,
        )),
        angular_velocity: Vec3::new(f32::NAN, f32::NAN, f32::NAN),
        roll: f32::NAN,
        pitch: f32::NAN,
        heading: f32::NAN,
    };

    let error = LEVEL.check(&all_nan).unwrap_err();
    assert_eq!(error.mismatches().len(), CHECKED_FIELDS);
}
