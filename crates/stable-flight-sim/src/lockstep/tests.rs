use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::testing::conformance::Scenario;

use super::*;

/// The simulator's native body state for `scenario`: the physical truth in NED.
fn native_fixture(scenario: &Scenario) -> Body {
    let [roll, pitch, heading] =
        [scenario.roll_deg, scenario.pitch_deg, scenario.heading_deg].map(f32::to_radians);
    let ned = |[n, e, u]: [f32; 3]| Vec3::new(n, e, -u);
    Body {
        position_ned_m: ned(scenario.position_neu_m),
        velocity_ned_mps: ned(scenario.velocity_neu_mps),
        acceleration_ned_mps2: ned(scenario.acceleration_neu_mps2),
        attitude: UnitQuaternion::from_euler_angles(roll, pitch, heading),
        angular_velocity_radps: Vec3::from(scenario.body_rates_degps.map(f32::to_radians)),
    }
}

fn state_for(scenario: &Scenario) -> VehicleState {
    vehicle_state(&native_fixture(scenario), Vec3::zeros())
}

stable_flight::conformance_tests! {
    state_for;
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
