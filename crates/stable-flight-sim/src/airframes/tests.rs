use approx::assert_relative_eq;
use assert2::check;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::{AttitudeSetpoint, ControlOutput, Mixer, Setpoint, Stabilizer, VehicleState};

use super::*;
use crate::motors::Motors;

#[test]
fn quad_x_is_valid() {
    let airframe = quad_x();
    check!(airframe.model.validate() == Ok(()));
    check!(airframe.guidance.validate() == Ok(()));
    check!(airframe.stabilizer.validate() == Ok(()));
}

/// The hover thrust, through the stabilizer's curve and the mixer, gives motors whose thrust
/// carries the weight.
#[test]
fn hover_thrust_carries_the_weight() {
    let airframe = quad_x();
    let mut stabilizer = CopterStabilizer::new(airframe.stabilizer);
    let level = VehicleState::default();
    let setpoint = Setpoint::Attitude(AttitudeSetpoint {
        thrust: airframe.guidance.altitude.hover_thrust,
        ..Default::default()
    });
    let output: ControlOutput = stabilizer.stabilize(0.004, &setpoint, &level);
    let mut mixer = airframe.mixer;
    let command = mixer.mix(&output);

    let mut motors = Motors::new(airframe.mixer.rows(), &airframe.model).expect("motors only");
    motors.spin(&command.motors, 1.0);
    let (thrust_n, _) = motors.wrench(&airframe.model);
    assert_relative_eq!(
        thrust_n,
        airframe.model.mass_kg * GRAVITY,
        max_relative = 1e-4
    );
    // Plausible hover: neither near idle nor near full.
    let hover = airframe.guidance.altitude.hover_thrust;
    check!((0.5..0.95).contains(&hover), "hover thrust {hover}");
}
