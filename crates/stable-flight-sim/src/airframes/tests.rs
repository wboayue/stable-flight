use approx::assert_relative_eq;
use assert2::check;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::{AttitudeSetpoint, ControlOutput, Mixer, Setpoint, Stabilizer, VehicleState};

use super::*;
use crate::motors::Motors;

#[test]
fn every_airframe_is_valid() {
    for (name, airframe) in all() {
        check!(airframe.model.validate() == Ok(()), "{name}");
        check!(airframe.guidance.validate() == Ok(()), "{name}");
        check!(airframe.stabilizer.validate() == Ok(()), "{name}");
    }
}

/// ArduPilot's ring model: 3 kg on a 0.35 m diagonal (its default frame).
#[test]
fn ring_inertia_puts_half_the_mass_on_the_motor_ring() {
    let inertia_kgm2 = ring_inertia(3.0, 0.35);
    assert_relative_eq!(inertia_kgm2.x, 3.0 * 0.25 * 0.175 * 0.175);
    check!(inertia_kgm2.x == inertia_kgm2.y);
    assert_relative_eq!(inertia_kgm2.z, 2.0 * inertia_kgm2.x);
}

#[test]
fn x_moment_arm_is_half_the_diagonal_at_45_degrees() {
    // A 250 mm diagonal: 125 mm arms, 88.4 mm from each axis.
    assert_relative_eq!(x_moment_arm_m(0.25), 0.088_388_35, max_relative = 1e-6);
}

#[test]
fn max_thrust_gives_the_thrust_to_weight() {
    let racer = racer().model;
    assert_relative_eq!(
        4.0 * racer.max_thrust_n / (racer.mass_kg * GRAVITY),
        8.0,
        max_relative = 1e-6
    );
}

/// The hover thrust, through the stabilizer's curve and the mixer, gives motors whose thrust
/// carries the weight.
#[test]
fn hover_thrust_carries_the_weight() {
    for (name, airframe) in all() {
        check_hover_thrust(name, airframe);
    }
}

fn check_hover_thrust(name: &str, airframe: Airframe) {
    let mut stabilizer = CopterStabilizer::new(airframe.stabilizer);
    let level = VehicleState::default();
    let setpoint = Setpoint::Attitude(AttitudeSetpoint {
        thrust: airframe.guidance.altitude.hover_thrust,
        ..Default::default()
    });
    let output: ControlOutput = stabilizer.stabilize(0.004, &setpoint, &level);
    let mut mixer = airframe.mixer;
    let command = mixer.mix(&output);

    let mut motors = Motors::new(airframe.mixer.rows(), &airframe.model.motor_layout());
    motors.spin(&command.motors, 1.0);
    let (thrust_n, _) = motors.wrench();
    assert_relative_eq!(
        thrust_n,
        airframe.model.mass_kg * GRAVITY,
        max_relative = 1e-4
    );
    // Plausible hover: neither near idle nor near full.
    let hover = airframe.guidance.altitude.hover_thrust;
    check!((0.5..0.95).contains(&hover), "{name} hover thrust {hover}");
}
