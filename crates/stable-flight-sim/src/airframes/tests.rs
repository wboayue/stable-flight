use approx::assert_relative_eq;
use assert2::check;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::stabilizers::vtol::TailsitterStabilizer;
use stable_flight::{AttitudeSetpoint, ControlOutput, Mixer, Setpoint, Stabilizer, VehicleState};

use super::*;
use crate::aero::{self, Wash};
use crate::body::Body;
use crate::lockstep::Dynamics;
use crate::motors::Motors;
use crate::wing::{FixedWing, Rotors};

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
    // Power-commanded motors, a 0.8 thrust factor, and a 1.5 thrust curve.
    check_hover_thrust("alia250_hover", alia250_hover());
}

#[test]
fn alia250_is_valid() {
    let airframe = alia250_hover();
    check!(airframe.model.validate() == Ok(()));
    check!(airframe.guidance.validate() == Ok(()));
    check!(airframe.stabilizer.validate() == Ok(()));
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

/// A body rate with its feedforward deflection makes the moment that neither does: the
/// surface cancels the rate damping, on each axis.
#[test]
fn rate_feedforward_cancels_rate_damping() {
    let model = plane().model;
    let airspeed_mps = 15.0;
    let feedforward = plane::rate_feedforward(&model, airspeed_mps);
    let air_mps = Vec3::new(airspeed_mps, 0.0, 0.0);
    let still_nm = aero::wrench(
        &model,
        air_mps,
        Vec3::zeros(),
        Vec3::zeros(),
        Wash::default(),
    )
    .torque_nm;
    for axis in 0..3 {
        let rates_radps = Vec3::ith(axis, 0.5);
        let deflection_rad = rates_radps
            .component_mul(&feedforward)
            .component_mul(&model.max_deflection_rad);
        let torque_nm = aero::wrench(
            &model,
            air_mps,
            rates_radps,
            deflection_rad,
            Wash::default(),
        )
        .torque_nm;
        assert_relative_eq!(torque_nm[axis], still_nm[axis], epsilon = 1e-4);
    }
}

#[test]
fn tailsitters_are_valid() {
    for (name, airframe) in [("tailsitter", tailsitter()), ("explora", explora())] {
        check!(airframe.model.validate() == Ok(()), "{name}");
        check!(airframe.guidance.validate() == Ok(()), "{name}");
        check!(airframe.stabilizer.validate() == Ok(()), "{name}");
    }
}

/// The eXplora hovers on the thrust RealFlight's does (`explora_vtol()`'s 0.82, #124).
#[test]
fn explora_hovers_at_realflights_thrust() {
    assert_relative_eq!(
        explora().guidance.hover.altitude.hover_thrust,
        0.82,
        max_relative = 1e-5
    );
}

/// ArduPilot's tailsitter is its plane with 1.5× the thrust (`SIM_Plane.cpp:78-81`), here
/// split over two motors.
#[test]
fn tailsitter_has_one_and_a_half_times_the_planes_thrust() {
    let (tailsitter, plane) = (tailsitter().model, plane().model);
    assert_relative_eq!(
        2.0 * tailsitter.wing.max_thrust_n,
        1.5 * plane.max_thrust_n,
        max_relative = 1e-6
    );
}

/// Nose up, the hover thrust, through the hover side's curve and the mixer, gives motors whose
/// thrust carries the weight.
#[test]
fn tailsitter_hover_thrust_carries_the_weight() {
    for airframe in [tailsitter(), explora()] {
        check_tailsitter_hover_thrust(airframe);
    }
}

fn check_tailsitter_hover_thrust(airframe: TailsitterAirframe) {
    let mut stabilizer = TailsitterStabilizer::new(airframe.stabilizer);
    let nose_up = VehicleState {
        attitude: UnitQuaternion::from_euler_angles(0.0, core::f32::consts::FRAC_PI_2, 0.0),
        ..Default::default()
    };
    let hover = airframe.guidance.hover.altitude.hover_thrust;
    let setpoint = Setpoint::Attitude(AttitudeSetpoint {
        thrust: hover,
        ..Default::default()
    });
    let output = stabilizer.stabilize(0.004, &setpoint, &nose_up);
    let mut mixer = airframe.mixer;
    let command = mixer.mix(&output);

    // Nose up in still air, after two steps far longer than the motor delay and lag: the
    // first starts the delay, the second runs on the command.
    let mut dynamics = FixedWing::new(
        airframe.model.wing,
        &airframe.mixer,
        Rotors::tailsitter(&airframe.model),
        UnitQuaternion::identity(),
    );
    dynamics.begin(100.0);
    let body = Body {
        attitude: nose_up.attitude,
        ..Body::default()
    };
    for _ in 0..2 {
        dynamics.loads(&command, &body, Vec3::zeros());
    }
    assert_relative_eq!(
        dynamics.lift_n(&body),
        airframe.model.wing.mass_kg * GRAVITY,
        max_relative = 1e-4
    );
    check!((0.5..0.95).contains(&hover), "hover thrust {hover}");
}

/// The quadplane's pusher commands are those its transition and waypoint tests were measured
/// on: at cruise trim, at the forward transition's cap, and for forward thrust at
/// `tan(pitch)` 0.2 (#448 gave the pusher its full range; the values above it keep these).
#[test]
fn quadplane_pusher_commands_are_as_measured() {
    use stable_flight::{ControlOutput, CruiseOutput, Mixer, Vec3};
    let airframe = super::quadplane();
    let mut mixer = airframe.mixer;
    let mut pusher = |thrust| {
        let cruise = CruiseOutput {
            thrust,
            moments: Vec3::zeros(),
        };
        mixer
            .mix(&ControlOutput {
                cruise: Some(cruise),
                ..Default::default()
            })
            .motors[4]
    };
    let guidance = airframe.guidance;
    // (what, pusher command, as measured)
    let cases = [
        ("cruise", pusher(guidance.plane.cruise_thrust), 0.3792),
        (
            "transition",
            pusher(guidance.transition.forward_thrust),
            0.9,
        ),
        ("forward thrust", pusher(0.2 * guidance.forward.gain), 0.216),
    ];
    for (what, command, expected) in cases {
        assert!((command - expected).abs() < 1e-4, "{what}: {command}");
    }
}
