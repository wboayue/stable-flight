use approx::assert_relative_eq;
use assert2::check;
use stable_flight::GRAVITY;

use super::*;
use crate::airframes;

/// The first two motors at `motors` and the first two servos (the elevons) at `elevons`.
fn command(motors: [f32; 2], elevons: [f32; 2]) -> ActuatorCommand {
    let mut command = ActuatorCommand::default();
    command.motors[..2].copy_from_slice(&motors);
    command.servos[..2].copy_from_slice(&elevons);
    command
}

fn plane() -> FixedWing {
    let airframe = airframes::plane();
    FixedWing::new(airframe.model, &airframe.mixer, Rotors::airplane())
}

fn tailsitter() -> FixedWing {
    let airframe = airframes::tailsitter();
    let rotors = Rotors {
        motor_arm_m: airframe.model.motor_arm_m,
        wash: Some(airframe.model.prop_wash),
        rest: UnitQuaternion::identity(),
    };
    FixedWing::new(airframe.model.wing, &airframe.mixer, rotors)
}

/// The loads on the airframe: prop thrust along the nose plus the aerodynamics, rotated to
/// the world, and the lift that would carry the weight on the ground.
#[test]
fn loads_combine_prop_and_aero() {
    let mut dynamics = plane();
    dynamics.begin(1.0);
    let body = Body {
        velocity_ned_mps: Vec3::new(15.0, 0.0, 0.0),
        ..Body::default()
    };
    let mut full = ActuatorCommand::default();
    full.motors[0] = 1.0;
    let loads = dynamics.loads(&full, &body, Vec3::zeros());
    let full_alpha = lag::factor(1.0, dynamics.model.motor_time_constant_s);
    let thrust_n = dynamics.model.max_thrust_n * full_alpha * full_alpha;
    let aero = dynamics.aero;
    assert_relative_eq!(
        loads.force_ned_n,
        aero.force_n + Vec3::new(thrust_n, 0.0, 0.0)
    );
    check!(loads.torque_nm == aero.torque_nm);
    check!(loads.damping_nm == Vec3::zeros());
    // At 15 m/s head-on, the wing's lift is short of the weight (it flies at α ≈ −2° only
    // because c_lift_0 is large): lift_n reads it.
    assert_relative_eq!(dynamics.lift_n(&body), -aero.force_n.z, max_relative = 1e-5);
    check!(dynamics.lift_n(&body) > 0.5 * dynamics.model.mass_kg * GRAVITY);
}

/// The tailsitter's torque in still air after one step far longer than the lags, so every
/// motor and servo runs at its command: only the motors and the surfaces in their wash act.
fn still_air_torque_nm(command: &ActuatorCommand) -> Vec3 {
    let mut dynamics = tailsitter();
    dynamics.begin(100.0);
    dynamics
        .loads(command, &Body::default(), Vec3::zeros())
        .torque_nm
}

/// In still air, `tailsitter_2p`'s moments come back as the mixer means them: differential
/// thrust yaws, and the elevons in the wash pitch together and roll apart.
#[test]
fn still_air_moments_follow_the_tailsitter_mixer() {
    // (case, motors, elevons, axis that turns, its sign)
    let cases = [
        ("left motor yaws right", [0.8, 0.6], [0.0, 0.0], 2, 1.0),
        ("right motor yaws left", [0.6, 0.8], [0.0, 0.0], 2, -1.0),
        ("elevons up pitch up", [0.7, 0.7], [0.5, 0.5], 1, 1.0),
        (
            "right elevon up rolls right",
            [0.7, 0.7],
            [-0.5, 0.5],
            0,
            1.0,
        ),
    ];
    for (case, motors, elevons, axis, sign) in cases {
        let torque_nm = still_air_torque_nm(&command(motors, elevons));
        check!(torque_nm[axis] * sign > 0.0, "{case}: {torque_nm:?}");
        for other in (0..3).filter(|other| *other != axis) {
            check!(torque_nm[other].abs() < 1e-4, "{case}: {torque_nm:?}");
        }
    }
}

/// Differential thrust yaws through the motor arm: `arm · (left − right)`.
#[test]
fn differential_thrust_yaws_through_the_arm() {
    let airframe = airframes::tailsitter();
    let (arm_m, max_thrust_n) = (airframe.model.motor_arm_m, airframe.model.wing.max_thrust_n);
    let torque_nm = still_air_torque_nm(&command([0.8, 0.6], [0.0, 0.0]));
    let expected_nm = arm_m * max_thrust_n * (0.8 * 0.8 - 0.6 * 0.6);
    assert_relative_eq!(torque_nm.z, expected_nm, max_relative = 1e-4);
}

/// The wash gives the elevons authority in still air, growing with thrust; without thrust, or
/// on the airplane (no wash), they have none.
#[test]
fn wash_scales_elevon_authority_with_thrust() {
    let pitch_nm = |throttle: f32| still_air_torque_nm(&command([throttle; 2], [0.5, 0.5])).y;
    check!(pitch_nm(0.0) == 0.0);
    // Thrust goes as the command squared, and so does the wash's pressure.
    assert_relative_eq!(pitch_nm(0.8), 4.0 * pitch_nm(0.4), max_relative = 1e-4);

    let mut airplane = plane();
    airplane.begin(100.0);
    let mut elevator = ActuatorCommand::default();
    elevator.motors[0] = 0.8;
    elevator.servos[1] = 0.5;
    let still = airplane.loads(&elevator, &Body::default(), Vec3::zeros());
    check!(still.torque_nm == Vec3::zeros());
}
