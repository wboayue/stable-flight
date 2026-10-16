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
    FixedWing::new(
        airframe.model,
        &airframe.mixer,
        Rotors::airplane(),
        UnitQuaternion::identity(),
    )
}

fn tailsitter() -> FixedWing {
    let airframe = airframes::tailsitter();
    FixedWing::new(
        airframe.model.wing,
        &airframe.mixer,
        Rotors::tailsitter(&airframe.model),
        UnitQuaternion::identity(),
    )
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

/// Thrust falls off with forward airspeed as `full_thrust_n` gives it, and a thrust line below
/// the center of gravity pitches the nose up by thrust times its offset.
#[test]
fn thrust_falls_off_with_airspeed_and_pitches_from_its_line() {
    // (falloff N per m/s, thrust line below the CG m)
    for (falloff, below_m) in [(0.0, 0.0), (0.5, 0.0), (0.5, 0.1), (0.0, -0.1)] {
        let airframe = airframes::plane();
        let model = FixedWingModel {
            thrust_falloff_n_per_mps: falloff,
            thrust_line_below_cg_m: below_m,
            ..airframe.model
        };
        let mut dynamics = FixedWing::new(
            model,
            &airframe.mixer,
            Rotors::airplane(),
            UnitQuaternion::identity(),
        );
        dynamics.begin(1.0);
        let body = Body {
            velocity_ned_mps: Vec3::new(15.0, 0.0, 0.0),
            ..Body::default()
        };
        let mut full = ActuatorCommand::default();
        full.motors[0] = 1.0;
        let loads = dynamics.loads(&full, &body, Vec3::zeros());
        let full_alpha = lag::factor(1.0, model.motor_time_constant_s);
        let thrust_n = model.full_thrust_n(15.0) * full_alpha * full_alpha;
        let aero = dynamics.aero;
        let label = format!("falloff {falloff}, line {below_m}");
        assert_relative_eq!(
            loads.force_ned_n,
            aero.force_n + Vec3::new(thrust_n, 0.0, 0.0)
        );
        assert_relative_eq!(
            loads.torque_nm,
            aero.torque_nm + Vec3::new(0.0, thrust_n * below_m, 0.0)
        );
        check!(
            (thrust_n < model.max_thrust_n * full_alpha * full_alpha) == (falloff > 0.0),
            "{label}"
        );
    }
}

/// The thrust at full command never goes negative, however fast the air.
#[test]
fn full_thrust_stops_at_zero() {
    let model = FixedWingModel {
        thrust_falloff_n_per_mps: 1.0,
        ..airframes::plane().model
    };
    check!(model.full_thrust_n(-5.0) == model.max_thrust_n);
    check!(model.full_thrust_n(1.0e6) == 0.0);
}

/// The tailsitter's torque moving at `velocity_mps` (body axes) through still air, after one
/// step far longer than the lags, so every motor and servo runs at its command.
fn torque_nm(command: &ActuatorCommand, velocity_mps: Vec3) -> Vec3 {
    let mut dynamics = tailsitter();
    dynamics.begin(100.0);
    let body = Body {
        velocity_ned_mps: velocity_mps,
        ..Body::default()
    };
    dynamics.loads(command, &body, Vec3::zeros()).torque_nm
}

/// [`torque_nm`] at rest: only the motors and the surfaces in their wash act.
fn still_air_torque_nm(command: &ActuatorCommand) -> Vec3 {
    torque_nm(command, Vec3::zeros())
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

/// Sinking tail first into the wash takes the elevons' authority: less pitch per deflection
/// the faster it sinks, none sinking as fast as the static jet (#123).
#[test]
fn sinking_into_the_wash_takes_elevon_authority() {
    // The elevons' pitch moment alone: deflected less centered, at the same motion.
    let elevon_pitch_nm = |sink_mps: f32| {
        let velocity_mps = Vec3::new(-sink_mps, 0.0, 0.0);
        torque_nm(&command([0.7; 2], [0.5, 0.5]), velocity_mps).y
            - torque_nm(&command([0.7; 2], [0.0, 0.0]), velocity_mps).y
    };
    let hover_nm = elevon_pitch_nm(0.0);
    check!(hover_nm > 0.0);
    check!(elevon_pitch_nm(2.0) < hover_nm);
    check!(elevon_pitch_nm(5.0) < elevon_pitch_nm(2.0));

    // The static jet at 0.7 on both motors.
    let airframe = airframes::tailsitter();
    let thrust_n = 2.0 * airframe.model.wing.max_thrust_n * 0.49;
    let disc_pa = thrust_n / airframe.model.prop_wash.disc_area_m2;
    let jet_mps = libm::sqrtf(2.0 * disc_pa / aero::AIR_DENSITY_KGPM3);
    check!(elevon_pitch_nm(jet_mps).abs() < 1e-3 * hover_nm);
}
