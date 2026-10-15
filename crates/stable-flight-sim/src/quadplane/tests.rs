use approx::assert_relative_eq;
use assert2::check;
use stable_flight::{GRAVITY, Vec3};

use super::*;
use crate::aero;
use crate::airframes::{self, PUSHER_SLOT};
use crate::body::Body;
use crate::fixed_wing::FixedWingModel;
use crate::lag;
use crate::lockstep::Dynamics;
use crate::plane::SimPlane;

const TICK: Duration = Duration::from_millis(4);

/// The lift rotors' slots on the airframe's hover mixer.
const LIFT: core::ops::Range<usize> = 0..4;

fn quadplane() -> SimQuadPlane {
    let airframe = airframes::quadplane();
    SimQuadPlane::new(airframe.model, &airframe.mixer).expect("valid airframe")
}

/// Every lift rotor at `lift`, the pusher at `pusher`, and the surfaces at `surfaces`.
fn command(lift: f32, pusher: f32, surfaces: [f32; 3]) -> ActuatorCommand {
    let mut command = ActuatorCommand::default();
    command.motors[LIFT].fill(lift);
    command.motors[usize::from(PUSHER_SLOT)] = pusher;
    command.servos[..3].copy_from_slice(&surfaces);
    command
}

/// The loads after one step of `command` from stopped motors, long enough (1 s) for the lags
/// to all but settle, level and moving through still air at `air_mps` (FRD).
fn loads(command: &ActuatorCommand, air_mps: Vec3) -> crate::body::Loads {
    let mut vehicle = quadplane();
    vehicle.dynamics.begin(1.0);
    let body = Body {
        velocity_ned_mps: air_mps,
        ..Body::default()
    };
    vehicle.dynamics.loads(command, &body, Vec3::zeros())
}

/// The command each motor runs at after one 1 s step from stopped toward `command`.
fn settled(command: f32, time_constant_s: f32) -> f32 {
    lag::factor(1.0, time_constant_s) * command
}

/// Lift rotors at the command whose thrust carries the weight, level and still: the force is
/// the weight straight up and the rotors turn nothing.
#[test]
fn lift_rotors_balance_the_weight_in_a_hover() {
    let model = airframes::quadplane().model;
    let weight_n = model.wing.mass_kg * GRAVITY;
    let rotor_n = weight_n / LIFT.len() as f32;
    let hover = libm::sqrtf(rotor_n / model.lift.max_thrust_n)
        / lag::factor(1.0, model.lift.motor_time_constant_s);
    let loads = loads(&command(hover, 0.0, [0.0; 3]), Vec3::zeros());
    assert_relative_eq!(
        loads.force_ned_n,
        Vec3::new(0.0, 0.0, -weight_n),
        max_relative = 1e-5
    );
    assert_relative_eq!(loads.torque_nm, Vec3::zeros(), epsilon = 1e-6);
}

/// The lift rotors thrust along body −z only and the pusher along body x only, whichever
/// slots the mixer drives.
#[test]
fn lift_rotors_and_pusher_thrust_on_their_own_axes() {
    let model = airframes::quadplane().model;

    let mut vehicle = quadplane();
    vehicle.dynamics.begin(1.0);
    let pusher = vehicle.dynamics.loads(
        &command(0.0, 1.0, [0.0; 3]),
        &Body::default(),
        Vec3::zeros(),
    );
    check!(vehicle.lift_thrusts_n() == [0.0; ActuatorCommand::SLOTS]);
    let pusher_n = model.wing.max_thrust_n * settled(1.0, model.wing.motor_time_constant_s).powi(2);
    assert_relative_eq!(pusher.force_ned_n, Vec3::new(pusher_n, 0.0, 0.0));

    let lift = loads(&command(1.0, 0.0, [0.0; 3]), Vec3::zeros());
    check!(lift.force_ned_n.x == 0.0);
    check!(lift.force_ned_n.y == 0.0);
    check!(lift.force_ned_n.z < 0.0);
}

/// Spinning lift rotors drag on the air's in-plane velocity in proportion to their thrust
/// (`LiftRotors::drag_n_per_mps` at full), on top of the same aerodynamics and pusher.
#[test]
fn spinning_lift_rotors_drag_in_their_plane() {
    let model = airframes::quadplane().model;
    let rotors = model.lift;
    // (case, air velocity FRD m/s, lift command)
    let cases = [
        ("hover thrust, forward", Vec3::new(10.0, 0.0, 0.0), 0.7),
        ("full thrust, sideways", Vec3::new(0.0, -5.0, 0.0), 1.0),
        ("full thrust, sinking", Vec3::new(0.0, 0.0, 3.0), 1.0),
        ("stopped, cruise", Vec3::new(15.0, 0.0, 0.0), 0.0),
    ];
    for (case, air_mps, lift) in cases {
        let spinning = loads(&command(lift, 0.5, [0.0; 3]), air_mps);
        let stopped = loads(&command(0.0, 0.5, [0.0; 3]), air_mps);
        let fraction = settled(lift, rotors.motor_time_constant_s).powi(2);
        let thrust_n = LIFT.len() as f32 * rotors.max_thrust_n * fraction;
        let drag_n = Vec3::new(air_mps.x, air_mps.y, 0.0) * (rotors.drag_n_per_mps * fraction);
        let expected = Vec3::new(0.0, 0.0, -thrust_n) - drag_n;
        let difference = spinning.force_ned_n - stopped.force_ned_n;
        check!(
            (difference - expected).norm() < 1e-4,
            "{case}: {difference:?} vs {expected:?}"
        );
    }
}

/// With the lift rotors stopped, the quadplane flies as the airplane on its wing does, bit
/// for bit: the rotors add no lift and no drag.
#[test]
fn lift_rotors_off_in_cruise_add_nothing() {
    let airframe = airframes::quadplane();
    let start = airframe.start;
    let mut quadplane = quadplane().with_start(start);
    let plane_mixer = stable_flight::mixers::TableMixer::plane();
    let mut plane = SimPlane::new(airframe.model.wing, &plane_mixer)
        .expect("valid airframe")
        .with_start(start);
    let surfaces = [0.1, -0.2, 0.05];
    quadplane.actuate(&command(0.0, 0.6, surfaces));
    let mut plane_command = ActuatorCommand::default();
    plane_command.motors[0] = 0.6;
    plane_command.servos[..3].copy_from_slice(&surfaces);
    plane.actuate(&plane_command);
    for _ in 0..1250 {
        let flown = quadplane.state(TICK);
        let reference = plane.state(TICK);
        check!(flown.position_m == reference.position_m);
        check!(flown.attitude == reference.attitude);
    }
}

/// The pusher and elevator that hold `model` level at `airspeed_mps`, and the angle of attack
/// (= pitch) it flies at: lift and thrust carry the weight, thrust the drag, and the elevator
/// zeroes the pitch moment.
fn cruise_trim(model: &FixedWingModel, airspeed_mps: f32) -> (f32, f32, f32) {
    let weight_n = model.mass_kg * GRAVITY;
    let elevator_per_rad =
        aero::pressure_area_n(model, airspeed_mps) * model.chord_m * model.aero.c_pitch_elevator;
    let trim = |alpha_rad: f32| {
        let air_mps = Vec3::new(libm::cosf(alpha_rad), 0.0, libm::sinf(alpha_rad)) * airspeed_mps;
        let wrench = |elevator_rad| {
            aero::wrench(
                model,
                air_mps,
                Vec3::zeros(),
                Vec3::new(0.0, elevator_rad, 0.0),
                0.0,
            )
        };
        let elevator_rad = -wrench(0.0).torque_nm.y / elevator_per_rad;
        let force_n = wrench(elevator_rad).force_n;
        // Gravity on the body axes, pitched up by α: (−W sin α, 0, W cos α).
        let thrust_n = weight_n * libm::sinf(alpha_rad) - force_n.x;
        let vertical_n = force_n.z + weight_n * libm::cosf(alpha_rad);
        (thrust_n, elevator_rad, vertical_n)
    };
    // Newton's method on the vertical balance.
    let mut alpha_rad = 0.0;
    for _ in 0..20 {
        let step_rad = 1e-4;
        let (_, _, residual_n) = trim(alpha_rad);
        let (_, _, ahead_n) = trim(alpha_rad + step_rad);
        alpha_rad -= residual_n * step_rad / (ahead_n - residual_n);
    }
    let (thrust_n, elevator_rad, _) = trim(alpha_rad);
    (thrust_n, elevator_rad, alpha_rad)
}

/// Altitude band for the pusher-only cruise, in m: measured 0.28 m off over 10 s (the
/// pusher and elevator lags' start-up transient, then a phugoid); 0.5 m leaves 1.8× margin.
const CRUISE_ALTITUDE_M: f32 = 0.5;

/// Airspeed band for the pusher-only cruise, in m/s: measured 0.22 m/s; 0.4 m/s leaves 1.8×
/// margin.
const CRUISE_AIRSPEED_MPS: f32 = 0.4;

/// From level flight at the airframe's trim airspeed with the lift rotors stopped, the
/// pusher and elevator at their trim hold altitude and airspeed open loop.
#[test]
fn pusher_alone_holds_level_flight_at_cruise_trim() {
    let airframe = airframes::quadplane();
    let model = airframe.model.wing;
    let airspeed_mps = airframe.start.velocity_mps.norm();
    let (thrust_n, elevator_rad, alpha_rad) = cruise_trim(&model, airspeed_mps);
    let start = Start {
        attitude: UnitQuaternion::from_euler_angles(0.0, alpha_rad, 0.0),
        ..airframe.start
    };
    let mut quadplane = quadplane().with_start(start);
    let trim = command(
        0.0,
        libm::sqrtf(thrust_n / model.max_thrust_n),
        [0.0, elevator_rad / model.max_deflection_rad.y, 0.0],
    );
    let (mut altitude_m, mut airspeed_off_mps) = (0.0f32, 0.0f32);
    for _ in 0..2500 {
        quadplane.actuate(&trim);
        let state = quadplane.state(TICK);
        altitude_m = altitude_m.max((state.position_m.z - start.position_m.z).abs());
        airspeed_off_mps =
            airspeed_off_mps.max((state.airspeed_mps.unwrap_or(f32::NAN) - airspeed_mps).abs());
    }
    check!(altitude_m < CRUISE_ALTITUDE_M);
    check!(airspeed_off_mps < CRUISE_AIRSPEED_MPS);
    check!(quadplane.lift_thrusts_n() == [0.0; ActuatorCommand::SLOTS]);
}

/// From resting on its gear, the lift rotors at full lift it off, wings level. (Open loop it
/// pitches up as it climbs: the air from above meets the wing and tail broadside, α −90°.)
#[test]
fn takes_off_vertically_on_the_lift_rotors() {
    let mut vehicle = quadplane();
    check!(vehicle.on_ground());
    vehicle.actuate(&command(1.0, 0.0, [0.0; 3]));
    let mut state = vehicle.state(Duration::ZERO);
    for _ in 0..250 {
        state = vehicle.state(TICK);
    }
    check!(!vehicle.on_ground());
    check!(state.position_m.z > 1.0);
    check!(state.roll_rad.abs() < 1e-4);
}

/// Motor mismatch scales a lift rotor by its slot, so a weak front-left rotor rolls the
/// quadplane left and pitches it down.
#[test]
fn motor_mismatch_scales_each_lift_rotor() {
    let mut thrust_scale = [1.0; ActuatorCommand::SLOTS];
    thrust_scale[0] = 0.8;
    let mut vehicle = quadplane()
        .with_perturbations(Perturbations {
            motor_thrust_scale: thrust_scale,
            ..Perturbations::default()
        })
        .expect("valid perturbations");
    vehicle.dynamics.begin(1.0);
    let loads = vehicle.dynamics.loads(
        &command(1.0, 0.0, [0.0; 3]),
        &Body::default(),
        Vec3::zeros(),
    );
    let thrusts_n = vehicle.lift_thrusts_n();
    assert_relative_eq!(thrusts_n[0], 0.8 * thrusts_n[1]);
    check!(loads.torque_nm.x < 0.0);
    check!(loads.torque_nm.y < 0.0);
}

#[test]
fn rejects_an_invalid_model() {
    let airframe = airframes::quadplane();
    let model = QuadPlaneModel {
        pusher_slot: 9,
        ..airframe.model
    };
    check!(SimQuadPlane::new(model, &airframe.mixer).err() == Some(ModelError::PusherSlot(9)));
}
