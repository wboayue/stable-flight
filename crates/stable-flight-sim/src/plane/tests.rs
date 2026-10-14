use approx::assert_relative_eq;
use assert2::check;
use stable_flight::GRAVITY;

use super::*;
use crate::airframes;

const TICK: Duration = Duration::from_millis(4);

fn plane() -> SimPlane {
    let airframe = airframes::plane();
    SimPlane::new(airframe.model, &airframe.mixer)
        .expect("valid airframe")
        .with_start(airframe.start)
}

/// The prop at `throttle` and the aileron, elevator, and rudder servos at `surfaces`.
fn command(throttle: f32, surfaces: [f32; 3]) -> ActuatorCommand {
    let mut command = ActuatorCommand::default();
    command.motors[0] = throttle;
    command.servos[..3].copy_from_slice(&surfaces);
    command
}

/// Runs `ticks` loop ticks: read state, then actuate `command`, as `FlightLoop::step` does.
fn fly(plane: &mut SimPlane, command: &ActuatorCommand, ticks: usize) -> VehicleState {
    let mut state = plane.state(Duration::ZERO);
    for _ in 0..ticks {
        state = plane.state(TICK);
        plane.actuate(command);
    }
    state
}

#[test]
fn starts_where_placed() {
    let airframe = airframes::plane();
    let mut plane = plane();
    let state = plane.state(Duration::ZERO);
    check!(state.position_m == airframe.start.position_m);
    check!(state.velocity_mps == airframe.start.velocity_mps);
    assert_relative_eq!(state.airspeed_mps.unwrap_or(f32::NAN), 15.0);
    check!(!plane.on_ground());
    check!(plane.time_s() == 0.0);
}

/// Each servo, through the plane mixer's slots, turns the airplane the frame contract's way.
#[test]
fn each_surface_turns_its_axis() {
    // (case, aileron/elevator/rudder servos, axis that must turn positive, its sign)
    let cases = [
        ("aileron right", [0.5, 0.0, 0.0], 0, 1.0),
        ("aileron left", [-0.5, 0.0, 0.0], 0, -1.0),
        ("elevator up", [0.0, 0.5, 0.0], 1, 1.0),
        ("rudder right", [0.0, 0.0, 0.5], 2, 1.0),
    ];
    let trim = command(0.6, [0.0; 3]);
    for (case, surfaces, axis, sign) in cases {
        let mut neutral = plane();
        let mut deflected = plane();
        let neutral = fly(&mut neutral, &trim, 50);
        let state = fly(&mut deflected, &command(0.6, surfaces), 50);
        let change = state.angular_velocity_radps - neutral.angular_velocity_radps;
        check!(change[axis] * sign > 0.0, "{case}: {change:?}");
    }
}

/// Specific energy (height plus speed), in m.
fn energy_m(state: &VehicleState) -> f32 {
    state.position_m.z + state.velocity_mps.norm_squared() / (2.0 * GRAVITY)
}

/// The prop adds energy and drag takes it: full throttle gains it, a glide loses it.
#[test]
fn the_prop_adds_energy_and_drag_takes_it() {
    let start_m = energy_m(&plane().state(Duration::ZERO));
    let mut powered = plane();
    let mut gliding = plane();
    let powered = fly(&mut powered, &command(1.0, [0.0; 3]), 250);
    let gliding = fly(&mut gliding, &command(0.0, [0.0; 3]), 250);
    check!(energy_m(&powered) > start_m + 1.0);
    check!(energy_m(&gliding) < start_m - 1.0);
}

#[test]
fn reports_flow_angles() {
    let mut plane = plane().with_wind(Wind {
        steady_mps: Vec3::new(0.0, -3.0, 0.0),
        gust: None,
    });
    plane.state(Duration::from_millis(1));
    // Flying north in a wind from the east (blowing west): air from the right.
    check!(plane.sideslip_rad() > 0.1);
    // Level at 15 m/s with the start's zero pitch: the wing meets the air head-on.
    check!(plane.angle_of_attack_rad().abs() < 0.01);
}

#[test]
fn servo_commands_wait_out_the_actuator_delay() {
    let mut plane = plane()
        .with_perturbations(Perturbations {
            actuator_delay_s: 0.02,
            ..Default::default()
        })
        .expect("valid perturbations");
    let mut undelayed = super::tests::plane();
    let roll = command(0.0, [1.0, 0.0, 0.0]);
    // Within the delay, the aileron hasn't moved: same as an airplane never commanded.
    plane.actuate(&roll);
    let delayed = plane.state(Duration::from_millis(16));
    let idle = undelayed.state(Duration::from_millis(16));
    check!(delayed.angular_velocity_radps == idle.angular_velocity_radps);
    let later = plane.state(Duration::from_millis(40));
    check!(later.angular_velocity_radps.x > idle.angular_velocity_radps.x + 0.01);
}

#[test]
fn rests_on_the_ground_by_default() {
    let airframe = airframes::plane();
    let mut plane = SimPlane::new(airframe.model, &airframe.mixer).expect("valid airframe");
    let state = fly(&mut plane, &command(0.0, [0.0; 3]), 25);
    check!(plane.on_ground());
    check!(state.position_m == Vec3::zeros());
}

#[test]
fn rejects_an_invalid_model_or_perturbations() {
    let airframe = airframes::plane();
    let model = FixedWingModel {
        chord_m: -1.0,
        ..airframe.model
    };
    check!(SimPlane::new(model, &airframe.mixer).err() == Some(ModelError::NotPositive("chord_m")));
    let delay = Perturbations {
        actuator_delay_s: 1.0,
        ..Default::default()
    };
    check!(plane().with_perturbations(delay).err() == Some(ModelError::DelayTooLong));
}

/// The loads on the airframe: prop thrust along the nose plus the aerodynamics, rotated to
/// the world, and the lift that would carry the weight on the ground.
#[test]
fn loads_combine_prop_and_aero() {
    let mut dynamics = plane().dynamics;
    dynamics.begin(1.0);
    let body = Body {
        velocity_ned_mps: Vec3::new(15.0, 0.0, 0.0),
        ..Body::default()
    };
    let loads = dynamics.loads(&command(1.0, [0.0; 3]), &body, Vec3::zeros());
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
