use core::f32::consts::FRAC_PI_2;

use approx::assert_relative_eq;
use assert2::check;
use stable_flight::euler_angles;

use super::*;
use crate::airframes;

const TICK: Duration = Duration::from_millis(4);

fn tailsitter() -> SimTailsitter {
    let airframe = airframes::tailsitter();
    SimTailsitter::new(airframe.model, &airframe.mixer).expect("valid airframe")
}

/// Both motors at `throttle`, elevons centered.
fn throttle(throttle: f32) -> ActuatorCommand {
    let mut command = ActuatorCommand::default();
    command.motors[..2].fill(throttle);
    command
}

/// Runs `ticks` loop ticks: read state, then actuate `command`, as `FlightLoop::step` does.
fn fly(tailsitter: &mut SimTailsitter, command: &ActuatorCommand, ticks: usize) -> VehicleState {
    let mut state = tailsitter.state(Duration::ZERO);
    for _ in 0..ticks {
        state = tailsitter.state(TICK);
        tailsitter.actuate(command);
    }
    state
}

/// The hover view's roll, pitch, and heading of `state`.
fn view_angles(state: &VehicleState) -> (f32, f32, f32) {
    euler_angles(&(state.attitude * View::tailsitter(()).rotation()))
}

/// It starts on its tail, nose up: 90° of airplane pitch, level in the hover view, heading
/// north there.
#[test]
fn starts_resting_on_its_tail() {
    let mut tailsitter = tailsitter();
    let state = tailsitter.state(Duration::ZERO);
    check!(state.position_m == Vec3::zeros());
    assert_relative_eq!(state.pitch_rad, FRAC_PI_2, epsilon = 1e-3);
    let (roll_rad, pitch_rad, heading_rad) = view_angles(&state);
    check!(roll_rad.abs() < 1e-5);
    check!(pitch_rad.abs() < 1e-5);
    check!(heading_rad.abs() < 1e-5);
    check!(tailsitter.on_ground());
    check!(tailsitter.time_s() == 0.0);
}

/// Thrust short of the weight leaves it on its tail; more climbs nose first (the cambered wing's
/// lift at zero α pushes it belly-ward a little on the way).
#[test]
fn climbs_nose_first_on_enough_thrust() {
    let mut resting = tailsitter();
    let state = fly(&mut resting, &throttle(0.5), 250);
    check!(resting.on_ground());
    check!(state.position_m.norm() < 1e-6);
    assert_relative_eq!(state.pitch_rad, FRAC_PI_2, epsilon = 1e-3);

    let mut climbing = tailsitter();
    let state = fly(&mut climbing, &throttle(1.0), 250);
    check!(!climbing.on_ground());
    check!(state.position_m.z > 1.0);
    check!(state.velocity_mps.z > 0.0);
}

/// Landing on its tail at an angle settles it back nose up, with its heading in the view
/// kept (but for the 10 ms it turns before touching down).
#[test]
fn lands_back_on_its_tail() {
    let mut tailsitter = tailsitter();
    let tilted = UnitQuaternion::from_euler_angles(0.0, 1.3, 0.4);
    tailsitter.lockstep.place(&Start {
        position_m: Vec3::new(0.0, 0.0, 0.01),
        velocity_mps: Vec3::new(0.0, 0.0, -1.0),
        attitude: tilted,
        angular_velocity_radps: Vec3::new(0.2, 0.0, 0.0),
    });
    let heading_rad = view_angles(&tailsitter.state(Duration::ZERO)).2;
    let state = fly(&mut tailsitter, &throttle(0.0), 25);
    check!(tailsitter.on_ground());
    let (roll_rad, pitch_rad, landed_heading_rad) = view_angles(&state);
    check!(roll_rad.abs() < 1e-4);
    check!(pitch_rad.abs() < 1e-4);
    assert_relative_eq!(landed_heading_rad, heading_rad, epsilon = 5e-3);
    check!(state.angular_velocity_radps == Vec3::zeros());
}

#[test]
fn rejects_an_invalid_model_or_perturbations() {
    let airframe = airframes::tailsitter();
    let model = TailsitterModel {
        motor_arm_m: 0.0,
        ..airframe.model
    };
    check!(
        SimTailsitter::new(model, &airframe.mixer).err()
            == Some(ModelError::NotPositive("motor_arm_m"))
    );
    let delay = Perturbations {
        actuator_delay_s: 1.0,
        ..Default::default()
    };
    check!(tailsitter().with_perturbations(delay).err() == Some(ModelError::DelayTooLong));
}
