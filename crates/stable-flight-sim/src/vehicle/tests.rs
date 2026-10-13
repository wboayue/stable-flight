use approx::assert_relative_eq;
use assert2::check;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::testing::conformance::Scenario;
use stable_flight::{GRAVITY, Vec3};

use super::*;
use crate::airframes;

const TICK: Duration = Duration::from_millis(4);

fn vehicle() -> SimVehicle {
    let airframe = airframes::quad_x();
    SimVehicle::new(airframe.model, &airframe.mixer).expect("valid airframe")
}

fn motors(command: f32) -> ActuatorCommand {
    ActuatorCommand {
        motors: [command; ActuatorCommand::SLOTS],
        ..Default::default()
    }
}

/// Runs `ticks` loop ticks: read state, then actuate `command`, as `FlightLoop::step` does.
fn fly(vehicle: &mut SimVehicle, command: &ActuatorCommand, ticks: usize) -> VehicleState {
    let mut state = vehicle.state(Duration::ZERO);
    for _ in 0..ticks {
        state = vehicle.state(TICK);
        vehicle.actuate(command);
    }
    state
}

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

#[test]
fn starts_at_rest_on_the_ground() {
    let mut vehicle = vehicle();
    let state = vehicle.state(Duration::ZERO);
    check!(state == vehicle_state(&Body::default(), Vec3::zeros()));
    check!(vehicle.on_ground());
    check!(vehicle.time_s() == 0.0);
}

#[test]
fn priming_call_does_not_step() {
    let mut vehicle = vehicle();
    vehicle.actuate(&motors(1.0));
    vehicle.state(Duration::ZERO);
    check!(vehicle.time_s() == 0.0);
    check!(vehicle.motor_thrusts_n() == [0.0; ActuatorCommand::SLOTS]);
}

#[test]
fn state_steps_the_last_actuated_command() {
    let mut vehicle = vehicle();
    vehicle.state(TICK);
    // Nothing commanded yet: the motors stay stopped.
    check!(vehicle.motor_thrusts_n()[0] == 0.0);
    vehicle.actuate(&motors(1.0));
    vehicle.state(TICK);
    check!(vehicle.motor_thrusts_n()[0] > 0.0);
    assert_relative_eq!(vehicle.time_s(), 0.008, epsilon = 1e-6);
}

#[test]
fn idle_stays_on_the_ground_at_rest() {
    let mut vehicle = vehicle();
    let state = fly(&mut vehicle, &motors(0.3), 250);
    check!(vehicle.on_ground());
    check!(state.position_m == Vec3::zeros());
    assert_relative_eq!(state.acceleration_mps2, Vec3::zeros(), epsilon = 1e-4);
}

#[test]
fn full_throttle_takes_off_and_climbs() {
    let mut vehicle = vehicle();
    let state = fly(&mut vehicle, &motors(1.0), 250);
    check!(!vehicle.on_ground());
    check!(state.position_m.z > 1.0);
    check!(state.velocity_mps.z > 0.0);
    check!(state.acceleration_mps2.z > 0.0);
}

#[test]
fn cutting_the_motors_falls_back_and_lands() {
    let mut vehicle = vehicle();
    fly(&mut vehicle, &motors(1.0), 125);
    // Coasting up after the cut: gravity plus drag against the climb, once the props spin down.
    let coasting = fly(&mut vehicle, &motors(0.0), 50);
    check!(coasting.velocity_mps.z > 0.0);
    check!(coasting.acceleration_mps2.z < -GRAVITY);
    let landed = fly(&mut vehicle, &motors(0.0), 2000);
    check!(vehicle.on_ground());
    check!(landed.position_m.z == 0.0);
    check!(landed.velocity_mps == Vec3::zeros());
}

/// A tick longer than the max step is split into equal steps no longer than it, so a slower
/// loop still integrates finely: a 10 ms tick at 1 ms steps matches ten 1 ms ticks.
#[test]
fn long_ticks_split_into_max_steps() {
    let model = MultirotorModel {
        max_step_s: 0.001,
        ..airframes::quad_x().model
    };
    let mixer = airframes::quad_x().mixer;
    let mut coarse = SimVehicle::new(model, &mixer).expect("valid airframe");
    let mut fine = coarse.clone();
    coarse.actuate(&motors(1.0));
    fine.actuate(&motors(1.0));
    let long = coarse.state(Duration::from_millis(10));
    let mut short = fine.state(Duration::ZERO);
    for _ in 0..10 {
        short = fine.state(Duration::from_millis(1));
    }
    assert_relative_eq!(long.position_m, short.position_m, epsilon = 1e-6);
    assert_relative_eq!(long.velocity_mps, short.velocity_mps, epsilon = 1e-5);
    assert_relative_eq!(coarse.time_s(), 0.010, epsilon = 1e-9);
}

#[test]
fn servos_are_ignored() {
    let mut vehicle = vehicle();
    let command = ActuatorCommand {
        servos: [1.0; ActuatorCommand::SLOTS],
        ..Default::default()
    };
    let state = fly(&mut vehicle, &command, 50);
    check!(vehicle.motor_thrusts_n() == [0.0; ActuatorCommand::SLOTS]);
    check!(state.position_m == Vec3::zeros());
}

#[test]
fn airspeed_is_against_the_wind() {
    let wind = Wind {
        steady_mps: Vec3::new(3.0, 4.0, 0.0),
        gust: None,
    };
    let mut vehicle = vehicle().with_wind(wind);
    // At rest on the ground in a 5 m/s wind.
    let state = vehicle.state(Duration::ZERO);
    assert_relative_eq!(state.airspeed_mps.unwrap_or(f32::NAN), 5.0, epsilon = 1e-5);
}

#[test]
fn wind_pushes_an_airborne_vehicle_downwind() {
    let wind = Wind {
        steady_mps: Vec3::new(0.0, 5.0, 0.0),
        gust: None,
    };
    let mut vehicle = vehicle().with_wind(wind);
    let state = fly(&mut vehicle, &motors(1.0), 500);
    check!(state.velocity_mps.y > 0.5);
    check!(state.position_m.x.abs() < 1e-3);
}

#[test]
fn rejects_an_invalid_model_or_a_servo_mixer() {
    let airframe = airframes::quad_x();
    let model = MultirotorModel {
        max_step_s: 0.0,
        ..airframe.model
    };
    check!(
        SimVehicle::new(model, &airframe.mixer).err()
            == Some(ModelError::NotPositive("max_step_s"))
    );
    check!(SimVehicle::new(airframe.model, &TableMixer::tri()).err() == Some(ModelError::Servo(0)));
}

#[test]
fn same_commands_give_the_same_state() {
    let run = || {
        let mut vehicle = vehicle().with_wind(Wind {
            steady_mps: Vec3::new(1.0, -2.0, 0.0),
            gust: None,
        });
        let mut command = motors(0.7);
        command.motors[0] = 0.75;
        fly(&mut vehicle, &command, 500)
    };
    check!(run() == run());
}
