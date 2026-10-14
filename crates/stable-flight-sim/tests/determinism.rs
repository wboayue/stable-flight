//! The same flight twice gives the same states, bit for bit, with or without perturbations, on
//! the copter, the airplane, and the tailsitter.

mod common;

use assert2::check;
use stable_flight::Vec3;
use stable_flight::commanders::{RouteEnd, WaypointFollower};
use stable_flight_sim::airframes;
use stable_flight_sim::{Gust, GyroNoise, Perturbations, Vibration, Wind};

use common::plane::{PlanePlant, plane_loop};
use common::tailsitter::{TailsitterPlant, forward_and_back, tailsitter_loop};
use common::{Plant, fly, sim_loop_with, waypoint};

/// A steady wind with a gust at 6 s.
fn gusty() -> Wind {
    Wind {
        steady_mps: Vec3::new(-2.0, 1.0, 0.0),
        gust: Some(Gust {
            start_s: 6.0,
            duration_s: 1.5,
            peak_mps: Vec3::new(0.0, 4.0, -1.0),
        }),
    }
}

/// A windy route with a gust under `perturbations`; every tick's state.
fn flight(perturbations: Perturbations) -> Vec<stable_flight::VehicleState> {
    let route = [waypoint(0.0, 0.0, 4.0), waypoint(10.0, 5.0, 6.0)];
    let follower = WaypointFollower::new(route)
        .expect("waypoints")
        .with_route_end(RouteEnd::Land);
    let plant = Plant {
        airframe: airframes::quad_x(),
        wind: gusty(),
        perturbations,
    };
    let mut flight_loop = sim_loop_with(&plant, follower);
    let mut states = Vec::new();
    fly(&mut flight_loop, 20.0, |flight_loop, _| {
        states.push(flight_loop.state().clone());
    });
    states
}

/// The airplane on its line through a crosswind and a gust; every tick's state.
fn plane_flight() -> Vec<stable_flight::VehicleState> {
    let plant = PlanePlant {
        wind: gusty(),
        ..PlanePlant::clean(airframes::plane())
    };
    let mut flight_loop = plane_loop(&plant, plant.line());
    let mut states = Vec::new();
    fly(&mut flight_loop, 20.0, |flight_loop, _| {
        states.push(flight_loop.state().clone());
    });
    states
}

/// The tailsitter taking off from its tail through a gusty wind, transitioning to plane
/// flight, and back to a hover; every tick's state.
fn tailsitter_flight() -> Vec<stable_flight::VehicleState> {
    let airframe = airframes::tailsitter();
    let plant = TailsitterPlant::in_wind(airframe, gusty());
    let mut flight_loop = tailsitter_loop(&plant, forward_and_back(&airframe, 30.0, 130.0));
    let mut states = Vec::new();
    fly(&mut flight_loop, 40.0, |flight_loop, _| {
        states.push(flight_loop.state().clone());
    });
    states
}

/// Every perturbation on, mild enough that the route still flies, noise seeded by `seed`.
fn all_on(seed: u64) -> Perturbations {
    let mut motor_thrust_scale = Perturbations::default().motor_thrust_scale;
    motor_thrust_scale[1] = 0.95;
    Perturbations {
        actuator_delay_s: 0.01,
        gyro: GyroNoise {
            rms_radps: Vec3::repeat(0.01),
            vibration: Some(Vibration {
                amplitude_radps: Vec3::repeat(0.02),
                frequency_hz: 90.0,
            }),
            seed,
        },
        disturbance_moment_nm: Vec3::new(0.005, -0.005, 0.0),
        motor_thrust_scale,
    }
}

#[test]
fn repeats_bit_for_bit() {
    let first = flight(Perturbations::default());
    check!(first.len() == 5000);
    check!(first == flight(Perturbations::default()));
}

#[test]
fn repeats_bit_for_bit_with_perturbations() {
    let first = flight(all_on(7));
    check!(first.len() == 5000);
    check!(first == flight(all_on(7)));
    check!(first != flight(all_on(8)));
}

#[test]
fn plane_repeats_bit_for_bit() {
    let first = plane_flight();
    check!(first.len() == 5000);
    check!(first == plane_flight());
}

#[test]
fn tailsitter_repeats_bit_for_bit() {
    let first = tailsitter_flight();
    check!(first.len() == 10_000);
    check!(first == tailsitter_flight());
}
