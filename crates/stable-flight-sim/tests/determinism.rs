//! The same flight twice gives the same states, bit for bit, with or without perturbations.

mod common;

use assert2::check;
use stable_flight::Vec3;
use stable_flight::commanders::{RouteEnd, WaypointFollower};
use stable_flight::testing::RecordingTelemetry;
use stable_flight_sim::airframes;
use stable_flight_sim::{Gust, GyroNoise, Perturbations, SimVehicle, Vibration, Wind};

use common::{fly, pipeline_on, waypoint};

/// A windy route with a gust under `perturbations`; every tick's state.
fn flight(perturbations: Perturbations) -> Vec<stable_flight::VehicleState> {
    let route = [waypoint(0.0, 0.0, 4.0), waypoint(10.0, 5.0, 6.0)];
    let follower = WaypointFollower::new(route)
        .expect("waypoints")
        .with_route_end(RouteEnd::Land);
    let wind = Wind {
        steady_mps: Vec3::new(-2.0, 1.0, 0.0),
        gust: Some(Gust {
            start_s: 6.0,
            duration_s: 1.5,
            peak_mps: Vec3::new(0.0, 4.0, -1.0),
        }),
    };
    let airframe = airframes::quad_x();
    let vehicle = SimVehicle::new(airframe.model, &airframe.mixer)
        .expect("valid airframe")
        .with_wind(wind)
        .with_perturbations(perturbations)
        .expect("valid perturbations");
    let telemetry = RecordingTelemetry::<{ common::RECORDED }>::new();
    let mut flight_loop = pipeline_on(airframe, vehicle, follower, telemetry);
    let mut states = Vec::new();
    fly(&mut flight_loop, 20.0, |flight_loop, _| {
        states.push(flight_loop.state().clone());
    });
    states
}

/// Every perturbation on, mild enough that the route still flies, noise seeded by `seed`.
fn all_on(seed: u64) -> Perturbations {
    let mut motor_thrust_scale = [1.0; 8];
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
