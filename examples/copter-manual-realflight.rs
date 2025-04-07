//! Reference: https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX
//!
//! This example demonstrates how to control a quadcopter in RealFlight using the Stable Flight library.
//! It uses the `Extreme3DCommander` to control the vehicle and the `RealFlightVehicle` to interface with RealFlight.
//! The vehicle is configured to use a `QuadCopterMixer` and a `CopterStabilizer` for flight control.
//! The `MavlinkPublisher` is used for telemetry.
//!
//! The example runs a flight orchestrator that continuously updates the vehicle state and applies control commands.
//!
//! The target frequency for the control loop is set to 250 Hz.

use std::f32::consts::PI;

use env_logger::Builder;

use realflight_bridge::RealFlightRemoteBridge;

use robomath::{vec2, vec3};
use stable_flight_sitl::commanders::extreme_3d::Extreme3DCommander;
use stable_flight_sitl::mixers::QuadCopterMixer;
use stable_flight_sitl::simulators::realflight::{ActuatorMapping, RealFlightConfiguration};
use stable_flight_sitl::simulators::RealFlightVehicle;
use stable_flight_sitl::stabilizers::controllers::*;
use stable_flight_sitl::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight_sitl::stabilizers::Gains;
use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};
use stable_flight_sitl::*;

fn main() {
    // Initialize logger
    Builder::from_default_env().format_timestamp_millis().init();

    // Initializes joystick commander to control the vehicle
    let commander = Extreme3DCommander::new();

    // Configure RealFlight vehicle with network connection and actuator mapping.
    // This mapping connects to a QuadCopterX model in RealFlight.
    // Motors are mapped to the first 4 channels (0-3) matching the RealFlight QuadCopterX model
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge: RealFlightRemoteBridge::new("10.211.55.3:8080").expect("Failed to create bridge"),
        mapping: ActuatorMapping {
            motors: [Some(2), Some(0), Some(1), Some(3), None, None, None, None],
            ..Default::default()
        },
    });

    let rates_controller = BodyRateController::with_config(&BodyRateControllerConfig {
        gains: Gains {
            p: 7.0,
            i: 0.00,
            d: 0.01,
        },
        max_integral: 100.0,
        max_angular_rates: vec3(3.0, 3.0, 3.0), // Max roll, pitch, yaw rates in rad/s
    });

    let attitude_controller = AttitudeController::with_config(&AttitudeControllerConfig {
        gains: Gains {
            p: 2.0,
            i: 0.2,
            d: 0.1,
        },
        max_integral: 100.0,
        max_angles: vec2(PI / 4.0, PI / 4.0), // 45° for roll and pitch
        max_rate: vec2(3.0, 3.0),             // Max roll, pitch, yaw rates in rad/s
    });

    let stabilizer_config = CopterStabilizerConfig {
        rates_controller,
        ..Default::default()
    };
    let stabilizer = CopterStabilizer::new(stabilizer_config);

    let mixer = QuadCopterMixer::default();

    let telemetry_configuration = MavlinkConfiguration {};
    let telemetry = MavlinkPublisher::new(telemetry_configuration);

    let mut orchestrator =
        FlightOrchestrator::new(commander, vehicle, stabilizer, mixer, telemetry);

    let target_frequency = 250.0; // Hz
    orchestrator.run(target_frequency);
}
