//! Reference: https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX
//!
//! Flies a quadcopter in RealFlight from a Logitech Extreme 3D joystick, using
//! hand-tuned stabilizer gains.
//!
//! The RealFlight host defaults to `10.211.55.3:8080`; override it with `--host`.
//!
//! ```text
//! cargo run --example copter-manual-realflight
//! cargo run --example copter-manual-realflight -- --host 192.168.1.20:8080
//! ```

use std::f32::consts::PI;

use clap::Parser;
use env_logger::Builder;

use realflight_bridge::RealFlightRemoteBridge;
use robomath::{vec2, vec3};
use stable_flight::mixers::QuadCopterMixer;
use stable_flight::stabilizers::Gains;
use stable_flight::stabilizers::controllers::*;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight_sitl::FlightOrchestrator;
use stable_flight_sitl::commanders::extreme_3d::Extreme3DCommander;
use stable_flight_sitl::simulators::RealFlightVehicle;
use stable_flight_sitl::simulators::realflight::{ActuatorMapping, RealFlightConfiguration};
use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};

/// Joystick-driven quadcopter flight in RealFlight.
#[derive(Parser)]
struct Args {
    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,
}

fn main() {
    // Initialize logger
    Builder::from_default_env().format_timestamp_millis().init();

    let args = Args::parse();

    // Joystick commander drives the vehicle attitude/rate directly.
    let commander = Extreme3DCommander::default();

    // Configure RealFlight vehicle with network connection and actuator mapping.
    // This mapping connects to a QuadCopterX model in RealFlight.
    // Motors are mapped to the first 4 channels (0-3) matching the RealFlight QuadCopterX model.
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge: RealFlightRemoteBridge::new(&args.host).expect("Failed to create bridge"),
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
            p: 3.0,
            i: 0.0,
            d: 0.1,
        },
        max_integral: 100.0,
        max_angles: vec2(PI / 4.0, PI / 4.0), // 45° for roll and pitch
        max_rate: vec2(3.0, 3.0),             // Max roll, pitch, yaw rates in rad/s
    });

    let stabilizer = CopterStabilizer::new(CopterStabilizerConfig {
        rates_controller,
        attitude_controller,
        ..Default::default()
    });

    let mixer = QuadCopterMixer::default();

    let telemetry = MavlinkPublisher::new(MavlinkConfiguration {});

    let mut orchestrator =
        FlightOrchestrator::new(commander, vehicle, stabilizer, mixer, telemetry);

    let target_frequency = 250.0; // Hz
    orchestrator.run(target_frequency);
}
