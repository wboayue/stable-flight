//! Flies a scripted figure-eight in RealFlight using a [`ScriptCommander`].
//!
//! The commander replays the waypoints in a CSV script (default `flights/figure_eight.csv`,
//! regenerate it with `cargo run --features tools --bin generate_figure_eight`), looping the
//! path until interrupted.
//!
//! The script path is the first argument (default `flights/figure_eight.csv`); the RealFlight
//! host defaults to `10.211.55.3:8080` via `--host`.
//!
//! ```text
//! cargo run --example copter-auto-realflight
//! cargo run --example copter-auto-realflight -- flights/wide.csv --host 192.168.1.20:8080
//! ```

use clap::Parser;
use env_logger::Builder;

use realflight_bridge::RealFlightRemoteBridge;
use stable_flight::mixers::QuadCopterMixer;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight_sitl::FlightOrchestrator;
use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::simulators::RealFlightVehicle;
use stable_flight_sitl::simulators::realflight::{ActuatorMapping, RealFlightConfiguration};
use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};

/// Scripted figure-eight flight in RealFlight.
#[derive(Parser)]
struct Args {
    /// Waypoint CSV to fly (columns: x,y,altitude,cruise_speed).
    #[arg(default_value = "flights/figure_eight.csv")]
    script: String,

    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,
}

fn main() {
    // Initialize logger
    Builder::from_default_env().format_timestamp_millis().init();

    let args = Args::parse();

    // Replays and loops the waypoints from the CSV script.
    let commander = ScriptCommander::new(&args.script).expect("failed to load flight script");

    // Configure RealFlight vehicle with network connection and actuator mapping.
    // This mapping connects to a QuadCopterX model in RealFlight.
    // Motors are mapped to the first 4 channels (0-3) matching the RealFlight QuadCopterX model.
    // Reference: https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge: RealFlightRemoteBridge::new(&args.host).expect("Failed to create bridge"),
        mapping: ActuatorMapping {
            motors: [Some(2), Some(0), Some(1), Some(3), None, None, None, None],
            ..Default::default()
        },
    });

    // Cascaded PID stabilizer for multirotors with default gains.
    let stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());

    let mixer = QuadCopterMixer::default();

    let telemetry = MavlinkPublisher::new(MavlinkConfiguration {});

    let mut orchestrator =
        FlightOrchestrator::new(commander, vehicle, stabilizer, mixer, telemetry);

    let target_frequency = 250.0; // Hz
    orchestrator.run(target_frequency);
}
