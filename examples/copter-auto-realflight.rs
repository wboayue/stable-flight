//! Flies a scripted figure-eight in RealFlight using a [`ScriptCommander`].
//!
//! The commander replays the waypoints in `flights/figure_eight.csv` (regenerate them
//! with `cargo run --example generate_figure_eight`), looping the path until interrupted.
//!
//! ```text
//! cargo run --example copter-auto-realflight
//! ```

use env_logger::Builder;

use realflight_bridge::RealFlightRemoteBridge;
use stable_flight_sitl::FlightOrchestrator;
use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::mixers::QuadCopterMixer;
use stable_flight_sitl::simulators::RealFlightVehicle;
use stable_flight_sitl::simulators::realflight::{ActuatorMapping, RealFlightConfiguration};
use stable_flight_sitl::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};

fn main() {
    // Initialize logger
    Builder::from_default_env().format_timestamp_millis().init();

    // Replays and loops the figure-eight waypoints from the CSV script.
    let commander = ScriptCommander::new("flights/figure_eight.csv")
        .expect("failed to load figure-eight script");

    // Configure RealFlight vehicle with network connection and actuator mapping.
    // This mapping connects to a QuadCopterX model in RealFlight.
    // Motors are mapped to the first 4 channels (0-3) matching the RealFlight QuadCopterX model.
    // Reference: https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge: RealFlightRemoteBridge::new("10.211.55.3:8080").expect("Failed to create bridge"),
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
