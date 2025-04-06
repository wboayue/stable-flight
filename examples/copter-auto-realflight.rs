use env_logger::Builder;
use log::{debug, error, info, warn};

use realflight_bridge::RealFlightRemoteBridge;
use stable_flight_sitl::commanders::extreme_3d::Extreme3DCommander;
use stable_flight_sitl::mixers::QuadCopterMixer;
use stable_flight_sitl::simulators::realflight::{ActuatorMapping, RealFlightConfiguration};
use stable_flight_sitl::simulators::RealFlightVehicle;
use stable_flight_sitl::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};
use stable_flight_sitl::FlightOrchestrator;

fn main() {
    // Initialize logger
    Builder::from_default_env().format_timestamp_millis().init();

    // Initializes a ScriptCommander to replay a pre-recorded flight trajectory
    // The "figure_eight.csv" file contains waypoints that define a figure-eight pattern
    //
    // The Commander trait's polymorphic design allows easy swapping of command sources:
    // - ScriptCommander: Replays pre-recorded CSV waypoint sequences (used here)
    // - SpektrumCommander: Maps physical controller inputs from Spektrum receiver
    // let commander = ScriptCommander::new("figure_eight.csv");
    let commander = Extreme3DCommander::new();

    // Configure RealFlight vehicle with network connection and actuator mapping.
    // This mapping connects to a QuadCopterX model in RealFlight.
    // Motors are mapped to the first 4 channels (0-3) matching the RealFlight QuadCopterX model
    // Reference: https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge: RealFlightRemoteBridge::new("10.211.55.3:8080").expect("Failed to create bridge"),
        mapping: ActuatorMapping {
            motors: [Some(2), Some(0), Some(1), Some(3), None, None, None, None],
            ..Default::default()
        },
    });

    println!("desired_yaw,estimated_yaw");

    // let vehicle = NullVehicle {};

    let controller_config = CopterStabilizerConfig::default();
    let controller = CopterStabilizer::new(controller_config);

    let mixer = QuadCopterMixer::default();

    let telemetry_configuration = MavlinkConfiguration {};

    let telemetry = MavlinkPublisher::new(telemetry_configuration);

    let mut stabilizer = FlightOrchestrator::new(commander, vehicle, controller, mixer, telemetry);

    let target_frequency = 250.0; // Hz
    stabilizer.run(target_frequency);
}
