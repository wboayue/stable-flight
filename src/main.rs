use env_logger::Builder;
use log::{debug, error, info, warn};

use stable_flight_sitl::commanders::extreme_3d::Extreme3DCommander;
use stable_flight_sitl::commanders::script::ScriptCommander;
use stable_flight_sitl::controllers::basic::BasicController;
use stable_flight_sitl::mixers::QuadCopterMixer;
use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};
use stable_flight_sitl::vehicles::realflight::{ActuatorMapping, RealFlightConfiguration};
use stable_flight_sitl::vehicles::{NullVehicle, RealFlightVehicle};
use stable_flight_sitl::FlightStabilizer;

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
        host: "10.211.55.3:8080".to_string(),
        mapping: ActuatorMapping {
            motors: [Some(0), Some(1), Some(2), Some(3), None, None, None, None],
            ..Default::default()
        },
    });

    // let vehicle = NullVehicle {};

    let controller = BasicController::default();

    let mixer = QuadCopterMixer::default();

    let telemetry_configuration = MavlinkConfiguration {};

    let telemetry = MavlinkPublisher::new(telemetry_configuration);

    let target_frequency = 250.0;

    let mut stabilizer = FlightStabilizer::new(
        target_frequency,
        commander,
        vehicle,
        controller,
        mixer,
        telemetry,
    );

    stabilizer.run();
}
