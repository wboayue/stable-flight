use stable_flight_sitl::commanders::script::ScriptCommander;
use stable_flight_sitl::controllers::basic::BasicController;
use stable_flight_sitl::mixers::QuadCopterMixer;
use stable_flight_sitl::vehicles::realflight::RealFlightConfiguration;
use stable_flight_sitl::vehicles::RealFlightVehicle;
use stable_flight_sitl::{ConsoleTelemetryPublisher, FlightStabilizer};

fn main() {
    let commander = ScriptCommander::new("figure_eight.csv");

    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        host: "".to_string(),
        mapping: Default::default(),
    });

    let controller = BasicController {};

    let mixer = QuadCopterMixer {};

    let telemetry = ConsoleTelemetryPublisher {};

    let target_frequency = 2.0;

    let stabilizer = FlightStabilizer::new(
        target_frequency,
        commander,
        vehicle,
        controller,
        mixer,
        telemetry,
    );

    stabilizer.run();
}
