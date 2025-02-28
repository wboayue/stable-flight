use stable_flight_sitl::commanders::script::ScriptCommander;
use stable_flight_sitl::realflight::RealFlightVehicle;
use stable_flight_sitl::{
    ConsoleTelemetryPublisher, FlightStabilizer, RateController, StabilizerConfiguration,
};

fn main() {
    let commander = ScriptCommander::new("foo");
    let vehicle = RealFlightVehicle::new();
    let controller = RateController {};
    let telemetry = ConsoleTelemetryPublisher {};

    let configuration = StabilizerConfiguration {
        target_frequency: 2.0,
        commander,
        vehicle,
        controller,
        telemetry,
    };

    let stabilizer = FlightStabilizer::new(configuration);
    stabilizer.run();
}
