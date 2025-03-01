use stable_flight_sitl::commanders::script::ScriptCommander;
use stable_flight_sitl::mixers::QuadCopterMixer;
use stable_flight_sitl::realflight::RealFlightVehicle;
use stable_flight_sitl::{
    ConsoleTelemetryPublisher, FlightStabilizer, RateController,
};

fn main() {
    let commander = ScriptCommander::new("foo");
    let vehicle = RealFlightVehicle::new();
    let controller = RateController {};
    let mixer = QuadCopterMixer {};
    let telemetry = ConsoleTelemetryPublisher {};

    let target_frequency=  2.0;

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
