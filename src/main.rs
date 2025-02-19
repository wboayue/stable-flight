use stable_flight_sitl::realflight::RealFlightVehicle;
use stable_flight_sitl::sticks::SpectrumStick;
use stable_flight_sitl::{FlightStabilizer, RateController, ConsoleTelemetryPublisher, StabilizerConfiguration};

fn main() {
    let stick = &SpectrumStick::new();
    let vehicle = &RealFlightVehicle::new();
    let controller = &RateController {};
    let telemetry = &ConsoleTelemetryPublisher {};

    let configuration = StabilizerConfiguration {
        target_frequency: 2.0,
        stick,
        vehicle,
        controller,
        telemetry
    };

    let stabilizer = FlightStabilizer::new(configuration);
    stabilizer.run();
}
