use stable_flight_sitl::sticks::SpectrumStick;
use stable_flight_sitl::{
    FlightStabilizer, RateController, RealFlightVehicle,
    StabilizerConfiguration,
};

fn main() {
    let receiver = &SpectrumStick::new();
    let vehicle = &RealFlightVehicle {};
    let flight_controller = &RateController {};

    let configuration = StabilizerConfiguration {
        target_frequency: 2.0,
        receiver,
        vehicle,
        controller: flight_controller,
    };

    let stabilizer = FlightStabilizer::new(configuration);
    stabilizer.stabilize();
}
