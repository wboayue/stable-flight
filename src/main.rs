use stable_flight_sitl::sticks::SpectrumStick;
use stable_flight_sitl::{
    FlightStabilizer, RateController, RealFlightVehicle,
    StabilizerConfiguration,
};

fn main() {
    let stick = &SpectrumStick::new();
    let vehicle = &RealFlightVehicle {};
    let controller = &RateController {};

    let configuration = StabilizerConfiguration {
        target_frequency: 2.0,
        stick,
        vehicle,
        controller,
    };

    let stabilizer = FlightStabilizer::new(configuration);
    stabilizer.stabilize();
}
