use stable_flight_sitl::{
    FlightStabilizer, RateController, RealFlightActuators, RealFlightVehicle, SpectrumRadioControl,
    StabilizerConfiguration,
};

fn main() {
    let receiver = &SpectrumRadioControl {};
    let vehicle = &RealFlightVehicle {};
    let flight_controller = &RateController {};
    let actuators = &RealFlightActuators {};

    let configuration = StabilizerConfiguration {
        target_frequency: 2.0,
        receiver,
        vehicle,
        controller: flight_controller,
        actuators,
    };

    let stabilizer = FlightStabilizer::new(configuration);
    stabilizer.stabilize();
}
