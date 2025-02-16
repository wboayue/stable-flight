use stable_flight_sitl::{Actuators, Controller, Receiver, RateController, RealFlightVehicle, SpectrumRadioControl, Vehicle, RealFlightActuators};


fn stabilize(controller: &dyn Receiver, vehicle: &dyn Vehicle, flight_controller: &dyn Controller, actuators: &dyn Actuators) {
    loop {
        // Get control inputs
        let controls = controller.get_controls();
        println!("Controls: {:?}", controls);

        // Get vehicle state
        let state = vehicle.get_state();

        // Compute desired state with inputs
        let desired_state = flight_controller.compute_desired_state(&controls, &state);
        
        // Compute control outputs (angles, rates, etc.)
        let control_outputs = flight_controller.compute_control_outputs(&desired_state, &state);

        // Command actuators 
        actuators.command_actuators(&control_outputs);
    }
}

fn main() {
    println!("Hello, world!");

    let controller = &SpectrumRadioControl {};
    let vehicle = &RealFlightVehicle {};
    let flight_controller = &RateController {};
    let actuators = &RealFlightActuators {};

    stabilize(controller, vehicle, flight_controller, actuators);
}
