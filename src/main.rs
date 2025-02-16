use stable_flight_sitl::{RadioControl, SpectrumRadioControl, Vehicle, Controller, RealFlightVehicle, RateController};


fn stabilize(controller: &dyn RadioControl, vehicle: &dyn Vehicle, flight_controller: &dyn Controller) {

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

}

fn main() {
    println!("Hello, world!");

    let controller = &SpectrumRadioControl {};
    let vehicle = &RealFlightVehicle {};
    let flight_controller = &RateController {};

    stabilize(controller, vehicle, flight_controller);
}
