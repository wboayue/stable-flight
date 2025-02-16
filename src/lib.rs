pub trait Receiver {
    fn get_controls(&self) -> Controls;
}

pub struct SpectrumRadioControl {}

impl Receiver for SpectrumRadioControl {
    fn get_controls(&self) -> Controls {
        Controls {}
    }
}

#[derive(Debug)]
pub struct Controls {}

#[derive(Debug)]
pub struct ControlOutput {}

pub struct VehicleState {}

pub trait Vehicle {
    fn get_state(&self) -> VehicleState;
}

pub struct RealFlightVehicle {} 

impl Vehicle for RealFlightVehicle {
    fn get_state(&self) -> VehicleState {
        VehicleState {}
    }
}

pub trait Controller {
    fn compute_desired_state(&self, controls: &Controls, state: &VehicleState) -> VehicleState;
    fn compute_control_outputs(&self, desired_state: &VehicleState, state: &VehicleState) -> Controls;
}

pub struct RateController {}

impl Controller for RateController {
    fn compute_desired_state(&self, controls: &Controls, state: &VehicleState) -> VehicleState {
        VehicleState {}
    }

    fn compute_control_outputs(&self, desired_state: &VehicleState, state: &VehicleState) -> Controls {
        Controls {}
    }
}

pub struct AngleController {}

pub trait Actuators {
    fn command_actuators(&self, controls: &Controls);
}

pub struct RealFlightActuators {}

impl Actuators for RealFlightActuators {
    fn command_actuators(&self, controls: &Controls) {
        println!("Commanding actuators: {:?}", controls);
    }
    
}