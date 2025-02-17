mod controllers;
pub mod sticks;

pub use controllers::{FlightStabilizer, StabilizerConfiguration};

pub trait Receiver {
    fn get_controls(&self) -> ControlInput;
}

#[derive(Debug)]
pub struct ControlInput {}

#[derive(Debug)]
pub struct ControlOutput {}

pub struct VehicleState {
    // flight mode
}

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
    fn compute_desired_state(&self, controls: &ControlInput, state: &VehicleState) -> VehicleState;
    fn compute_control_outputs(
        &self,
        desired_state: &VehicleState,
        state: &VehicleState,
    ) -> ControlOutput;
}

pub struct RateController {}

impl Controller for RateController {
    fn compute_desired_state(&self, controls: &ControlInput, state: &VehicleState) -> VehicleState {
        VehicleState {}
    }

    fn compute_control_outputs(
        &self,
        desired_state: &VehicleState,
        state: &VehicleState,
    ) -> ControlOutput {
        ControlOutput {}
    }
}

pub struct AngleController {}

pub trait Actuators {
    fn command_actuators(&self, controls: &ControlOutput);
}

pub struct RealFlightActuators {}

impl Actuators for RealFlightActuators {
    fn command_actuators(&self, controls: &ControlOutput) {
        println!("Commanding actuators: {:?}", controls);
    }
}
