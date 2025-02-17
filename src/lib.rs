mod stabilizer;
pub mod sticks;
mod controllers;
mod realflight;

pub use stabilizer::{FlightStabilizer, StabilizerConfiguration};
pub use controllers::{RateController, AngleController};

pub trait Stick {
    fn get_controls(&self) -> StickInput;
}

#[derive(Debug)]
pub struct StickInput {
    pub channels: [f64; 12],
}

#[derive(Debug)]
pub struct ControlOutput {}

pub struct VehicleState {
    // flight mode
}

pub trait Vehicle {
    fn get_state(&self) -> VehicleState;
    fn command_actuators(&self, controls: &ControlOutput);
}

pub struct RealFlightVehicle {}

impl Vehicle for RealFlightVehicle {
    fn get_state(&self) -> VehicleState {
        VehicleState {}
    }

    fn command_actuators(&self, controls: &ControlOutput) {
        println!("Commanding actuators: {:?}", controls);
    }
}

pub trait Controller {
    fn compute_desired_state(&self, controls: &StickInput, state: &VehicleState) -> VehicleState;
    fn compute_control_outputs(
        &self,
        desired_state: &VehicleState,
        state: &VehicleState,
    ) -> ControlOutput;
}
