mod controllers;
pub mod realflight;
mod stabilizer;
pub mod sticks;

use std::time::Duration;

pub use controllers::{AngleController, RateController};
pub use stabilizer::{FlightStabilizer, StabilizerConfiguration};

pub trait Stick {
    fn get_controls(&self) -> StickInput;
}

#[derive(Debug)]
pub struct StickInput {
    pub channels: [f32; 12],
}

#[derive(Debug)]
pub struct ControlOutput {}

pub struct VehicleState {
    // flight mode
}

pub trait Vehicle {
    fn get_state(&self, dt: Duration) -> VehicleState;
    fn command_actuators(&self, dt: Duration, stick: &StickInput, controls: &ControlOutput);
}

pub trait Controller {
    fn compute_desired_state(&self, controls: &StickInput, state: &VehicleState) -> VehicleState;
    fn compute_control_outputs(
        &self,
        desired_state: &VehicleState,
        state: &VehicleState,
    ) -> ControlOutput;
}
