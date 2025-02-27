use super::{ControlOutput, Controller, StickInput, VehicleState};

pub mod quad;
pub mod tailsitter;

pub struct RateController {}

impl Controller for RateController {
    fn compute_desired_state(&self, _controls: &StickInput, _state: &VehicleState) -> VehicleState {
        VehicleState::default()
    }

    fn control(
        &self,
        _desired_state: &VehicleState,
        _state: &VehicleState,
    ) -> ControlOutput {
        ControlOutput {}
    }
}

pub struct AngleController {}
