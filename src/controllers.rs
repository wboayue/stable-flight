use super::{ControlOutput, Controller, StickInput, VehicleState};

pub struct RateController {}

impl Controller for RateController {
    fn compute_desired_state(&self, _controls: &StickInput, _state: &VehicleState) -> VehicleState {
        VehicleState {}
    }

    fn compute_control_outputs(
        &self,
        _desired_state: &VehicleState,
        _state: &VehicleState,
    ) -> ControlOutput {
        ControlOutput {}
    }
}

pub struct AngleController {}
