use crate::commanders::Command;

use super::{ControlOutput, Controller, StickInput, VehicleState};

pub mod quad;
pub mod tailsitter;

pub struct RateController {}

impl Controller for RateController {
    fn control(&self, _desired_state: &Command, _state: &VehicleState) -> ControlOutput {
        ControlOutput {}
    }
}

pub struct AngleController {}
