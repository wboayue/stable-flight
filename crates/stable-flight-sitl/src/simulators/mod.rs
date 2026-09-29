//!  Defines components for interacting with a physical aircraft, whether it’s a real vehicle or a simulated one.
//!
pub mod realflight;

use std::time::Duration;

pub use realflight::RealFlightVehicle;

use crate::{Actuator, ActuatorCommand, StateEstimator, VehicleState};

pub struct NullVehicle {}

impl StateEstimator for NullVehicle {
    fn estimate_state(&mut self, _dt: Duration) -> VehicleState {
        VehicleState::default()
    }
}

impl Actuator for NullVehicle {
    fn actuate(&mut self, _command: &ActuatorCommand) {}
}
