//!  Defines components for interacting with a physical aircraft, whether it’s a real vehicle or a simulated one.
//!
pub mod realflight;

use std::time::Duration;

pub use realflight::RealFlightVehicle;

use crate::{ActuatorCommand, Vehicle, VehicleState};

pub struct NullVehicle {}

impl Vehicle for NullVehicle {
    fn estimate_state(&self, _dt: Duration) -> VehicleState {
        VehicleState::default()
    }

    fn actuate(&mut self, _command: &ActuatorCommand) {}
}
