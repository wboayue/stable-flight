//! Placeholder pipeline components until real sensors, ESCs, and a transmitter are wired in
//! (#88). They let the full `FlightLoop` run on target so its timing and flash cost are real:
//! the estimate and motor commands pass through [`black_box`], so the optimizer can't fold the
//! pipeline away.

use core::hint::black_box;
use core::time::Duration;

use stable_flight::{
    Actuator, ActuatorCommand, AttitudeCommand, Command, Commander, StateEstimator, VehicleState,
};

/// Always commands zero throttle, level, disarmed. Replaced by a receiver-driven commander.
pub struct HoldCommander;

impl Commander for HoldCommander {
    fn next_command(&mut self, _dt: f32, _current_state: &VehicleState) -> Command {
        Command::Attitude(AttitudeCommand::default())
    }
}

/// Reports the vehicle level and at rest at the origin. Replaced by the IMU/baro/mag fusion
/// estimator ported from ark-fpv-discovery.
pub struct HoldEstimator;

impl StateEstimator for HoldEstimator {
    fn estimate_state(&mut self, _dt: Duration) -> VehicleState {
        black_box(VehicleState::default())
    }
}

/// Discards motor commands. Replaced by the ESC (PWM/DShot) output driver.
pub struct NullActuator;

impl Actuator for NullActuator {
    fn actuate(&mut self, actuator_command: &ActuatorCommand) {
        black_box(actuator_command);
    }
}
