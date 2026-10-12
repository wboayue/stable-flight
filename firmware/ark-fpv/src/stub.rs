//! Placeholder pipeline components until real sensors, ESCs, and a transmitter are wired in.
//! They let the full `FlightLoop` run on target so its timing and flash cost are real:
//! the command, estimate, and motor commands pass through [`black_box`], so the optimizer can't
//! fold the pipeline (or the guidance paths a constant command never takes) away. The core
//! `testing` doubles aren't reused: that feature is dev-only and carries formatting code.

use core::hint::black_box;
use core::time::Duration;

use stable_flight::{
    Actuator, ActuatorCommand, Command, Commander, ControlInput, StateSource, VehicleState,
};

/// Whether [`HoldCommander`] arms: only with the `bench-armed` feature.
const HOLD_ARMED: bool = cfg!(feature = "bench-armed");

/// Always commands zero throttle, level; disarmed unless built with `bench-armed`, which
/// makes every tick run guidance, the stabilizer, and the mixer so the loop's timing is real.
/// `arm` passes through [`black_box`], so both builds link the full pipeline and flash cost
/// is the same. TODO(#134): replace with a receiver-driven commander.
pub struct HoldCommander;

impl Commander for HoldCommander {
    fn next_command(&mut self, _dt: f32, _current_state: &VehicleState) -> Command {
        black_box(Command::new(HOLD_ARMED, ControlInput::default()))
    }
}

/// Reports the vehicle level and at rest at the origin. Replaced by the IMU/baro/mag fusion
/// estimator ported from ark-fpv-discovery.
pub struct HoldState;

impl StateSource for HoldState {
    fn state(&mut self, _dt: Duration) -> VehicleState {
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
