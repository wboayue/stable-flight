//! Combinators that build one pipeline component out of several.
//!
//! - [`ComposedVehicle`] joins an independent [`StateEstimator`] and [`Actuator`] into a
//!   [`Vehicle`], e.g. an IMU/estimator and an ESC driver on hardware. The two halves don't see
//!   each other, so it doesn't fit an estimator that needs the actuator's data (such as a
//!   simulator bridge's truth state); that case needs a different combinator.
//! - `(A, B)` is a [`TelemetryPublisher`] that fans each record out to both sinks; nest tuples
//!   for more. `()` is the no-op sink.
//!
//! Both are allocation-free and add no dynamic dispatch.
//!
//! ```
//! # use std::time::Duration;
//! # use stable_flight::{ActuatorCommand, Command, ControlOutput, Saturation, Setpoint, VehicleState};
//! use stable_flight::{TelemetryPublisher, TelemetryRecord};
//!
//! #[derive(Default)]
//! struct Counter(u32);
//! impl TelemetryPublisher for Counter {
//!     fn publish(&mut self, _record: &TelemetryRecord) {
//!         self.0 += 1;
//!     }
//! }
//!
//! // A ground-station sink, a log sink, and `()` where a slot needs no sink.
//! let mut sinks = (Counter::default(), (Counter::default(), ()));
//! # let (command, setpoint, state, control, actuators) = (Command::default(), Setpoint::Attitude(Default::default()), VehicleState::default(), ControlOutput::default(), ActuatorCommand::default());
//! # let record = TelemetryRecord { dt: Duration::from_millis(4), command: &command, setpoint: &setpoint, state: &state, control: &control, actuators: &actuators, saturation: Saturation::default() };
//! sinks.publish(&record);
//! assert_eq!((sinks.0.0, sinks.1.0.0), (1, 1));
//! ```
//!
//! [`Vehicle`]: crate::Vehicle

use core::time::Duration;

use crate::{
    Actuator, ActuatorCommand, StateEstimator, TelemetryPublisher, TelemetryRecord, VehicleState,
};

#[cfg(test)]
mod tests;

/// A [`Vehicle`](crate::Vehicle) made of a separate estimator and actuator.
///
/// `estimate_state` goes to `E`, `actuate` to `A`; `Vehicle` itself comes from the blanket
/// impl. Use it when sensing and actuation are different components, which is the normal
/// case on hardware.
///
/// # Examples
///
/// ```
/// use std::time::Duration;
/// use stable_flight::{
///     Actuator, ActuatorCommand, ComposedVehicle, StateEstimator, Vehicle, VehicleState,
/// };
///
/// struct Imu;
/// impl StateEstimator for Imu {
///     fn estimate_state(&mut self, _dt: Duration) -> VehicleState {
///         VehicleState::default()
///     }
/// }
///
/// struct Escs {
///     last: ActuatorCommand,
/// }
/// impl Actuator for Escs {
///     fn actuate(&mut self, command: &ActuatorCommand) {
///         self.last = *command;
///     }
/// }
///
/// fn fly(vehicle: &mut impl Vehicle) {
///     let _state = vehicle.estimate_state(Duration::from_millis(4));
///     vehicle.actuate(&ActuatorCommand { motors: [0.5; 8], ..Default::default() });
/// }
///
/// let mut vehicle = ComposedVehicle::new(Imu, Escs { last: ActuatorCommand::default() });
/// fly(&mut vehicle);
/// assert_eq!(vehicle.actuator().last.motors[0], 0.5);
/// ```
#[derive(Debug, Clone, Default)]
pub struct ComposedVehicle<E, A> {
    estimator: E,
    actuator: A,
}

impl<E, A> ComposedVehicle<E, A> {
    /// Pairs an estimator with an actuator.
    pub fn new(estimator: E, actuator: A) -> Self {
        ComposedVehicle {
            estimator,
            actuator,
        }
    }

    /// The estimator half.
    pub fn estimator(&self) -> &E {
        &self.estimator
    }

    /// The actuator half.
    pub fn actuator(&self) -> &A {
        &self.actuator
    }

    /// Splits back into `(estimator, actuator)`.
    pub fn into_parts(self) -> (E, A) {
        (self.estimator, self.actuator)
    }
}

impl<E: StateEstimator, A> StateEstimator for ComposedVehicle<E, A> {
    fn estimate_state(&mut self, dt: Duration) -> VehicleState {
        self.estimator.estimate_state(dt)
    }
}

impl<E, A: Actuator> Actuator for ComposedVehicle<E, A> {
    fn actuate(&mut self, actuator_command: &ActuatorCommand) {
        self.actuator.actuate(actuator_command);
    }
}

/// The no-op sink: discards every record. Use it where a loop needs a publisher but nothing
/// should be published, or as a placeholder slot in a tuple fan-out.
impl TelemetryPublisher for () {
    fn publish(&mut self, _record: &TelemetryRecord) {}
}

/// Fans each record out to both publishers, first `A` then `B`.
impl<A: TelemetryPublisher, B: TelemetryPublisher> TelemetryPublisher for (A, B) {
    fn publish(&mut self, record: &TelemetryRecord) {
        self.0.publish(record);
        self.1.publish(record);
    }
}
