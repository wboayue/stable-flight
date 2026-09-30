//! Combinators that build one pipeline component out of several.
//!
//! - [`ComposedVehicle`] joins a [`StateEstimator`] and an [`Actuator`] into a [`Vehicle`]:
//!   firmware pairs an IMU/estimator with an ESC driver; sensor-synthesis SITL pairs a
//!   truth-driven estimator with a simulator's actuator.
//! - `(A, B)` is a [`TelemetryPublisher`] that fans each record out to both sinks (MAVLink and
//!   a file log, say). Nest tuples for more: `(A, (B, C))`.
//!
//! Both are allocation-free and add no dynamic dispatch.
//!
//! [`Vehicle`]: crate::Vehicle

use std::time::Duration;

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

/// Fans each record out to both publishers, first `A` then `B`.
impl<A: TelemetryPublisher, B: TelemetryPublisher> TelemetryPublisher for (A, B) {
    fn publish(&mut self, record: &TelemetryRecord) {
        self.0.publish(record);
        self.1.publish(record);
    }
}
