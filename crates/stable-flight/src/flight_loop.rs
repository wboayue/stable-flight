//! One control-loop tick, independent of how ticks are scheduled.
//!
//! [`FlightLoop`] owns the five pipeline components and runs them in order for a single tick.
//! It has no clock, threads, or signal handling: a runner decides when to call
//! [`FlightLoop::step`] and with what `dt` (a wall-clock runner in `stable-flight-sitl` today;
//! a lockstep simulator runner or a hardware timer interrupt later).

use std::time::Duration;

use crate::{
    Commander, Mixer, Stabilizer, TelemetryPublisher, TelemetryRecord, Vehicle, VehicleState,
};

#[cfg(test)]
mod tests;

/// Sequences one control tick: commander → estimate → stabilize → mix → actuate → telemetry.
///
/// Call [`prime`](FlightLoop::prime) once before the first [`step`](FlightLoop::step).
///
/// # Examples
///
/// ```
/// use std::time::Duration;
/// use stable_flight::mixers::QuadCopterMixer;
/// use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
/// use stable_flight::{
///     Actuator, ActuatorCommand, AttitudeCommand, Command, Commander, FlightLoop,
///     StateEstimator, VehicleState,
/// };
///
/// struct Hover;
/// impl Commander for Hover {
///     fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
///         Command::Attitude(AttitudeCommand { throttle: 0.5, arm: true, ..Default::default() })
///     }
/// }
///
/// struct StillAir;
/// impl StateEstimator for StillAir {
///     fn estimate_state(&mut self, _dt: Duration) -> VehicleState {
///         VehicleState::default()
///     }
/// }
/// impl Actuator for StillAir {
///     fn actuate(&mut self, _command: &ActuatorCommand) {}
/// }
///
/// let mut flight_loop = FlightLoop::new(
///     Hover,
///     StillAir,
///     CopterStabilizer::new(CopterStabilizerConfig::default()),
///     QuadCopterMixer::default(),
///     (), // no telemetry
/// );
///
/// flight_loop.prime();
/// for _ in 0..3 {
///     flight_loop.step(Duration::from_millis(4)); // 250 Hz
/// }
/// ```
///
/// The struct itself is unbounded (bounds live on the `impl`), so types that hold a
/// `FlightLoop` — runners, firmware statics — needn't repeat the five trait bounds.
pub struct FlightLoop<C, V, S, M, T> {
    commander: C,
    vehicle: V,
    stabilizer: S,
    mixer: M,
    telemetry: T,
    /// Latest estimate; the commander sees the previous tick's state.
    state: VehicleState,
}

impl<C, V, S, M, T> FlightLoop<C, V, S, M, T>
where
    C: Commander,
    V: Vehicle,
    S: Stabilizer,
    M: Mixer,
    T: TelemetryPublisher,
{
    /// Assembles a loop from its five components. Does not touch the vehicle.
    pub fn new(commander: C, vehicle: V, stabilizer: S, mixer: M, telemetry: T) -> Self {
        FlightLoop {
            commander,
            vehicle,
            stabilizer,
            mixer,
            telemetry,
            state: VehicleState::default(),
        }
    }

    /// Takes the initial estimate (with `dt = Duration::ZERO`, per the [`StateEstimator`]
    /// convention) and primes the stabilizer from it. Call once before the first tick.
    ///
    /// [`StateEstimator`]: crate::StateEstimator
    pub fn prime(&mut self) {
        self.state = self.vehicle.estimate_state(Duration::ZERO);
        self.stabilizer.prime(&self.state);
    }

    /// Runs one tick with loop period `dt`.
    ///
    /// The commander sees the state from the previous tick; the stabilizer sees the fresh
    /// estimate. Telemetry receives a [`TelemetryRecord`] after actuation. Allocation-free.
    pub fn step(&mut self, dt: Duration) {
        let dt_secs = dt.as_secs_f32();

        let command = self.commander.next_command(dt_secs, &self.state);
        self.state = self.vehicle.estimate_state(dt);
        let control = self.stabilizer.stabilize(dt_secs, &command, &self.state);
        let actuators = self.mixer.mix(&control);
        self.vehicle.actuate(&actuators);

        self.telemetry.publish(&TelemetryRecord {
            dt,
            command: &command,
            state: &self.state,
            control: &control,
            actuators: &actuators,
        });
    }

    /// The most recent state estimate.
    pub fn state(&self) -> &VehicleState {
        &self.state
    }

    /// Splits the loop back into `(commander, vehicle, stabilizer, mixer, telemetry)`, e.g. to
    /// inspect test doubles after a run or to hand the vehicle to another runner.
    ///
    /// ```
    /// # use std::time::Duration;
    /// # use stable_flight::mixers::QuadCopterMixer;
    /// # use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
    /// # use stable_flight::{Actuator, ActuatorCommand, AttitudeCommand, Command, Commander, StateEstimator, VehicleState};
    /// use stable_flight::FlightLoop;
    ///
    /// # struct Hover;
    /// # impl Commander for Hover {
    /// #     fn next_command(&mut self, _: f32, _: &VehicleState) -> Command {
    /// #         Command::Attitude(AttitudeCommand::default())
    /// #     }
    /// # }
    /// /// Counts actuations.
    /// #[derive(Default)]
    /// struct Bench {
    ///     actuations: u32,
    /// }
    /// # impl StateEstimator for Bench {
    /// #     fn estimate_state(&mut self, _: Duration) -> VehicleState { VehicleState::default() }
    /// # }
    /// impl Actuator for Bench {
    ///     fn actuate(&mut self, _: &ActuatorCommand) {
    ///         self.actuations += 1;
    ///     }
    /// }
    ///
    /// let mut flight_loop = FlightLoop::new(
    ///     Hover,
    ///     Bench::default(),
    ///     CopterStabilizer::new(CopterStabilizerConfig::default()),
    ///     QuadCopterMixer::default(),
    ///     (),
    /// );
    /// flight_loop.prime();
    /// flight_loop.step(Duration::from_millis(4));
    ///
    /// let (_, bench, _, _, _) = flight_loop.into_parts();
    /// assert_eq!(bench.actuations, 1);
    /// ```
    pub fn into_parts(self) -> (C, V, S, M, T) {
        (
            self.commander,
            self.vehicle,
            self.stabilizer,
            self.mixer,
            self.telemetry,
        )
    }
}
