//! Test doubles for the pipeline traits. Enabled by the `testing` feature (and in this
//! crate's own tests); off by default.
//!
//! Use these in tests of anything built on the core — runners, backends, firmware glue — instead
//! of writing per-crate mocks. They allocate nothing, so they also work in `no_std` tests.
//!
//! - [`NullVehicle`]: returns a fixed state and counts calls.
//! - [`ConstantCommander`]: emits the same [`Command`] every tick.
//! - [`RecordingTelemetry`]: keeps copies of the last `N` ticks in a fixed-size ring.
//! - [`conformance`]: physical scenarios that check a backend's `VehicleState` against the
//!   frame contract, and [`conformance_tests!`](crate::conformance_tests) to run them.
//!
//! Recover a double after running a [`FlightLoop`](crate::FlightLoop) with
//! [`FlightLoop::into_parts`](crate::FlightLoop::into_parts).
//!
//! ```toml
//! [dev-dependencies]
//! stable-flight = { version = "0.1", features = ["testing"] }
//! ```

use core::time::Duration;

use crate::{
    Actuator, ActuatorCommand, Command, Commander, ControlOutput, Setpoint, StateEstimator,
    TelemetryPublisher, TelemetryRecord, VehicleState,
};

pub mod conformance;

#[cfg(test)]
mod tests;

/// A vehicle that reports a fixed state, ignores commands, and counts both.
///
/// # Examples
///
/// ```
/// use std::time::Duration;
/// use stable_flight::testing::NullVehicle;
/// use stable_flight::{Actuator, ActuatorCommand, StateEstimator};
///
/// let mut vehicle = NullVehicle::default();
/// vehicle.estimate_state(Duration::ZERO);
/// vehicle.actuate(&ActuatorCommand::default());
/// assert_eq!((vehicle.estimates(), vehicle.actuations()), (1, 1));
/// ```
#[derive(Debug, Clone, Default)]
pub struct NullVehicle {
    state: VehicleState,
    estimates: u64,
    actuations: u64,
    last_actuation: Option<ActuatorCommand>,
}

impl NullVehicle {
    /// A vehicle at rest at the origin (`VehicleState::default()`).
    pub fn new() -> Self {
        Self::default()
    }

    /// A vehicle that always reports `state`.
    pub fn with_state(state: VehicleState) -> Self {
        NullVehicle {
            state,
            ..Self::default()
        }
    }

    /// How many times `estimate_state` was called (including priming).
    pub fn estimates(&self) -> u64 {
        self.estimates
    }

    /// How many times `actuate` was called.
    pub fn actuations(&self) -> u64 {
        self.actuations
    }

    /// The most recent actuator command, if any.
    pub fn last_actuation(&self) -> Option<&ActuatorCommand> {
        self.last_actuation.as_ref()
    }
}

impl StateEstimator for NullVehicle {
    fn estimate_state(&mut self, _dt: Duration) -> VehicleState {
        self.estimates += 1;
        self.state.clone()
    }
}

impl Actuator for NullVehicle {
    fn actuate(&mut self, actuator_command: &ActuatorCommand) {
        self.actuations += 1;
        self.last_actuation = Some(*actuator_command);
    }
}

/// A commander that emits the same command every tick. The default is a disarmed,
/// zero-throttle attitude command.
///
/// # Examples
///
/// ```
/// use stable_flight::testing::ConstantCommander;
/// use stable_flight::{AttitudeCommand, Command, Commander, ControlInput, VehicleState};
///
/// let hover = Command::armed(ControlInput::Attitude(AttitudeCommand {
///     throttle: 0.5,
///     ..Default::default()
/// }));
/// let mut commander = ConstantCommander::new(hover);
/// let command = commander.next_command(0.004, &VehicleState::default());
/// assert!(command.arm);
/// assert!(matches!(
///     command.input,
///     ControlInput::Attitude(AttitudeCommand { throttle: 0.5, .. })
/// ));
/// ```
#[derive(Debug, Default, Clone, Copy)]
pub struct ConstantCommander {
    command: Command,
}

impl ConstantCommander {
    /// Emits `command` every tick.
    pub fn new(command: Command) -> Self {
        ConstantCommander { command }
    }
}

impl Commander for ConstantCommander {
    fn next_command(&mut self, _dt: f32, _current_state: &VehicleState) -> Command {
        self.command
    }
}

/// An owned copy of one [`TelemetryRecord`]; fields mean what they do there.
#[derive(Debug, Clone)]
#[allow(missing_docs)]
pub struct RecordedTick {
    pub dt: Duration,
    pub command: Command,
    pub setpoint: Setpoint,
    pub state: VehicleState,
    pub control: ControlOutput,
    pub actuators: ActuatorCommand,
}

impl RecordedTick {
    /// Borrows this tick as a [`TelemetryRecord`], e.g. to replay it into another sink.
    pub fn as_record(&self) -> TelemetryRecord<'_> {
        TelemetryRecord {
            dt: self.dt,
            command: &self.command,
            setpoint: &self.setpoint,
            state: &self.state,
            control: &self.control,
            actuators: &self.actuators,
        }
    }
}

impl From<&TelemetryRecord<'_>> for RecordedTick {
    fn from(record: &TelemetryRecord<'_>) -> Self {
        RecordedTick {
            dt: record.dt,
            command: *record.command,
            setpoint: *record.setpoint,
            state: record.state.clone(),
            control: *record.control,
            actuators: *record.actuators,
        }
    }
}

/// A telemetry sink that keeps the last `N` ticks (a fixed-size ring, no allocation) and
/// counts all of them.
///
/// Ticks are stored inline, roughly 200 bytes each, so the recorder is about `200 × N` bytes
/// wherever it lives. Keep `N` small (hundreds) for a stack-allocated recorder; `N = 10_000`
/// overflows a default test thread's stack. `N = 0` only counts.
///
/// # Examples
///
/// ```
/// use std::time::Duration;
/// use stable_flight::testing::RecordingTelemetry;
/// use stable_flight::{
///     ActuatorCommand, Command, ControlOutput, Setpoint, TelemetryPublisher, TelemetryRecord,
///     VehicleState,
/// };
///
/// let mut telemetry = RecordingTelemetry::<2>::new();
/// for thrust in [0.1, 0.2, 0.3] {
///     telemetry.publish(&TelemetryRecord {
///         dt: Duration::from_millis(4),
///         command: &Command::default(),
///         setpoint: &Setpoint::Attitude(Default::default()),
///         state: &VehicleState::default(),
///         control: &ControlOutput { thrust, ..Default::default() },
///         actuators: &ActuatorCommand::default(),
///     });
/// }
///
/// assert_eq!(telemetry.total(), 3);
/// let kept: Vec<f32> = telemetry.iter().map(|tick| tick.control.thrust).collect();
/// assert_eq!(kept, [0.2, 0.3]); // oldest first; 0.1 was overwritten
/// ```
#[derive(Debug, Clone)]
pub struct RecordingTelemetry<const N: usize> {
    ticks: [Option<RecordedTick>; N],
    total: u64,
}

impl<const N: usize> RecordingTelemetry<N> {
    /// An empty recorder.
    pub fn new() -> Self {
        RecordingTelemetry {
            ticks: core::array::from_fn(|_| None),
            total: 0,
        }
    }

    /// Ticks published so far, including ones no longer kept.
    pub fn total(&self) -> u64 {
        self.total
    }

    /// The most recent tick, if any.
    pub fn last(&self) -> Option<&RecordedTick> {
        let newest = self.total.checked_sub(1)?;
        self.slot(newest)
    }

    /// The kept ticks, oldest first (at most `N`).
    pub fn iter(&self) -> impl Iterator<Item = &RecordedTick> {
        let kept = self.total.min(N as u64);
        (self.total - kept..self.total).filter_map(|index| self.slot(index))
    }

    fn slot(&self, index: u64) -> Option<&RecordedTick> {
        if N == 0 {
            return None;
        }
        self.ticks[(index % N as u64) as usize].as_ref()
    }
}

impl<const N: usize> Default for RecordingTelemetry<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> TelemetryPublisher for RecordingTelemetry<N> {
    fn publish(&mut self, record: &TelemetryRecord) {
        if N > 0 {
            self.ticks[(self.total % N as u64) as usize] = Some(RecordedTick::from(record));
        }
        self.total += 1;
    }
}
