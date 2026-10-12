//! Test doubles for the pipeline traits. Enabled by the `testing` feature (and in this
//! crate's own tests); off by default.
//!
//! Use these in tests of anything built on the core — runners, backends, firmware glue — instead
//! of writing per-crate mocks. They allocate nothing, so they also work in `no_std` tests.
//!
//! - [`NullVehicle`]: returns a fixed state and counts calls.
//! - [`ConstantCommander`]: emits the same [`Command`] every tick.
//! - [`RecordingTelemetry`]: keeps copies of the last `N` ticks in a fixed-size ring.
//! - [`settled_yaw_rate`]: closed-loop check of a body-rate config's yaw loop against a
//!   constant torque, e.g. an airframe's measured hover yaw imbalance.
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

use crate::controllers::{BodyRateController, BodyRateControllerConfig};
use crate::{
    Actuator, ActuatorCommand, Command, Commander, ControlOutput, FailsafeState, FlightModeReport,
    Saturation, Setpoint, StateSource, TelemetryPublisher, TelemetryRecord, Vec3, VehicleState,
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
/// use stable_flight::{Actuator, ActuatorCommand, StateSource};
///
/// let mut vehicle = NullVehicle::default();
/// vehicle.state(Duration::ZERO);
/// vehicle.actuate(&ActuatorCommand::default());
/// assert_eq!((vehicle.state_reads(), vehicle.actuations()), (1, 1));
/// ```
#[derive(Debug, Clone, Default)]
pub struct NullVehicle {
    state: VehicleState,
    state_reads: u64,
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

    /// How many times `state` was called (including priming).
    pub fn state_reads(&self) -> u64 {
        self.state_reads
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

impl StateSource for NullVehicle {
    fn state(&mut self, _dt: Duration) -> VehicleState {
        self.state_reads += 1;
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
    pub saturation: Saturation,
    pub failsafe: Option<FailsafeState>,
    pub flight_mode: FlightModeReport,
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
            saturation: self.saturation,
            failsafe: self.failsafe,
            flight_mode: self.flight_mode,
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
            saturation: record.saturation,
            failsafe: record.failsafe,
            flight_mode: record.flight_mode,
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
///     ActuatorCommand, Command, ControlOutput, Saturation, Setpoint, TelemetryPublisher,
///     TelemetryRecord, VehicleState,
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
///         saturation: Saturation::default(),
///         failsafe: None,
///         flight_mode: Default::default(),
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

/// Yaw acceleration of the [`settled_yaw_rate`] model per unit of normalized yaw moment, in
/// rad/s².
const YAW_MODEL_GAIN: f32 = 20.0;

/// Tick of the [`settled_yaw_rate`] simulation, in seconds (250 Hz).
const YAW_MODEL_DT: f32 = 0.004;

/// Simulated time [`settled_yaw_rate`] runs before reading the rate, in seconds.
const YAW_MODEL_DURATION_S: f32 = 60.0;

/// The yaw rate in rad/s that a [`BodyRateController`] with `config` settles at while
/// commanding zero rate against a constant yaw `disturbance` torque (normalized moment, nose
/// right positive).
///
/// It flies a first-order yaw model, `ω̇ = 20 · (moment − disturbance)`, from rest for 60 s at
/// 250 Hz. The integral cancels the disturbance up to its authority `yaw_gains.i ·
/// max_integral`, so the result is 0 below it and `−(disturbance − i · max_integral) / p` above
/// (a steady acro drift). The model has no inertia beyond the integrator, so it checks
/// authority and steady state, not tuning.
///
/// # Examples
///
/// ```
/// use stable_flight::controllers::{BodyRateControllerConfig, Gains};
/// use stable_flight::testing::settled_yaw_rate;
///
/// let config = BodyRateControllerConfig {
///     yaw_gains: Gains { p: 0.6, i: 0.2, d: 0.0 },
///     max_integral: 1.0,
///     ..Default::default()
/// };
/// // Within the integral's 0.2 authority: held at zero rate.
/// assert!(settled_yaw_rate(&config, 0.15).abs() < 1e-3);
/// // Past it: a steady counterclockwise drift of (0.4 - 0.2) / 0.6 rad/s.
/// assert!((settled_yaw_rate(&config, 0.4) + 0.2 / 0.6).abs() < 1e-3);
/// ```
pub fn settled_yaw_rate(config: &BodyRateControllerConfig, disturbance: f32) -> f32 {
    let mut controller = BodyRateController::default();
    let mut rate_radps = 0.0;
    for _ in 0..(YAW_MODEL_DURATION_S / YAW_MODEL_DT) as usize {
        let moment = controller.control(
            config,
            YAW_MODEL_DT,
            Vec3::zeros(),
            Vec3::new(0.0, 0.0, rate_radps),
        );
        rate_radps += YAW_MODEL_GAIN * (moment.z - disturbance) * YAW_MODEL_DT;
    }
    rate_radps
}
