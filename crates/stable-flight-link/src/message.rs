//! The protocol's messages.

use core::time::Duration;

use serde::{Deserialize, Serialize};
use stable_flight::{
    ActuatorCommand, Command, ControlOutput, Saturation, Setpoint, TelemetryRecord, VehicleState,
    params,
};

use crate::frame::{Message, sealed::Sealed};

#[cfg(test)]
mod tests;

/// A parameter name, at most [`params::MAX_NAME_LEN`] bytes (see [`stable_flight::params`]).
pub type ParamName = heapless::String<{ params::MAX_NAME_LEN }>;

/// Longest [`Downlink::Status`] text in bytes.
pub const MAX_STATUS_LEN: usize = 64;

/// [`Downlink::Status`] text, at most [`MAX_STATUS_LEN`] bytes.
pub type StatusText = heapless::String<MAX_STATUS_LEN>;

/// Companion/host → flight controller.
///
/// # Examples
///
/// ```
/// use stable_flight_link::stable_flight::{AttitudeCommand, Command, ControlInput};
/// use stable_flight_link::{MAX_FRAME_LEN, Message, Uplink};
///
/// let hover = Uplink::Command(Command::armed(ControlInput::Attitude(AttitudeCommand {
///     throttle: 0.5,
///     ..Default::default()
/// })));
/// let mut frame = [0u8; MAX_FRAME_LEN];
/// let len = hover.encode(&mut frame).unwrap();
/// assert_eq!(Uplink::decode(&mut frame[..len]), Ok(hover));
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Uplink {
    /// Keeps the link alive; the flight controller tracks staleness from it.
    Heartbeat,
    /// What to fly, as a core [`Command`].
    Command(Command),
    /// Arm (`true`) or disarm request, subject to the flight controller's arm policy.
    Arm(bool),
    /// Set one parameter by name.
    SetParam {
        /// Parameter name.
        name: ParamName,
        /// New value, in the parameter's own units.
        value: f32,
    },
    /// Ask for every parameter; answered with a [`Downlink::Param`] each.
    RequestParams,
}

/// Flight controller → companion/host.
///
/// # Examples
///
/// ```
/// use stable_flight_link::{Downlink, MAX_FRAME_LEN, Message};
///
/// let heartbeat = Downlink::Heartbeat { armed: false, mode: 0, link_ok: true };
/// let mut frame = [0u8; MAX_FRAME_LEN];
/// let len = heartbeat.encode(&mut frame).unwrap();
/// assert_eq!(Downlink::decode(&mut frame[..len]), Ok(heartbeat));
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Downlink {
    /// Flight controller status.
    Heartbeat {
        /// Motors armed.
        armed: bool,
        /// Flight mode.
        mode: u8,
        /// The flight controller is hearing the companion.
        link_ok: bool,
    },
    /// Estimated vehicle state.
    State(VehicleState),
    /// One tick's full telemetry, for debugging and tuning; sent at a lower rate.
    Record(Record),
    /// One parameter's value.
    Param {
        /// Parameter name.
        name: ParamName,
        /// Current value, in the parameter's own units.
        value: f32,
        /// Index of this parameter.
        index: u16,
        /// Number of parameters.
        count: u16,
    },
    /// Human-readable status text.
    Status(StatusText),
}

impl Sealed for Uplink {}
impl Message for Uplink {}
impl Sealed for Downlink {}
impl Message for Downlink {}

/// Owned copy of a core [`TelemetryRecord`], which borrows its parts and so can't be
/// deserialized. Same fields, same meaning.
///
/// # Examples
///
/// ```
/// use core::time::Duration;
/// use stable_flight_link::Record;
/// use stable_flight_link::stable_flight::{
///     ActuatorCommand, Command, ControlOutput, Saturation, Setpoint, TelemetryRecord,
///     VehicleState,
/// };
///
/// let (command, state) = (Command::default(), VehicleState::default());
/// let setpoint = Setpoint::Attitude(Default::default());
/// let (control, actuators) = (ControlOutput::default(), ActuatorCommand::default());
/// let telemetry = TelemetryRecord {
///     dt: Duration::from_millis(4),
///     command: &command,
///     setpoint: &setpoint,
///     state: &state,
///     control: &control,
///     actuators: &actuators,
///     saturation: Saturation::default(),
/// };
///
/// let record = Record::from(&telemetry);
/// assert_eq!(record.telemetry().dt, telemetry.dt);
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Record {
    /// Nominal loop period.
    pub dt: Duration,
    /// Command the commander produced.
    pub command: Command,
    /// Setpoint guidance derived from `command`.
    pub setpoint: Setpoint,
    /// Estimated vehicle state the stabilizer acted on.
    pub state: VehicleState,
    /// Stabilizer output.
    pub control: ControlOutput,
    /// Mixer output sent to the actuators.
    pub actuators: ActuatorCommand,
    /// Limits hit this tick.
    pub saturation: Saturation,
}

impl Record {
    /// Borrows this record as a core [`TelemetryRecord`], e.g. to feed a host-side
    /// `TelemetryPublisher`.
    pub fn telemetry(&self) -> TelemetryRecord<'_> {
        TelemetryRecord {
            dt: self.dt,
            command: &self.command,
            setpoint: &self.setpoint,
            state: &self.state,
            control: &self.control,
            actuators: &self.actuators,
            saturation: self.saturation,
        }
    }
}

impl From<&TelemetryRecord<'_>> for Record {
    fn from(record: &TelemetryRecord<'_>) -> Self {
        Self {
            dt: record.dt,
            command: *record.command,
            setpoint: *record.setpoint,
            state: record.state.clone(),
            control: *record.control,
            actuators: *record.actuators,
            saturation: record.saturation,
        }
    }
}
