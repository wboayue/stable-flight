//! The protocol's messages.

use core::time::Duration;

use serde::{Deserialize, Serialize};
use stable_flight::{
    ActuatorCommand, Command, ControlMode, ControlOutput, FailsafeState, FlightModeReport,
    FlightRegime, Saturation, Setpoint, TelemetryRecord, VehicleState, params,
};

use crate::frame::{Message, sealed::Sealed};

#[cfg(test)]
mod tests;

/// A parameter name, at most [`params::MAX_NAME_LEN`] bytes (see [`stable_flight::params`]).
/// `u8` length: smaller in memory, same on the wire.
pub type ParamName = heapless::String<{ params::MAX_NAME_LEN }, u8>;

/// The uplinks that steer the vehicle, for a link-fed commander:
/// [`Uplink::Control`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Control {
    /// Keeps the link alive; the flight controller tracks staleness from it.
    Heartbeat,
    /// What to fly, as a core [`Command`].
    Command(Command),
    /// Arm (`true`) or disarm request, subject to the flight controller's arm policy.
    Arm(bool),
    /// The regime to fly in, e.g. a ground station's VTOL transition; guidance decides whether
    /// and when it is flown ([`FlightModeReport`]).
    Regime(FlightRegime),
}

/// A parameter request from the companion: [`Uplink::Param`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ParamRequest {
    /// Set one parameter by name, in its own units. Echoed with the **value sent** if accepted
    /// (exact under the core `Params` stores-or-rejects contract, where a degree param's
    /// read-back may differ in the last bit) and the current value if rejected; an unknown name
    /// gets no reply.
    Set {
        /// Parameter name.
        name: ParamName,
        /// New value.
        value: f32,
    },
    /// List every parameter, one [`ParamValue`] each, in index order. Restarts a listing
    /// already in progress.
    List,
}

/// One parameter's value: [`Downlink::Param`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParamValue {
    /// Parameter name.
    pub name: ParamName,
    /// Value, in the parameter's own units.
    pub value: f32,
    /// Index of this parameter, `0..count`, stable across the flight controller's lifetime.
    pub index: u16,
    /// Number of parameters.
    pub count: u16,
}

/// Longest [`Downlink::Status`] text in bytes.
pub const MAX_STATUS_LEN: usize = 64;

/// [`Downlink::Status`] text, at most [`MAX_STATUS_LEN`] bytes.
/// `u8` length: smaller in memory, same on the wire.
pub type StatusText = heapless::String<MAX_STATUS_LEN, u8>;

/// Companion/host → flight controller.
///
/// # Examples
///
/// ```
/// use stable_flight_link::stable_flight::{AttitudeCommand, Command, ControlInput};
/// use stable_flight_link::{Control, MAX_FRAME_LEN, Message, Uplink};
///
/// let hover = Uplink::Control(Control::Command(Command::armed(ControlInput::Attitude(
///     AttitudeCommand {
///         throttle: 0.5,
///         ..Default::default()
///     },
/// ))));
/// let mut frame = [0u8; MAX_FRAME_LEN];
/// let len = hover.encode(&mut frame).unwrap();
/// assert_eq!(Uplink::decode(&mut frame[..len]), Ok(hover));
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Uplink {
    /// Steers the vehicle; for a link-fed commander.
    Control(Control),
    /// Set a parameter or list them all; answered with [`Downlink::Param`]s. See
    /// [`ParamServer`](crate::params::ParamServer).
    Param(ParamRequest),
}

/// Flight controller → companion/host.
///
/// # Examples
///
/// ```
/// use stable_flight_link::stable_flight::{ControlMode, FlightModeReport};
/// use stable_flight_link::{Downlink, MAX_FRAME_LEN, Message};
///
/// let heartbeat = Downlink::Heartbeat {
///     armed: false,
///     mode: ControlMode::Attitude,
///     flight_mode: FlightModeReport::default(),
///     link_ok: true,
/// };
/// let mut frame = [0u8; MAX_FRAME_LEN];
/// let len = heartbeat.encode(&mut frame).unwrap();
/// assert_eq!(Downlink::decode(&mut frame[..len]), Ok(heartbeat));
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[expect(
    clippy::large_enum_variant,
    reason = "no heap to box a Record in no_std; a message lives on the stack for one encode"
)]
pub enum Downlink {
    /// Flight controller status.
    Heartbeat {
        /// Motors armed.
        armed: bool,
        /// Control mode: the kind of the commanded input (sticks, waypoint, land), not the
        /// regime; that is `flight_mode`.
        mode: ControlMode,
        /// The regime guidance flies, and why it isn't the requested one
        /// ([`TelemetryRecord::flight_mode`]).
        flight_mode: FlightModeReport,
        /// The flight controller is hearing the companion: `false` while a failsafe flies the
        /// fallback ([`FailsafeState::Fallback`]), `true` otherwise, including without a failsafe.
        link_ok: bool,
    },
    /// Estimated vehicle state.
    State(VehicleState),
    /// Control output: commanded thrust and the limits hit, sent with each state.
    Output(Output),
    /// One tick's full telemetry, for debugging and tuning; opt-in, at a lower rate.
    Record(Record),
    /// One parameter's value: a listing entry or a write's echo.
    Param(ParamValue),
    /// Human-readable status text.
    Status(StatusText),
}

impl Sealed for Uplink {}
impl Message for Uplink {}
impl Sealed for Downlink {}
impl Message for Downlink {}

/// What the control loop is putting out: [`Downlink::Output`]. Small enough to stream with
/// every [`Downlink::State`], so a companion gets thrust and saturation without a [`Record`].
///
/// # Examples
///
/// ```
/// use stable_flight_link::stable_flight::Saturation;
/// use stable_flight_link::{Downlink, MAX_FRAME_LEN, Message, Output};
///
/// let mut saturation = Saturation::default();
/// saturation.tilt = true;
/// let output = Downlink::Output(Output { thrust: 0.5, saturation });
/// let mut frame = [0u8; MAX_FRAME_LEN];
/// let len = output.encode(&mut frame).unwrap();
/// assert_eq!(Downlink::decode(&mut frame[..len]), Ok(output));
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Output {
    /// Normalized collective thrust the stabilizer commanded on the latest tick
    /// ([`ControlOutput::thrust`]).
    pub thrust: f32,
    /// Limits hit on any tick since the previous [`Downlink::Output`], so a saturation shorter
    /// than the send interval still shows.
    pub saturation: Saturation,
}

/// Owned copy of a core [`TelemetryRecord`], which borrows its parts and so can't be
/// deserialized. Same fields, same meaning.
///
/// # Examples
///
/// ```
/// use core::time::Duration;
/// use stable_flight_link::Record;
/// use stable_flight_link::stable_flight::{
///     ActuatorCommand, Command, ControlOutput, FlightModeReport, Saturation, Setpoint,
///     TelemetryRecord, VehicleState,
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
///     failsafe: None,
///     flight_mode: FlightModeReport::default(),
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
    /// Which commander a failsafe let fly; `None` without a failsafe.
    pub failsafe: Option<FailsafeState>,
    /// The flight mode guidance flew, and why it isn't the requested regime.
    pub flight_mode: FlightModeReport,
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
            failsafe: self.failsafe,
            flight_mode: self.flight_mode,
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
            failsafe: record.failsafe,
            flight_mode: record.flight_mode,
        }
    }
}
