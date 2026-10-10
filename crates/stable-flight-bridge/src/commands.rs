//! Commands a ground station sends the flight controller: see [`CommandService`].

use std::time::{Duration, Instant};

use mavlink::MavHeader;
use mavlink::common::*;
use stable_flight_link::stable_flight::{ControlInput, ControlMode, LandCommand, Vec2};
use stable_flight_link::{Control, Uplink};

use crate::encode;
use crate::outbox::Outbox;
use crate::setpoints;

#[cfg(test)]
mod tests;

/// What the service needs of a live flight controller.
#[derive(Debug, Clone, Copy)]
pub(crate) struct FcView {
    /// Armed, per its latest heartbeat.
    pub(crate) armed: bool,
    /// Its mode, per its latest heartbeat.
    pub(crate) mode: ControlMode,
    /// Its latest horizontal position (north, east), if known and finite.
    pub(crate) position_m: Option<Vec2>,
}

/// What the flight controller's heartbeat must show to confirm a request.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Confirmation {
    Armed(bool),
    Mode(ControlMode),
}

impl Confirmation {
    /// Whether a heartbeat reporting `armed` and `mode` shows it.
    fn shown_by(self, armed: bool, mode: ControlMode) -> bool {
        match self {
            Confirmation::Armed(arm) => arm == armed,
            Confirmation::Mode(expected) => expected == mode,
        }
    }
}

/// A request the flight controller hasn't confirmed yet.
#[derive(Debug, Clone, Copy)]
struct Pending {
    command: MavCmd,
    confirmation: Confirmation,
    deadline: Instant,
    sender: MavHeader,
}

/// Translates the ground station's commands for the flight controller and acks them:
///
/// - `MAV_CMD_COMPONENT_ARM_DISARM` (param1 1 arms, 0 disarms) goes out as `Control::Arm`. The
///   force variants in param2 (QGC's "Force Arm", 2989, and "Emergency Stop", 21196) act the
///   same: the flight controller has no arming checks to skip and honors every disarm. One for
///   the state the vehicle is already in is `ACCEPTED` at once.
/// - `MAV_CMD_NAV_LAND` (from offboard apps: QGC offers no Land for a generic autopilot) goes
///   out as a land at the vehicle's latest position; the latitude and longitude in param5/6
///   are ignored. The flight controller disarms on touchdown. A land while disarmed is
///   `DENIED`, one before any position has arrived is `TEMPORARILY_REJECTED`, and one while
///   already landing is `ACCEPTED` at once.
///
/// A request that goes out is acked `ACCEPTED` only once the flight controller's heartbeat
/// shows it (the arm state, or the land mode), and `FAILED` if it doesn't within
/// `ack_timeout`. QGC waits 1.2 s and never retries an arm, so the timeout stays under that.
/// A request with no live flight controller, or while another is pending, is
/// `TEMPORARILY_REJECTED`.
///
/// Every other command is left to the caller ([`on_command_long`](Self::on_command_long)
/// returns `false`).
#[derive(Debug)]
pub(crate) struct CommandService {
    ack_timeout: Duration,
    pending: Option<Pending>,
}

impl CommandService {
    /// Fails an unconfirmed request after `ack_timeout`.
    pub(crate) fn new(ack_timeout: Duration) -> Self {
        Self {
            ack_timeout,
            pending: None,
        }
    }

    /// Handles a `COMMAND_LONG` from `sender` received at `now`, with the flight controller if
    /// it is alive. Returns whether the command was this service's.
    pub(crate) fn on_command_long(
        &mut self,
        sender: &MavHeader,
        data: &COMMAND_LONG_DATA,
        fc: Option<FcView>,
        now: Instant,
        out: &mut Outbox,
    ) -> bool {
        let request = match data.command {
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM => arm_disarm(data.param1 >= 0.5, fc),
            MavCmd::MAV_CMD_NAV_LAND => land(fc),
            _ => return false,
        };
        let request = if self.pending.is_some() {
            Err(MavResult::MAV_RESULT_TEMPORARILY_REJECTED)
        } else {
            request
        };
        match request {
            Ok((uplink, confirmation)) => {
                out.to_fc.push(uplink);
                self.pending = Some(Pending {
                    command: data.command,
                    confirmation,
                    deadline: now + self.ack_timeout,
                    sender: *sender,
                });
            }
            Err(result) => ack(sender, data.command, result, out),
        }
        true
    }

    /// The flight controller's heartbeat reported `armed` in `mode`: confirms a matching
    /// request.
    pub(crate) fn on_fc_heartbeat(&mut self, armed: bool, mode: ControlMode, out: &mut Outbox) {
        if let Some(pending) = self
            .pending
            .filter(|p| p.confirmation.shown_by(armed, mode))
        {
            ack(
                &pending.sender,
                pending.command,
                MavResult::MAV_RESULT_ACCEPTED,
                out,
            );
            self.pending = None;
        }
    }

    /// Fails a request still unconfirmed at `now`.
    pub(crate) fn poll(&mut self, now: Instant, out: &mut Outbox) {
        if let Some(pending) = self.pending.filter(|p| now >= p.deadline) {
            ack(
                &pending.sender,
                pending.command,
                MavResult::MAV_RESULT_FAILED,
                out,
            );
            self.pending = None;
        }
    }
}

/// What to send for a request, and the heartbeat that confirms it; or the ack to answer with
/// at once.
type Request = Result<(Uplink, Confirmation), MavResult>;

/// An arm (`arm`) or disarm request.
fn arm_disarm(arm: bool, fc: Option<FcView>) -> Request {
    match fc {
        None => Err(MavResult::MAV_RESULT_TEMPORARILY_REJECTED),
        Some(fc) if fc.armed == arm => Err(MavResult::MAV_RESULT_ACCEPTED),
        Some(_) => Ok((Uplink::Control(Control::Arm(arm)), Confirmation::Armed(arm))),
    }
}

/// A land at the vehicle's latest position.
fn land(fc: Option<FcView>) -> Request {
    let Some(fc) = fc else {
        return Err(MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
    };
    if !fc.armed {
        return Err(MavResult::MAV_RESULT_DENIED);
    }
    if fc.mode == ControlMode::Land {
        return Err(MavResult::MAV_RESULT_ACCEPTED);
    }
    let Some(position_m) = fc.position_m else {
        return Err(MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
    };
    Ok((
        setpoints::uplink(ControlInput::Land(LandCommand { position_m })),
        Confirmation::Mode(ControlMode::Land),
    ))
}

/// Queues an ack of `command` with `result` for `sender`.
fn ack(sender: &MavHeader, command: MavCmd, result: MavResult, out: &mut Outbox) {
    let ack = encode::command_ack(sender, command, result);
    out.to_gcs.push(MavMessage::COMMAND_ACK(ack));
}
