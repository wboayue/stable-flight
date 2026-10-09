//! Commands a ground station sends the flight controller: see [`CommandService`].

use std::time::{Duration, Instant};

use mavlink::MavHeader;
use mavlink::common::*;
use stable_flight_link::{Control, Uplink};

use crate::encode;
use crate::outbox::Outbox;

#[cfg(test)]
mod tests;

/// An arm or disarm request the flight controller hasn't confirmed yet.
#[derive(Debug, Clone, Copy)]
struct PendingArm {
    arm: bool,
    deadline: Instant,
    sender: MavHeader,
}

/// Translates the ground station's commands for the flight controller and acks them:
///
/// - `MAV_CMD_COMPONENT_ARM_DISARM` (param1 1 arms, 0 disarms) goes out as `Control::Arm`. The
///   force variants in param2 (QGC's "Force Arm", 2989, and "Emergency Stop", 21196) act the
///   same: the flight controller has no arming checks to skip and honors every disarm. It is acked `ACCEPTED` only once the flight controller's heartbeat
///   shows the requested state, and `FAILED` if it doesn't within `ack_timeout`. QGC waits
///   1.2 s and never retries an arm, so the timeout stays under that. A request with no live
///   flight controller, or while another is pending, is `TEMPORARILY_REJECTED`; one for the
///   state the vehicle is already in is `ACCEPTED` at once.
///
/// Every other command is left to the caller ([`on_command_long`](Self::on_command_long)
/// returns `false`).
#[derive(Debug)]
pub(crate) struct CommandService {
    ack_timeout: Duration,
    pending: Option<PendingArm>,
}

impl CommandService {
    /// Fails an unconfirmed arm or disarm after `ack_timeout`.
    pub(crate) fn new(ack_timeout: Duration) -> Self {
        Self {
            ack_timeout,
            pending: None,
        }
    }

    /// Handles a `COMMAND_LONG` from `sender` received at `now`, with the flight controller's
    /// arm state if it is alive. Returns whether the command was this service's.
    pub(crate) fn on_command_long(
        &mut self,
        sender: &MavHeader,
        data: &COMMAND_LONG_DATA,
        fc_armed: Option<bool>,
        now: Instant,
        out: &mut Outbox,
    ) -> bool {
        if data.command != MavCmd::MAV_CMD_COMPONENT_ARM_DISARM {
            return false;
        }
        let arm = data.param1 >= 0.5;
        let result = match fc_armed {
            None => MavResult::MAV_RESULT_TEMPORARILY_REJECTED,
            Some(_) if self.pending.is_some() => MavResult::MAV_RESULT_TEMPORARILY_REJECTED,
            Some(armed) if armed == arm => MavResult::MAV_RESULT_ACCEPTED,
            Some(_) => {
                out.to_fc.push(Uplink::Control(Control::Arm(arm)));
                self.pending = Some(PendingArm {
                    arm,
                    deadline: now + self.ack_timeout,
                    sender: *sender,
                });
                return true;
            }
        };
        ack(sender, result, out);
        true
    }

    /// The flight controller's heartbeat reported `armed`: confirms a matching request.
    pub(crate) fn on_fc_heartbeat(&mut self, armed: bool, out: &mut Outbox) {
        if let Some(pending) = self.pending.filter(|p| p.arm == armed) {
            ack(&pending.sender, MavResult::MAV_RESULT_ACCEPTED, out);
            self.pending = None;
        }
    }

    /// Fails a request still unconfirmed at `now`.
    pub(crate) fn poll(&mut self, now: Instant, out: &mut Outbox) {
        if let Some(pending) = self.pending.filter(|p| now >= p.deadline) {
            ack(&pending.sender, MavResult::MAV_RESULT_FAILED, out);
            self.pending = None;
        }
    }
}

/// Queues an ack of `MAV_CMD_COMPONENT_ARM_DISARM` with `result` for `sender`.
fn ack(sender: &MavHeader, result: MavResult, out: &mut Outbox) {
    let ack = encode::command_ack(sender, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, result);
    out.to_gcs.push(MavMessage::COMMAND_ACK(ack));
}
