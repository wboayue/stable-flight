//! How the flight controller presents itself to a ground station: the heartbeat, and the answers
//! QGC's connect sequence waits for. See [`ConnectService`].

use std::time::{Duration, Instant};

use mavlink::common::*;
use mavlink::{MavHeader, MessageData};
use stable_flight_link::stable_flight::ControlMode;

use crate::encode;
use crate::interval::Interval;
use crate::outbox::Outbox;

#[cfg(test)]
mod tests;

/// Sends `HEARTBEAT` for the flight controller and answers, without it, what a ground station
/// asks while connecting:
///
/// - `HEARTBEAT` every `heartbeat_interval`, the first as soon as the flight controller
///   appears. The caller polls only while the flight controller is alive.
/// - `MISSION_REQUEST_LIST`: an empty `MISSION_COUNT` of the requested type.
/// - `COMMAND_LONG`: `REQUEST_MESSAGE(AUTOPILOT_VERSION)` is acked, then answered; anything else
///   is acked `UNSUPPORTED`, so the ground station moves on without retrying.
///
/// Two answers are placeholders that move out when their real owner arrives (#154): the
/// `UNSUPPORTED` fallback moves to `Bridge`'s routing once a second service handles
/// `COMMAND_LONG` (arming), and `MISSION_REQUEST_LIST` moves to a missions service once
/// missions are served.
#[derive(Debug)]
pub(crate) struct ConnectService {
    vehicle_type: MavType,
    heartbeat: Interval,
}

impl ConnectService {
    /// Reports `vehicle_type` in heartbeats sent every `heartbeat_interval`.
    pub(crate) fn new(vehicle_type: MavType, heartbeat_interval: Duration) -> Self {
        Self {
            vehicle_type,
            heartbeat: Interval::new(heartbeat_interval),
        }
    }

    /// The flight controller appeared (or came back) at `now`: show the ground station at once.
    pub(crate) fn fc_appeared(&mut self, now: Instant, _out: &mut Outbox) {
        self.heartbeat.start(now);
    }

    /// Sends the heartbeat if due at `now`, reporting `armed` and `mode`.
    pub(crate) fn poll(&mut self, now: Instant, armed: bool, mode: ControlMode, out: &mut Outbox) {
        if self.heartbeat.due(now) {
            out.to_gcs
                .push(MavMessage::HEARTBEAT(stable_flight_mavlink::heartbeat(
                    self.vehicle_type,
                    armed,
                    mode,
                )));
        }
    }

    /// Answers a `MISSION_REQUEST_LIST` from `sender`: there is no mission, fence, or rally plan.
    pub(crate) fn on_mission_request_list(
        &self,
        sender: &MavHeader,
        data: &MISSION_REQUEST_LIST_DATA,
        out: &mut Outbox,
    ) {
        out.to_gcs
            .push(MavMessage::MISSION_COUNT(encode::empty_mission_count(
                data.mission_type,
                sender.system_id,
                sender.component_id,
            )));
    }

    /// Acks a `COMMAND_LONG` from `sender`, and answers a request for `AUTOPILOT_VERSION`.
    pub(crate) fn on_command_long(
        &self,
        sender: &MavHeader,
        data: &COMMAND_LONG_DATA,
        out: &mut Outbox,
    ) {
        let autopilot_version = data.command == MavCmd::MAV_CMD_REQUEST_MESSAGE
            && data.param1 == AUTOPILOT_VERSION_DATA::ID as f32;
        let result = if autopilot_version {
            MavResult::MAV_RESULT_ACCEPTED
        } else {
            MavResult::MAV_RESULT_UNSUPPORTED
        };
        out.to_gcs.push(MavMessage::COMMAND_ACK(encode::command_ack(
            data.command,
            result,
            sender.system_id,
            sender.component_id,
        )));
        if autopilot_version {
            out.to_gcs
                .push(MavMessage::AUTOPILOT_VERSION(encode::autopilot_version()));
        }
    }
}
