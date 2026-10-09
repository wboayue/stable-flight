//! The translation between the native link and MAVLink, with no I/O: [`Bridge`] takes the
//! messages each side sent and queues what to send in an [`Outbox`]. The caller owns the
//! transports and the clock.

use std::time::{Duration, Instant};

use mavlink::common::*;
use mavlink::{MavHeader, MessageData};
use stable_flight_link::stable_flight::ControlMode;
use stable_flight_link::{Downlink, ParamRequest, Uplink};

use crate::encode;
use crate::params::{ParamCache, param_name};

#[cfg(test)]
mod tests;

/// Identity and timing of a [`Bridge`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct BridgeConfig {
    /// MAVLink system id the flight controller appears as.
    pub(crate) system_id: u8,
    /// MAVLink component id the flight controller appears as.
    pub(crate) component_id: u8,
    /// The vehicle type the heartbeat reports.
    pub(crate) vehicle_type: MavType,
    /// How long after its last heartbeat the flight controller counts as gone.
    pub(crate) fc_timeout: Duration,
    /// Interval between `HEARTBEAT`s to the ground station.
    pub(crate) heartbeat_interval: Duration,
}

impl Default for BridgeConfig {
    /// System and component id 1 (the autopilot, which QGC requires), a quadrotor, a 3 s
    /// timeout, and 1 Hz heartbeats.
    fn default() -> Self {
        Self {
            system_id: 1,
            component_id: 1,
            vehicle_type: MavType::MAV_TYPE_QUADROTOR,
            fc_timeout: Duration::from_secs(3),
            heartbeat_interval: Duration::from_secs(1),
        }
    }
}

/// What to send after handling a message: uplinks to the flight controller and MAVLink to the
/// ground station, stamped with the flight controller's ids.
#[derive(Debug, Default)]
pub(crate) struct Outbox {
    pub(crate) to_fc: Vec<Uplink>,
    pub(crate) to_gcs: Vec<MavMessage>,
}

/// The flight controller's latest heartbeat.
#[derive(Debug, Clone, Copy)]
struct FcStatus {
    last_seen: Instant,
    armed: bool,
    mode: ControlMode,
}

/// Presents a flight controller on the native link as a MAVLink autopilot. The message-by-message
/// mapping is in the crate README; the invariants behind it:
///
/// - The flight controller is the only judge of a param: writes and listings go to it, and its
///   replies go out unchanged. The cache only answers reads, and is cleared whenever the flight
///   controller (re)appears, since its table may have changed.
/// - `HEARTBEAT` goes out only while the flight controller is alive (heard from within
///   `fc_timeout`), so the ground station sees link loss.
/// - Only MAVLink addressed to the flight controller's system and component (or broadcast) is
///   handled.
#[derive(Debug)]
pub(crate) struct Bridge {
    config: BridgeConfig,
    fc: Option<FcStatus>,
    next_heartbeat: Option<Instant>,
    params: ParamCache,
}

impl Bridge {
    pub(crate) fn new(config: BridgeConfig) -> Self {
        Self {
            config,
            fc: None,
            next_heartbeat: None,
            params: ParamCache::default(),
        }
    }

    /// The flight controller was heard from within `fc_timeout`.
    fn fc_alive(&self, now: Instant) -> bool {
        self.fc
            .is_some_and(|fc| now.saturating_duration_since(fc.last_seen) < self.config.fc_timeout)
    }

    /// Handles one message from the flight controller, received at `now`.
    pub(crate) fn on_downlink(&mut self, msg: &Downlink, now: Instant, out: &mut Outbox) {
        match msg {
            Downlink::Heartbeat { armed, mode, .. } => {
                if !self.fc_alive(now) {
                    // New or back: its table may have changed, so refill the cache, and show
                    // the ground station the link right away.
                    self.params.clear();
                    out.to_fc.push(Uplink::Param(ParamRequest::List));
                    self.next_heartbeat = Some(now);
                }
                self.fc = Some(FcStatus {
                    last_seen: now,
                    armed: *armed,
                    mode: *mode,
                });
            }
            Downlink::Param(param) => {
                out.to_gcs
                    .push(MavMessage::PARAM_VALUE(encode::param_value(param)));
                self.params.insert(param.clone());
            }
            _ => {}
        }
    }

    /// Handles one MAVLink message from the ground station, sent by `header`'s system and
    /// component.
    pub(crate) fn on_mavlink(&mut self, header: &MavHeader, msg: &MavMessage, out: &mut Outbox) {
        match msg {
            MavMessage::PARAM_REQUEST_LIST(data)
                if self.addressed(data.target_system, data.target_component) =>
            {
                out.to_fc.push(Uplink::Param(ParamRequest::List));
            }
            MavMessage::PARAM_REQUEST_READ(data)
                if self.addressed(data.target_system, data.target_component) =>
            {
                let cached = match usize::try_from(data.param_index) {
                    Ok(index) => self.params.get(index),
                    // A negative index means read by name.
                    Err(_) => param_name(&data.param_id).and_then(|name| self.params.find(&name)),
                };
                out.to_gcs
                    .extend(cached.map(|p| MavMessage::PARAM_VALUE(encode::param_value(p))));
            }
            MavMessage::PARAM_SET(data)
                if self.addressed(data.target_system, data.target_component) =>
            {
                self.param_set(data, out);
            }
            MavMessage::MISSION_REQUEST_LIST(data)
                if self.addressed(data.target_system, data.target_component) =>
            {
                out.to_gcs
                    .push(MavMessage::MISSION_COUNT(encode::empty_mission_count(
                        data.mission_type,
                        header.system_id,
                        header.component_id,
                    )));
            }
            MavMessage::COMMAND_LONG(data)
                if self.addressed(data.target_system, data.target_component) =>
            {
                self.command_long(header, data, out);
            }
            _ => {}
        }
    }

    /// Sends what is due at `now`: the heartbeat.
    pub(crate) fn poll(&mut self, now: Instant, out: &mut Outbox) {
        let Some(fc) = self.fc.filter(|_| self.fc_alive(now)) else {
            return;
        };
        if self.next_heartbeat.is_some_and(|due| now >= due) {
            out.to_gcs
                .push(MavMessage::HEARTBEAT(stable_flight_mavlink::heartbeat(
                    self.config.vehicle_type,
                    fc.armed,
                    fc.mode,
                )));
            self.next_heartbeat = Some(now + self.config.heartbeat_interval);
        }
    }

    /// `target_system` / `target_component` select the flight controller (0 is broadcast).
    fn addressed(&self, target_system: u8, target_component: u8) -> bool {
        (target_system == 0 || target_system == self.config.system_id)
            && (target_component == 0 || target_component == self.config.component_id)
    }

    fn param_set(&self, data: &PARAM_SET_DATA, out: &mut Outbox) {
        let Some(name) = param_name(&data.param_id) else {
            return;
        };
        if data.param_type == MavParamType::MAV_PARAM_TYPE_REAL32 {
            // Unknown to the cache may still be known to the flight controller; it decides.
            out.to_fc.push(Uplink::Param(ParamRequest::Set {
                name,
                value: data.param_value,
            }));
        } else if let Some(current) = self.params.find(&name) {
            // Every param is REAL32: reject by echoing the current value.
            out.to_gcs
                .push(MavMessage::PARAM_VALUE(encode::param_value(current)));
        }
    }

    fn command_long(&self, header: &MavHeader, data: &COMMAND_LONG_DATA, out: &mut Outbox) {
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
            header.system_id,
            header.component_id,
        )));
        if autopilot_version {
            out.to_gcs
                .push(MavMessage::AUTOPILOT_VERSION(encode::autopilot_version()));
        }
    }
}
