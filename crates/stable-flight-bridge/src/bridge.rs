//! The translation between the native link and MAVLink, with no I/O: [`Bridge`] takes the
//! messages each side sent and queues what to send in an [`Outbox`]. The caller owns the
//! transports and the clock.

use std::time::{Duration, Instant};

use mavlink::MavHeader;
use mavlink::common::*;
use stable_flight_link::Downlink;
use stable_flight_link::stable_flight::ControlMode;
use stable_flight_mavlink::FlatEarth;

use crate::connect::ConnectService;
use crate::outbox::Outbox;
use crate::params::ParamService;
use crate::state::StateService;

#[cfg(test)]
mod tests;

/// Identity and timing of a [`Bridge`].
#[derive(Debug, Clone, Copy)]
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
    /// Where local position sits on the earth; `None` sends no global position or home.
    pub(crate) earth: Option<FlatEarth>,
    /// Interval between attitude, position, and `VFR_HUD` messages.
    pub(crate) state_interval: Duration,
    /// Interval between `HOME_POSITION`s and `EXTENDED_SYS_STATE`s, which also go out on change.
    pub(crate) slow_interval: Duration,
}

impl Default for BridgeConfig {
    /// System and component id 1 (the autopilot, which QGC requires), a quadrotor, a 3 s
    /// timeout, 1 Hz heartbeats, no origin, state at 10 Hz, and home and landed state at 1 Hz.
    fn default() -> Self {
        Self {
            system_id: 1,
            component_id: 1,
            vehicle_type: MavType::MAV_TYPE_QUADROTOR,
            fc_timeout: Duration::from_secs(3),
            heartbeat_interval: Duration::from_secs(1),
            earth: None,
            state_interval: Duration::from_millis(100),
            slow_interval: Duration::from_secs(1),
        }
    }
}

/// The flight controller's latest heartbeat.
#[derive(Debug, Clone, Copy)]
struct FcStatus {
    last_seen: Instant,
    armed: bool,
    mode: ControlMode,
}

/// Presents a flight controller on the native link as a MAVLink autopilot. The message-by-message
/// mapping is in the crate README.
///
/// `Bridge` tracks whether the flight controller is alive (heard from within `fc_timeout`),
/// decides whether a MAVLink message is for it (its system and component, or broadcast), and
/// routes each message to the service that handles it: [`ParamService`], [`ConnectService`],
/// or [`StateService`]. When the flight controller appears or comes back after a timeout, every
/// service is told; state is taken, and the heartbeat and state are sent, only while it is
/// alive, so the ground station sees link loss and never stale state.
#[derive(Debug)]
pub(crate) struct Bridge {
    system_id: u8,
    component_id: u8,
    fc_timeout: Duration,
    fc: Option<FcStatus>,
    params: ParamService,
    connect: ConnectService,
    state: StateService,
}

impl Bridge {
    pub(crate) fn new(config: BridgeConfig) -> Self {
        Self {
            system_id: config.system_id,
            component_id: config.component_id,
            fc_timeout: config.fc_timeout,
            fc: None,
            params: ParamService::default(),
            connect: ConnectService::new(config.vehicle_type, config.heartbeat_interval),
            state: StateService::new(config.earth, config.state_interval, config.slow_interval),
        }
    }

    /// The header MAVLink goes out with: the flight controller's system and component. The
    /// sequence is the connection's to set.
    pub(crate) fn header(&self) -> MavHeader {
        MavHeader {
            system_id: self.system_id,
            component_id: self.component_id,
            sequence: 0,
        }
    }

    /// The flight controller's latest status, if it was heard from within `fc_timeout`.
    fn alive_fc(&self, now: Instant) -> Option<FcStatus> {
        self.fc
            .filter(|fc| now.saturating_duration_since(fc.last_seen) < self.fc_timeout)
    }

    /// Handles one message from the flight controller, received at `now`.
    pub(crate) fn on_downlink(&mut self, msg: &Downlink, now: Instant, out: &mut Outbox) {
        match msg {
            Downlink::Heartbeat { armed, mode, .. } => {
                if self.alive_fc(now).is_none() {
                    self.params.fc_appeared(now, out);
                    self.connect.fc_appeared(now, out);
                    self.state.fc_appeared(now, out);
                }
                self.fc = Some(FcStatus {
                    last_seen: now,
                    armed: *armed,
                    mode: *mode,
                });
            }
            Downlink::Param(param) => self.params.on_param(param, out),
            Downlink::State(state) => {
                if let Some(fc) = self.alive_fc(now) {
                    self.state.on_state(state, fc.armed);
                }
            }
            Downlink::Record(record) if self.alive_fc(now).is_some() => {
                self.state.on_record(record);
            }
            // Status text has no MAVLink side yet.
            _ => {}
        }
    }

    /// Handles one MAVLink message from the ground station, sent by `header`'s system and
    /// component. Only a message addressed to the flight controller reaches a service.
    pub(crate) fn on_mavlink(&mut self, header: &MavHeader, msg: &MavMessage, out: &mut Outbox) {
        if !target(msg).is_some_and(|(system, component)| self.addressed(system, component)) {
            return;
        }
        match msg {
            MavMessage::PARAM_REQUEST_LIST(_) => self.params.on_request_list(out),
            MavMessage::PARAM_REQUEST_READ(data) => self.params.on_request_read(data, out),
            MavMessage::PARAM_SET(data) => self.params.on_set(data, out),
            MavMessage::MISSION_REQUEST_LIST(data) => {
                self.connect.on_mission_request_list(header, data, out);
            }
            MavMessage::COMMAND_LONG(data) => self.connect.on_command_long(header, data, out),
            _ => {}
        }
    }

    /// Sends what is due at `now`, while the flight controller is alive: the heartbeat and state.
    pub(crate) fn poll(&mut self, now: Instant, out: &mut Outbox) {
        if let Some(fc) = self.alive_fc(now) {
            self.connect.poll(now, fc.armed, fc.mode, out);
            self.state.poll(now, out);
        }
    }

    /// `target_system` / `target_component` select the flight controller (0 is broadcast).
    fn addressed(&self, target_system: u8, target_component: u8) -> bool {
        (target_system == 0 || target_system == self.system_id)
            && (target_component == 0 || target_component == self.component_id)
    }
}

/// The system and component a message the bridge routes is addressed to; `None` for every
/// other message, so a newly routed message type is ignored until it is listed here.
fn target(msg: &MavMessage) -> Option<(u8, u8)> {
    match msg {
        MavMessage::PARAM_REQUEST_LIST(d) => Some((d.target_system, d.target_component)),
        MavMessage::PARAM_REQUEST_READ(d) => Some((d.target_system, d.target_component)),
        MavMessage::PARAM_SET(d) => Some((d.target_system, d.target_component)),
        MavMessage::MISSION_REQUEST_LIST(d) => Some((d.target_system, d.target_component)),
        MavMessage::COMMAND_LONG(d) => Some((d.target_system, d.target_component)),
        _ => None,
    }
}
