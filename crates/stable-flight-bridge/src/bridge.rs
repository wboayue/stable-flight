//! The translation between the native link and MAVLink, with no I/O: [`Bridge`] takes the
//! messages each side sent and queues what to send in an [`Outbox`]. The caller owns the
//! transports and the clock.

use std::time::{Duration, Instant};

use mavlink::MavHeader;
use mavlink::common::*;
use stable_flight_link::stable_flight::ControlMode;
use stable_flight_link::{Control, Downlink, Uplink};
use stable_flight_mavlink::FlatEarth;

use crate::commands::{CommandService, FcView};
use crate::connect::ConnectService;
use crate::encode;
use crate::interval::Interval;
use crate::metadata::ComponentMetadata;
use crate::outbox::Outbox;
use crate::params::ParamService;
use crate::setpoints::SetpointService;
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
    /// Where the param metadata is served, for `COMPONENT_METADATA`; `None` serves none.
    pub(crate) metadata: Option<ComponentMetadata>,
    /// How long after its last message the ground station counts as gone.
    pub(crate) gcs_timeout: Duration,
    /// Interval between `Control::Heartbeat`s to the flight controller, which keep its link
    /// fresh while the ground station is alive.
    pub(crate) link_heartbeat_interval: Duration,
    /// How long an arm or disarm may go unconfirmed by the flight controller's heartbeat before
    /// it is acked `FAILED`; under QGC's 1.2 s wait.
    pub(crate) arm_ack_timeout: Duration,
    /// Speed cap, in m/s, for position targets from offboard apps, which carry none.
    pub(crate) offboard_speed_mps: f32,
}

impl Default for BridgeConfig {
    /// System and component id 1 (the autopilot, which QGC requires), a quadrotor, a 3 s
    /// timeout, 1 Hz heartbeats, no origin, state at 10 Hz, home and landed state at 1 Hz, no
    /// param metadata, a 3 s ground-station timeout, 5 Hz link heartbeats, a 1 s arm ack
    /// timeout, and position targets flown at most 5 m/s.
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
            metadata: None,
            gcs_timeout: Duration::from_secs(3),
            link_heartbeat_interval: Duration::from_millis(200),
            arm_ack_timeout: Duration::from_secs(1),
            offboard_speed_mps: 5.0,
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
/// `Bridge` tracks whether the flight controller is alive (heard from within `fc_timeout`) and
/// whether the ground station is (any message within `gcs_timeout`), decides whether a MAVLink
/// message is for the flight controller (its system and component, or broadcast), and routes
/// each message to the service that handles it: [`ParamService`], [`ConnectService`],
/// [`CommandService`], [`SetpointService`], or [`StateService`]. A `COMMAND_LONG` no service handles is acked
/// `UNSUPPORTED`, so the ground station moves on without retrying.
///
/// When the flight controller appears or comes back after a timeout, every service is told;
/// state is taken, and the heartbeat and state are sent, only while it is alive, so the ground
/// station sees link loss and never stale state. `Control::Heartbeat`s go to the flight
/// controller only while both are alive, so losing the ground station (or this bridge) lets
/// the flight controller's failsafe take over.
#[derive(Debug)]
pub(crate) struct Bridge {
    system_id: u8,
    component_id: u8,
    fc_timeout: Duration,
    fc: Option<FcStatus>,
    gcs_timeout: Duration,
    /// When the ground station was last heard from.
    gcs_last_seen: Option<Instant>,
    link_heartbeat: Interval,
    params: ParamService,
    connect: ConnectService,
    commands: CommandService,
    setpoints: SetpointService,
    state: StateService,
}

impl Bridge {
    pub(crate) fn new(config: BridgeConfig) -> Self {
        Self {
            system_id: config.system_id,
            component_id: config.component_id,
            fc_timeout: config.fc_timeout,
            fc: None,
            gcs_timeout: config.gcs_timeout,
            gcs_last_seen: None,
            link_heartbeat: Interval::new(config.link_heartbeat_interval),
            params: ParamService::default(),
            connect: ConnectService::new(
                config.vehicle_type,
                config.heartbeat_interval,
                config.metadata,
            ),
            commands: CommandService::new(config.arm_ack_timeout),
            setpoints: SetpointService::new(config.offboard_speed_mps),
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
            .filter(|fc| heard_within(Some(fc.last_seen), now, self.fc_timeout))
    }

    /// Handles one message from the flight controller, received at `now`.
    pub(crate) fn on_downlink(&mut self, msg: &Downlink, now: Instant, out: &mut Outbox) {
        let alive = self.alive_fc(now);
        match msg {
            Downlink::Heartbeat { armed, mode, .. } => {
                if alive.is_none() {
                    self.params.fc_appeared(now, out);
                    self.connect.fc_appeared(now, out);
                    self.state.fc_appeared(now, out);
                }
                self.fc = Some(FcStatus {
                    last_seen: now,
                    armed: *armed,
                    mode: *mode,
                });
                self.commands.on_fc_heartbeat(*armed, *mode, out);
            }
            Downlink::Param(param) => self.params.on_param(param, out),
            Downlink::State(state) => {
                if let Some(fc) = alive {
                    self.state.on_state(state, fc.armed, fc.mode);
                }
            }
            Downlink::Output(output) if alive.is_some() => {
                self.state.on_output(output);
            }
            Downlink::Status(text) => {
                out.to_gcs
                    .push(MavMessage::STATUSTEXT(encode::statustext(text.as_str())));
            }
            _ => {}
        }
    }

    /// Handles one MAVLink message from the ground station, sent by `header`'s system and
    /// component and received at `now`. Any message shows the ground station is alive; only one
    /// addressed to the flight controller reaches a service.
    pub(crate) fn on_mavlink(
        &mut self,
        header: &MavHeader,
        msg: &MavMessage,
        now: Instant,
        out: &mut Outbox,
    ) {
        if !self.gcs_alive(now) {
            self.link_heartbeat.start(now);
        }
        self.gcs_last_seen = Some(now);
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
            MavMessage::MANUAL_CONTROL(data) => self.setpoints.on_manual_control(data, out),
            MavMessage::SET_POSITION_TARGET_LOCAL_NED(data) => {
                self.setpoints.on_position_target(data, out);
            }
            MavMessage::SET_ATTITUDE_TARGET(data) => self.setpoints.on_attitude_target(data, out),
            MavMessage::COMMAND_LONG(data) => {
                let fc = self.alive_fc(now).map(|fc| FcView {
                    armed: fc.armed,
                    mode: fc.mode,
                    position_m: self.state.position_m(),
                });
                let handled = self.connect.on_command_long(header, data, out)
                    || self.commands.on_command_long(header, data, fc, now, out);
                if !handled {
                    let ack = encode::command_ack(
                        header,
                        data.command,
                        MavResult::MAV_RESULT_UNSUPPORTED,
                    );
                    out.to_gcs.push(MavMessage::COMMAND_ACK(ack));
                }
            }
            _ => {}
        }
    }

    /// Sends what is due at `now`: while the flight controller is alive, the heartbeat and state,
    /// and while the ground station is too, the link heartbeat; and fails an arm request the
    /// flight controller never confirmed.
    pub(crate) fn poll(&mut self, now: Instant, out: &mut Outbox) {
        self.commands.poll(now, out);
        let Some(fc) = self.alive_fc(now) else {
            return;
        };
        self.connect.poll(now, fc.armed, fc.mode, out);
        self.state.poll(now, out);
        // `&&` short-circuits: the timer only advances while the ground station is alive, and
        // restarts when it comes back.
        if self.gcs_alive(now) && self.link_heartbeat.due(now) {
            out.to_fc.push(Uplink::Control(Control::Heartbeat));
        }
    }

    /// Whether the ground station was heard from within `gcs_timeout` of `now`.
    fn gcs_alive(&self, now: Instant) -> bool {
        heard_within(self.gcs_last_seen, now, self.gcs_timeout)
    }

    /// `target_system` / `target_component` select the flight controller (0 is broadcast).
    fn addressed(&self, target_system: u8, target_component: u8) -> bool {
        (target_system == 0 || target_system == self.system_id)
            && (target_component == 0 || target_component == self.component_id)
    }
}

/// Whether a peer last heard at `last_seen` (`None`: never) counts as alive at `now`.
fn heard_within(last_seen: Option<Instant>, now: Instant, timeout: Duration) -> bool {
    last_seen.is_some_and(|seen| now.saturating_duration_since(seen) < timeout)
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
        // `MANUAL_CONTROL` names only the system.
        MavMessage::MANUAL_CONTROL(d) => Some((d.target, 0)),
        MavMessage::SET_POSITION_TARGET_LOCAL_NED(d) => Some((d.target_system, d.target_component)),
        MavMessage::SET_ATTITUDE_TARGET(d) => Some((d.target_system, d.target_component)),
        _ => None,
    }
}
