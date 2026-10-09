//! Publishes telemetry as MAVLink v2 messages, e.g. to a ground control station.
//!
//! [`MavlinkPublisher`] sequences the streams; the payloads, home, and landed state come from
//! `stable-flight-mavlink`, shared with the bridge, and each stream is paced by its own
//! drift-free timer.

use std::io;
use std::mem;
use std::time::{Duration, Instant};

use log::warn;
use mavlink::common::{MavLandedState, MavMessage, MavType};
use mavlink::{MavConnection, MavHeader, MavlinkVersion};
use thiserror::Error;

use super::throttle::ErrorThrottle;
use stable_flight::timing::Every;
use stable_flight::{TelemetryPublisher, TelemetryRecord};
use stable_flight_mavlink::{FlatEarth, HomeTracker, InvalidOrigin, LandDetector};

#[cfg(test)]
mod tests;

/// Where the local origin sits on the earth; part of [`MavlinkConfiguration`].
pub use stable_flight_mavlink::{DEFAULT_ORIGIN, GeodeticOrigin};

/// Address [`MavlinkConfiguration::default`] sends to: UDP to a local ground station.
pub const DEFAULT_ADDRESS: &str = "udpout:127.0.0.1:14550";

/// Heartbeat and the slow-changing state streams (home, landed state).
const SLOW_STREAM_INTERVAL: Duration = Duration::from_secs(1);
const STREAM_INTERVAL: Duration = Duration::from_millis(100);

/// Where and as whom a [`MavlinkPublisher`] sends.
#[derive(Debug, Clone, PartialEq)]
pub struct MavlinkConfiguration {
    /// A `mavlink::connect` address, e.g. `udpout:127.0.0.1:14550` or `tcpout:host:5760`.
    pub address: String,
    /// MAVLink system id in every message header.
    pub system_id: u8,
    /// MAVLink component id in every message header.
    pub component_id: u8,
    /// The vehicle type the heartbeat reports, e.g. `MAV_TYPE_QUADROTOR`.
    pub vehicle_type: MavType,
    /// Where local position sits on the earth; `None` sends no GLOBAL_POSITION_INT or
    /// HOME_POSITION.
    pub origin: Option<GeodeticOrigin>,
}

impl Default for MavlinkConfiguration {
    /// [`DEFAULT_ADDRESS`], system and component id 1, a quadrotor, [`DEFAULT_ORIGIN`].
    fn default() -> Self {
        MavlinkConfiguration {
            address: DEFAULT_ADDRESS.to_owned(),
            system_id: 1,
            component_id: 1,
            vehicle_type: MavType::MAV_TYPE_QUADROTOR,
            origin: Some(DEFAULT_ORIGIN),
        }
    }
}

/// Returned by [`MavlinkPublisher::connect`] and [`MavlinkPublisher::with_origin`].
#[derive(Debug, Error)]
pub enum MavlinkError {
    /// The address didn't parse or its connection couldn't be opened.
    #[error("failed to open MAVLink connection {address}")]
    Connect {
        /// The configured [`MavlinkConfiguration::address`].
        address: String,
        /// Why `mavlink::connect` failed.
        #[source]
        source: io::Error,
    },
    /// A coordinate is out of range or not finite (see [`GeodeticOrigin`]).
    #[error(transparent)]
    InvalidOrigin(#[from] InvalidOrigin),
}

/// A [`TelemetryPublisher`] that sends each tick's state as MAVLink messages: heartbeat at
/// 1 Hz; VFR_HUD, local position, global position, and attitude at 10 Hz each; home position
/// and extended system state at 1 Hz and whenever they change. Every stream sends on the
/// first `publish` it has data for, so a ground station sees the link immediately.
///
/// The heartbeat reports the arm state and the command's mode as
/// [`stable_flight_mavlink::heartbeat`] maps them.
///
/// Home is the position at the latest arming (the first finite one from it on), or before
/// the first arm, the first finite one.
/// GLOBAL_POSITION_INT and HOME_POSITION are sent only with a [`GeodeticOrigin`]
/// ([`with_origin`](Self::with_origin)) and once there is a home; GLOBAL_POSITION_INT is
/// skipped on a tick whose position isn't finite, and its `relative_alt` is measured from home.
///
/// EXTENDED_SYS_STATE's landed state is estimated from the arm state and height above home:
/// disarmed is on the ground; armed, in the air above 0.5 m, landed again below 0.2 m. It is
/// undefined while armed with no home or a non-finite altitude. Landing armed well above
/// home (a rooftop, a hill) keeps reporting in the air until disarmed.
///
/// A failed send is logged (rate-limited) and dropped; it never stops the flight loop.
///
/// # Examples
///
/// ```no_run
/// use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};
///
/// let telemetry = MavlinkPublisher::connect(MavlinkConfiguration::default())?;
/// # Ok::<(), stable_flight_sitl::telemetry::mavlink::MavlinkError>(())
/// ```
pub struct MavlinkPublisher {
    connection: Box<dyn MavConnection<MavMessage> + Send>,
    system_id: u8,
    component_id: u8,
    /// The vehicle type the heartbeat reports.
    vehicle_type: MavType,
    sequence: u8,
    start_time: Instant,
    heartbeat: Every,
    vfr_hud: Every,
    local_position: Every,
    attitude: Every,
    /// Global position stream: the projection and its timer, when an origin is set.
    global_position: Option<(FlatEarth, Every)>,
    /// HOME_POSITION timer.
    home_position: Every,
    /// Home: see [`HomeTracker`].
    home: HomeTracker,
    /// Home was latched since the last HOME_POSITION check, so send it now.
    home_changed: bool,
    /// EXTENDED_SYS_STATE timer.
    extended_sys_state: Every,
    /// Estimates the landed state from the arm state and height above home.
    land_detector: LandDetector,
    /// The latest estimate, to send EXTENDED_SYS_STATE as soon as it changes.
    landed_state: MavLandedState,
    errors: ErrorThrottle,
}

impl MavlinkPublisher {
    /// Opens `config.address` with MAVLink v2 and publishes on it.
    ///
    /// # Errors
    ///
    /// [`MavlinkError::Connect`] if the address is malformed or the connection can't be
    /// opened (e.g. a TCP peer refuses it); [`MavlinkError::InvalidOrigin`] for an
    /// out-of-range `config.origin`.
    pub fn connect(config: MavlinkConfiguration) -> Result<Self, MavlinkError> {
        let mut connection = mavlink::connect::<MavMessage>(&config.address).map_err(|source| {
            MavlinkError::Connect {
                address: config.address.clone(),
                source,
            }
        })?;
        connection.set_protocol_version(MavlinkVersion::V2);
        let publisher =
            Self::with_connection(Box::new(connection), config.system_id, config.component_id)
                .with_vehicle_type(config.vehicle_type);
        match config.origin {
            Some(origin) => publisher.with_origin(origin),
            None => Ok(publisher),
        }
    }

    /// Publishes on an already-open connection (a custom transport, or a test double),
    /// stamping headers with `system_id` and `component_id`. The connection's protocol
    /// version is left as configured. Reports a quadrotor (change it with
    /// [`with_vehicle_type`](Self::with_vehicle_type)), with no origin (add one with
    /// [`with_origin`](Self::with_origin)).
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use mavlink::{MavConnection, MavlinkVersion, common::MavMessage};
    /// use stable_flight_sitl::telemetry::mavlink::MavlinkPublisher;
    ///
    /// let mut connection = mavlink::connect::<MavMessage>("tcpout:127.0.0.1:5760")?;
    /// connection.set_protocol_version(MavlinkVersion::V2);
    /// let telemetry = MavlinkPublisher::with_connection(Box::new(connection), 42, 1);
    /// # Ok::<(), std::io::Error>(())
    /// ```
    pub fn with_connection(
        connection: Box<dyn MavConnection<MavMessage> + Send>,
        system_id: u8,
        component_id: u8,
    ) -> Self {
        MavlinkPublisher {
            connection,
            system_id,
            component_id,
            vehicle_type: MavType::MAV_TYPE_QUADROTOR,
            sequence: 0,
            start_time: Instant::now(),
            heartbeat: Every::new(SLOW_STREAM_INTERVAL),
            vfr_hud: Every::new(STREAM_INTERVAL),
            local_position: Every::new(STREAM_INTERVAL),
            attitude: Every::new(STREAM_INTERVAL),
            global_position: None,
            home_position: Every::new(SLOW_STREAM_INTERVAL),
            home: HomeTracker::default(),
            home_changed: false,
            extended_sys_state: Every::new(SLOW_STREAM_INTERVAL),
            land_detector: LandDetector::default(),
            landed_state: MavLandedState::MAV_LANDED_STATE_UNDEFINED,
            errors: ErrorThrottle::default(),
        }
    }

    /// Reports `vehicle_type` in the heartbeat instead of a quadrotor.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use mavlink::{MavlinkVersion, MavConnection, common::{MavMessage, MavType}};
    /// use stable_flight_sitl::telemetry::mavlink::MavlinkPublisher;
    ///
    /// let mut connection = mavlink::connect::<MavMessage>("udpout:127.0.0.1:14550")?;
    /// connection.set_protocol_version(MavlinkVersion::V2);
    /// let telemetry = MavlinkPublisher::with_connection(Box::new(connection), 1, 1)
    ///     .with_vehicle_type(MavType::MAV_TYPE_VTOL_TAILSITTER_QUADROTOR);
    /// # Ok::<(), std::io::Error>(())
    /// ```
    pub fn with_vehicle_type(mut self, vehicle_type: MavType) -> Self {
        self.vehicle_type = vehicle_type;
        self
    }

    /// Also sends GLOBAL_POSITION_INT and HOME_POSITION, placing local position (0, 0, 0)
    /// at `origin`.
    ///
    /// # Errors
    ///
    /// [`MavlinkError::InvalidOrigin`] if a coordinate is out of range or not finite.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use mavlink::{MavlinkVersion, MavConnection, common::MavMessage};
    /// use stable_flight_sitl::telemetry::mavlink::{DEFAULT_ORIGIN, MavlinkPublisher};
    ///
    /// let mut connection = mavlink::connect::<MavMessage>("udpout:127.0.0.1:14550")?;
    /// connection.set_protocol_version(MavlinkVersion::V2);
    /// let telemetry =
    ///     MavlinkPublisher::with_connection(Box::new(connection), 1, 1).with_origin(DEFAULT_ORIGIN)?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn with_origin(mut self, origin: GeodeticOrigin) -> Result<Self, MavlinkError> {
        self.global_position = Some((FlatEarth::new(origin)?, Every::new(STREAM_INTERVAL)));
        Ok(self)
    }

    fn send(&mut self, message: MavMessage) {
        let header = MavHeader {
            system_id: self.system_id,
            component_id: self.component_id,
            sequence: self.sequence,
        };
        self.sequence = self.sequence.wrapping_add(1);

        // A dropped telemetry frame must not take down the flight loop; log (rate-limited) and move on.
        match self.connection.send(&header, &message) {
            Ok(_) => self.errors.reset(),
            Err(e) => {
                if let Some(suppressed) = self.errors.record() {
                    warn!(
                        "Failed to send MAVLink message: {e} ({suppressed} earlier failures suppressed)"
                    );
                }
            }
        }
    }

    fn time_boot_ms(&self) -> u32 {
        self.start_time.elapsed().as_millis() as u32
    }
}

impl TelemetryPublisher for MavlinkPublisher {
    fn publish(&mut self, record: &TelemetryRecord) {
        let TelemetryRecord {
            dt,
            command,
            state,
            control,
            ..
        } = *record;
        let time_boot_ms = self.time_boot_ms();
        self.home_changed |= self.home.update(command.arm, state.position_m);
        let home_m = self.home.home_m();

        if self.heartbeat.ready(dt) {
            self.send(MavMessage::HEARTBEAT(stable_flight_mavlink::heartbeat(
                self.vehicle_type,
                command.arm,
                command.input.mode(),
            )));
        }
        if self.vfr_hud.ready(dt) {
            self.send(MavMessage::VFR_HUD(stable_flight_mavlink::vfr_hud(
                state,
                control.thrust,
            )));
        }
        if self.local_position.ready(dt) {
            self.send(MavMessage::LOCAL_POSITION_NED(
                stable_flight_mavlink::local_position_ned(state, time_boot_ms),
            ));
        }
        if let Some((earth, every)) = &mut self.global_position
            && every.ready(dt)
            && let Some(home_m) = home_m
            && let Some(data) =
                stable_flight_mavlink::global_position_int(earth, state, home_m.z, time_boot_ms)
        {
            self.send(MavMessage::GLOBAL_POSITION_INT(data));
        }
        // `|`, not `||`: the timer advances every tick.
        let home_due = self.home_position.ready(dt) | mem::take(&mut self.home_changed);
        if home_due
            && let Some((earth, _)) = &self.global_position
            && let Some(home_m) = home_m
        {
            self.send(MavMessage::HOME_POSITION(
                stable_flight_mavlink::home_position(earth, home_m),
            ));
        }
        let height_m = home_m.map_or(f32::NAN, |home_m| state.position_m.z - home_m.z);
        let landed_state = self.land_detector.update(command.arm, height_m);
        let landed_changed = mem::replace(&mut self.landed_state, landed_state) != landed_state;
        if self.extended_sys_state.ready(dt) | landed_changed {
            self.send(MavMessage::EXTENDED_SYS_STATE(
                stable_flight_mavlink::extended_sys_state(landed_state),
            ));
        }
        if self.attitude.ready(dt) {
            self.send(MavMessage::ATTITUDE(stable_flight_mavlink::attitude(
                state,
                time_boot_ms,
            )));
        }
    }
}
