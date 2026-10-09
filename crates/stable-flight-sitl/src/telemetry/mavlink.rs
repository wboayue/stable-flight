//! Publishes telemetry as MAVLink v2 messages, e.g. to a ground control station.
//!
//! [`MavlinkPublisher`] sequences the streams; the payloads come from pure functions in
//! `messages`, and each stream is paced by its own drift-free timer.

use std::io;
use std::mem;
use std::time::{Duration, Instant};

use log::warn;
use mavlink::{MavConnection, MavHeader, MavlinkVersion, common::MavMessage};
use thiserror::Error;

use super::every::Every;
use super::throttle::ErrorThrottle;
use stable_flight::{TelemetryPublisher, TelemetryRecord, Vec3};

mod messages;
#[cfg(test)]
mod tests;

use messages::FlatEarth;

/// Address [`MavlinkConfiguration::default`] sends to: UDP to a local ground station.
pub const DEFAULT_ADDRESS: &str = "udpout:127.0.0.1:14550";

/// Heartbeat and the slow-changing state streams (home).
const SLOW_STREAM_INTERVAL: Duration = Duration::from_secs(1);
const STREAM_INTERVAL: Duration = Duration::from_millis(100);

/// Where the local origin sits on the earth: [`MavlinkConfiguration::default`] places it at
/// 34°53'03.8"N 120°24'12.6"W, at sea level.
pub const DEFAULT_ORIGIN: GeodeticOrigin = GeodeticOrigin {
    latitude_deg: 34.884389,
    longitude_deg: -120.4035,
    altitude_m: 0.0,
};

/// The geodetic point that local position (0, 0, 0) maps to, for ground stations that
/// plot global position. `VehicleState` is local, so this is the only source of latitude
/// and longitude.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeodeticOrigin {
    /// Latitude, north positive; strictly between -90 and 90 (the projection degenerates at
    /// the poles).
    pub latitude_deg: f64,
    /// Longitude, east positive; -180 to 180.
    pub longitude_deg: f64,
    /// Altitude above mean sea level of the backend's altitude datum (0 for a sea-level datum).
    pub altitude_m: f64,
}

impl GeodeticOrigin {
    fn is_valid(&self) -> bool {
        self.latitude_deg.abs() < 90.0
            && self.longitude_deg.abs() <= 180.0
            && self.altitude_m.is_finite()
    }
}

/// Where and as whom a [`MavlinkPublisher`] sends.
#[derive(Debug, Clone, PartialEq)]
pub struct MavlinkConfiguration {
    /// A `mavlink::connect` address, e.g. `udpout:127.0.0.1:14550` or `tcpout:host:5760`.
    pub address: String,
    /// MAVLink system id in every message header.
    pub system_id: u8,
    /// MAVLink component id in every message header.
    pub component_id: u8,
    /// Where local position sits on the earth; `None` sends no GLOBAL_POSITION_INT or
    /// HOME_POSITION.
    pub origin: Option<GeodeticOrigin>,
}

impl Default for MavlinkConfiguration {
    /// [`DEFAULT_ADDRESS`], system and component id 1, [`DEFAULT_ORIGIN`].
    fn default() -> Self {
        MavlinkConfiguration {
            address: DEFAULT_ADDRESS.to_owned(),
            system_id: 1,
            component_id: 1,
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
    #[error("invalid geodetic origin {0:?}")]
    InvalidOrigin(GeodeticOrigin),
}

/// A [`TelemetryPublisher`] that sends each tick's state as MAVLink messages: heartbeat at
/// 1 Hz; VFR_HUD, local position, global position, and attitude at 10 Hz each; home position
/// at 1 Hz and whenever home moves. Every stream sends on the first `publish` it has data
/// for, so a ground station sees the link immediately.
///
/// The heartbeat reports the command kind as standard `base_mode` flags: rate is
/// `MANUAL_INPUT`, attitude `MANUAL_INPUT | STABILIZE`, waypoint `STABILIZE | GUIDED | AUTO`.
///
/// Home is the position at the latest arming, or before the first arm, the first finite one.
/// GLOBAL_POSITION_INT and HOME_POSITION are sent only with a [`GeodeticOrigin`]
/// ([`with_origin`](Self::with_origin)) and once there is a home; GLOBAL_POSITION_INT is
/// skipped on a tick whose position isn't finite, and its `relative_alt` is measured from home.
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
    sequence: u8,
    start_time: Instant,
    heartbeat: Every,
    vfr_hud: Every,
    local_position: Every,
    attitude: Every,
    /// Global position stream: the projection and its timer, when an origin is set.
    global_position: Option<(FlatEarth, Every)>,
    home_position: Every,
    /// Local position home is at; see [`Self::update_home`].
    home_m: Option<Vec3>,
    /// Home was latched since the last HOME_POSITION check, so send it now.
    home_changed: bool,
    /// The previous tick's arm state, to find the arming edge.
    was_armed: bool,
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
            Self::with_connection(Box::new(connection), config.system_id, config.component_id);
        match config.origin {
            Some(origin) => publisher.with_origin(origin),
            None => Ok(publisher),
        }
    }

    /// Publishes on an already-open connection (a custom transport, or a test double),
    /// stamping headers with `system_id` and `component_id`. The connection's protocol
    /// version is left as configured. No origin: add one with [`with_origin`](Self::with_origin).
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
            sequence: 0,
            start_time: Instant::now(),
            heartbeat: Every::new(SLOW_STREAM_INTERVAL),
            vfr_hud: Every::new(STREAM_INTERVAL),
            local_position: Every::new(STREAM_INTERVAL),
            attitude: Every::new(STREAM_INTERVAL),
            global_position: None,
            home_position: Every::new(SLOW_STREAM_INTERVAL),
            home_m: None,
            home_changed: false,
            was_armed: false,
            errors: ErrorThrottle::default(),
        }
    }

    /// Also sends GLOBAL_POSITION_INT and HOME_POSITION, placing local position (0, 0, 0) at `origin`.
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
        if !origin.is_valid() {
            return Err(MavlinkError::InvalidOrigin(origin));
        }
        self.global_position = Some((FlatEarth::new(origin), Every::new(STREAM_INTERVAL)));
        Ok(self)
    }

    /// Latches home at each arming (the takeoff point), or at the first finite position
    /// before the first arm. A position with a non-finite axis never becomes home.
    fn update_home(&mut self, armed: bool, position_m: Vec3) {
        let arming = armed && !self.was_armed;
        self.was_armed = armed;
        if position_m.iter().all(|c| c.is_finite()) && (arming || self.home_m.is_none()) {
            self.home_m = Some(position_m);
            self.home_changed = true;
        }
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
        self.update_home(command.arm, state.position_m);

        if self.heartbeat.ready(dt) {
            self.send(MavMessage::HEARTBEAT(messages::heartbeat(command)));
        }
        if self.vfr_hud.ready(dt) {
            self.send(MavMessage::VFR_HUD(messages::vfr_hud(control, state)));
        }
        if self.local_position.ready(dt) {
            self.send(MavMessage::LOCAL_POSITION_NED(
                messages::local_position_ned(state, time_boot_ms),
            ));
        }
        if let Some((earth, every)) = &mut self.global_position
            && every.ready(dt)
            && let Some(home_m) = self.home_m
            && let Some(data) = messages::global_position_int(earth, state, home_m.z, time_boot_ms)
        {
            self.send(MavMessage::GLOBAL_POSITION_INT(data));
        }
        // `|`, not `||`: the timer advances every tick.
        let home_due = self.home_position.ready(dt) | mem::take(&mut self.home_changed);
        if home_due
            && let Some((earth, _)) = &self.global_position
            && let Some(home_m) = self.home_m
        {
            self.send(MavMessage::HOME_POSITION(messages::home_position(
                earth, home_m,
            )));
        }
        if self.attitude.ready(dt) {
            self.send(MavMessage::ATTITUDE(messages::attitude(
                state,
                time_boot_ms,
            )));
        }
    }
}
