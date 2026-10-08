//! Publishes telemetry as MAVLink v2 messages, e.g. to a ground control station.
//!
//! [`MavlinkPublisher`] sequences the streams; the payloads come from pure functions in
//! `messages`, and each stream is paced by its own drift-free timer.

use std::io;
use std::time::{Duration, Instant};

use log::warn;
use mavlink::{MavConnection, MavHeader, MavlinkVersion, common::MavMessage};
use thiserror::Error;

use super::every::Every;
use super::throttle::ErrorThrottle;
use stable_flight::{TelemetryPublisher, TelemetryRecord};

mod messages;
#[cfg(test)]
mod tests;

/// Address [`MavlinkConfiguration::default`] sends to: UDP to a local ground station.
pub const DEFAULT_ADDRESS: &str = "udpout:127.0.0.1:14550";

const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(1);
const STREAM_INTERVAL: Duration = Duration::from_millis(100);

/// Where and as whom a [`MavlinkPublisher`] sends.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MavlinkConfiguration {
    /// A `mavlink::connect` address, e.g. `udpout:127.0.0.1:14550` or `tcpout:host:5760`.
    pub address: String,
    /// MAVLink system id in every message header.
    pub system_id: u8,
    /// MAVLink component id in every message header.
    pub component_id: u8,
}

impl Default for MavlinkConfiguration {
    /// [`DEFAULT_ADDRESS`], system and component id 1.
    fn default() -> Self {
        MavlinkConfiguration {
            address: DEFAULT_ADDRESS.to_owned(),
            system_id: 1,
            component_id: 1,
        }
    }
}

/// Returned by [`MavlinkPublisher::connect`].
#[derive(Debug, Error)]
pub enum MavlinkError {
    /// The address didn't parse or its connection couldn't be opened.
    #[error("failed to open MAVLink connection {address}")]
    Connect {
        address: String,
        #[source]
        source: io::Error,
    },
}

/// A [`TelemetryPublisher`] that sends each tick's state as MAVLink messages: heartbeat at
/// 1 Hz; global position, VFR_HUD, local position, and attitude at 10 Hz each.
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
    global_position: Every,
    vfr_hud: Every,
    local_position: Every,
    attitude: Every,
    errors: ErrorThrottle,
}

impl MavlinkPublisher {
    /// Opens `config.address` with MAVLink v2 and publishes on it.
    ///
    /// # Errors
    ///
    /// [`MavlinkError::Connect`] if the address is malformed or the connection can't be
    /// opened (e.g. a TCP peer refuses it).
    pub fn connect(config: MavlinkConfiguration) -> Result<Self, MavlinkError> {
        let mut connection = mavlink::connect::<MavMessage>(&config.address).map_err(|source| {
            MavlinkError::Connect {
                address: config.address.clone(),
                source,
            }
        })?;
        connection.set_protocol_version(MavlinkVersion::V2);
        Ok(Self::with_connection(config, Box::new(connection)))
    }

    /// Publishes on an already-open connection (a custom transport, or a test double). Its
    /// protocol version is left as configured; `config.address` is unused.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use mavlink::{MavConnection, MavlinkVersion, common::MavMessage};
    /// use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};
    ///
    /// let mut connection = mavlink::connect::<MavMessage>("tcpout:127.0.0.1:5760")?;
    /// connection.set_protocol_version(MavlinkVersion::V2);
    /// let config = MavlinkConfiguration { system_id: 42, ..Default::default() };
    /// let telemetry = MavlinkPublisher::with_connection(config, Box::new(connection));
    /// # Ok::<(), std::io::Error>(())
    /// ```
    pub fn with_connection(
        config: MavlinkConfiguration,
        connection: Box<dyn MavConnection<MavMessage> + Send>,
    ) -> Self {
        MavlinkPublisher {
            connection,
            system_id: config.system_id,
            component_id: config.component_id,
            sequence: 0,
            start_time: Instant::now(),
            heartbeat: Every::new(HEARTBEAT_INTERVAL),
            global_position: Every::new(STREAM_INTERVAL),
            vfr_hud: Every::new(STREAM_INTERVAL),
            local_position: Every::new(STREAM_INTERVAL),
            attitude: Every::new(STREAM_INTERVAL),
            errors: ErrorThrottle::default(),
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
            dt, state, control, ..
        } = *record;
        let time_boot_ms = self.time_boot_ms();

        if self.heartbeat.ready(dt) {
            self.send(MavMessage::HEARTBEAT(messages::heartbeat()));
        }
        if self.global_position.ready(dt) {
            self.send(MavMessage::GLOBAL_POSITION_INT(messages::global_position(
                time_boot_ms,
            )));
        }
        if self.vfr_hud.ready(dt) {
            self.send(MavMessage::VFR_HUD(messages::vfr_hud(control, state)));
        }
        if self.local_position.ready(dt) {
            self.send(MavMessage::LOCAL_POSITION_NED(
                messages::local_position_ned(state, time_boot_ms),
            ));
        }
        if self.attitude.ready(dt) {
            self.send(MavMessage::ATTITUDE(messages::attitude(
                state,
                time_boot_ms,
            )));
        }
    }
}
