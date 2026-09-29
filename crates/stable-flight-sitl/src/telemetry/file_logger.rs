use std::fs::{File, OpenOptions};
use std::io::{self, Write};
use std::thread::sleep;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use byteorder::{BigEndian, WriteBytesExt};
use log::warn;
use mavlink::common::*;
use mavlink::{MavHeader, MavlinkVersion};

use super::throttle::ErrorThrottle;
use crate::{Command, TelemetryPublisher, VehicleState};

/// Writes telemetry frames to `generated_telemetry.tlog`.
///
/// Write failures are logged (rate-limited) and skipped; they never interrupt the flight loop.
pub struct FileLogger {
    file: File,
    errors: ErrorThrottle,
}

impl Default for FileLogger {
    fn default() -> Self {
        Self::new()
    }
}

impl FileLogger {
    pub fn new() -> FileLogger {
        let file = OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open("generated_telemetry.tlog")
            .expect("Failed to open telemetry log file");

        FileLogger {
            file,
            errors: ErrorThrottle::default(),
        }
    }

    pub fn log(&mut self, data: &[u8]) {
        let result = self.file.write_all(data);
        self.handle_write(result);
    }

    /// Writes one `.tlog` frame (big-endian µs timestamp, then the packet) and flushes it.
    fn write_frame(&mut self, timestamp: u64, packet: &[u8]) -> io::Result<()> {
        self.file.write_u64::<BigEndian>(timestamp)?;
        self.file.write_all(packet)?;
        self.file.flush()
    }

    /// Ends the failure episode on success; reports failures through the throttle.
    /// Returns whether the write succeeded.
    fn handle_write(&mut self, result: io::Result<()>) -> bool {
        match result {
            Ok(()) => {
                self.errors.reset();
                true
            }
            Err(e) => {
                if let Some(suppressed) = self.errors.record() {
                    warn!(
                        "Failed to write telemetry log: {e} ({suppressed} earlier failures suppressed)"
                    );
                }
                false
            }
        }
    }
}

impl TelemetryPublisher for FileLogger {
    fn publish(&mut self, dt: Duration, command: &Command, telemetry: &VehicleState) {
        // Define MAVLink header (system ID and component ID)
        let header = MavHeader {
            system_id: 1,    // Simulated system ID
            component_id: 1, // Simulated component ID
            sequence: 0,     // Sequence number (will increment)
        };

        // Simulate generating 10 HEARTBEAT messages with a 1-second interval
        for i in 0..10 {
            // Create a HEARTBEAT message
            let heartbeat = HEARTBEAT_DATA {
                mavtype: MavType::MAV_TYPE_QUADROTOR,
                autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
                base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode: 0,
                system_status: MavState::MAV_STATE_ACTIVE,
                mavlink_version: 3,
            };

            let message = MavMessage::HEARTBEAT(heartbeat);

            // Create a MAVLink message frame
            // let mut frame = mavlink::MavFrame::new(header, MavMessage::HEARTBEAT(heartbeat));

            let mut frame = mavlink::MavFrame {
                header,
                msg: message,
                protocol_version: MavlinkVersion::V2,
            };

            // Increment sequence number for the next message
            frame.header.sequence = (frame.header.sequence + 1) % 255;

            let mut raw_packet = [0_u8; 1024];
            frame.ser(&mut raw_packet);

            // Get the current timestamp in microseconds since UNIX epoch
            // A clock stepped before the epoch (e.g. by NTP) records 0 rather than panicking.
            let timestamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map_or(0, |elapsed| elapsed.as_micros() as u64);

            let result = self.write_frame(timestamp, &raw_packet);
            if !self.handle_write(result) {
                return;
            }

            // Print info about the generated message
            println!(
                "[{}] Generated HEARTBEAT message, packet length: {}",
                timestamp,
                raw_packet.len()
            );

            // Simulate a delay between messages (1 second)
            sleep(Duration::from_secs(1));
        }
    }
}
