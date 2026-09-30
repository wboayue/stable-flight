use std::time::{Duration, Instant};

use log::warn;
use mavlink::{MavConnection, MavHeader, MavlinkVersion, common::*};

use super::throttle::ErrorThrottle;
use stable_flight::{ControlOutput, TelemetryPublisher, TelemetryRecord, VehicleState};

pub struct MavlinkConfiguration {
    // connection: String,
    //     system_id: 1,    // Your system ID
    //     component_id: 1, // Your component ID
}

pub struct MavlinkPublisher {
    #[expect(dead_code, reason = "holds the geodetic origin once #97 lands")]
    config: MavlinkConfiguration,
    connection: Box<dyn mavlink::MavConnection<MavMessage>>,
    sequence: u8,
    start_time: Instant,
    heartbeat_interval: Duration,
    heartbeat_bucket: Duration,
    global_position_interval: Duration,
    global_position_bucket: Duration,
    local_position_interval: Duration,
    local_position_bucket: Duration,
    attitude_interval: Duration,
    attitude_bucket: Duration,
    errors: ErrorThrottle,
}

impl MavlinkPublisher {
    pub fn new(config: MavlinkConfiguration) -> Self {
        // Establish MAVLink connection (UDP example)
        let mut connection =
            mavlink::connect("udpout:127.0.0.1:14550").expect("Failed to connect to MAVLink");

        // Set protocol version
        connection.set_protocol_version(MavlinkVersion::V2);

        MavlinkPublisher {
            config,
            connection: Box::new(connection),
            sequence: 0,
            start_time: Instant::now(),
            heartbeat_interval: Duration::from_secs_f32(1.0 / 10.0),
            heartbeat_bucket: Duration::from_secs(0),
            global_position_interval: Duration::from_secs_f32(1.0 / 10.0),
            global_position_bucket: Duration::from_secs(0),
            local_position_interval: Duration::from_secs_f32(1.0 / 10.0),
            local_position_bucket: Duration::from_secs(0),
            attitude_interval: Duration::from_secs_f32(1.0 / 10.0),
            attitude_bucket: Duration::from_secs(0),
            errors: ErrorThrottle::default(),
        }
    }

    pub fn send_message(&mut self, message: &MavMessage) {
        let header = MavHeader {
            system_id: 1,    // Your system ID
            component_id: 1, // Your component ID
            sequence: self.sequence,
        };

        self.sequence = self.sequence.wrapping_add(1);

        // A dropped telemetry frame must not take down the flight loop; log (rate-limited) and move on.
        match self.connection.send(&header, message) {
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

    pub fn send_heartbeat(&mut self) {
        let heartbeat = HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_QUADROTOR,
            autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
            base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        };

        self.send_message(&MavMessage::HEARTBEAT(heartbeat));
    }

    pub fn time_boot_ms(&self) -> u32 {
        self.start_time.elapsed().as_millis() as u32
    }

    pub fn send_local_position(&mut self, estimated_state: &VehicleState) {
        let local_position_msg = LOCAL_POSITION_NED_DATA {
            time_boot_ms: self.time_boot_ms(),
            x: estimated_state.position.x.value, // North position (meters)
            y: estimated_state.position.y.value, // East position (meters)
            z: estimated_state.position.z.value, // Down position (meters)
            vx: estimated_state.velocity.x.value, // North velocity (m/s)
            vy: estimated_state.velocity.y.value, // East velocity (m/s)
            vz: estimated_state.velocity.z.value, // Down velocity (m/s)
        };

        self.send_message(&MavMessage::LOCAL_POSITION_NED(local_position_msg));
    }

    pub fn send_global_position(&mut self, _state: &VehicleState) {
        // Hard-coded until a geodetic origin exists (#97).
        // Send GPS position at 5 Hz (every 200ms)
        let gps_msg = GLOBAL_POSITION_INT_DATA {
            time_boot_ms: self.time_boot_ms(),
            lat: 371234567,      // 37.1234567° N
            lon: -1221234567,    // 122.1234567° W
            alt: 100000,         // 100m above sea level (mm)
            relative_alt: 50000, // 50m above home (mm)
            vx: 100,             // 1 m/s north (cm/s)
            vy: 50,              // 0.5 m/s east (cm/s)
            vz: 10,              // 0.1 m/s down (cm/s)
            hdg: 9000,           // 90° heading (degrees * 100)
        };
        self.send_message(&MavMessage::GLOBAL_POSITION_INT(gps_msg));
    }

    pub fn send_attitude(&mut self, state: &VehicleState) {
        let attitude_msg = ATTITUDE_DATA {
            time_boot_ms: self.time_boot_ms(),
            roll: state.attitude.x,
            pitch: state.attitude.y,
            yaw: state.attitude.z,
            rollspeed: state.angular_velocity.x,
            pitchspeed: state.angular_velocity.y,
            yawspeed: state.angular_velocity.z,
        };

        self.send_message(&MavMessage::ATTITUDE(attitude_msg));
    }
    /// Send VFR_HUD message with telemetry data.
    ///
    /// Throttle is the stabilizer's commanded thrust, so it is reported for every command kind.
    pub fn send_vfr_hud(&mut self, control: &ControlOutput, state: &VehicleState) {
        let throttle = throttle_percent(control.thrust);

        let vfr_hud_msg = VFR_HUD_DATA {
            airspeed: state.velocity.x.value,
            groundspeed: state.velocity.y.value,
            heading: state.attitude.z as i16,
            throttle,
            climb: state.velocity.z.value,
            alt: state.position.z.value, // Altitude (meters)
        };

        self.send_message(&MavMessage::VFR_HUD(vfr_hud_msg));
    }
}

impl TelemetryPublisher for MavlinkPublisher {
    fn publish(&mut self, record: &TelemetryRecord) {
        let TelemetryRecord {
            dt, state, control, ..
        } = *record;

        self.heartbeat_bucket += dt;
        self.global_position_bucket += dt;
        self.local_position_bucket += dt;
        self.attitude_bucket += dt;

        // Send heartbeat at 1 Hz
        if self.heartbeat_bucket >= self.heartbeat_interval {
            self.heartbeat_bucket = Duration::from_secs(0);
            self.send_heartbeat();
        }

        if self.global_position_bucket >= self.global_position_interval {
            self.global_position_bucket = Duration::from_secs(0);
            self.send_global_position(state);
            self.send_vfr_hud(control, state);
        }

        if self.local_position_bucket >= self.local_position_interval {
            self.local_position_bucket = Duration::from_secs(0);
            self.send_local_position(state);
        }

        if self.attitude_bucket >= self.attitude_interval {
            self.attitude_bucket = Duration::from_secs(0);
            self.send_attitude(state);
        }
    }
}

/// Normalized thrust (0.0–1.0) as the 0–100 % throttle VFR_HUD expects; out-of-range clamps.
fn throttle_percent(thrust: f32) -> u16 {
    (thrust.clamp(0.0, 1.0) * 100.0).round() as u16
}

#[cfg(test)]
mod tests {
    use assert2::check;

    use super::*;

    #[test]
    fn maps_normalized_thrust_to_throttle_percent() {
        // (thrust, expected percent)
        let cases = [
            (0.0, 0),
            (0.5, 50),
            (0.734, 73),
            (1.0, 100),
            (-0.2, 0),
            (1.7, 100),
            (f32::NAN, 0),
        ];

        for (thrust, expected) in cases {
            check!(throttle_percent(thrust) == expected, "thrust {thrust}");
        }
    }
}
