use std::time::{Duration, Instant};

use mavlink::{common::*, MavHeader, MavlinkVersion};

use crate::{TelemetryPublisher, VehicleState};

pub struct MavlinkConfiguration {
    // connection: String,
    //     system_id: 1,    // Your system ID
    //     component_id: 1, // Your component ID
}

pub struct MavlinkPublisher {
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
            connection,
            sequence: 0,
            start_time: Instant::now(),
            heartbeat_interval: Duration::from_secs(1),
            heartbeat_bucket: Duration::from_secs(0),
            global_position_interval: Duration::from_secs(1),
            global_position_bucket: Duration::from_secs(0),
            local_position_interval: Duration::from_secs_f32(1.0 / 5.0),
            local_position_bucket: Duration::from_secs(0),
            attitude_interval: Duration::from_secs(1),
            attitude_bucket: Duration::from_secs(0),
        }
    }

    pub fn send_message(&mut self, message: &MavMessage) {
        let header = MavHeader {
            system_id: 1,    // Your system ID
            component_id: 1, // Your component ID
            sequence: self.sequence,
        };

        self.sequence = self.sequence.wrapping_add(1);

        self.connection
            .send(&header, message)
            .expect("Failed to send message");
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
            x: estimated_state.position.x,  // North position (meters)
            y: estimated_state.position.y,  // East position (meters)
            z: estimated_state.position.z,  // Down position (meters)
            vx: estimated_state.velocity.x, // North velocity (m/s)
            vy: estimated_state.velocity.y, // East velocity (m/s)
            vz: estimated_state.velocity.z, // Down velocity (m/s)
        };

        self.send_message(&MavMessage::LOCAL_POSITION_NED(local_position_msg));
    }

    pub fn send_global_position(&mut self, telemetry: &VehicleState) {
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

    pub fn send_attitude(&mut self, telemetry: &VehicleState) {
        let attitude_msg = ATTITUDE_DATA {
            time_boot_ms: self.time_boot_ms(),
            roll: telemetry.orientation.x,
            pitch: telemetry.orientation.y,
            yaw: telemetry.orientation.z,
            rollspeed: telemetry.angular_velocity.x,
            pitchspeed: telemetry.angular_velocity.y,
            yawspeed: telemetry.angular_velocity.z,
        };

        self.send_message(&MavMessage::ATTITUDE(attitude_msg));
    }
}

impl TelemetryPublisher for MavlinkPublisher {
    fn send_telemetry(&mut self, dt: Duration, telemetry: &VehicleState) {
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
            self.send_global_position(telemetry);
        }

        if self.local_position_bucket >= self.local_position_interval {
            self.local_position_bucket = Duration::from_secs(0);
            self.send_local_position(telemetry);
        }

        if self.attitude_bucket >= self.attitude_interval {
            self.attitude_bucket = Duration::from_secs(0);
            self.send_attitude(telemetry);
        }
    }
}
