use std::thread;
use std::time::{Duration, Instant};

use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};
use stable_flight_sitl::{TelemetryPublisher, VehicleState}; // Import common MAVLink messages

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let telemetry_configuration = MavlinkConfiguration {};

    let mut publisher = MavlinkPublisher::new(telemetry_configuration);

    // Simulated flight loop

    let dt = Duration::from_millis(100); // 10Hz update rate

    loop {
        let start = Instant::now();

        let state = VehicleState::default(); // Simulated vehicle state
        publisher.send_telemetry(dt, &state);

        // Sleep for 100ms (10Hz update rate)
        thread::sleep(dt - start.elapsed());
    }
}

// Helper function to get system time in milliseconds
fn get_time_ms() -> u32 {
    use std::time::SystemTime;
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_millis() as u32
}

// fn send_heartbeat<T : mavlink::Message>(
//     conn: &mut Box<dyn MavConnection<T>>,
//     header: MavHeader
// ) -> Result<(), Box<dyn std::error::Error>> {
//     let heartbeat = HEARTBEAT_DATA {
//         custom_mode: 0,
//         mavtype: MavType::MAV_TYPE_QUADROTOR,
//         autopilot: MavAutopilot::MAV_AUTOPILOT_PX4,
//         base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
//         system_status: MavState::MAV_STATE_ACTIVE,
//         mavlink_version: 3,
//     };
//     conn.send(&header, &MavMessage::HEARTBEAT(heartbeat))?;
//     Ok(())
// }

// Add these dependencies to your Cargo.toml:
// [dependencies]
// mavlink = "0.9"
// Add this if you get build errors:
// [build-dependencies]
// mavlink = "0.9"
