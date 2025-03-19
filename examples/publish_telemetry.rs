use std::thread;
use std::time::{Duration, Instant};

use stable_flight_sitl::commanders::{AttitudeCommand, Command};
use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};
use stable_flight_sitl::{TelemetryPublisher, VehicleState}; // Import common MAVLink messages

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let telemetry_configuration = MavlinkConfiguration {};

    let mut publisher = MavlinkPublisher::new(telemetry_configuration);

    // Simulated flight loop

    let dt = Duration::from_millis(100); // 10Hz update rate
    let command = Command::Attitude(AttitudeCommand {
        throttle: 0.5,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        arm: true,
        mode: 0, // Placeholder for mode, replace with actual mode logic
    }); // Placeholder for command, replace with actual command logic

    loop {
        let start = Instant::now();

        let state = VehicleState::default(); // Simulated vehicle state
        publisher.send_telemetry(dt, &command, &state);

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
