use std::thread;
use std::time::{Duration, Instant};

use stable_flight::{
    ActuatorCommand, AttitudeCommand, Command, ControlInput, ControlOutput, Saturation, Setpoint,
    TelemetryPublisher, TelemetryRecord, VehicleState,
};
use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut publisher = MavlinkPublisher::connect(MavlinkConfiguration::default())?;

    // Simulated flight loop

    let dt = Duration::from_millis(100); // 10Hz update rate
    let command = Command::armed(ControlInput::Attitude(AttitudeCommand {
        throttle: 0.5,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        mode: 0, // Placeholder for mode, replace with actual mode logic
    })); // Placeholder for command, replace with actual command logic

    loop {
        let start = Instant::now();

        let state = VehicleState::default(); // Simulated vehicle state
        publisher.publish(&TelemetryRecord {
            dt,
            command: &command,
            setpoint: &Setpoint::Attitude(Default::default()),
            state: &state,
            control: &ControlOutput::default(),
            actuators: &ActuatorCommand::default(),
            saturation: Saturation::default(),
        });

        // Sleep for 100ms (10Hz update rate)
        thread::sleep(dt - start.elapsed());
    }
}
