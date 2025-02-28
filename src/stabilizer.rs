use std::{thread, time::Duration};

use crate::{commanders::Commander, Controller, TelemetryPublisher, Vehicle};

pub struct StabilizerConfiguration<'a> {
    pub target_frequency: f64, // 200 Hz
    pub commander: &'a dyn Commander,
    pub vehicle: &'a dyn Vehicle,
    pub controller: &'a dyn Controller,
    pub telemetry: &'a dyn TelemetryPublisher,
}

pub struct FlightStabilizer<'a> {
    configuration: StabilizerConfiguration<'a>,
}

impl FlightStabilizer<'_> {
    pub fn new(configuration: StabilizerConfiguration) -> FlightStabilizer {
        FlightStabilizer { configuration }
    }

    pub fn commander(&self) -> &dyn Commander {
        self.configuration.commander
    }

    pub fn run(&self) {
        println!(
            "Starting stabilization with target frequency: {} Hz",
            self.configuration.target_frequency
        );

        let dt = Duration::from_secs_f64(1.0 / self.configuration.target_frequency);

        loop {
            let start = std::time::Instant::now();
            let end = start + dt;

            // Get control inputs
            let command = self.configuration.commander.next(dt);
            println!("{:?} Controls: {:?}", start.elapsed(), command);

            // Get vehicle state
            let state = self.configuration.vehicle.get_state(dt);

            // Compute desired state with inputs
            let desired_state = self
                .configuration
                .controller
                .compute_desired_state(&command, &state);

            // Compute control outputs (angles, rates, etc.)
            let control_outputs = self
                .configuration
                .controller
                .control(&desired_state, &state);

            // mix control outputs
            // self.configuration.vehicle.mix_control_outputs(&control_outputs);

            // Command actuators
            self.configuration
                .vehicle
                .command_actuators(dt, &command, &control_outputs);

            // Send telemetry
            self.configuration.telemetry.send_telemetry(&state);

            thread::sleep(end - std::time::Instant::now());
        }
    }
}
