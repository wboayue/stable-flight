use std::{thread, time::Duration};

use crate::{Controller, Stick, Vehicle};

pub struct StabilizerConfiguration<'a> {
    pub target_frequency: f64, // 200 Hz
    pub stick: &'a dyn Stick,
    pub vehicle: &'a dyn Vehicle,
    pub controller: &'a dyn Controller,
}

pub struct FlightStabilizer<'a> {
    configuration: StabilizerConfiguration<'a>,
}

impl FlightStabilizer<'_> {
    pub fn new(configuration: StabilizerConfiguration) -> FlightStabilizer {
        FlightStabilizer { configuration }
    }

    pub fn stabilize(&self) {
        println!(
            "Starting stabilization with target frequency: {} Hz",
            self.configuration.target_frequency
        );

        let dt = Duration::from_secs_f64(1.0 / self.configuration.target_frequency);

        loop {
            let start = std::time::Instant::now();
            let end = start + dt;

            // Get control inputs
            let stick = self.configuration.stick.get_controls();
            println!("{:?} Controls: {:?}", start.elapsed(), stick);

            // Get vehicle state
            let state = self.configuration.vehicle.get_state(dt);

            // Compute desired state with inputs
            let desired_state = self
                .configuration
                .controller
                .compute_desired_state(&stick, &state);

            // Compute control outputs (angles, rates, etc.)
            let control_outputs = self
                .configuration
                .controller
                .compute_control_outputs(&desired_state, &state);

            // Command actuators
            self.configuration
                .vehicle
                .command_actuators(dt, &stick, &control_outputs);

            // Send telemetry

            // hit the loop rate

            // flight mode

            thread::sleep(end - std::time::Instant::now());
        }
    }
}
