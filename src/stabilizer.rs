use std::thread;

use crate::{Actuators, Controller, Receiver, Vehicle};

pub struct StabilizerConfiguration<'a> {
    pub target_frequency: f64, // 200 Hz
    pub receiver: &'a dyn Receiver,
    pub vehicle: &'a dyn Vehicle,
    pub controller: &'a dyn Controller,
    pub actuators: &'a dyn Actuators,
}

pub struct FlightStabilizer<'a> {
    configuration: StabilizerConfiguration<'a>,
}

impl FlightStabilizer<'_> {
    pub fn new(configuration: StabilizerConfiguration) -> FlightStabilizer {
        FlightStabilizer {
            configuration: configuration,
        }
    }

    pub fn stabilize(&self) {
        println!(
            "Starting stabilization with target frequency: {} Hz",
            self.configuration.target_frequency
        );

        loop {
            let start = std::time::Instant::now();
            let target_duration =
                std::time::Duration::from_secs_f64(1.0 / self.configuration.target_frequency);
            let end = start + target_duration;

            // Get control inputs
            let controls = self.configuration.receiver.get_controls();
            println!("{:?} Controls: {:?}", start, controls);

            // Get vehicle state
            let state = self.configuration.vehicle.get_state();

            // Compute desired state with inputs
            let desired_state = self
                .configuration
                .controller
                .compute_desired_state(&controls, &state);

            // Compute control outputs (angles, rates, etc.)
            let control_outputs = self
                .configuration
                .controller
                .compute_control_outputs(&desired_state, &state);

            // Command actuators
            self.configuration
                .actuators
                .command_actuators(&control_outputs);

            // Send telemetry

            // hit the loop rate

            // flight mode

            thread::sleep(end - std::time::Instant::now());
        }
    }
}
