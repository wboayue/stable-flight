//! # Flight Stabilizer Module
//!
//! This module implements the core stabilization loop for a flight control system.
//! It is responsible for maintaining the desired state of the aircraft by coordinating
//! command generation, state feedback, control computations, and telemetry transmission.
//!
//! ## Overview
//!
//! The Flight Stabilizer continuously executes a control loop at a specified target frequency.
//! During each loop iteration, it:
//!
//! - Retrieves the next flight control command from a `Commander`.
//! - Acquires the current state of the vehicle via a `Vehicle` interface.
//! - Computes the desired state and control outputs using a `Controller`.
//! - Commands the vehicle's actuators with the calculated outputs.
//! - Sends telemetry data through a `TelemetryPublisher`.
//!
//! This design leverages dependency injection to decouple the stabilizer from specific
//! implementations of command generation, vehicle state management, control logic, and telemetry.
//!
//! ## Components
//!
//! - **StabilizerConfiguration**: A configuration struct that aggregates the following:
//!   - `target_frequency`: The desired update rate (in Hz) for the control loop.
//!   - References to objects implementing the `Commander`, `Vehicle`, `Controller`, and `TelemetryPublisher` traits.
//!
//! - **FlightStabilizer**: The main struct that encapsulates the stabilization loop.
//!   It uses the configuration to execute the following steps repeatedly:
//!   1. Calculate the loop time slice (`dt`) based on the target frequency.
//!   2. Obtain control inputs from the `Commander`.
//!   3. Retrieve the current vehicle state.
//!   4. Compute the desired state and control outputs using the `Controller`.
//!   5. Command the vehicle actuators.
//!   6. Publish telemetry data.
//!   7. Sleep for the remainder of the loop duration to maintain a consistent update rate.
//!
//! ## Timing and Loop Considerations
//!
//! The loop is designed to run at a fixed rate determined by `target_frequency` (e.g., 200 Hz).
//! The duration `dt` is computed as the reciprocal of this frequency. After performing all operations,
//! the loop calculates the remaining time until the next cycle and sleeps accordingly.
use std::{thread, time::Duration};

use crate::{commanders::Commander, Controller, Mixer, TelemetryPublisher, Vehicle};

pub struct FlightStabilizer<
    T: Commander,
    U: Vehicle,
    V: Controller,
    W: Mixer,
    X: TelemetryPublisher,
> {
    target_frequency: f64, // 200 Hz
    commander: T,
    vehicle: U,
    controller: V,
    mixer: W,
    telemetry: X,
}

impl<T: Commander, U: Vehicle, V: Controller, W: Mixer, X: TelemetryPublisher>
    FlightStabilizer<T, U, V, W, X>
{
    pub fn new(
        target_frequency: f64,
        commander: T,
        vehicle: U,
        controller: V,
        mixer: W,
        telemetry: X,
    ) -> FlightStabilizer<T, U, V, W, X> {
        FlightStabilizer {
            target_frequency,
            commander,
            vehicle,
            controller,
            mixer,
            telemetry,
        }
    }

    pub fn run(&self) {
        println!(
            "Starting stabilization with target frequency: {} Hz",
            self.target_frequency
        );

        let dt = Duration::from_secs_f64(1.0 / self.target_frequency);

        loop {
            let start = std::time::Instant::now();
            let end = start + dt;

            // Get control inputs
            let command = self.commander.next(dt);
            println!("{:?} Controls: {:?}", start.elapsed(), command);

            // Get vehicle state
            let state = self.vehicle.estimate_state(dt);

            // Compute control outputs (angles, rates, etc.)
            // pass command and state to controller
            let control_outputs = self.controller.control(&command, &state);

            // mix control outputs
            let actuator_command = self.mixer.mix_control_outputs(&control_outputs);

            // Command actuators
            self.vehicle.command_actuators(actuator_command);

            // Send telemetry
            self.telemetry.send_telemetry(&state);

            thread::sleep(end - std::time::Instant::now());
        }
    }
}
