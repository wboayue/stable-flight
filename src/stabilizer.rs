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

use signal_hook::{consts::SIGINT, iterator::Signals};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Instant;
use std::{thread, time::Duration};

use log::info;

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
    running: Arc<AtomicBool>,
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
        let running = Arc::new(AtomicBool::new(true));

        // Set up signal handling in a separate thread
        let r = running.clone();
        std::thread::spawn(move || {
            let mut signals = Signals::new([SIGINT]).unwrap();
            for _ in signals.forever() {
                info!("Received shutdown signal, initiating graceful shutdown...");
                r.store(false, Ordering::SeqCst);
                break;
            }
        });

        FlightStabilizer {
            target_frequency,
            commander,
            vehicle,
            controller,
            mixer,
            telemetry,
            running,
        }
    }

    pub fn run(&mut self) {
        let target_duration = Duration::from_secs_f64(1.0 / self.target_frequency);

        println!(
            "Starting stabilization with target frequency: {} Hz. Delta {:?}",
            self.target_frequency, target_duration
        );

        // Start time for the loop
        let mut next_time = Instant::now();

        // Optional: For tracking actual frequency
        let mut iteration_count = 0;
        let start_time = Instant::now();

        while self.running.load(Ordering::SeqCst) {
            // Get control inputs
            let command = self.commander.next(target_duration);
            info!("controls: {:?}", command);

            // Get vehicle state
            let state = self.vehicle.estimate_state(target_duration);

            // Compute control outputs (angles, rates, etc.)
            // pass command and state to controller
            let control_outputs = self.controller.control(target_duration, &command, &state);

            // mix control outputs
            let actuator_command = self.mixer.mix(&control_outputs);

            // Command actuators
            self.vehicle.actuate(actuator_command);

            // Send telemetry
            self.telemetry.send_telemetry(target_duration, &state);

            // Increment iteration counter
            iteration_count += 1;

            // Calculate when the next iteration should start
            next_time += target_duration;

            // Calculate how much time we need to sleep to hit the target
            let sleep_time = next_time
                .checked_duration_since(Instant::now())
                .unwrap_or(Duration::ZERO);

            // Sleep to maintain the target frequency
            thread::sleep(sleep_time);

            // Optional: Print actual frequency every 200 iterations
            if iteration_count % 200 == 0 {
                let elapsed = start_time.elapsed().as_secs_f64();
                let actual_freq = iteration_count as f64 / elapsed;
                println!("Actual frequency: {:.2} Hz", actual_freq);
                println!("State: {:?}", state);
            }
        }
    }
}
