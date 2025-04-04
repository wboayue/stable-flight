//! Coordinates the activities between components of the flight control system.
//!
//! This module implements the core stabilization loop for the flight control system.
//! It is responsible for maintaining the desired state of the aircraft by coordinating
//! command generation, state feedback, control computations, and telemetry transmission.

use signal_hook::{consts::SIGINT, iterator::Signals};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Instant;
use std::{thread, time::Duration};

use log::info;

use crate::{Commander, Controller, Mixer, TelemetryPublisher, Vehicle};

/// Encapsulates the stabilization loop.
///
/// The FlightOrchestrator coordinates components of the flight control system,
/// including command generation, vehicle state estimation, control computation,
/// actuator command generation, and telemetry publishing. It continuously
/// executes a control loop at a specified target frequency.
///
/// During each loop iteration, it:
///
/// - Retrieves the next flight control command from a [Commander].
/// - Acquires the current state of the vehicle via a [Vehicle] interface.
/// - Computes control outputs for desired state using a [Controller].
/// - Mixes the control outputs into actuator commands using a [Mixer].
/// - Actuates the [Vehicle] with output from the Mixer.
/// - Publishes telemetry data via a [TelemetryPublisher].
///
/// This design leverages dependency injection to decouple the orchestrator from specific
/// implementations of command generation, vehicle state management, control logic, and telemetry.
pub struct FlightOrchestrator<
    T: Commander,
    U: Vehicle,
    V: Controller,
    W: Mixer,
    X: TelemetryPublisher,
> {
    commander: T,
    vehicle: U,
    controller: V,
    mixer: W,
    telemetry: X,
    running: Arc<AtomicBool>,
}

impl<T: Commander, U: Vehicle, V: Controller, W: Mixer, X: TelemetryPublisher>
    FlightOrchestrator<T, U, V, W, X>
{
    /// Creates and configures a new [FlightOrchestrator].
    pub fn new(
        commander: T,
        vehicle: U,
        controller: V,
        mixer: W,
        telemetry: X,
    ) -> FlightOrchestrator<T, U, V, W, X> {
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

        FlightOrchestrator {
            commander,
            vehicle,
            controller,
            mixer,
            telemetry,
            running,
        }
    }

    /// Runs the stabilization loop at a specified target frequency.
    ///
    /// The loop is designed to run at a fixed rate determined by `target_frequency` (e.g., 200 Hz).
    /// The duration `dt` is computed as the reciprocal of this frequency. After performing all operations,
    // the loop calculates the remaining time until the next cycle and sleeps accordingly.
    pub fn run(&mut self, target_frequency: f64) {
        assert!(
            target_frequency.is_sign_positive(),
            "The target frequency must be positive."
        );

        let target_duration = 1.0 / target_frequency;
        assert!(
            target_duration.is_finite(),
            "The value is not a finite number."
        );
        let target_duration = Duration::from_secs_f64(target_duration);

        // println!(
        //     "Starting stabilization with target frequency: {} Hz. Delta {:?}",
        //     self.target_frequency, target_duration
        // );

        // Start time for the loop
        let mut next_time = Instant::now();

        // Optional: For tracking actual frequency
        let mut iteration_count = 0;
        let start_time = Instant::now();
        let mut state = self.vehicle.estimate_state(target_duration);

        while self.running.load(Ordering::SeqCst) {
            // Get control inputs
            let command = self.commander.next(target_duration, &state);

            // Get vehicle state
            state = self.vehicle.estimate_state(target_duration);

            // Compute control outputs (angles, rates, etc.)
            // pass command and state to controller
            let control_outputs = self.controller.control(target_duration, &command, &state);

            // mix control outputs
            let actuator_command = self.mixer.mix(&control_outputs);

            // Command actuators
            self.vehicle.actuate(&actuator_command);

            // Send telemetry
            self.telemetry
                .send_telemetry(target_duration, &command, &state);

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
        }
    }
}
