//! Coordinates communication and execution flow between all system components.
//!
//! This module implements the core stabilization loop for the flight control system.
//! It is responsible for maintaining the desired state of the aircraft by coordinating
//! command generation, state feedback, control computations, and telemetry transmission.
//!
//! # Design Pattern
//!
//! The orchestrator follows a dependency injection pattern, allowing for flexible
//! configuration and easy testing. Each component ([Commander], [Vehicle], [Stabilizer],
//! [Mixer], and [TelemetryPublisher]) can be independently replaced with different
//! implementations to adapt to various scenarios.
//!
//! # Example
//!
//! ```ignore
//! use stable_flight_sitl::commanders::Extreme3DCommander;
//! use stable_flight_sitl::controllers::basic::BasicController;
//! use stable_flight_sitl::mixers::QuadCopterMixer;
//! use stable_flight_sitl::vehicles::RealFlightVehicle;
//! use stable_flight_sitl::telemetry::mavlink::MavlinkPublisher;
//! use stable_flight_sitl::FlightOrchestrator;
//!
//! // Create components with appropriate configurations
//! let commander = Extreme3DCommander::new();
//! let vehicle = RealFlightVehicle::new(/* configuration */);
//! let controller = BasicController::new(/* configuration */);
//! let mixer = QuadCopterMixer::default();
//! let telemetry = MavlinkPublisher::new(/* configuration */);
//!
//! // Create and run the orchestrator
//! let mut orchestrator = FlightOrchestrator::new(
//!     commander, vehicle, controller, mixer, telemetry
//! );
//!
//! // Run at 250 Hz
//! orchestrator.run(250.0);
//! ```

// Unix-specific imports
#[cfg(unix)]
use signal_hook::{consts::SIGINT, iterator::Signals};

// Windows-specific imports
#[cfg(windows)]
use windows_sys::Win32::System::Console::{SetConsoleCtrlHandler, CTRL_BREAK_EVENT, CTRL_C_EVENT};

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Instant;
use std::{thread, time::Duration};

use log::{debug, info};

use crate::{Commander, Mixer, Stabilizer, TelemetryPublisher, Vehicle};

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
/// - Computes control outputs for desired state using a [Stabilizer].
/// - Mixes the control outputs into actuator commands using a [Mixer].
/// - Actuates the [Vehicle] with output from the Mixer.
/// - Publishes telemetry data via a [TelemetryPublisher].
///
/// The orchestrator also handles system signals like SIGINT (Ctrl+C) to provide
/// a graceful shutdown mechanism.
pub struct FlightOrchestrator<
    T: Commander,
    U: Vehicle,
    V: Stabilizer,
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

impl<T: Commander, U: Vehicle, V: Stabilizer, W: Mixer, X: TelemetryPublisher>
    FlightOrchestrator<T, U, V, W, X>
{
    /// Creates and configures a new [FlightOrchestrator].
    ///
    /// # Parameters
    ///
    /// * `commander` - Component that generates flight control commands
    /// * `vehicle` - Represents the physical aircraft and provides state estimation
    /// * `controller` - Computes control outputs based on commands and current state
    /// * `mixer` - Converts abstract control outputs to concrete actuator commands
    /// * `telemetry` - Handles transmission of telemetry data
    ///
    /// # Returns
    ///
    /// A new [FlightOrchestrator] instance configured with the provided components.
    ///
    /// # Signal Handling
    ///
    /// Sets up a signal handler for SIGINT (Ctrl+C) to enable graceful shutdown.
    /// When a SIGINT is received, the `running` flag is set to false, which will
    /// cause the control loop to exit after the current iteration.
    pub fn new(
        commander: T,
        vehicle: U,
        controller: V,
        mixer: W,
        telemetry: X,
    ) -> FlightOrchestrator<T, U, V, W, X> {
        let running = Arc::new(AtomicBool::new(true));
        let running_clone = running.clone();

        // Platform-specific signal handling
        #[cfg(unix)]
        {
            thread::spawn(move || {
                let mut signals =
                    Signals::new([SIGINT]).expect("Failed to initialize signal handler");
                for _ in signals.forever() {
                    info!("Received SIGINT, initiating graceful shutdown...");
                    running_clone.store(false, Ordering::SeqCst);
                    break;
                }
            });
        }

        #[cfg(windows)]
        {
            unsafe {
                // Define a handler function for Windows console events
                extern "system" fn handler(event: u32) -> i32 {
                    if event == CTRL_C_EVENT || event == CTRL_BREAK_EVENT {
                        let running = unsafe { &*RUNNING.as_ptr() };
                        info!("Received Ctrl+C or Ctrl+Break, initiating graceful shutdown...");
                        running.store(false, Ordering::SeqCst);
                        return 1; // Handled
                    }
                    0 // Not handled
                }

                // Static variable to pass `running` to the handler
                static mut RUNNING: *const AtomicBool = std::ptr::null();
                RUNNING = running_clone.as_ref() as *const _;

                // Register the handler
                let success = SetConsoleCtrlHandler(Some(handler), 1);
                if success == 0 {
                    panic!("Failed to set Windows console control handler");
                }
            }
        }

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
    /// the loop calculates the remaining time until the next cycle and sleeps accordingly.
    ///
    /// # Parameters
    ///
    /// * `target_frequency` - The desired frequency in Hz at which the control loop should run
    ///
    /// # Panics
    ///
    /// This function will panic if:
    /// - The target frequency is not positive (must be > 0)
    /// - The calculated duration is not finite
    ///
    /// # Process Flow
    ///
    /// 1. Calculate the target duration from the frequency
    /// 2. Initialize timing variables
    /// 3. Begin the main control loop:
    ///    1. Get the next command from the commander
    ///    2. Get the current vehicle state
    ///    3. Compute control outputs using the controller
    ///    4. Mix control outputs to actuator commands
    ///    5. Apply actuator commands to the vehicle
    ///    6. Publish telemetry
    ///    7. Sleep to maintain the target frequency
    /// 4. Continue until a shutdown signal is received
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
            let control_outputs = self.controller.stabilize(target_duration, &command, &state);

            // Mix control outputs
            let actuator_command = self.mixer.mix(&control_outputs);

            // Command actuators
            self.vehicle.actuate(&actuator_command);

            // Send telemetry
            self.telemetry
                .send_telemetry(target_duration, &command, &state);

            iteration_count += 1;

            // Calculate when the next iteration should start
            next_time += target_duration;

            // Calculate how much time we need to sleep to hit the target
            let sleep_time = next_time
                .checked_duration_since(Instant::now())
                .unwrap_or(Duration::ZERO);

            if iteration_count % 100 == 0 {
                debug!(
                    "Iteration {}: frequency {:.2} Hz",
                    iteration_count,
                    iteration_count as f64 / start_time.elapsed().as_secs_f64()
                );
            }

            // Sleep to maintain the target frequency
            thread::sleep(sleep_time);
        }

        // At this point we've exited the loop due to running being set to false
        info!("Flight orchestrator shutting down gracefully");

        // Calculate actual frequency achieved (optional metrics)
        let elapsed = start_time.elapsed();
        if iteration_count > 0 && !elapsed.is_zero() {
            let actual_frequency = iteration_count as f64 / elapsed.as_secs_f64();
            info!(
                "Control loop ran for {} iterations at an average frequency of {:.2} Hz",
                iteration_count, actual_frequency
            );
        }
    }
}
