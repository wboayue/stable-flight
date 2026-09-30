//! Runs the flight loop in real time until shutdown.
//!
//! [`FlightOrchestrator`] wraps a core [`FlightLoop`], which sequences one tick, and adds the
//! host concerns: wall-clock pacing at a target frequency, loop-rate metrics, and graceful
//! shutdown on SIGINT (Unix) / Ctrl-C (Windows). Each component ([`Commander`], [`Vehicle`],
//! [`Stabilizer`], [`Mixer`], and [`TelemetryPublisher`]) is injected and can be replaced
//! independently.
//!
//! # Example
//!
//! ```ignore
//! use stable_flight::mixers::QuadCopterMixer;
//! use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
//! use stable_flight_sitl::commanders::Extreme3DCommander;
//! use stable_flight_sitl::simulators::RealFlightVehicle;
//! use stable_flight_sitl::telemetry::mavlink::MavlinkPublisher;
//! use stable_flight_sitl::FlightOrchestrator;
//!
//! // Create components with appropriate configurations
//! let commander = Extreme3DCommander::default();
//! let vehicle = RealFlightVehicle::new(/* configuration */);
//! let stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
//! let mixer = QuadCopterMixer::default();
//! let telemetry = MavlinkPublisher::new(/* configuration */);
//!
//! // Create and run the orchestrator
//! let mut orchestrator = FlightOrchestrator::new(
//!     commander, vehicle, stabilizer, mixer, telemetry
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
use windows_sys::Win32::System::Console::{CTRL_BREAK_EVENT, CTRL_C_EVENT, SetConsoleCtrlHandler};

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;
use std::{thread, time::Duration};

use log::{debug, info};

use stable_flight::{Commander, FlightLoop, Mixer, Stabilizer, TelemetryPublisher, Vehicle};

#[cfg(test)]
mod tests;

/// Runs a [`FlightLoop`] at a fixed wall-clock rate until a shutdown signal arrives.
///
/// Each iteration calls [`FlightLoop::step`] (one tick; see its docs for the order), then
/// sleeps until the next period. The orchestrator also handles
/// SIGINT (Ctrl+C) to provide a graceful shutdown mechanism.
pub struct FlightOrchestrator<
    T: Commander,
    U: Vehicle,
    V: Stabilizer,
    W: Mixer,
    X: TelemetryPublisher,
> {
    flight_loop: FlightLoop<T, U, V, W, X>,
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
    /// * `stabilizer` - Computes control outputs based on commands and current state
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
        stabilizer: V,
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
                if signals.forever().next().is_some() {
                    info!("Received SIGINT, initiating graceful shutdown...");
                    running_clone.store(false, Ordering::SeqCst);
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
            flight_loop: FlightLoop::new(commander, vehicle, stabilizer, mixer, telemetry),
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
    /// 3. Prime the flight loop ([`FlightLoop::prime`])
    /// 4. Each period, run one tick ([`FlightLoop::step`]) and sleep to maintain the target frequency
    /// 5. Continue until a shutdown signal is received
    pub fn run(&mut self, target_frequency: f64) {
        assert!(
            target_frequency > 0.0,
            "The target frequency must be positive."
        );

        let target_dt = 1.0 / target_frequency as f32;
        let period = Duration::from_secs_f32(target_dt);

        // Start time for the loop
        let mut next_time = Instant::now();

        // Optional: For tracking actual frequency
        let mut iteration_count = 0;
        let start_time = Instant::now();

        self.flight_loop.prime();

        while self.running.load(Ordering::SeqCst) {
            self.flight_loop.step(period);

            iteration_count += 1;

            // Calculate when the next iteration should start
            next_time += period;

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
