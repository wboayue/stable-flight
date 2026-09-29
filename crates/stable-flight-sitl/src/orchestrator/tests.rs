use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use crate::*;

// Mock implementations for dependencies
struct MockCommander;
impl Commander for MockCommander {
    fn next_command(&mut self, _: f32, _: &VehicleState) -> Command {
        Command::Attitude(AttitudeCommand::default())
    }
}

struct MockVehicle;
impl Vehicle for MockVehicle {
    fn estimate_state(&mut self, _: Duration) -> VehicleState {
        VehicleState::default()
    }
    fn actuate(&mut self, _: &ActuatorCommand) {}
}

struct MockStabilizer;
impl Stabilizer for MockStabilizer {
    fn stabilize(&mut self, _: f32, _: &Command, _: &VehicleState) -> ControlOutput {
        ControlOutput::default()
    }

    fn prime(&mut self, state: &VehicleState) {}
}

struct MockMixer;
impl Mixer for MockMixer {
    fn mix(&self, _: &ControlOutput) -> ActuatorCommand {
        ActuatorCommand::default()
    }
}

struct MockTelemetry;
impl TelemetryPublisher for MockTelemetry {
    fn publish(&mut self, _: Duration, _: &Command, _: &VehicleState) {}
}

#[test]
fn test_flight_orchestrator_initialization() {
    let orchestrator = FlightOrchestrator::new(
        MockCommander,
        MockVehicle,
        MockStabilizer,
        MockMixer,
        MockTelemetry,
    );

    assert!(orchestrator.running.load(Ordering::SeqCst));
}

#[test]
#[should_panic(expected = "The target frequency must be positive.")]
fn test_flight_orchestrator_negative_frequency() {
    let mut orchestrator = FlightOrchestrator::new(
        MockCommander,
        MockVehicle,
        MockStabilizer,
        MockMixer,
        MockTelemetry,
    );

    orchestrator.run(-50.0);
}

#[test]
fn test_flight_orchestrator_shutdown_signal() {
    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();

    let mut orchestrator = FlightOrchestrator {
        commander: MockCommander,
        vehicle: MockVehicle,
        stabilizer: MockStabilizer,
        mixer: MockMixer,
        telemetry: MockTelemetry,
        running: running_clone,
    };

    // Simulate external shutdown signal
    orchestrator.running.store(false, Ordering::SeqCst);

    orchestrator.run(50.0);

    assert!(!running.load(Ordering::SeqCst));
}

#[test]
fn test_flight_orchestrator_single_iteration() {
    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();
    let running_checker = running.clone();

    let mut orchestrator = FlightOrchestrator {
        commander: MockCommander,
        vehicle: MockVehicle,
        stabilizer: MockStabilizer,
        mixer: MockMixer,
        telemetry: MockTelemetry,
        running: running,
    };

    // Run the orchestrator in a separate thread and terminate after one iteration
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(10));
        running_clone.store(false, Ordering::SeqCst);
    });

    orchestrator.run(1000.0);

    assert!(!running_checker.load(Ordering::SeqCst));
}

#[test]
fn test_flight_orchestrator_high_frequency() {
    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();
    let running_checker = running.clone();

    let mut orchestrator = FlightOrchestrator {
        commander: MockCommander,
        vehicle: MockVehicle,
        stabilizer: MockStabilizer,
        mixer: MockMixer,
        telemetry: MockTelemetry,
        running,
    };

    // Run at a high frequency and terminate quickly
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(50));
        running_clone.store(false, Ordering::SeqCst);
    });

    orchestrator.run(5000.0); // 5 kHz frequency

    assert!(!running_checker.load(Ordering::SeqCst));
}
