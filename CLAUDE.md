# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in the Stable Flight SITL repository. All code contributions must adhere to the following Rust coding standards, prioritizing simplicity, modularity, comprehensive testing, performance, and safety-critical reliability for aerospace applications.

## Code Quality Priorities

**Target**: Low Complexity (8+/10), High Modularity (8+/10), High Testability (9+/10), High Performance (8+/10)

### 1. Complexity Management
- Keep functions under 20 lines and cyclomatic complexity under 4, unless justified for complex flight control algorithms (document rationale in comments).
- Extract magic numbers into documented constants with descriptive names (e.g., `const MAX_ROLL_RATE: f32 = 200.0; // Maximum roll rate in deg/s`).
- Prefer early returns to reduce nesting; avoid deeply nested conditionals.
- Break complex calculations (e.g., PID control, quaternion math) into well-named helper functions.
- Use clear, self-explanatory variable names (e.g., `roll_angle` instead of `r`).
- Document unavoidable complexity with inline comments explaining the "why."

### 2. Modularity & Design
- Write idiomatic Rust following the single-responsibility principle.
- Use traits for components (e.g., `Commander`, `Vehicle`, `Stabilizer`) to enable polymorphism and dependency injection.
- Prefer generics and associated types to reduce coupling (e.g., `trait Stabilizer<T: VehicleState>`).
- Organize code into focused modules (`stabilizers`, `mixers`, `orchestrator`) with minimal public APIs.
- Separate configuration, state, and behavior into distinct types (e.g., `StabilizerConfig`, `VehicleState`, `CopterStabilizer`).
- Make dependencies explicit and injectable (e.g., pass `Commander` trait objects).
- Use Rustdoc for documentation: one-line summary, parameter descriptions, return types, and examples.
- Ensure public APIs are SemVer-compliant; deprecate features explicitly with `#[deprecated]`.

### 3. Error Handling
- Use `Result<T, E>` or `Option<T>` instead of panics in library code.
- Define custom error types with `thiserror` (e.g., `#[derive(thiserror::Error)] enum StabilizerError { ... }`).
- Validate inputs early and return descriptive errors (e.g., `Err(StabilizerError::InvalidPitchAngle)`).
- Handle edge cases (e.g., sensor noise, out-of-range inputs) gracefully.
- Use the `?` operator for clean error propagation.

### 4. Performance & Concurrency
- Optimize for low-latency, real-time performance (e.g., minimize allocations in stabilization loops).
- Use `no_std` where possible for embedded systems; justify standard library usage.
- Handle concurrency explicitly: use `Arc`, `Mutex`, or `RwLock` for shared state; prefer `async/await` for non-blocking I/O (e.g., telemetry).
- Benchmark critical components (e.g., `FlightOrchestrator` loop) using `criterion`.
- Minimize copying with references and slices; use `Cow` for conditional cloning.

### 5. Unsafe Code
- Use `unsafe` only for hardware interfacing or performance-critical math (e.g., SIMD for quaternion operations).
- Justify `unsafe` usage with detailed comments on safety guarantees.
- Encapsulate `unsafe` in safe abstractions with clear invariants.
- Test `unsafe` code paths exhaustively, including boundary conditions.

### 6. Testing Excellence
- Write comprehensive tests covering normal, boundary, edge cases, and error conditions.
- Prioritize unit tests for isolated components (e.g., `CopterStabilizer`); use integration tests for system interactions (e.g., `Commander` to `Vehicle`).
- Use table-driven tests for control algorithms (e.g., PID responses).
- Apply property-based testing with `proptest` for mathematical invariants (e.g., quaternion normalization).
- Organize tests in modules with descriptive names (e.g., `tests::pid_controller`).
- Ensure test isolation: no shared state, proper cleanup.
- Provide debugging support with custom error messages.
- Use test helpers for common setup (e.g., mock `Vehicle` implementations).
- Use `#[test]`, `assert2` crate for meaningful assertions.

### 7. Documentation
- Provide Rustdoc with summaries, parameter details, return types, and examples.
- Update `README.md` with project overview, setup, usage, and example code.
- Document safety-critical assumptions (e.g., sensor accuracy, loop timing).

### 8. Code Review Checklist
Before submitting code, verify:
- Functions meet complexity budget or have justified exceptions.
- Magic numbers are named constants with documentation.
- Error handling uses `Result`/`Option`; no panics in library code.
- Public API is minimal, documented, and SemVer-compliant.
- Dependencies are explicit and mockable/injectable.
- Tests cover unit, integration, table-driven, and property-based cases.
- Test names clearly describe scenarios.
- All error paths and edge cases are tested.
- Performance-critical code is benchmarked.
- `unsafe` code is justified, encapsulated, and tested.
- Documentation includes Rustdoc, `README.md`, and examples.

## Commands

### Building and Running
```bash
# Build the project
cargo build

# Run tests
cargo test

# Generate and save coverage report
cargo tarpaulin -o html

# Run a specific example (e.g., copter-auto-realflight)
cargo run --example copter-auto-realflight

# Run with a specific commander type
cargo run -- --commander spektrum
cargo run -- --script sequence

# Benchmark critical components
cargo bench
```

## Architecture Overview

Stable Flight SITL is a Rust-powered flight controller designed for VTOL drones using Software-in-the-Loop (SITL) simulation. The architecture emphasizes modularity, testability, and real-time performance through trait-based polymorphism and dependency injection.

### Core Components
1. **FlightOrchestrator**: Coordinates the stabilization loop
   - Runs at a fixed frequency (e.g., 250Hz).
   - Sequences data flow between components.
   - Handles graceful shutdown via signals.
   - Configurable via `OrchestratorConfig` for frequency and logging.

2. **Commander**: Generates control commands
   - Trait: `Commander` with method `fn get_command(&self) -> Result<Command, CommanderError>`.
   - Implementations:
     - `Extreme3DCommander`: Logitech Extreme 3D joystick input.
     - `SpektrumCommander`: Spektrum controller input.
     - `ScriptCommander`: Predefined script commands.
   - Mockable for testing via trait objects.

3. **Vehicle**: Represents physical or simulated vehicle
   - Trait: `Vehicle` with methods for state estimation and actuation.
   - Implementation: `RealFlightVehicle` for RealFlight simulator.
   - State: `VehicleState` with position, orientation, and rates.
   - Configurable via `RealFlightConfiguration`.

4. **Stabilizer**: Implements flight control logic
   - Trait: `Stabilizer` with method `fn compute_control(&self, state: &VehicleState, command: &Command) -> Result<ControlOutput, StabilizerError>`.
   - Implementation: `CopterStabilizer` for multirotors.
   - Uses cascaded PID loops for rate, attitude, and position control.
   - Supports VTOL stabilization for hybrid aircraft.
   - Configurable via `StabilizerConfig` for PID gains.

5. **Mixer**: Converts control outputs to actuator commands
   - Trait: `Mixer` with method `fn mix(&self, control: &ControlOutput) -> Result<ActuatorCommands, MixerError>`.
   - Implementations:
     - `QuadCopterMixer`: Maps outputs to four motors in X configuration.
     - `TailsitterMixer`: Maps outputs for VTOL/tailsitter configurations.
   - Configurable via `ActuatorMapping`.

6. **TelemetryPublisher**: Publishes vehicle state data
   - Trait: `TelemetryPublisher` with method `fn publish(&self, state: &VehicleState) -> Result<(), TelemetryError>`.
   - Implementations:
     - `MavlinkPublisher`: Sends telemetry via MAVLink protocol.
     - `FileLogger`: Logs telemetry to a file.
   - Configurable via `TelemetryConfig`.

### Command Types
- **RateCommand**: Controls angular rates (roll, pitch, yaw).
- **AttitudeCommand**: Controls orientation angles.
- **WaypointCommand**: Autonomous navigation to positions.
- Defined in `Command` enum with `thiserror`-based error handling.

### Control Flow
The stabilization loop runs at a fixed frequency (e.g., 250Hz) in `FlightOrchestrator`:
1. Call `Commander::get_command` to fetch command.
2. Call `Vehicle::estimate_state` to get `VehicleState`.
3. Call `Stabilizer::compute_control` to compute `ControlOutput`.
4. Call `Mixer::mix` to generate `ActuatorCommands`.
5. Call `Vehicle::actuate` to apply commands.
6. Call `TelemetryPublisher::publish` to log state.

### Example Control Loop
```rust
fn run_loop(&self) -> Result<(), OrchestratorError> {
    let command = self.commander.get_command()?;
    let state = self.vehicle.estimate_state()?;
    let control = self.stabilizer.compute_control(&state, &command)?;
    let actuators = self.mixer.mix(&control)?;
    self.vehicle.actuate(&actuators)?;
    self.telemetry.publish(&state)?;
    Ok(())
}
```

## Testing

The project uses Rust's testing framework with comprehensive unit and integration tests. Tests are organized within the same modules as the code they test:

```
src/
  stabilizers/
    controllers/
      attitude/
        tests.rs  # Tests for attitude PID controller
      body_rate/
        tests.rs  # Tests for rate PID controller
    copter/
      tests.rs  # Tests for CopterStabilizer
  mixers/
    quad/
      tests.rs  # Tests for QuadCopterMixer
  orchestrator/
    tests.rs  # Tests for FlightOrchestrator
```

### Testing Strategies
- **Unit Tests**: Test individual components (e.g., `CopterStabilizer::compute_control`).
- **Table-Driven Tests**: Test control algorithms with multiple inputs/outputs.
  ```rust
  #[test]
  fn test_pid_controller() {
      let tests = vec![
          (0.0, 0.0, 0.0), // Input, setpoint, expected output
          (1.0, 0.0, -2.0),
      ];
      for (input, setpoint, expected) in tests {
          assert_approx_eq!(pid.compute(input, setpoint), expected);
      }
  }
  ```
- **Property-Based Tests**: Use `proptest` for invariants (e.g., quaternion normalization).
- **Integration Tests**: Test component interactions (e.g., `Commander` to `Vehicle`).
- **Benchmarks**: Measure performance of critical paths (e.g., stabilization loop).

Use the `approx` crate for floating-point comparisons and `mockall` for mock implementations of traits.

## Simulator Configuration

The flight controller supports the RealFlight simulator with "RealFlight Link" enabled. To configure:

1. Install RealFlight simulator (Windows; use virtualization on macOS).
2. Enable RealFlight Link: Settings → Physics → Quality → RealFlight Link Enabled.
3. Configure the bridge connection:
   ```rust
   use stable_flight_sitl::vehicle::{RealFlightVehicle, RealFlightConfiguration, ActuatorMapping};
   let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
       bridge: RealFlightRemoteBridge::new("10.211.55.3:8080")?,
       mapping: ActuatorMapping {
           motors: [Some(2), Some(0), Some(1), Some(3), None, None, None, None],
           ..Default::default()
       },
   })?;
   ```

### Test Models
- **QuadcopterX**: Standard quadcopter model for testing stabilization.
- **eXplora VTOL**: Tailsitter model for testing VTOL transitions.
- Additional models available in the ArduPilot repository.

## Enhanced Workflow
When contributing code:
1. Clarify requirements (e.g., control frequency, sensor types).
2. Design for testability (e.g., mockable `Commander` and `Vehicle` traits).
3. Implement with complexity and performance budgets in mind.
4. Extract reusable patterns (e.g., PID loops) into traits or generic functions.
5. Test comprehensively using unit, table-driven, property-based, and integration tests.
6. Review for complexity, modularity, test coverage, and performance.
7. Refactor if complexity or latency exceeds targets.
