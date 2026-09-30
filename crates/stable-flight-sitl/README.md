# stable-flight-sitl

Simulator-independent software-in-the-loop harness for the Stable Flight flight controller: commanders, telemetry, and the real-time runner. Simulator backends are separate crates, starting with [`stable-flight-sitl-realflight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl-realflight). Part of the [Stable Flight](https://github.com/wboayue/stable-flight) workspace; see the [project README](https://github.com/wboayue/stable-flight#readme) for simulator setup and the runnable examples, and [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md) for design rationale.

## Components

The RealFlight examples (in the backend crate) wire the same five-component pipeline and are copy-paste starting points for your own setup. Let's walk through the components a `FlightLoop` needs: a **Commander**, **Vehicle**, **Stabilizer**, **Mixer**, and **TelemetryPublisher**. The traits and data types come from the [`stable-flight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight) core crate; this crate provides the simulation-side implementations and the real-time runner.

### Commander

Commanders generate Rate, Attitude, or Position control commands processed by the controller. Commands may come from input devices, scripts, or external systems. Currently, commanders are available to support scripting, Spektrum controllers, and Extreme3D joysticks. You can support new input devices by implementing the Commander trait.

The following example demonstrates creating a commander for the Extreme3D joystick:

```rust
// Default: Attitude mode
let commander = Extreme3DCommander::new()
    .expect("Failed to initialize Extreme3D commander");

// Or specify mode explicitly for Rate mode
let commander = Extreme3DCommander::with_mode(Mode::Rate)
    .expect("Failed to initialize Extreme3D commander");
```

Use `new()` for Attitude mode (default) or `with_mode()` to start in Rate mode. Mode can also be switched at runtime via D-pad buttons.

The SpektrumCommander currently doesn't expose configuration options.

### Vehicle

Vehicles retrieve state estimates and actuate motors and control surfaces. Each simulator backend is its own crate that depends only on the core; RealFlight's is [`stable-flight-sitl-realflight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl-realflight), whose README shows how to configure it. To support another simulator, create a `stable-flight-sitl-<simulator>` crate that implements `StateEstimator` (read state) and `Actuator` (send commands) for a type that talks to it; `Vehicle` is implemented automatically for any type with both. It has to meet the frame and timing contracts described in [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#simulator-backends).

### Conformance kit

The `conformance` feature adds `stable_flight_sitl::conformance`, a table of physical scenarios (level, 30° right roll, 15° nose up, heading east, climbing, yaw-right rate). A backend's tests build the simulator's native report for each scenario, run it through the backend, and check the resulting `VehicleState`:

```toml
[dev-dependencies]
stable-flight-sitl = { version = "0.1.0", features = ["conformance"] }
```

```rust
use stable_flight_sitl::conformance::ROLL_RIGHT_30;

let state = backend_state_for(&ROLL_RIGHT_30); // your mock-simulator fixture
ROLL_RIGHT_30.check(&state).unwrap(); // Err lists every field off contract
```

### Stabilizer and Mixer

The stabilizer (cascaded PID control) and mixer (thrust/moments → motor and surface commands) live in the [`stable-flight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight) core crate so the same code can run on hardware. See its README for configuration; the examples here use `CopterStabilizer` and `QuadCopterMixer`.

### TelemetryPublisher

TelemetryPublishers publish the system state to external systems. There is a publisher implemented for MAVLink messages:

```rust
// Telemetry via MAVLink protocol
let telemetry = MavlinkPublisher::new(MavlinkConfiguration {});
```

### Running the Loop

With all components configured, assemble a core `FlightLoop` and drive it with this crate's `RealTimeRunner`, stopping on a `ShutdownSignal`:

```rust
let mut flight_loop = FlightLoop::new(commander, vehicle, stabilizer, mixer, telemetry);

// Ctrl-C (SIGINT) requests a graceful stop between ticks.
let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");

// Prime the loop, then tick at 250 Hz until shutdown.
let stats = RealTimeRunner::new(250.0)
    .expect("valid loop frequency")
    .run(&mut flight_loop, &shutdown);
```

`run` returns `RunStats` (ticks, elapsed). `ShutdownSignal::new()` gives an unconnected flag you can `trigger()` yourself, e.g. in tests. Pacing and shutdown behaviour are described in [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#real-time-loop-management).
