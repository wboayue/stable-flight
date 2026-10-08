# stable-flight-sitl

Simulator-independent software-in-the-loop harness for the Stable Flight flight controller: commanders, telemetry, and the real-time runner. Simulator backends are separate crates, starting with [`stable-flight-sitl-realflight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl-realflight). Part of the [Stable Flight](https://github.com/wboayue/stable-flight) workspace; see the [project README](https://github.com/wboayue/stable-flight#readme) for simulator setup and the runnable examples, and [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md) for design rationale.

## Components

The RealFlight examples (in the backend crate) wire the same six-component pipeline and are copy-paste starting points for your own setup. Let's walk through the components a `FlightLoop` needs: a **Commander**, **Vehicle**, **Guidance**, **Stabilizer**, **Mixer**, and **TelemetryPublisher**. The traits and data types come from the [`stable-flight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight) core crate; this crate provides the commanders, telemetry, and the real-time runner.

### Commander

Commanders generate Rate, Attitude, or Position control commands processed by guidance. Commands may come from input devices, scripts, or external systems. Currently, commanders are available to support scripting, Spektrum controllers, and Extreme3D joysticks. You can support new input devices by implementing the Commander trait.

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

`SpektrumCommander::new()` likewise returns a `Result`; it doesn't expose configuration options.

### Vehicle

Vehicles retrieve state estimates and actuate motors and control surfaces. Each simulator backend is its own crate that builds on the core and never depends on this crate (except for its examples); RealFlight's is [`stable-flight-sitl-realflight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl-realflight), whose README shows how to configure it. To support another simulator, create a `stable-flight-sitl-<simulator>` crate that implements `StateEstimator` (read state) and `Actuator` (send commands) for a type that talks to it; `Vehicle` is implemented automatically for any type with both. It has to meet the frame and timing contracts ([architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#simulator-backends)) and cover the core's conformance scenarios (`stable_flight::conformance_tests!`, `testing` feature).

### Guidance, Stabilizer, and Mixer

Guidance (commands → attitude/rate setpoints, including waypoint navigation), the stabilizer (cascaded PID control), and the mixer (thrust/moments → motor and surface commands) live in the [`stable-flight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight) core crate so the same code can run on hardware. See its README for configuration; the RealFlight examples use `CopterGuidance`, `CopterStabilizer`, and `QuadCopterMixer`.

### TelemetryPublisher

TelemetryPublishers publish the system state to external systems. There is a publisher implemented for MAVLink messages:

```rust
// Telemetry via MAVLink protocol
let telemetry = MavlinkPublisher::connect(MavlinkConfiguration::default())?;
```

The default configuration sends to QGroundControl's default port and places the local origin at a fixed latitude/longitude (`DEFAULT_ORIGIN`) so the vehicle shows on the map; set `origin` to move it, or to `None` to stop sending global position.

### Running the Loop

With all components configured, assemble a core `FlightLoop` and drive it with this crate's `RealTimeRunner`, stopping on a `ShutdownSignal`:

```rust
let mut flight_loop = FlightLoop::new(commander, vehicle, guidance, stabilizer, mixer, telemetry);

// Ctrl-C (SIGINT) requests a graceful stop between ticks.
let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");

// Prime the loop, then tick at 250 Hz until shutdown.
let stats = RealTimeRunner::new(250.0)
    .expect("valid loop frequency")
    .run(&mut flight_loop, &shutdown);
```

`run` returns `RunStats` (ticks, elapsed). `ShutdownSignal::new()` gives an unconnected flag you can `trigger()` yourself, e.g. in tests. Pacing and shutdown behaviour are described in [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#real-time-loop-management).
