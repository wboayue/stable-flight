# stable-flight-sitl

Software-in-the-loop flight controller for VTOL drones, driven by the [RealFlight](https://www.realflight.com) simulator. Part of the [Stable Flight](https://github.com/wboayue/stable-flight) workspace; see the [project README](https://github.com/wboayue/stable-flight#readme) for simulator setup and the runnable examples, and [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md) for design rationale.

## Components

Each example wires the same five-component pipeline and is a copy-paste starting point for your own setup. Let's walk through the components a `FlightLoop` needs: a **Commander**, **Vehicle**, **Stabilizer**, **Mixer**, and **TelemetryPublisher**. The traits and data types come from the [`stable-flight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight) core crate; this crate provides the simulation-side implementations and the real-time runner.

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

Vehicles are responsible for retrieving state estimates from the vehicle and actuating its motors and control surfaces. There is currently one vehicle implementation for the RealFlight simulator. To support another simulator, implement `StateEstimator` (read state) and `Actuator` (send commands) for a type that talks to it; `Vehicle` is implemented automatically for any type with both.

The following example demonstrates configuring the RealFlight vehicle:

```rust
let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
    bridge: RealFlightRemoteBridge::new("10.211.55.3:8080").expect("Failed to create bridge"),
    mapping: ActuatorMapping {
        motors: [Some(2), Some(0), Some(1), Some(3), None, None, None, None],
        ..Default::default()
    },
});
```

The RealFlightVehicle is configured with a `RealFlightLocalBridge` or `RealFlightRemoteBridge`, depending on whether the flight stabilizer is running on the same host as the simulator. The `ActuatorMapping` maps logical motors and control surfaces to specific channels for the RealFlight vehicle model.

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

`RealTimeRunner` paces ticks on the wall clock (deadlines accumulate, so a slow tick is caught up rather than shifting the schedule) and returns `RunStats` (ticks, elapsed). `ShutdownSignal::install()` wires SIGINT (Unix) or Ctrl-C/Ctrl-Break (Windows); `ShutdownSignal::new()` gives an unconnected flag you can `trigger()` yourself, e.g. in tests.
