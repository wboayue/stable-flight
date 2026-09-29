# stable-flight-sitl

Software-in-the-loop flight controller for VTOL drones, driven by the [RealFlight](https://www.realflight.com) simulator. Part of the [Stable Flight](https://github.com/wboayue/stable-flight) workspace; see the [project README](https://github.com/wboayue/stable-flight#readme) for simulator setup and the runnable examples, and [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md) for design rationale.

## Components

Each example wires the same five-component pipeline and is a copy-paste starting point for your own setup. Let's walk through the components a `FlightOrchestrator` needs: a **Commander**, **Vehicle**, **Stabilizer**, **Mixer**, and **TelemetryPublisher**.

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

Vehicles are responsible for retrieving state estimates from the vehicle and actuating its motors and control surfaces. There is currently one vehicle implementation for the RealFlight simulator. To support another simulator, you would implement the Vehicle trait and interface it with the new simulator.

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

### Stabilizer

Stabilizers calculate the difference between the vehicle's current state and the desired state, generating `ControlOutput`s that the mixer then translates into actuator commands. This component contains the core flight stabilization logic. `CopterStabilizer` targets multirotors; a VTOL stabilizer lives in `stabilizers::vtol` for airframes that transition between flight modes. This is where you would start if you want to implement custom flight control logic.

The simplest setup uses default gains:

```rust
let stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
```

For tuned gains, build the inner PID controllers explicitly (see [`examples/copter-manual-realflight.rs`](https://github.com/wboayue/stable-flight/blob/main/crates/stable-flight-sitl/examples/copter-manual-realflight.rs) for a full example):

```rust
let rates_controller = BodyRateController::with_config(&BodyRateControllerConfig {
    gains: Gains { p: 7.0, i: 0.0, d: 0.01 },
    max_integral: 100.0,
    max_angular_rates: vec3(3.0, 3.0, 3.0),
});

let attitude_controller = AttitudeController::with_config(&AttitudeControllerConfig {
    gains: Gains { p: 3.0, i: 0.0, d: 0.1 },
    max_integral: 100.0,
    max_angles: vec2(PI / 4.0, PI / 4.0),
    max_rate: vec2(3.0, 3.0),
});

let stabilizer = CopterStabilizer::new(CopterStabilizerConfig {
    rates_controller,
    attitude_controller,
    ..Default::default()
});
```

### Mixer

Mixers convert control outputs into motor commands and control surface movements:

```rust
let mixer = QuadCopterMixer::default();
```

### TelemetryPublisher

TelemetryPublishers publish the system state to external systems. There is a publisher implemented for MAVLink messages:

```rust
// Telemetry via MAVLink protocol
let telemetry = MavlinkPublisher::new(MavlinkConfiguration {});
```

### Running the Orchestrator

With all components configured, assemble and run the `FlightOrchestrator`:

```rust
let mut orchestrator = FlightOrchestrator::new(
    commander,
    vehicle,
    stabilizer,
    mixer,
    telemetry,
);

let target_frequency = 250.0; // Hz
orchestrator.run(target_frequency);
```

The orchestrator runs the control loop at the given frequency until it receives SIGINT (Unix) or Ctrl-C (Windows), then shuts down cleanly.
