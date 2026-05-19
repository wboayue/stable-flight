# Stable Flight SITL

## Overview

Stable Flight SITL provides the foundation for a Rust-powered flight controller designed for VTOL drones. It utilizes Software-in-the-Loop (SITL) simulation to safely refine flight control logic before real-world deployment. My goal is to create an easily extendable, robust, and approachable flight controller leveraging [Rust's advantages](https://wboayue.com/posts/flight-controller-why-rust/) in systems programming. See the following document for an [architecture overview](https://github.com/wboayue/stable-flight-sitl/blob/main/architecture.md).

Starting with simulation allowed me to quickly iterate on flight stabilizer logic and test different airframes without worrying about building hardware. Simulated crashes cut costs and speed up testing compared to real-world failures.

Perfect for anyone keen to explore flight controllers, tweak control logic, or script aerobatic flights.

## Simulator Choice

Currently, the flight controller supports software-in-the-loop simulation with the [RealFlight](https://www.realflight.com) simulator. RealFlight was chosen for its realistic physics model, API for external drone control, wide selection of RC aircraft, and ability to define custom aircraft models. RealFlight powers the current setup, but the design supports the addition of simulators like X-Plane in the future.

## Test Models

The flight controller logic was initially developed using the following models:

* [QuadcopterX](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX): A go-to for mastering control theory basics.
* [eXplora VTOL](https://github.com/robustini/eXploraVTOL/tree/main/simulation): A tricky tail-sitter testing smooth copter-to-plane transitions.

The ArduPilot repository also provides a helpful [collection of models](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models).

## Prerequisites

* **Rust**: Install Rust using rustup. [rustup](https://rustup.rs/)
* **RealFlight Simulator**: The flight stabilizer was developed using RealFlight Evolution but should work with older versions as well. RealFlight runs natively on Windows, but can also run on macOS using virtualization software like Parallels.

After installing RealFlight, you will need to enable **RealFlight Link**. You can find this option under:
**Settings → Physics → Quality → RealFlight Link Enabled**.

## Getting Started

Stable Flight SITL is a library crate; runnable entrypoints live in `examples/`. Pick one that matches the input you want:

```bash
# Joystick (Logitech Extreme 3D) controlling a quadcopter in RealFlight
cargo run --example copter-manual-realflight

# Scripted commands driving a quadcopter in RealFlight
cargo run --example copter-auto-realflight

# Joystick smoke test (no simulator required)
cargo run --example extreme_3d_commander

# MAVLink telemetry demo
cargo run --example publish_telemetry
```

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

For tuned gains, build the inner PID controllers explicitly (see `examples/copter-manual-realflight.rs` for a full example):

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

## Conclusion

This project provides a solid foundation, but there is still work ahead to make it fly on real hardware. Specifically, the appropriate architecture needs to be selected for running the flight stabilizer on an embedded device. I initially plan to target the Matek Wing H753. The [RTIC](https://rtic.rs/2/book/en/) and [Embassy](https://embassy.dev/) frameworks have been explored and both look promising. Additionally, until now, state data has come directly from the simulator. On real hardware, noisy sensor data needs to be  read and filtered to generate reliable state estimates. I'm excited to tackle these next steps and get a drone airborne with stable flight.

## Getting Involved

Feel free to [open an issue](https://github.com/wboayue/stable-flight-sitl/issues), submit a pull request, or reach out with questions or suggestions.
