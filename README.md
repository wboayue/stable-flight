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

Out of the box, the flight stabilizer is configured to control a quadcopter via the Spektrum controller or scripted commands.

You can run the stabilizer locally alongside the simulator or remotely using a local proxy.

To run with a Spektrum controller, use the following command:

```bash
stable-flight-sitl --commander spectrum
```

To run a scripted sequence, use this command:

```bash
stable-flight-sitl --script spectrum
```

These commands provide a quick starting point, but the real strength of the system lies in its extensibility. The flight stabilizer is configured in `main.rs`, or you can create your own binary for customized setups.

Let's dive into configuring a `FlightStabilizer`. The stabilizer depends on the following components: a **Commander**, **Vehicle**, **Controller**, **Mixer**, and **TelemetryPublisher**.

### Commander

Commanders generate Rate, Attitude, or Position control commands processed by the controller. Commands may come from input devices, scripts, or external systems. Currently, commanders are available to support scripting, Spektrum controllers, and Extreme3D joysticks. You can support new input devices by implementing the Commander trait.

The following example demonstrates creating a commander for the Extreme3D joystick:

```rust
// Configure joystick commander
let commander = Extreme3DCommander::new(Mode::Attitude)
    .expect("Failed to initialize Extreme3D commander");
```

Pick the commander `Mode` (`Attitude`, `Rate`, etc.) that matches the control inputs you want to emit.

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

### Controller

Controllers calculate the difference between the vehicle's current state and the desired state, generating ActuatorCommand outputs to move the vehicle toward the desired state. This component contains the core flight stabilization logic. There are currently two controller implementations: one for airframes that don't change modes, and another for airframes like VTOLs that transition between flight modes. This is where you would start if you want to implement custom flight control logic.

The following example demonstrates configuring the basic controller with PID gains:

```rust
let controller = BasicController::new(&BasicControllerConfig {
    rate_gains: Gains {
        p: 1.0,
        i: 0.0,
        d: 0.0,
    },
    angle_gains: Gains {
        p: 2.0,
        i: 0.2,
        d: 0.1,
    },
    altitude_gains: Gains {
        p: 3.0,
        i: 0.3,
        d: 0.15,
    },
    lateral_gains: Gains {
        p: 1.5,
        i: 0.15,
        d: 0.075,
    },
    yaw_gains: Gains {
        p: 1.2,
        i: 0.12,
        d: 0.06,
    },
    thrust_curve_exponent: 2.0,
});
```

This configuration sets gains for individual PID controllers.

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

### Running the Stabilizer

With dependencies created and configured, you can create and run a flight stabilizer:

```rust
// Assemble the flight stabilizer with all components
let mut stabilizer = FlightStabilizer::new(
    target_frequency,
    commander,
    vehicle,
    controller,
    mixer,
    telemetry,
);

// Start the control loop
stabilizer.run();
```

## Conclusion

This project provides a solid foundation, but there is still work ahead to make it fly on real hardware. Specifically, the appropriate architecture needs to be selected for running the flight stabilizer on an embedded device. I initially plan to target the Matek Wing H753. The [RTIC](https://rtic.rs/2/book/en/) and [Embassy](https://embassy.dev/) frameworks have been explored and both look promising. Additionally, until now, state data has come directly from the simulator. On real hardware, noisy sensor data needs to be  read and filtered to generate reliable state estimates. I'm excited to tackle these next steps and get a drone airborne with stable flight.

## Getting Involved

Feel free to [open an issue](https://github.com/wboayue/stable-flight-sitl/issues), submit a pull request, or reach out with questions or suggestions.
