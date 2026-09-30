# Stable Flight

## Overview

Stable Flight provides the foundation for a Rust-powered flight controller designed for VTOL drones. It utilizes Software-in-the-Loop (SITL) simulation to safely refine flight control logic before real-world deployment. My goal is to create an easily extendable, robust, and approachable flight controller leveraging [Rust's advantages](https://wboayue.com/posts/flight-controller-why-rust/) in systems programming. See the following document for an [architecture overview](architecture.md).

Starting with simulation allowed me to quickly iterate on flight stabilizer logic and test different airframes without worrying about building hardware. Simulated crashes cut costs and speed up testing compared to real-world failures.

Perfect for anyone keen to explore flight controllers, tweak control logic, or script aerobatic flights.

## Crates

The repository is a Cargo workspace:

| Crate | Purpose |
|---|---|
| [`stable-flight`](crates/stable-flight) | Flight-control core: pipeline traits, cascaded-PID stabilizers, mixers, and `FlightLoop` (one control tick). No simulator or I/O dependencies |
| [`stable-flight-sitl`](crates/stable-flight-sitl) | Software-in-the-loop harness: RealFlight vehicle, commanders, telemetry, the real-time `RealTimeRunner` + `ShutdownSignal`, and the runnable examples |
| [`stable-flight-tools`](crates/stable-flight-tools) | Developer binaries (unpublished): `plot_flight` and `generate_figure_eight` |

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

Runnable entrypoints are the `stable-flight-sitl` examples (`crates/stable-flight-sitl/examples/`); run them from the repository root. Pick one that matches the input you want:

```bash
# Joystick (Logitech Extreme 3D) controlling a quadcopter in RealFlight
# (RealFlight host defaults to 10.211.55.3:8080; override with --host)
cargo run --example copter-manual-realflight

# Scripted figure-eight in RealFlight (script path is the first arg, --host to override host)
cargo run --example copter-auto-realflight

# Joystick smoke test (no simulator required)
cargo run --example extreme_3d_commander

# MAVLink telemetry demo
cargo run --example publish_telemetry

# Regenerate flights/figure_eight.csv (flags like --half-width resize the path)
cargo run --bin generate_figure_eight
```

Scripted flights are CSV waypoint files under `flights/` (`x,y,altitude,cruise_speed`,
in meters and m/s); `ScriptCommander::new(path)` loads and loops them.

To inspect a scripted path without flying it, render it as a static 3D image with the
`plot_flight` binary (altitude on the vertical axis, path segments colored by cruise speed,
a ground-projected shadow for depth). It reads any waypoint CSV in the `flights/` format and
lives in the unpublished `stable-flight-tools` crate, so it never weighs down the library:

```bash
# Render to SVG or PNG — the output extension selects the format
cargo run --bin plot_flight -- flights/figure_eight.csv figure_eight.svg

# Rotate the camera with --pitch/--yaw (degrees); e.g. near top-down reads the horizontal shape
cargo run --bin plot_flight -- flights/figure_eight.csv top.png --pitch 80 --yaw 0
```

```
Usage: plot_flight [OPTIONS] <INPUT> <OUTPUT>

Arguments:
  <INPUT>   Waypoint CSV to render (columns: x,y,altitude,cruise_speed)
  <OUTPUT>  Output image path; the .svg or .png extension selects the format

Options:
      --pitch <DEG>  Camera tilt above the horizontal, in degrees (default ~28.6)
      --yaw <DEG>    Camera rotation about the vertical axis, in degrees (default ~28.6)
```

Each example wires the same five-component pipeline. The [`stable-flight` README](crates/stable-flight/README.md) covers the pipeline, stabilizers, and mixers; the [`stable-flight-sitl` README](crates/stable-flight-sitl/README.md) covers the simulator-side components and the real-time runner.

## Conclusion

This project provides a solid foundation, but there is still work ahead to make it fly on real hardware. Specifically, the appropriate architecture needs to be selected for running the flight stabilizer on an embedded device. I initially plan to target the Matek Wing H753. The [RTIC](https://rtic.rs/2/book/en/) and [Embassy](https://embassy.dev/) frameworks have been explored and both look promising. Additionally, until now, state data has come directly from the simulator. On real hardware, noisy sensor data needs to be  read and filtered to generate reliable state estimates. I'm excited to tackle these next steps and get a drone airborne with stable flight.

## Getting Involved

Feel free to [open an issue](https://github.com/wboayue/stable-flight/issues), submit a pull request, or reach out with questions or suggestions.
