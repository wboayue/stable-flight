# Stable Flight

## Overview

Stable Flight provides the foundation for a Rust-powered flight controller designed for VTOL drones. It utilizes Software-in-the-Loop (SITL) simulation to safely refine flight control logic before real-world deployment. My goal is to create an easily extendable, robust, and approachable flight controller leveraging [Rust's advantages](https://wboayue.com/posts/flight-controller-why-rust/) in systems programming. See the following document for an [architecture overview](architecture.md).

Starting with simulation allowed me to quickly iterate on flight stabilizer logic and test different airframes without worrying about building hardware. Simulated crashes cut costs and speed up testing compared to real-world failures.

Perfect for anyone keen to explore flight controllers, tweak control logic, or script aerobatic flights.

## Crates

The repository is a Cargo workspace:

| Crate | Purpose |
|---|---|
| [`stable-flight`](crates/stable-flight) | Flight-control core: pipeline traits, copter guidance, cascaded-PID stabilizers, mixers, and `FlightLoop` (one control tick). `no_std`, with no simulator or I/O dependencies |
| [`stable-flight-link`](crates/stable-flight-link) | Wire protocol between the flight controller and a companion computer: `Uplink`/`Downlink` messages, versioned COBS frames, and the byte-level `Link` trait. `no_std` |
| [`stable-flight-mavlink`](crates/stable-flight-mavlink) | MAVLink message builders for the bridge: pure functions from core values to MAVLink payloads |
| [`stable-flight-bridge`](crates/stable-flight-bridge) | Companion-computer binary translating the native link to MAVLink for ground stations (QGC): params and state today, commands next |
| [`stable-flight-sitl`](crates/stable-flight-sitl) | Simulator-independent software-in-the-loop harness: commanders, the real-time `RealTimeRunner` + `ShutdownSignal`, and the backend conformance kit |
| [`stable-flight-sitl-realflight`](crates/stable-flight-sitl-realflight) | RealFlight backend: `RealFlightVehicle`, tuned `airframes`, and the RealFlight examples |
| [`stable-flight-tools`](crates/stable-flight-tools) | Developer binaries (unpublished): `plot_flight` and `generate_figure_eight` |

Board firmware lives outside the workspace, one nested workspace per board:

| Board | Firmware | Status |
|---|---|---|
| [ARK FPV](https://arkelectron.com/product/ark-fpv/) (STM32H743) | [`firmware/ark-fpv`](firmware/ark-fpv) | Stub: runs `FlightLoop` at 250 Hz on RTIC 2; no sensors or motors yet |

## Supported Simulators

Each simulator backend is its own crate, so a backend's dependencies never reach the others.

| Simulator | Crate | Status |
|---|---|---|
| [RealFlight](https://www.realflight.com) | [`stable-flight-sitl-realflight`](crates/stable-flight-sitl-realflight) | Flies QuadCopterX (manual and scripted) |

RealFlight was chosen for its realistic physics model, API for external drone control, wide selection of RC aircraft, and ability to define custom aircraft models. Other simulators (Gazebo, JSBSim, X-Plane) can be added as `stable-flight-sitl-<simulator>` crates.

## Test Models

The flight controller logic was initially developed using the following models:

* [QuadcopterX](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX): A go-to for mastering control theory basics.
* [eXplora VTOL](https://github.com/robustini/eXploraVTOL/tree/main/simulation): A tricky tail-sitter testing smooth copter-to-plane transitions.

The ArduPilot repository also provides a helpful [collection of models](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models).

## Prerequisites

* **Rust**: Install Rust using rustup. [rustup](https://rustup.rs/)
* **RealFlight Simulator**: The flight stabilizer was developed using RealFlight Evolution but should work with older versions as well. RealFlight runs natively on Windows, but can also run on macOS using virtualization software like Parallels.

After installing RealFlight, enable RealFlight Link as described in the [`stable-flight-sitl-realflight` README](crates/stable-flight-sitl-realflight/README.md#setup).

## Getting Started

Runnable entrypoints are the crate examples (`crates/*/examples/`); run them from the repository root. Pick one that matches the input you want:

```bash
# The bridge: the examples send params and telemetry over the native link, and it speaks
# MAVLink to QGC (UDP 14550). Start it once; it keeps running across flights. Or: just bridge
cargo run -p stable-flight-bridge

# Joystick (Logitech Extreme 3D) controlling a quadcopter in RealFlight
# (RealFlight host defaults to 10.211.55.3:8080; override with --host). Or: just fly-manual
cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight

# Scripted figure-eight in RealFlight (script path is the first arg, --host to override host). Or: just fly-auto
cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight

# Joystick smoke test (no simulator required)
cargo run --example extreme_3d_commander

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

Each example wires the same six-component pipeline. The [`stable-flight` README](crates/stable-flight/README.md) covers the pipeline, guidance, stabilizers, and mixers; the [`stable-flight-sitl` README](crates/stable-flight-sitl/README.md) covers the commanders, telemetry, and real-time runner; the [`stable-flight-sitl-realflight` README](crates/stable-flight-sitl-realflight/README.md) covers the RealFlight vehicle and airframes.

## Conclusion

This project provides a solid foundation, but there is still work ahead to make it fly on real hardware. The first board is the ARK FPV (STM32H743) running [RTIC 2](https://rtic.rs/2/book/en/); its [firmware](firmware/ark-fpv) already runs the flight loop on target, with placeholder sensing and actuation. Until now, state data has come directly from the simulator. On real hardware, noisy sensor data needs to be read and filtered to generate reliable state estimates, and motor output, arming, and failsafes need to land ([#88](https://github.com/wboayue/stable-flight/issues/88)). I'm excited to tackle these next steps and get a drone airborne with stable flight.

## Getting Involved

Feel free to [open an issue](https://github.com/wboayue/stable-flight/issues), submit a pull request, or reach out with questions or suggestions.
