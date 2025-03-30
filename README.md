# Stable Flight SITL

## Project Overview

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



## Getting Involved

Feel free to [open an issue](https://github.com/wboayue/stable-flight-sitl/issues), submit a pull request, or reach out with questions or suggestions.
