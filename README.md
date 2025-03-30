# Stable Flight SITL

## Project Overview

Stable Flight SITL provides the foundation for a Rust-based flight controller designed for VTOL drones. It utilizes Software-in-the-Loop (SITL) simulation to safely refine flight control logic before real-world deployment. My goal is to create an easily extendable, robust, and approachable flight controller leveraging [Rust's advantages](https://wboayue.com/posts/flight-controller-why-rust/) in systems programming.

See the following document for an [architecture overview](https://github.com/wboayue/stable-flight-sitl/blob/main/architecture.md).

Starting with simulation allowed me to quickly iterate on flight stabilizer logic and test different airframes without worrying about building hardware. Simulated crashes saved significant time and money compared to real-world testing.

This project should be particularly interesting to those wanting to learn how a flight controller works, experiment with custom control logic or aircraft models, or even script aerobatic maneuvers.

## Simulator Choice

Currently, my flight controller supports software-in-the-loop simulation with the [RealFlight](https://www.realflight.com) simulator. It is designed to be easily extended to support other flight simulators as well. I chose RealFlight for its realistic physics model, its API for external drone control, its wide selection of RC aircraft, and its ability to define custom models. Although RealFlight is only available on Windows, I have successfully run it on my Mac using Parallels. While RealFlight powers the current setup, the controller is designed to easily support simulators like X-Plane in the future.

## Test Models

The flight controller logic was initially developed using the following models:

* [QuadcopterX](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX): Quadcopters provide a solid foundation, widely referenced in control theory for mastering core principles.
* [eXplora VTOL](https://github.com/robustini/eXploraVTOL/tree/main/simulation): Tail-sitters are among the most challenging frames to control, requiring careful handling of transitions between flight modes (copter ↔ plane) and orientations (vertical ↔ horizontal).

The ArduPilot repository also provides a helpful [collection of models](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models).

## Getting Involved

Feel free to open an issue, submit a pull request, or reach out with questions or suggestions.
