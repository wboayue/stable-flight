# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Initial release of the flight-control core, extracted from `stable-flight-sitl`:
  the pipeline traits (`Commander`, `StateEstimator`, `Actuator`, `Vehicle`,
  `Stabilizer`, `Mixer`, `TelemetryPublisher`), their data types (`Command` and
  its variants, `VehicleState`, `ControlOutput`, `ActuatorCommand`,
  `TelemetryRecord`), the `stabilizers` module (`CopterStabilizer`, PID
  `controllers`, `Gains`), and the `mixers` module (`QuadCopterMixer`,
  placeholder `TailSitterMixer`). Earlier history of these items is in the
  `stable-flight-sitl` changelog.
- `FlightLoop`: owns the five pipeline components and runs one control tick per
  `step(dt)` (commander → estimate → stabilize → mix → actuate → telemetry), with
  `prime()` for the initial `Duration::ZERO` estimate and stabilizer priming. No
  clock, threads, or signals; runners decide when to step. Allocation-free.

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight
