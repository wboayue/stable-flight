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
  `into_parts()` returns the five components after a run.
- `commanders::WaypointFollower<W: AsRef<[WaypointCommand]>>`: a `Commander` that
  follows and loops a waypoint list by horizontal acceptance radius (default
  `DEFAULT_ACCEPTANCE_RADIUS`, 3 m; `with_acceptance_radius` to override).
  Works over an array, `&'static` slice, or `Vec`, and never allocates, so
  waypoint flight can run on firmware. `new` returns `EmptyWaypoints` for an
  empty list.
- `ComposedVehicle<E, A>`: a `Vehicle` built from a separate `StateEstimator`
  and `Actuator` (e.g. IMU/estimator + ESC driver), with `estimator()`,
  `actuator()`, and `into_parts()`.
- `impl TelemetryPublisher for (A, B)` fans each tick out to both publishers
  (nest tuples for more sinks), and `impl TelemetryPublisher for ()` is the
  no-op sink.
- `testing` feature (off by default) with allocation-free test doubles in
  `stable_flight::testing`: `NullVehicle` (fixed state, counts estimates and
  actuations, keeps the last command), `ConstantCommander`, and
  `RecordingTelemetry<N>` (ring of the last `N` ticks as `RecordedTick`s, which
  `as_record()` can replay).

### Fixed

- Waypoint flight far from the target no longer panics the flight loop. The
  lateral controller's roll/pitch request is saturated to the normalized
  [-1, 1] range (it previously passed an unbounded `accel / g`, e.g. -4.6),
  its integral term is clamped to the configured `max_integral` (it was
  unbounded), and `AttitudeController::control` saturates out-of-range or NaN
  inputs instead of `assert!`ing. Far from a waypoint the vehicle now commands
  full tilt (`max_angles`) toward it. A NaN target, position, or attitude
  estimate no longer sticks in the lateral or attitude integral.
  `LateralController::with_config` now rejects a negative or non-finite
  `max_integral` (it would otherwise panic inside the loop).

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight
