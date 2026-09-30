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
- Backend contracts in rustdoc: `VehicleState` documents its frame contract
  (translation north/east/up, attitude NED world → FRD body, SI units, the sign of
  each angle and body rate), and `Vehicle` documents the per-tick timing contract
  every backend must meet.
- `testing::conformance` (`testing` feature): physical scenarios (level, banked,
  nose up, heading east, banked climb east, climbing, moving northeast, yaw and
  combined rates) with the `VehicleState` each must produce, `Scenario::check`
  (allocation-free `ConformanceError`), and the `conformance_tests!` macro, which
  generates one test per scenario plus a coverage check. docs.rs renders the
  `testing` feature.
- `QuadCopterMixer` derives `Debug`, `Clone`, `Copy`, and `PartialEq`.
- The crate is `no_std` (including the `testing` feature), so it builds for
  bare-metal targets such as `thumbv7em-none-eabihf`. `uom` and `robomath` are
  pulled in without default features; `libm` is a new dependency.

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
- Holding yaw in attitude mode no longer panics the flight loop after a few
  minutes. The heading-hold target grew without bound until the yaw
  controller's `[-2π, 2π]` `assert!` fired; it is now wrapped to `[-π, π)` each
  tick. `YawController::control` accepts any finite target (it already works on
  the wrapped error), and a non-finite target or estimate returns a zero rate
  without corrupting the controller state or kicking the derivative afterwards.
  A heading-hold target that is NaN (e.g. primed from a NaN estimate) re-latches
  from the next valid estimate instead of disabling heading hold.
- Out-of-range or non-finite commands and estimates no longer panic the flight
  loop (see architecture.md, "Input Policy"):
  - `QuadCopterMixer::mix` saturates thrust to [0, 1] and moments to [-1, 1] (NaN →
    0) instead of asserting; rate-mode throttle, which was never clamped, used to
    reach this assert.
  - `CopterStabilizer` saturates attitude-command throttle and yaw instead of
    asserting (roll and pitch saturate in the attitude controller).
  - `BodyRateController::control` returns zero moments on non-finite desired or
    estimated rates instead of panicking.
  - `AttitudeController::control` returns zero rates on a non-finite error.
  - `AltitudeController::control` returns hover thrust on a non-finite altitude or
    velocity error, and keeps it out of the integral. Before, a NaN estimate stuck
    there and cut the motors for good once the mixer saturated NaN thrust to 0.
  - `LateralController::control` returns level (no tilt) on a non-finite error; an
    infinite error used to command full tilt.
  - Rate-mode NaN rate commands mean "hold zero rate" (per-axis saturation);
    `clamp` let NaN through to zero every moment.
  - `AttitudeController::with_config` and `YawController::with_config` reject a
    negative or non-finite `max_integral`/`max_rate`, which would otherwise make a
    per-tick `clamp` panic.

  Both controllers skip the derivative on the first finite tick after a
  non-finite one.

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight
