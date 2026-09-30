# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Initial release of the flight-control core, extracted from `stable-flight-sitl`:
  the pipeline traits (`Commander`, `StateEstimator`, `Actuator`, `Vehicle`,
  `Guidance`, `Stabilizer`, `Mixer`, `TelemetryPublisher`), their data types
  (`Command` and its variants, `Setpoint` and its variants, `VehicleState`,
  `ControlOutput`, `ActuatorCommand`, `TelemetryRecord`), the `guidance` module
  (`CopterGuidance`), the `stabilizers` module (`CopterStabilizer`), the
  `controllers` module (PID controllers and `Gains`, formerly
  `stabilizers::controllers` and `stabilizers::Gains`), and the `mixers` module (`QuadCopterMixer`,
  placeholder `TailSitterMixer`). Earlier history of these items is in the
  `stable-flight-sitl` changelog.
- `FlightLoop`: owns the six pipeline components and runs one control tick per
  `step(dt)` (commander → estimate → guide → stabilize → mix → actuate →
  telemetry), with `prime()` for the initial `Duration::ZERO` estimate and
  guidance and stabilizer priming. No clock, threads, or signals; runners decide
  when to step. Allocation-free. `into_parts()` returns the six components after
  a run.
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
- `Guidance` trait (`guide(dt, &Command, &VehicleState) -> Setpoint`, `prime`) and
  `guidance::copter::CopterGuidance` / `CopterGuidanceConfig` (altitude and lateral
  gains): the outer loop, split out of `CopterStabilizer`. It flies waypoints and
  owns the attitude-mode heading-hold target. `Setpoint` is `Attitude(AttitudeSetpoint)`
  (normalized roll/pitch, absolute heading, linear thrust) or `Rate(RateSetpoint)`.
  `TelemetryRecord` and `RecordedTick` gain the tick's `setpoint`.
- `CopterGuidanceConfig::navigation_period` (default `DEFAULT_NAVIGATION_PERIOD`,
  20 ms): how often the waypoint lateral loop runs; `0.0` runs it every tick.
  `CopterGuidance::new` panics on a negative or non-finite period.
- The crate is `no_std` (including the `testing` feature), so it builds for
  bare-metal targets such as `thumbv7em-none-eabihf`. It depends on `uom` and
  `robomath` without their `std` features; enable them yourself if you need them.

### Changed

- **Breaking:** `Stabilizer` is the inner loop only: `stabilize` takes a
  `&Setpoint` instead of a `&Command`. `CopterStabilizer` no longer has the
  altitude and lateral controllers or the heading-hold target (now in
  `CopterGuidance`), and `CopterStabilizerConfig` drops `altitude_gains` and
  `lateral_gains` (now `CopterGuidanceConfig`). Wire `CopterGuidance` into
  `FlightLoop::new` between the vehicle and the stabilizer. Flight behaviour is
  unchanged: the guidance + stabilizer pair matches the old stabilizer bit for bit.
- **Breaking:** `CopterStabilizer`'s controller and `max_rates` fields are no
  longer public, and `CopterStabilizer::config` is gone (it was a stale copy).
- **Breaking:** the PID controllers and `Gains` moved from
  `stabilizers::controllers`/`stabilizers` to `controllers`, since guidance uses
  them too (`use stable_flight::controllers::{Gains, AttitudeController, …}`).
- Waypoint flight runs the lateral position loop at 50 Hz instead of every tick
  (250 Hz in the RealFlight examples), holding its roll/pitch in between; the
  altitude loop still runs every tick. Set `navigation_period: 0.0` for the old
  behaviour.
- The attitude-mode heading-hold target slews at 0.025 rad/s at full yaw stick,
  scaled by `dt`. It moved a fixed 0.0001 rad per tick, so its rate depended on the
  loop rate; the two are the same at 250 Hz.
- `CopterStabilizer`'s thrust curve saturates its input (NaN → 0), so it meets the
  input policy itself instead of relying on callers. No change for the built-in
  flight modes, which already saturated; only a custom `Guidance` sending
  out-of-range thrust sees it.
- `Guidance::prime` and `Stabilizer::prime` default to doing nothing.

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
  - Attitude-command throttle and yaw saturate instead of asserting: yaw in
    `CopterGuidance`, throttle in `CopterStabilizer`'s thrust curve, roll and pitch
    in the attitude controller.
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
