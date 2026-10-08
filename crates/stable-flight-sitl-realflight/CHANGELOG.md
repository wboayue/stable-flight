# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- New crate: the RealFlight backend, moved out of `stable-flight-sitl`
  (`simulators::realflight`). `RealFlightVehicle`, `RealFlightConfiguration`, and
  `ActuatorMapping` are at the crate root, and `realflight_bridge` (2.x) is re-exported.
  Never depends on `stable-flight-sitl` (a dev-dependency for the examples).
- `airframes::Airframe<G, S, M>` (guidance and stabilizer configs, mixer, and channel mapping) and
  `airframes::quadcopter_x()`, the QuadCopterX model's tuned gains with
  `QuadCopterMixer` and its mapping, flight-checked in manual attitude mode, auto
  hover, and the figure-eight after the frame fix (#62). Its guidance uses the core
  defaults with its measured hover thrust (0.82), and both examples fly it.
- `vehicle_state(&SimulatorState) -> VehicleState`: the frame conversion
  `RealFlightVehicle` uses, as a pure function, for recorded or replayed reports.
- `ActuatorMapping` derives `Debug`, `Clone`, `Copy`, `PartialEq`, `Eq`, and `Hash`.
- The priming `estimate_state(Duration::ZERO)` logs an error when its exchange
  fails (it still returns the default state).
- The `copter-manual-realflight` and `copter-auto-realflight` examples (moved from
  `stable-flight-sitl`); run with `-p stable-flight-sitl-realflight`, or
  `just fly-manual` / `just fly-auto`.
- The `capture-state-realflight` example (`just capture`): flies the joystick or a
  waypoint script (`--script`) and records every raw `SimulatorState` and the
  channels sent to CSV, for checking frame conversions.

### Changed

- `copter-auto-realflight` flies the script once and hovers on the last waypoint; `--loop`
  restores looping ([#126](https://github.com/wboayue/stable-flight/issues/126)). Waypoint
  legs are flown at the script's cruise speed
  ([#109](https://github.com/wboayue/stable-flight/issues/109)).
- **Breaking:** `airframes::quadcopter_x` guidance uses the cascaded `LateralControllerConfig`
  (same response below the speed limit) ([#109](https://github.com/wboayue/stable-flight/issues/109)).
- **Breaking:** `RealFlightVehicle` fills the renamed plain-`f32` `VehicleState` fields
  (`position_m`, `velocity_mps`, `acceleration_mps2`); the `uom` dependency is removed
  ([#141](https://github.com/wboayue/stable-flight/issues/141)).
- **Breaking:** `vehicle_state` and `airframes` use the core's unit-suffixed names
  (`roll_rad`, `angular_velocity_radps`, `max_rate_radps`, `navigation_period_s`, …)
  ([#161](https://github.com/wboayue/stable-flight/issues/161)).

### Fixed

- `airframes::quadcopter_x` yaw rate I raised from 0.2 to 0.6. The model needs a steady
  ~0.40 yaw moment to hover, past the old integral authority (0.2), so acro drifted ~19°/s
  counterclockwise with centered sticks and attitude mode held heading a few degrees off
  ([#192](https://github.com/wboayue/stable-flight/issues/192)).
- `RealFlightVehicle` meets the `VehicleState` frame contract, per a live capture
  ([#62](https://github.com/wboayue/stable-flight/issues/62)): angles and rates are
  converted to radians once (they were ~57× too small), position is north/east
  (RealFlight reports east/north), vertical velocity and acceleration are
  up-positive, heading and yaw rate are clockwise-positive (RealFlight's are
  counterclockwise), the quaternion is remapped from RealFlight's `(w, y, x, -z)`
  axes, and acceleration comes from the body specific force (RealFlight's world
  acceleration is unreliable). Every conformance test runs.

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight-sitl-realflight
