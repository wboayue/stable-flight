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
  defaults except for damped lateral gains, and both examples fly it.
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

### Fixed

- `RealFlightVehicle` meets the `VehicleState` frame contract, per a live capture
  ([#62](https://github.com/wboayue/stable-flight/issues/62)): angles and rates are
  converted to radians once (they were ~57× too small), position is north/east
  (RealFlight reports east/north), vertical velocity and acceleration are
  up-positive, heading and yaw rate are clockwise-positive (RealFlight's are
  counterclockwise), the quaternion is remapped from RealFlight's `(w, y, x, -z)`
  axes, and acceleration comes from the body specific force (RealFlight's world
  acceleration is unreliable). Every conformance test runs.

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight-sitl-realflight
