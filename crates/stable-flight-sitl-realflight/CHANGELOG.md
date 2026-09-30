# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- New crate: the RealFlight backend, moved out of `stable-flight-sitl`
  (`simulators::realflight`). `RealFlightVehicle`, `RealFlightConfiguration`, and
  `ActuatorMapping` are at the crate root, and `realflight_bridge` is re-exported.
  Never depends on `stable-flight-sitl` (a dev-dependency for the examples).
- `airframes::Airframe<G, S, M>` (guidance and stabilizer configs, mixer, and channel mapping) and
  `airframes::quadcopter_x()`, the QuadCopterX model's tuned gains with
  `QuadCopterMixer` and its mapping. Its guidance uses the core defaults, so
  `copter-auto-realflight` runs the waypoint lateral loop at 50 Hz.
- `ActuatorMapping` derives `Debug`, `Clone`, `Copy`, `PartialEq`, `Eq`, and `Hash`.
- The priming `estimate_state(Duration::ZERO)` logs an error when its exchange
  fails (it still returns the default state).
- The `copter-manual-realflight` and `copter-auto-realflight` examples (moved from
  `stable-flight-sitl`); run with `-p stable-flight-sitl-realflight`, or
  `just fly-manual` / `just fly-auto`.

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight-sitl-realflight
