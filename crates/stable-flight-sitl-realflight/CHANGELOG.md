# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- New crate: the RealFlight backend, moved out of `stable-flight-sitl`
  (`simulators::realflight`). `RealFlightVehicle`, `RealFlightConfiguration`, and
  `ActuatorMapping` are at the crate root, and `realflight_bridge` is re-exported.
  Depends on the `stable-flight` core only.
- `airframes::quadcopter_x()`: the QuadCopterX model's tuned stabilizer gains and
  channel mapping.
- `ActuatorMapping` derives `Debug`, `Clone`, and `PartialEq`.
- The `copter-manual-realflight` and `copter-auto-realflight` examples (moved from
  `stable-flight-sitl`); run with `-p stable-flight-sitl-realflight`, or
  `just fly-manual` / `just fly-auto`.

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight-sitl-realflight
