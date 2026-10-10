# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Not yet released. Until the first release this section summarizes what the crate provides,
not each change; per-change entries start after it.

### Added

- **`RealFlightVehicle`:** a core `Vehicle` over RealFlight's FlightAxis link (via the
  re-exported `realflight_bridge`).
  - Converts RealFlight state to the core frame contract; `vehicle_state` exposes the
    conversion as a pure function.
  - Maps mixer outputs to RF channels with `ActuatorMapping`.
  - Covers every core conformance scenario.
- **Airframes:** `airframes::quadcopter_x()`, the flight-checked QuadCopterX tune (guidance and
  stabilizer configs, mixer, channel mapping).
- **Examples:**
  - All serve params and telemetry over the native link (UDP) for `stable-flight-bridge`.
  - `copter-manual-realflight`: joystick (Extreme 3D, or Spektrum with `--commander spektrum`).
  - `copter-auto-realflight`: scripted waypoints; holds at the end, `--loop` to repeat.
  - `capture-state-realflight`: records raw simulator state and channels to CSV.
  - `copter-companion-realflight`: flown from the link (`FailsafeCommander<LinkCommander,
    HoldCommander>`), holding position when the link goes quiet.

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight-sitl-realflight
