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
  - Survives a lost connection: `with_reconnect` rebuilds the bridge with backoff and disables
    RC on it again; failed exchanges are logged at most once a second, and exchanges over
    20 ms (`SLOW_EXCHANGE`) at `debug`.
  - A stalled proxy or simulator can't hang the tick: built on `realflight-bridge` 2.1, whose
    calls time out (100 ms by default) as a connection error, which the reconnect handles.
  - Covers every core conformance scenario.
- **Airframes:** `airframes::quadcopter_x()`, the flight-checked QuadCopterX tune (guidance and
  stabilizer configs, mixer, channel mapping).
- **Examples:**
  - The flight and capture examples serve params and telemetry over the native link (UDP) for
    `stable-flight-bridge`.
  - The flight examples reconnect to RealFlight after a lost connection (capture does not).
  - `copter-manual-realflight`: joystick (Extreme 3D, or Spektrum with `--commander spektrum`).
  - `copter-auto-realflight`: scripted waypoints; `--end hold` (default), `loop`, or `land` (lands on the last waypoint and disarms).
  - `capture-state-realflight`: records raw simulator state and channels to CSV.
  - `copter-companion-realflight`: flown from the link (`FailsafeCommander<LinkCommander,
    HoldCommander>`), holding position when the link goes quiet, or finishing a land.
  - `exchange-latency-realflight`: times the RealFlight round trip alone, through the proxy or
    directly (`--local`).

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight-sitl-realflight
