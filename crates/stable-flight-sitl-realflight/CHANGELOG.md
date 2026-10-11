# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **`copter-autotune-realflight` example** (`just fly-autotune`): the core relay autotune
  (#149) on QuadCopterX. Climbs to the hover point, tunes the roll and pitch rate loops, logs each stage's
  measurement and gains, prints the tuned gains as an `airframes.rs` snippet, then lands and
  disarms. `--rate-p-scale` starts from bad rate gains; `--stages angle|all` adds the opt-in angle and heading stages; `--record`
  writes every tick to CSV for diagnosing a run. `--step-test` (#298) appends step tests of
  roll and pitch rate and angle, logging each step's response and a summary per loop
  (`--stages none --step-test` measures the hand tune); steps during a stall run again.

## [0.1.0] - 2026-10-10

First release.

### Added

- **`RealFlightVehicle`:** a core `Vehicle` over RealFlight's FlightAxis link (via the
  re-exported `realflight_bridge`).
  - Converts RealFlight state to the core frame contract; `vehicle_state` exposes the
    conversion as a pure function.
  - Maps the mixer's motor and servo slots to RF channels with `ActuatorMapping`, rescaling
    servos to RealFlight's 0–1 range; `validate` and `try_new` reject a missing or shared
    channel (`MappingError`), so a bad mapping fails at construction, not in the tick.
  - Survives a lost connection: `with_reconnect` rebuilds the bridge with backoff and disables
    RC on it again; failed exchanges are logged at most once a second, and exchanges over
    20 ms (`SLOW_EXCHANGE`) at `debug`.
  - A stalled proxy or simulator can't hang the tick: built on `realflight-bridge` 2.1, whose
    calls time out as a connection error. On one, the vehicle closes the connection at once,
    so the proxy (one client at a time) accepts the reconnect. `REMOTE_IO_TIMEOUT` (250 ms),
    the examples' I/O timeout, pairs with a proxy run with `--request-timeout-ms 150`.
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

[unreleased]: https://github.com/wboayue/stable-flight/compare/stable-flight-sitl-realflight-v0.1.0...HEAD
[0.1.0]: https://github.com/wboayue/stable-flight/releases/tag/stable-flight-sitl-realflight-v0.1.0
