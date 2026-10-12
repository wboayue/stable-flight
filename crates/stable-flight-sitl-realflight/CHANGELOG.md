# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Body rotation** (#314): `RealFlightConfiguration::body_rotation` and
  `Airframe::body_rotation`, the vehicle's body frame expressed in RealFlight's, for models whose
  RealFlight frame isn't the vehicle's (the eXplora VTOL tailsitter reads level on its tail; its
  vehicle frame is the airplane's, +90° pitch). `vehicle_state` applies it to the attitude, body
  rates, and body specific force. `airframes::quadcopter_x()` uses identity.

- **`copter-autotune-realflight` example** (`just fly-autotune`): the core autotune (#149,
  #298) on QuadCopterX. Climbs to the hover point, tunes roll and pitch rate, then roll and
  pitch angle, by step-response search (`--stages search`, the default: relay D, searched P,
  kept only if no worse than the start), logs each judgment and baseline vs verify, prints the
  tuned gains as an `airframes.rs` snippet, then lands and disarms. `--stages rate` runs the
  relay autotune instead (`angle|all` add relay angle and heading stages); `--rate-p-scale`
  starts from bad rate gains; `--record` writes every tick to CSV for diagnosing a run.
  `--step-test` appends step tests of roll and pitch rate and angle, logging each step's
  response and a summary per loop (`--stages none --step-test` measures the hand tune). A step
  during which the runner skipped more than two ticks at once runs again.
- **`airframes::explora_vtol()`** (#121): the eXplora VTOL tailsitter's mixer
  (`TableMixer::tailsitter_2p_hover`), channel mapping (motors on channels 5/6, elevons on
  1/2), and +90° pitch body rotation, with interim copter gains for flying it in RealFlight's
  hover-level frame until the nose-up stabilizer (#122).
- **`tailsitter-manual-realflight` example** (`just fly-tailsitter`, #121): joystick hover of
  the eXplora, on the copter stabilizer in RealFlight's hover-level frame with an adapter to the
  airplane-frame mixer (interim until #122); `--record` writes every tick to CSV.
- **`ActuatorMapping::reversed_servos`** (#121): servo slots negated before they reach their
  channel, for a surface the model moves the other way from the mixer's convention.
- **`CHANNEL_COUNT`:** RealFlight's 12 channels, exported.
- **`capture`** (#120): raw RealFlight captures as CSV, every `SimulatorState` with the
  channels sent. `RecordingBridge` records under any pipeline (`capture-state-realflight` uses
  it); `read_capture` reads a capture back (columns by name; a cut-off last row is dropped).
- **`channel-sweep-realflight` example** (#120): drives a model's raw channels on the ground
  with no flight loop (neutral, each servo full both ways, each motor alone, a hop) and records
  a capture, for mapping a new model's channels and checking the frame contract on it.
  `--neutral` sets any channel's rest value; `--tail-s` keeps recording after the last step.

### Changed

- **Breaking:** `RealFlightConfiguration` and `Airframe` have a new `body_rotation` field, and
  `vehicle_state` takes the body rotation as a second argument (#314). Pass
  `UnitQuaternion::identity()`, or the airframe's `body_rotation`, for the previous behavior.
- **Breaking:** `ActuatorMapping` has a new `reversed_servos` field (#121); struct literals
  add `..Default::default()` (all `false`, the previous behavior).
- **Breaking:** `airframes::quadcopter_x()`'s mixer is `TableMixer<4>` (`TableMixer::quad_x`),
  following the core's removal of `QuadCopterMixer` (#245).

- **`vehicle_state`'s Euler fields come from the quaternion** (#120), not RealFlight's own Euler
  angles, so `roll_rad`, `pitch_rad`, and `heading_rad` always agree with `attitude`. They match
  RealFlight's away from ±90° pitch; near it both are degenerate (a tailsitter's hover).

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
