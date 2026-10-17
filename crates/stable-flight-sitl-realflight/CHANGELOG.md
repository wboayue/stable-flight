# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- `airframes::explora_vtol()` and `titan_cobra()` fly plane waypoints by total energy control
  (#454) on ArduPilot's defaults, with their plane airspeeds as its least and most.
- `airframes::explora_vtol()` shapes its hover legs at 1 m/s² (`GUID_LAT_ACC`): after the
  back transition, a leg along the span peaked at 11° of roll instead of 35° (#418).

### Added

- `airframes::titan_cobra()`: the ArduPilot SITL_Models Titan Cobra VTOL quadplane on
  `TableMixer::quadplane_vtail`, its channels mapped from a sweep (#433). Flown in
  RealFlight: hover (measured trim, autotuned roll rate), surfaces checked in flight, tail
  trimmed +0.2, transitions at ArduPilot's 11 m/s, forward thrust in copter flight; its
  pusher values are shares of the full pusher (cruise 0.47, ArduPilot's `TRIM_THROTTLE`;
  forward thrust gain 1.08; #448).
  `quadplane-auto-realflight` flies it from a waypoint script (`flights/cobra_hover.csv`,
  `cobra_transition_lap.csv`, `cobra_legs.csv`; `--end`, `--record` with
  `QuadPlaneColumns`, `--param NAME=VALUE`), and `copter-autotune-realflight --airframe
  titan-cobra` tunes its lift rotors.
- `surface-probe-realflight`: checks which way a quadplane's surfaces push, in copter flight
  at speed, from the lift rotors' push-back against a cruise-moment pulse per axis (#433).
- `ActuatorMapping::servo_trims`: each servo slot's neutral offset, ArduPilot's
  `SERVOn_TRIM`, checked by `validate` (#433). **Breaking:** a struct literal needs the
  field (or `..Default::default()`), `ActuatorMapping` is no longer `Eq` or `Hash`, and
  `MappingError` gains `InvalidTrim`.

- `flights/explora_transition_lap.csv`: the eXplora transition mission with one plane lap,
  for transition checks; its back transition starts where the two-lap mission's does.

- `copter-manual-realflight --record <csv>` (`just fly-manual`): one row per tick of body
  rates, attitude, moments, the attitude setpoint, and the stick yaw (#337).

- `flights/explora_transition.csv` and `tailsitter-auto-realflight` plane legs (#124): the
  script's regime column drives transitions; the example logs mode changes and takes
  `--record` like the manual one.

- `RealFlightConfiguration::from_airframe(&airframe, bridge)`: takes the channel mapping and
  body rotation from an `Airframe`; the flight examples use it (#271).

- `vehicle_state` fills `VehicleState::airspeed_mps` from RealFlight's airspeed (#181).

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
  (`TableMixer::tailsitter_2p`), channel mapping (left/right motors on channels 6/5,
  elevons on 1/2, the left reversed), +90° pitch body rotation, and copter gains in the
  tailsitter `View`'s axes (#122), hand-tuned in flight (#123; roll/pitch angle P 4, #327),
  with the measured hover thrust, a thrust floor and sink guard that keep the elevons in the
  propwash, and a tilt-boost ceiling that leaves differential-thrust headroom (#123).
- **`tailsitter-manual-realflight` example** (`just fly-tailsitter`, #121, #122): joystick
  hover of the eXplora, on the copter guidance and stabilizer in the core's tailsitter `View`;
  `--record` writes every tick to CSV (body rates and actuators, the attitude in the view, and
  guidance's setpoint, the climb rate, and the stick throttle, #123).
- **`tailsitter-auto-realflight` example** (`just fly-tailsitter-auto`, #123): the eXplora
  through a waypoint script in hover, wired as `copter-auto-realflight` with the tailsitter
  `View`; `--end hold|loop|land`. Its scripts: `flights/explora_figure_eight.csv` (small and
  slow, centered on the spawn point; the default) and `flights/explora_hover.csv` (5 m over
  it).
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

- `airframes::explora_vtol()`: waypoint descents are held to 1.5 m/s (`max_descent_mps`, was
  the 3 m/s default). At 3 m/s the nose swung ±20° in hover descents (#414).
- `airframes::explora_vtol()` flies the back transition at thrust 0.75, down from hover
  (0.84), with core's default sink guard: it climbs about 9 m after the back request, not 18 m
  (#343).
- `airframes::explora_vtol()` returns `TailsitterGuidanceConfig`/`TailsitterStabilizerConfig`
  (hover as before, with the view pitch cap raised to 85° for the forward ramp and sticks
  kept at 30°; plane and transition starting values), and the tailsitter examples fly
  `TailsitterGuidance`/`TailsitterStabilizer`; the manual example takes the regime from the
  sticks, logs mode changes, and records mode, airspeed, and plane setpoints (#124).

- **realflight-bridge 2.2** (#270): a request RealFlight doesn't answer in time on the proxy
  (`BridgeError::Simulator`) is retried on the same bridge instead of forcing a reconnect, and
  the startup aircraft reset is retried up to three times. A reconnect whose `disable_rc` times
  out that way fails and backs off. The proxy's defaults now fit RealFlight: run it without
  `--request-timeout-ms`.
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

### Fixed

- `flights/figure_eight.csv` and `flights/explora_figure_eight.csv` start and end over their
  airframe's spawn point, so `--end land` touches down where the flight took off (#345).

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
