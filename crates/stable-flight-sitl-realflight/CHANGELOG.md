# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `copter-autotune-realflight --airframe explora` (#539): the eXplora tailsitter's hover side
  on its own mixer, guidance and stabilizer flown in `View::tailsitter`'s nose-up frame, where
  the autotune and the inverted check read its tilt and rates. `--param NAME=VALUE` sets a
  param before takeoff.

### Changed

- **Breaking (behavior):** `airframes::quadcopter_x()`'s rate I terms take core's new limits,
  0.5 of moment (#374): roll and pitch held 0.2 under the old limit, yaw 1.73. The eXplora
  and Titan Cobra keep what theirs held, in the new unit.
- **Breaking (behavior):** `airframes::explora_vtol()` hovers on RealFlight's autotune of it
  (#539): rate P 0.6504/1.4217/1.0191 (roll, pitch, yaw in the hover view), angle P
  2.3554/9, and the default 40 Hz gyro low-pass in place of none. The hand tune's rate steps
  ended at 0.55–0.80 of their request; the autotuned roll and pitch steps end within 0.07 of it.

## [0.2.0] - 2026-10-30

### Added

- `RealFlightVehicle::state_is_stale`: the tick's exchange failed or the link is down, so
  `state` holds the last report while the aircraft flies on unseen. `copter-autotune-realflight`
  tells its autotune (`Autotune::set_state_stale`), lands only on a fresh state, stops with
  exit status 1 after 30 s stale, and cuts the link on cue with `--drop-link SECS@AT` (#309).
- `StartError`: why `RealFlightVehicle::try_new` couldn't start a flight, an invalid mapping or
  a failed aircraft reset (#523).
- `airframes::explora_maneuvers()` (#342): scripted maneuvers on the eXplora, rolls and loops
  at full thrust and doublets at its cruise thrust; `tailsitter-auto-realflight` flies it, and
  `flights/explora_acro.csv` flies doublets, a roll, and a loop on plane legs.
- `TrueWind` and `RealFlightVehicle::true_wind` (#473): the wind RealFlight reports, north and
  east, as a handle updated on each exchange, to log beside a wind estimate.
  `tailsitter-auto-realflight --record` adds it as `true_wind_north_mps` and
  `true_wind_east_mps`, with the hover view's specific force (`view_force_x_mps2`,
  `view_force_y_mps2`) to fit drag coefficients from.
- `tailsitter-auto-realflight --param NAME=VALUE` sets a param before takeoff, and
  `flights/explora_descent.csv` flies a 60 → 20 m hover descent over the eXplora's spawn
  point (#417).
- `flights/explora_heading_box.csv`: the eXplora holds over its spawn point facing 8
  headings 45° apart, 20 s each, at 40 m and 60 m (#470).
- `flights/explora_wind_cal.csv` and `flights/explora_wind_check.csv` (#473): eXplora hover
  holds to fit the wind estimate's drag coefficients and check them.
- `tailsitter-companion-realflight` example (#333): the eXplora flown from the native link
  under a hold failsafe, its regime requested over the link (QGC's transition button through
  the bridge).
- `copter-auto-realflight --record`: one CSV row per tick (`CopterColumns`).
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
  #298, #518) on QuadCopterX. Climbs to the hover point, tunes roll and pitch rate, then roll
  and pitch angle, by design search (`StagePlan::copter_design_search`: each axis fitted and
  designed, then each loop's P searched by steps from its design, kept only if no worse), logs
  each fit and judgment and baseline vs verify, prints the tuned gains as an `airframes.rs`
  snippet, then lands and disarms. `--rate-p-scale` starts from bad rate gains; `--record` writes every tick to CSV for diagnosing a run.
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

- The Titan Cobra flies the RealFlight design search's hover gains: rate P 0.4847 / 0.3970
  (roll / pitch), D 0, I at 0.2 of P, `max_integral` 3; yaw rate 1.8091 / 0.5427; angle P
  7.1338 / 6.5084 (were rate 0.5867 / 0.5711 with D 0.0131 / 0.0123, yaw 0.3 / 0.3, angle P
  3 / 7.2219). Roll rate rises in 76 ms (113), yaw rate in 466 ms where it never rose in 1 s,
  roll angle in 161 ms with overshoot 0.05 where 3 of 6 steps never settled (#538).
- The QuadCopterX flies the RealFlight autotune's yaw rate gains, P 1.1532 and I 0.3460 (was
  P 0.6, I 0.6), which damp its heading loop: 30° heading turns overshoot 7% and settle within
  1° in 1.6 s, where they overshot 25% and rang for 4.2 s, and any kick swung the heading for
  seconds (#383, #558). 0.5 rad/s yaw steps overshoot 0.01 and rise in 0.50–0.89 s.
  `flights/heading_steps.csv` flies the turns.
- `copter-autotune-realflight` tunes yaw rate after the roll and pitch loops, and `--step-test`
  step-tests it. The QuadCopterX flies a 5 rad yaw integral limit (core's default), so the
  autotune's yaw I holds its 0.40 hover yaw trim; the eXplora and the Titan Cobra keep 1 rad
  (#296).
- The eXplora's hover loops fly without core's new gyro low-pass (`gyro_cutoff_hz: 0`): with
  little rate D they lose the margin to its lag on the sim tailsitter (#373, #550). The
  QuadCopterX and the Titan Cobra fly the 40 Hz default.
- **Breaking:** `RealFlightVehicle::try_new` returns `StartError` and fails when RealFlight
  doesn't reset the aircraft, handing control back to RC; `new` panics on it. A failed reset
  was a warning, and the flight started from wherever the last one left the aircraft, upside
  down after a crash (#523).
- `copter-autotune-realflight` exits with status 1, RC handed back, when the vehicle stays
  inverted for 3 s or the aircraft reset fails, so unattended runs don't wait on a crash (#523).
- `channel-sweep-realflight` exits with status 1, RC handed back, when the aircraft reset fails,
  instead of sweeping from wherever the last run left the aircraft (#523).
- `copter-autotune-realflight` flags a freeze (`Autotune::note_stall`) on a tick over 50 ms
  after the last, not on the runner's resyncs, which at about 223 Hz come every few seconds
  with no tick late by much (#524).
- The Titan Cobra's mixer runs its lift rotors at thrust factor 0.8, twice the default's moment
  range, with hover thrust rescaled so the hover holds the same motor command (#438). Its
  nose-heavy trim took 0.37 of the default's nose-up range; nose-up steps clipped 15× as often
  as nose-down. `copter-autotune-realflight --airframe titan-cobra` mixes at the same config.
  Its rate gains are halved to match, and its hover gains come from the autotune at 250 Hz:
  rate P 0.59 roll, 0.57 pitch, pitch angle P 7.2; every loop's steps rise and settle.
- `copter-autotune-realflight` waits out RealFlight's slow stretches (#438): under 225 Hz over
  0.5 s (`TickRate`), the running step stops and runs again once the rate is back over 240 Hz,
  and settling waits. Stalled steps are logged as stalled, and the host turning slow and back at `info`.
- The Titan Cobra's copter legs brake along a curve at 1.2 m/s² (`GUID_LAT_BRK`, #488):
  uncapped 300 m legs stop 3 m short to 0.1 m past their waypoints, where the linear position
  loop overshot 15–20 m.
- **Breaking:** the script examples (`copter-auto-realflight`, `tailsitter-auto-realflight`,
  `quadplane-auto-realflight`, `surface-probe-realflight`, `copter-autotune-realflight`,
  `capture-state-realflight`) fly scripts relative to where the vehicle starts (#494), and
  `flights/*.csv` start at 0,0 and height 0: the same route at any field. Touchdown logs
  report where, from the start.
- `airframes::explora_vtol()` and `airframes::titan_cobra()` fall back to a hover below 15 m
  in plane flight (`TRAN_FW_MIN_HGT`, #450), half the scripts' 30 m plane legs.
- `airframes::explora_vtol()` turns a wingtip into the wind while holding and landing
  (`GUID_WND_HDG_SPD` 1 m/s, #470): belly into RealFlight's 2 m/s wind it pinned its tilt cap
  and drifted.
- `airframes::explora_vtol()` damps its hover roll rate loop (`STAB_RATE_RLL_D` 0.03, #476):
  descending with the wind 20° behind a wingtip, it rolled ±12–16° on a 1.2 s cycle to
  touchdown; with D it holds within 4°.
- `airframes::quadcopter_x()` and `titan_cobra()` fly copter waypoint legs nose first and
  yield a heading the wind holds off (core's defaults, #452, #431); `explora_vtol()` holds
  its heading on hover legs and never yields it: facing a leg flies it belly first.
- `airframes::titan_cobra()` slows copter waypoint legs while its nose is off them (core's
  `OffNoseSpeed` default, #437): it flew the probe's legs back tail first at 16 m/s and
  flipped.
- `airframes::explora_vtol()` and `titan_cobra()` fly plane waypoints by total energy control
  (#454) on ArduPilot's defaults, with their plane airspeeds as its least and most.
- `airframes::explora_vtol()` turns to a waypoint's heading at 30°/s (`GUID_HDG_RATE`), the
  rate that turned it 90° in 3 mph wind without losing the hold (#417, #470).
- `airframes::explora_vtol()` shapes its hover legs at 1 m/s² (`GUID_LAT_ACC`): after the
  back transition, a leg along the span peaked at 11° of roll instead of 35° (#418).
- `airframes::explora_vtol()` estimates the hover wind (#473): drag linear in the air's
  speed, 4.76/s across the belly and 0.24/s along the span, fitted to RealFlight's wind; it
  reads it within 4° and 22% at 1.7–2.3 m/s.
- `airframes::explora_vtol()`'s plane rate loop (#342): feedforward 0.1 roll and 0.05 pitch
  (PX4's 0.5 overshot 30°/s doublets 3.2× and 3.4×), pitch I 0.8, and pitch trim 0.09 (its
  level nose-up moment).
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

[unreleased]: https://github.com/wboayue/stable-flight/compare/stable-flight-sitl-realflight-v0.2.0...HEAD
[0.2.0]: https://github.com/wboayue/stable-flight/releases/tag/stable-flight-sitl-realflight-v0.2.0
[0.1.0]: https://github.com/wboayue/stable-flight/releases/tag/stable-flight-sitl-realflight-v0.1.0
