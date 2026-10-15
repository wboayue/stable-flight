# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Lift+cruise transition** (#391): `guidance::vtol::LiftTransition` (`LiftTransitionConfig`,
  `LiftTransitionStep`), a quadplane's copter ⇄ plane state machine. Forward, the lift rotors
  hold hover until a valid airspeed reaches the gate (default 9 m/s), then their lift share
  ramps to 0 (5 s); without airspeed it times out to copter (15 s). Back, the share ramps to 1
  (3 s), completing below a forward speed (5 m/s) or after 10 s. Params `TRAN_ASPD_MIN`,
  `TRAN_FW_RAMP`, `TRAN_TIMEOUT`, `TRAN_MIN_HGT`, `TRAN_VT_RAMP`, `TRAN_VT_SPD`,
  `TRAN_VT_TIMEOUT`, with metadata.
- **Lift+cruise mixing** (#391): `MixRow::pusher` (a forward motor flying the cruise thrust,
  not desaturated with the lift motors; `MixRow::is_pusher`) and the `TableMixer::quadplane()`
  preset (`quad_x` lift rotors on motors 0–3, pusher on 4, `plane` surfaces on servos 0–2).
  With `ControlOutput::cruise` set, servo rows fly its moments; without it, they fly
  `moments` as before and pushers stay stopped.
- Tailsitter back-transition sink guard (#343): `TransitionConfig::back_sink_limit_mps` and
  `back_sink_gain` (`TRAN_VT_SINK_LIM`, `TRAN_VT_SINK_GN`; default 1 m/s and 0.4 per m/s)
  raise the back ramp's thrust while it sinks, so a ramp too weak to hold the nose up doesn't
  stall and fall. Thrust saturation is flagged while it acts.
- `FlightLoop::vehicle()`, beside `commander()`, `guidance()`, and `stabilizer()` (#348).
- `TableMixer::plane()`: a conventional airplane preset (motor 0; servos 0 ailerons,
  1 elevator, 2 rudder), flown by the simulator's fixed-wing model (#348).
- `TableMixer::rows` and `MixRow::{roll, pitch, yaw}`: read a mixer table back, e.g. for a
  simulator that derives motor torques from it (#347).
- **Plane waypoint navigation** (#124): `PlaneGuidance` flies `Waypoint`s. It follows the
  leg's line with L1 (ArduPilot's lateral acceleration, from the ground velocity so a
  crosswind crab is flown), loiters clockwise around a waypoint with no leg, holds the
  waypoint's altitude with pitch (PI), and its `cruise_speed_mps` as airspeed with throttle
  (PI around cruise; cruise plus a climb share without airspeed). New `PlaneGuidanceConfig`
  fields and `PGUID_*` params.

- **Tailsitter wiring** (#124): `guidance::vtol::TailsitterGuidance` (hover guidance in the
  tailsitter view and `PlaneGuidance` under a `Transition`: the forward ramp on the hover side,
  the back ramp on the plane side, `Land` asks for copter, a roll past the limit cuts a
  transition short, a thrust blend at each handoff, and stick throttle held at cruise after
  the handoff into plane until the stick reaches it) and `stabilizers::vtol::TailsitterStabilizer`
  (each setpoint on its side, the incoming side primed). `Transition::cut_short`;
  `TransitionConfig` gains `forward_thrust`, `back_thrust`, `handoff_s`, `roll_limit_rad`;
  `Setpoint::with_thrust`.
- **Params** for the plane and transition configs (`PSTAB_*`, `PGUID_*`, `TRAN_*`, with
  metadata); the tailsitter composites chain their parts' tables.

- **Plane control** (#124): `stabilizers::plane::PlaneStabilizer` (PX4's fixed-wing
  structure: yaw-free attitude error over a time constant, level-turn feedforward on pitch and
  yaw rate faded past 70° bank, and `controllers::PlaneRateController`, a PI + feedforward rate
  loop scaled by airspeed with its integral held below minimum airspeed) and
  `guidance::plane::PlaneGuidance` (FBWA sticks, acro rate sticks, absolute attitude and rates,
  a wings-level placeholder for waypoints). New setpoints `Setpoint::Plane(PlaneSetpoint)` and
  `Setpoint::PlaneRate(BodyRateSetpoint)`; `PlaneRate` flies at any attitude (rolls, loops).
  `FlightModeReport::plane_only`; `ModeRefusal::RollError`.
- **Absolute rates** (#124): `ControlInput::AbsoluteRate(AbsoluteRateCommand)` (rad/s and
  thrust, a stream; `ControlMode::AbsoluteRate`) for maneuvers and offboard rates.
  `CopterGuidance` flies it as the new `Setpoint::BodyRate(BodyRateSetpoint)`, which
  `CopterStabilizer` tracks at absolute rates saturated to its full-scale rates.

- **`guidance::vtol`** (#181): `Transition`, the copter ⇄ plane state machine a VTOL
  guidance owns. It moves toward the requested `FlightRegime`, ramps a frame-free tilt target
  (the nose's angle from vertical) through `ToPlane`/`ToCopter`, completes on tilt (and a
  minimum airspeed when valid), flags the controller handoff, and refuses below a height
  above the arming point. A forward timeout falls back to copter until copter is requested;
  a back timeout completes. `ThrustHandoff` blends thrust across a handoff. Defaults follow
  ArduPilot's tailsitter (50°/s, 45°) and PX4's 15 s timeout. Not wired into a guidance
  yet (#124).

- **Flight regimes** (#181): `Command::regime` requests a `FlightRegime` (`Copter`, the
  default, or `Plane`) every tick, like `arm`; `Command::with_regime` sets it. Guidance owns
  the flown `FlightMode` (`Copter`, `Plane`, `ToPlane`, `ToCopter`) and reports it with an
  optional `ModeRefusal` as a `FlightModeReport` through `Guidance::flight_mode` (default
  `None`: copter only, recorded as `FlightModeReport::copter_only`, which refuses `Plane` as
  `Unsupported`). `TelemetryRecord::flight_mode` carries it; `View` forwards it.
- **`VehicleState::airspeed_mps`** (`Option<f32>`): speed relative to the air mass, `None`
  without air data; `valid_airspeed_mps` drops a NaN or negative reading. The conformance kit expects ground speed (still air) and passes a backend
  that reports `None`.

- **`tuning`:** building blocks for PID autotune (#149). `RelayTuner` measures a loop's
  ultimate gain and period by relay feedback (hysteresis, relay ramp, adaptive bias for a
  steady offset, raising the relay when cycles stay inconsistent, timeout and amplitude
  aborts); `gains_from_ultimate` turns the result into
  `Gains` by Tyreus–Luyben or no-overshoot Ziegler–Nichols, as PID, PI, PD, or P
  (`ControllerForm::has_integral`/`has_derivative`).
- **Autotune probes** (#149, #298): `Probe` excites and measures one loop inside the tick,
  fed the loop's `LoopSignals` and answering with an `Injection` (a replaced output or
  setpoint): `ProbeConfig::Relay` (a `RelayTuner` seeded with the loop's trim) or
  `ProbeConfig::Step` (a `StepProbe`: holds a step for a fixed window and reports
  `StepMetrics`: overshoot, rise, settling, bounce-back, final value, clipped time;
  `StepSummary` aggregates them). It reports `ProbeStatus` and, when done, a `Measurement`. The `Tunable` trait (`start_probe`, `probe`,
  `stop_probe`), implemented by `CopterStabilizer` on a `TunedLoop` (roll/pitch/yaw rate,
  roll/pitch angle, or heading), hosts one; the loop is handed back with the measured trim as
  its I term when the probe finishes, on `stop_probe`, or when a rate tick interrupts an angle
  or heading probe. `prime` ends a probe. `FlightLoop` forwards the three calls, and adds
  `armed()`. `TunedLoop::gains`/`gains_mut` read or retune a loop's gains in a
  `CopterStabilizerConfig`.
- **`tuning::autotune::Autotune`** (#149, #298): executes a plan of stages between ticks
  (`StagePlan`: a `TunedLoop` and a `Method`; `Method::Relay(RelayPlan)` measures once and
  sets the rule's gains; `Method::StepTest(StepTestPlan)` measures alternating steps and
  keeps the gains, repeating clipped or stalled steps (`Autotune::note_stall`) up to
  `max_invalid` and `max_stalled` in a row (`AutotuneAbort::Invalid`, `Stalled`);
  `Method::Search(SearchPlan)` finds the largest gain of one `Term` whose ± step pairs meet a
  `Criterion`, by bracketing and bisection from gains a relay may seed and cap
  (`SearchRelay`), and keeps it only if its verify flies no worse than a baseline on the start
  gains (`Acceptance`); `StagePlan::copter_search` tunes roll and pitch rate this way
  (`copter_rate_search`: relay PD sets D, step overshoot sets P under `0.6·Ku`), then roll and
  pitch angle P (`SearchPlan::copter_angle`, 5° steps: `COPTER_ANGLE_SEARCH_STEP`); a
  bracket point passes on `SearchPlan::confirm` pairs in a row; candidate gains fly only
  during their step, and an `Envelope` breach while they fly fails only the candidate (the
  committed gains get `Envelope::recovery_s` to recover); presets
  `COPTER_RATE_STEP`, `COPTER_ANGLE_STEP`, `COPTER_STEP_SETTLE`; copter defaults in `StagePlan::copter_sequence`: roll and pitch rate,
  PD keeping each loop's I; the angle, heading, and yaw-rate stages are opt-in, see
  `COPTER_SEQUENCE`). Each stage's decisions run as a pure procedure; the executor settles
  before each probe (`Envelope::settle`, or the method's `Settle`), aborts on an `Envelope` breach (back to the last committed config),
  commits each stage's gains through `set_stabilizer_config`, and reverts to the starting
  config on disarm; reports `AutotuneEvent`s for logging (`StageStarted`, `Measured`,
  `Tried`, `StageDone` with its `Outcome`, ...). Works with any `CopterTunable` stabilizer (`Tunable` on `TunedLoop` +
  `Configurable`). `FlightLoop`'s accessors and config setters no longer require the pipeline
  bounds.
- **`mixers::TableMixer`** (#245): one table-driven mixer for multirotors and
  thrust-and-servo frames. Each `MixRow` drives a motor or servo `Slot` with roll, pitch, and
  yaw factors; motors share airmode-style desaturation, servos clamp on their own (both
  reported in `Saturation`). `MixerConfig` (thrust factor, yaw ratio) is set at construction;
  `TableMixer::new` validates a custom table (`MixerError`). Presets: `quad_x`, `quad_plus`,
  `hex_x`, `hex_plus`, `octo_x`, `y6`, `x8`, `tri` (tail tilt servo), `bi` (tilting rotors),
  `tailsitter_2p` (2 motors and 2 elevons, airplane frame, #121).
- **`View`** (#122): runs a `Guidance` or `Stabilizer` in a body frame rotated by a
  constant: the state goes in rotated, a stabilizer's moments come back in the body frame, and
  config, params, and autotune probes pass through. `View::tailsitter` (90° in pitch) flies a
  tailsitter's nose-up hover on the copter controllers, as ArduPilot's tailsitter view does.
- **`VehicleState::rotate_body`:** the state in another body frame (attitude, body rates, Euler
  fields), for a constant mount or view rotation.
- **`ActuatorCommand::SLOTS`:** the number of motor and of servo slots (8).
- **`ThrustLimits`** (#123): `CopterGuidanceConfig::thrust` bounds guidance's thrust while
  flying, all off by default. **Thrust floor:** `min_thrust` (param `GUID_THR_MIN`, default
  0: none) holds every thrust at or above it while flying, so a tailsitter's elevons keep the
  propwash they need to hold the nose up (as ArduPilot's `Q_TAILSIT_MIN_VO`). A piloted
  throttle at or below `IDLE_THROTTLE` (0.05) passes through, so the motors can idle on the
  ground; waypoint and land flight always get it. Raising a thrust flags `thrust` saturation.
  **Sink guard and boost ceiling:** piloted flight sinking faster than `sink_limit_mps`
  (`GUID_SINK_LIM`) gets `sink_gain` (`GUID_SINK_GAIN`) more thrust per m/s past it, since
  direct throttle can't hold a descent rate and a sinking tailsitter loses its elevons; `max_boost_thrust` (`GUID_BOOST_MAX`) stops the attitude-mode
  tilt boost short of full thrust, keeping motor headroom for the moments.
- **`Setpoint::thrust`:** the linear thrust either variant carries.

### Changed

- **Breaking:** `ControlOutput` has a `cruise` field (`Option<CruiseOutput>`, #391): a
  lift+cruise VTOL's pusher thrust and control-surface moments, flown alongside the lift
  rotors' `thrust` and `moments`. Struct literals add `cruise: None` or
  `..Default::default()`; `ControlOutput::new` sets `None`. `View` rotates the cruise moments
  with the others.
- **Breaking:** `MixerError::PusherBeforeMotor` (the enum is `#[non_exhaustive]`).
- **Breaking:** `guidance::copter::LAND_LEASH_M` is now `ALTITUDE_LEASH_M`: it leashes waypoint
  descents as well as lands (#400).
- Tailsitter back transitions add thrust while sinking by default (the sink guard, #343): a
  `TransitionConfig` built on `..Default::default()` gets it with no change.
  `back_sink_limit_mps: 0.0` restores the fixed `back_thrust`.
- **Breaking:** `AttitudeSetpoint`, `RateSetpoint`, and `BodyRateSetpoint` have an `idle`
  field, read through `Setpoint::idle()` (#326): struct literals add `idle: false` or
  `..Default::default()`. `CopterGuidance` sets it while a piloted throttle is at or below
  `IDLE_THROTTLE` (or NaN), and `CopterStabilizer` clears its integrals on it instead of on
  zero thrust. A disarmed tick's telemetry setpoint is idle.
- **Breaking:** `AttitudeSetpoint` and `PlaneSetpoint` have a `transition` field (#388):
  struct literals add `transition: false` or `..Default::default()`. A tailsitter ramp sets it,
  and the stabilizer saturates its pitch at ±π/2 instead of `max_angles_rad` or
  `max_pitch_rad` (ArduPilot ramps past its limits the same way).
- **Breaking:** `AutotuneEvent::Lowered`. A search whose start is above its maximum (the
  relay's ceiling) lowers it to the relay's rule gains, P at most the ceiling, and commits
  them before anything else flies, so the vehicle settles on them and they stay if the stage
  keeps or fails; a gain above the ceiling could ring once disturbed and fail the stage
  `NotSettled` (#375).
- **Breaking:** `AutotuneEvent` is `#[non_exhaustive]`: a `match` on it needs a wildcard arm,
  and new events stop being breaking.
- An autotune relay that can't settle starts at `Envelope::settle_timeout_s` instead of
  failing the stage: it replaces the loop's output, so a loop ringing on its own gains still
  measures (#375).
- `WaypointFollower` reaches a waypoint only within an altitude band as well as the
  horizontal radius (or past it), as PX4 does: default `DEFAULT_ALTITUDE_ACCEPTANCE_M` 0.8 m,
  `with_altitude_acceptance`/`set_altitude_acceptance` to change it, and
  `PLANE_ALTITUDE_ACCEPTANCE_M` 10 m for wing-borne legs.
- **Breaking:** `LateralController::control` takes `limit`, the acceleration the caller last
  capped; the integral doesn't grow along it. `CopterGuidance` passes its capped tilt request
  (#357).
- `LateralControllerConfig::default()` has a velocity I of 0.4 and `max_integral` 12.5 m (±5
  m/s², the default tilt cap), so a waypoint hold trims a steady wind instead of settling
  downwind (#357).
- `PlaneStabilizerConfig`, `PlaneGuidanceConfig`, and `TransitionConfig` check each field on
  its own (no cross-field rules), so each param's range is fixed; out-of-order airspeeds
  clamp, and a `forward_max_tilt_rad` below `forward_done_rad` leaves the forward transition
  to its timeout.

- `CopterStabilizer` takes its attitude error from the attitude quaternion (#124): the
  shortest tilt of the thrust axis onto the setpoint's (roll and pitch relative to the current
  heading), then the heading error weighted by the cosine of the tilt, as body rates. Near
  level it matches the per-axis Euler difference it replaces; far over (a tailsitter's view
  in transition, view pitch toward ±90°) it stays defined and no longer treats Euler rates as
  body rates.

- **Breaking:** `Guidance::guide` takes the requested `FlightRegime` after `dt` (#181). A
  copter-only guidance ignores it.
- **Breaking:** `Command`, `VehicleState`, `TelemetryRecord`, and `testing::RecordedTick` have
  new fields (`regime`, `airspeed_mps`, `flight_mode`); struct literals need them or
  `..Default::default()`.

- **`CopterStabilizer` clears its integrals at zero thrust** (#123): a setpoint with thrust 0
  (or NaN) zeroes the rate, attitude, and yaw I terms after the tick, so they don't wind up
  against a resting attitude on the ground and kick at liftoff (as ArduPilot does at zero
  throttle).
- **Breaking:** `CopterGuidanceConfig` has a new `thrust` field (`ThrustLimits`, #123); struct
  literals add `..Default::default()` (no limits, the previous behavior).

- **Breaking:** roll and pitch have separate gains in the rate and attitude loops (#289).
  `BodyRateControllerConfig` and `AttitudeControllerConfig` replace `gains` with `roll_gains`
  and `pitch_gains`, and the params `STAB_RATE_P/I/D` and `STAB_ATT_P/I/D` become
  `STAB_RATE_RLL_*`, `STAB_RATE_PIT_*`, `STAB_ATT_RLL_*`, and `STAB_ATT_PIT_*`. To keep a
  shared tune, set both to the old value.

### Removed

- **Breaking:** `AttitudeCommand::mode` and `RateCommand::mode` (unspecified, never read)
  and `WaypointCommand::roll_rad` (never read) (#181). The regime is `Command::regime`.

- **Breaking:** `mixers::QuadCopterMixer` (#245). Use `TableMixer::quad_x()`, which mixes the
  same; its type is `TableMixer<4>`.
- **Breaking:** the `mixers::TailSitterMixer` placeholder (#121). Use
  `TableMixer::tailsitter_2p()`.

### Fixed

- `CopterGuidance` waypoint descents keep their altitude target at most `ALTITUDE_LEASH_M`
  (1 m) below the vehicle (#400). A hover that starts its descent still climbing, as a
  tailsitter does after a back transition, no longer sinks past `max_descent_mps` once the
  climb stops (sim fast ramp: 3.89 → 3.35 m/s against a 3 m/s limit; 4.31 → 3.29 with the
  eXplora's thrust floor).
- `CopterStabilizer` keeps its integrals at zero thrust in flight (#326): the altitude loop
  shooting up past a waypoint no longer drops the yaw trim, and a tailsitter's thrust handoff
  after a back transition no longer steps the moments. A piloted throttle resting just above
  zero (up to `IDLE_THROTTLE`) on the ground now clears them.
- A tailsitter completes its transitions below its stabilizers' limits (#388): the back
  ramp no longer stops at the plane pitch cap (30° by default) and flies to the 15 s timeout,
  nor the forward ramp at the hover tilt cap.
- Autotune's gain search tunes down from a too-high start (#361): the result is held to the
  baseline only when the baseline met the criterion (a too-high rate P settles fast and kept
  itself as `Kept(Worse)`), a start above the search's maximum flies no baseline, and a
  baseline whose steps keep clipping no longer fails the stage `Invalid`.
- A waypoint straight over the vehicle is no longer reached on the ground: `RouteEnd::Land`
  climbs to it before landing (instead of staying armed on the ground), and `RouteEnd::Hold`
  completes at its altitude (#359).
- The attitude error recovers from exactly inverted: a 180° tilt turns about body `x` instead
  of giving no correction (#124).

## [0.1.0] - 2026-10-10

First release.

### Added

- **Pipeline traits and data:** `Commander`, `StateSource` + `Actuator` (blanket
  `Vehicle`), `Guidance`, `Stabilizer`, `Mixer`, `TelemetryPublisher`, and `Configurable`;
  `Command { arm, input }` with `ControlInput` (attitude, rate, waypoint, offboard absolute
  attitude, land; its kind as `ControlMode` via `mode()`), `Setpoint`, `VehicleState`,
  `ControlOutput`, `ActuatorCommand` (generic `motors` and `servos` slots, their meaning set
  by the mixer), and `TelemetryRecord`. Rustdoc specifies the backend
  frame contract (`VehicleState`) and timing contract (`Vehicle`).
- **`FlightLoop`:** runs one tick per `step(dt)` (command → estimate → guide → stabilize →
  mix → actuate → telemetry) with no clock or allocation. `Command::arm` gates the motors and
  re-primes on arming. Guidance and stabilizer configs can be retuned between ticks.
- **Copter guidance** (`CopterGuidance`):
  - waypoint flight: an altitude loop with gravity feedforward and tilt compensation, a
    position → velocity → acceleration lateral cascade capped at the waypoint's cruise speed,
    a waypoint tilt cap, and a descent limit (`max_descent_mps`: the altitude target slews
    down, so a low waypoint descends instead of cutting thrust);
  - land (`LandCommand`): hold position and descend at `land_speed_mps`;
  - piloted attitude mode with heading hold and throttle tilt boost (none when inverted);
  - rate passthrough;
  - absolute attitude (`AbsoluteAttitudeCommand`: angles flown as given, the heading as the
    heading-hold target, thrust not boosted);
  - heading carried across mode switches.
- **Copter stabilizer** (`CopterStabilizer`): cascaded attitude/heading → body-rate PID, with
  a thrust curve; a non-finite gyro axis zeroes only its own moment. **Controllers** (`controllers`): altitude, lateral, attitude, yaw, and body
  rate, with validated configs (`InvalidConfig`).
- **Mixers:** `QuadCopterMixer` with airmode-style desaturation (moments kept, collective gives
  way, no lift at zero thrust); `TailSitterMixer` placeholder.
- **Commanders:** `WaypointFollower` (allocation-free; loops, holds, or lands at the route's
  end, flies legs at cruise speed, advances past a waypoint missed wide; `RouteEnd::Land`
  disarms on touchdown). `TouchdownDetector` decides touchdown during a land from the vertical
  speed (a descent, then stopped for 1 s). `FailsafeCommander` flies a
  link-fed `LinkedCommander` and falls back to a `Fallback` (`HoldCommander`: hold position,
  or finish an armed land the link dropped)
  when it goes stale, returning after a recovery period; the fallback never arms, a disarm is
  always honored, the primary's arm is synced to the vehicle at the handover
  (`LinkedCommander::sync_arm`), and sticks take back an armed vehicle only once they move
  (`takeover_stick_delta`), reporting `FailsafeState::AwaitingSticks` meanwhile; an armed land
  disarms on touchdown, syncing the primary, whichever commander flies it.
  `ControlInput::is_sticks` names stick-style input and `is_stream` input that must keep
  arriving (sticks and absolute attitudes). `Commander::failsafe` reports the state into
  `TelemetryRecord::failsafe` (`FailsafeState`).
- **Params** (`params`): every guidance and stabilizer gain and limit readable, settable,
  and listable by index or by a MAVLink-sized name (`STAB_RATE_P`, `GUID_TILT_MAX`), with
  angles in degrees. Writes go through `set_config`, so an invalid value is rejected. `FlightLoop`
  exposes both components' params. Retuning an integral gain carries the I term over.
- **Timing** (`timing::Every`): dt-driven fixed-rate scheduling for streams slower than the
  loop, without drift.
- **Saturation reporting:** per-tick `Saturation` flags (tilt, thrust, rate-loop moments,
  mixer) from each stage, merged into `TelemetryRecord`.
- **Composition:** `ComposedVehicle` (separate state source + actuator) and tuple fan-out of
  telemetry publishers.
- **`testing` feature:** allocation-free doubles (`NullVehicle`, `ConstantCommander`,
  `RecordingTelemetry`), `conformance` scenarios with `conformance_tests!` for backends, and
  `settled_yaw_rate` for checking rate gains.
- **`param-metadata` feature:** `params::metadata` describes every guidance and stabilizer
  param for ground stations: description, units, category and group, and the valid range
  (`ParamMeta`, `Bound`, `ParamMetadata`), checked against `validate` by tests. Stays `no_std`;
  firmware leaves it off.
- **`serde` feature:** `Serialize`/`Deserialize` on the pipeline data (`Command`,
  `ControlInput`, `Setpoint`, `VehicleState`, `ControlOutput`, `ActuatorCommand`,
  `Saturation`), for wire protocols such as `stable-flight-link`. Stays `no_std`.
- **Math:** `Vec2`/`Vec3`, `GRAVITY`, NaN-safe `euler_angles`, `wrap_angle`, and a `nalgebra`
  re-export.
- **`no_std`**, including the `testing` feature; builds for `thumbv7em-none-eabihf`. The tick
  never panics on bad input (saturate, NaN as neutral), only on `dt <= 0`.

[unreleased]: https://github.com/wboayue/stable-flight/compare/stable-flight-v0.1.0...HEAD
[0.1.0]: https://github.com/wboayue/stable-flight/releases/tag/stable-flight-v0.1.0
