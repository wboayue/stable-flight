# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
  keeps the gains, repeating clipped or stalled steps (`Autotune::note_stall`);
  `Method::Search(SearchPlan)` finds the largest gain of one `Term` whose ± step pairs meet a
  `Criterion`, by bracketing and bisection from gains a relay may seed and cap
  (`SearchRelay`), and keeps it only if its verify flies no worse than a baseline on the start
  gains (`Acceptance`); `StagePlan::copter_rate_search` tunes roll and pitch rate this way
  (relay PD sets D, step overshoot sets P under `0.6·Ku`); candidate gains fly only during
  their step; presets
  `COPTER_RATE_STEP`, `COPTER_ANGLE_STEP`, `COPTER_STEP_SETTLE`; copter defaults in `StagePlan::copter_sequence`: roll and pitch rate,
  PD keeping each loop's I; the angle, heading, and yaw-rate stages are opt-in, see
  `COPTER_SEQUENCE`). Each stage's decisions run as a pure procedure; the executor settles
  before each probe (`Envelope::settle`, or the method's `Settle`), aborts on an `Envelope` breach (back to the last committed config),
  commits each stage's gains through `set_stabilizer_config`, and reverts to the starting
  config on disarm; reports `AutotuneEvent`s for logging (`StageStarted`, `Measured`,
  `Tried`, `StageDone` with its `Outcome`, ...). Works with any `CopterTunable` stabilizer (`Tunable` on `TunedLoop` +
  `Configurable`). `FlightLoop`'s accessors and config setters no longer require the pipeline
  bounds.

### Changed

- **Breaking:** roll and pitch have separate gains in the rate and attitude loops (#289).
  `BodyRateControllerConfig` and `AttitudeControllerConfig` replace `gains` with `roll_gains`
  and `pitch_gains`, and the params `STAB_RATE_P/I/D` and `STAB_ATT_P/I/D` become
  `STAB_RATE_RLL_*`, `STAB_RATE_PIT_*`, `STAB_ATT_RLL_*`, and `STAB_ATT_PIT_*`. To keep a
  shared tune, set both to the old value.

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
