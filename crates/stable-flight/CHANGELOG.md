# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Not yet released. Until the first release this section summarizes what the crate provides,
not each change; per-change entries start after it.

### Added

- **Pipeline traits and data:** `Commander`, `StateSource` + `Actuator` (blanket
  `Vehicle`), `Guidance`, `Stabilizer`, `Mixer`, `TelemetryPublisher`, and `Configurable`;
  `Command { arm, input }` with `ControlInput` (attitude, rate, waypoint, offboard absolute
  attitude, land; its kind as `ControlMode` via `mode()`), `Setpoint`, `VehicleState`,
  `ControlOutput`, `ActuatorCommand`, and `TelemetryRecord`. Rustdoc specifies the backend
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
  a thrust curve. **Controllers** (`controllers`): altitude, lateral, attitude, yaw, and body
  rate, with validated configs (`InvalidConfig`).
- **Mixers:** `QuadCopterMixer` with airmode-style desaturation (moments kept, collective gives
  way, no lift at zero thrust); `TailSitterMixer` placeholder.
- **Commanders:** `WaypointFollower` (allocation-free; loops, holds, or lands at the route's
  end, flies legs at cruise speed, advances past a waypoint missed wide; `RouteEnd::Land`
  disarms on touchdown). `TouchdownDetector` decides touchdown during a land from the vertical
  speed (a descent, then stopped for 1 s). `FailsafeCommander` flies a
  link-fed `LinkedCommander` and falls back to a `Fallback` (`HoldCommander`: hold position)
  when it goes stale, returning after a recovery period; the fallback never arms, a disarm is
  always honored, the primary's arm is synced to the vehicle at the handover
  (`LinkedCommander::sync_arm`), and sticks take back an armed vehicle only once they move
  (`takeover_stick_delta`), reporting `FailsafeState::AwaitingSticks` meanwhile.
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

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight
