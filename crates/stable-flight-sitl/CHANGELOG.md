# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `Command`, `AttitudeCommand`, `RateCommand`, `ControlOutput`, and
  `ActuatorCommand` now derive `Clone` and `Copy` (plain data), so telemetry
  sinks can keep copies of a tick's `TelemetryRecord`.
- `plot_flight` binary renders any waypoint CSV (`flights/*.csv`) as a static 3D
  flight path — horizontal `x`/`y` plane, altitude on the vertical axis, segments
  colored by cruise speed, each waypoint labeled with its 1-based script order.
  Emits SVG or PNG (chosen by output extension), with `--pitch`/`--yaw` flags to
  adjust the 3D camera angle. Lives in the unpublished `stable-flight-tools`
  crate so the library never builds `plotters`:
  `cargo run --bin plot_flight -- flights/figure_eight.csv out.svg`.
- `commanders::script::load_waypoints` public function loads and parses a waypoint
  CSV into a `Vec<WaypointCommand>` without building a `ScriptCommander`.
- `ScriptCommander` loads a CSV waypoint script (`x,y,altitude,cruise_speed`, in
  meters and m/s) from `flights/`, then follows and loops the waypoints by a
  configurable horizontal acceptance radius. Adds the public `from_waypoints`
  constructor, `with_acceptance_radius`, and a typed `ScriptError`.
- `generate_figure_eight` binary (in `stable-flight-tools`) writes a
  figure-eight waypoint script from a Gerono lemniscate. Every dimension is a `clap`
  flag (`--output`, `--waypoints`, `--half-width`, `--height-scale`, `--alt-mid`,
  `--alt-amp`, `--cruise`), defaulting to a consumer-drone envelope (~60 m × ~30 m,
  altitude sweeping 15–40 m): `cargo run --bin generate_figure_eight`.
- `flights/figure_eight.csv` scripted figure-eight flight path.
- `WaypointCommand` now derives `Clone` and `Copy`.
- Package metadata (`description`, `authors`, `license`, `repository`) in
  `Cargo.toml`.

### Changed

- **Breaking:** `SpectrumCommander` renamed `SpektrumCommander` (the brand's spelling)
  ([#85](https://github.com/wboayue/stable-flight/issues/85)).
- **Breaking:** `SpektrumCommander::new` returns `Result<Self, CommanderError>` instead of
  panicking when Gilrs fails to initialize, and `read_channels` takes `&mut self`
  (matching `Extreme3DCommander`) ([#83](https://github.com/wboayue/stable-flight/issues/83)).
- `ScriptCommander` fills the core's unit-suffixed `WaypointCommand` fields
  (`position_m`, `altitude_m`, `cruise_speed_mps`); the CSV columns are unchanged
  ([#161](https://github.com/wboayue/stable-flight/issues/161)).
- **Breaking:** `MavlinkPublisher::connect(config) -> Result<_, MavlinkError>` and
  `MavlinkPublisher::with_connection(connection, system_id, component_id)` replace
  `MavlinkPublisher::new`,
  which panicked when the connection failed to open. `MavlinkConfiguration` now holds
  the `address` (default `DEFAULT_ADDRESS`, `udpout:127.0.0.1:14550`), `system_id`, and
  `component_id` (default 1), and implements `Default`. The `send_*` and `time_boot_ms`
  methods are private; `publish` is the only entry point
  ([#163](https://github.com/wboayue/stable-flight/issues/163)).
- **Breaking:** state and waypoint vectors are `nalgebra` types (via the core's
  `Vec2`/`Vec3` aliases and `stable_flight::nalgebra`) instead of `robomath`, which is
  no longer a dependency ([#87](https://github.com/wboayue/stable-flight/issues/87)).
  See the core changelog for the migration.
- **Breaking:** `FlightOrchestrator` (and the `orchestrator` module) is replaced
  by three pieces with one job each: the core `stable_flight::FlightLoop` (one
  tick), `RealTimeRunner` (wall-clock pacing), and `ShutdownSignal` (graceful
  stop). Migrate `FlightOrchestrator::new(c, v, s, m, t).run(hz)` to
  `RealTimeRunner::new(hz)?.run(&mut FlightLoop::new(c, v, g, s, m, t),
  &ShutdownSignal::install()?)`, where `g` is a `Guidance` such as the core
  `CopterGuidance` (split out of the stabilizer). `RealTimeRunner::new` returns
  `InvalidFrequency` for a non-positive or non-finite rate, or one whose period
  isn't a representable non-zero `Duration`, instead of panicking in `run`, and
  `run` returns `RunStats`. Building a loop no longer installs a signal handler.
  A second Ctrl-C while shutdown is pending now terminates the process, so a
  wedged loop can still be killed.
- `ScriptCommander` now delegates waypoint following to the core
  `stable_flight::commanders::WaypointFollower`; its API and behaviour are
  unchanged.
- **Breaking:** the flight-control core moved to the new `stable-flight` crate:
  the pipeline traits and data types formerly at this crate's root (`Commander`,
  `Vehicle`, `StateEstimator`, `Actuator`, `Stabilizer`, `Mixer`,
  `TelemetryPublisher`, `Command`, `VehicleState`, `ControlOutput`,
  `ActuatorCommand`, `TelemetryRecord`, …) and the `stabilizers` and `mixers`
  modules. Import them from `stable_flight` (e.g. `use stable_flight::{Commander,
  VehicleState}`, `stable_flight::mixers::QuadCopterMixer`). No re-exports are
  provided beyond the crate itself (`stable_flight_sitl::stable_flight`). Entries
  below that describe those items now apply to `stable-flight`.
- **Breaking:** `TelemetryPublisher::publish` takes a single `&TelemetryRecord`
  (new public struct borrowing the tick's `dt`, `command`, `state`, `control`
  output, and `actuators`) instead of `(dt, command, state)`. Publishers can now
  see stabilizer and mixer output, and new fields no longer change the signature.
- **Breaking:** `Vehicle` is now `Vehicle: StateEstimator + Actuator` with a
  blanket impl. Implement `StateEstimator` and `Actuator` instead of `Vehicle`, and
  import those traits where `estimate_state`/`actuate` are called on a concrete
  type. `StateEstimator::estimate_state` now takes `&mut self` to match.
- **Breaking:** trait methods renamed for clarity: `Commander::next` →
  `next_command`, `Stabilizer::set_initial_state` → `prime`, and
  `TelemetryPublisher::send_telemetry` → `publish`. Update implementations and
  call sites. `next_command` and `prime` keep their signatures; `publish` also
  changed shape (see the `TelemetryRecord` entry above).
- The repository is now the `stable-flight` Cargo workspace; this crate lives in
  `crates/stable-flight-sitl` and its `repository` URL points at
  `github.com/wboayue/stable-flight`. The dev binaries moved to the unpublished
  `stable-flight-tools` crate, so this crate no longer has `plot`/`tools`
  features or `plotters`/`clap` dependencies.
- **Breaking:** `ScriptCommander::new` now returns `Result<Self, ScriptError>`
  and reads the given CSV file, instead of being infallible and returning a
  single fixed waypoint.
- The `copter-auto-realflight` example flies the scripted figure-eight via
  `ScriptCommander` rather than the joystick commander. Each RealFlight example
  is now self-contained (no shared wiring module).
- The RealFlight examples parse arguments with `clap`: both accept `--host`
  (default `10.211.55.3:8080`) instead of a hard-coded address, and
  `copter-auto-realflight` takes the script path as its first argument (default
  `flights/figure_eight.csv`).
- Upgraded `mavlink` requirement from 0.16.1 to 0.17.1. `mavlink::connect` now
  returns a concrete connection that `MavlinkPublisher` boxes internally.
- Upgraded `uom` requirement from 0.37.0 to 0.38.0, together with
  `realflight-bridge` 1.0.0 → 1.1.0 so both share the same `uom` version.

### Removed

- **Breaking:** `Default` impls for `Extreme3DCommander` and `SpektrumCommander`; they
  panicked on joystick init failure. Use the fallible `new()`
  ([#83](https://github.com/wboayue/stable-flight/issues/83)).
- **Breaking:** the `simulators` module and the RealFlight backend
  (`RealFlightVehicle`, `RealFlightConfiguration`, `ActuatorMapping`), along with
  the `realflight-bridge` dependency and the `copter-*-realflight` examples. They
  moved to the new `stable-flight-sitl-realflight` crate:
  `use stable_flight_sitl_realflight::{ActuatorMapping, RealFlightConfiguration, RealFlightVehicle};`.
- Unused `env_logger` dependency; `uom` is now a dev-dependency only.
- **Breaking:** `simulators::NullVehicle`. Use `stable_flight::testing::NullVehicle`
  (enable the core `testing` feature in `[dev-dependencies]`), which also counts
  calls and records the last actuator command.
- **Breaking:** the empty `stabilizers::vtol` module. It held only a reference
  link; no VTOL stabilizer exists yet.
- **Breaking:** the public `ScriptCommander::path` field; the commander now owns
  an internal waypoint list.
- Unused `rand` dependency.
- **Breaking:** `telemetry::file_logger::FileLogger`. Its `publish` ignored the
  record and stalled the loop ~10 s per tick writing placeholder heartbeats; it
  was never wired into a flight loop. The `byteorder` dependency goes with it.

### Fixed

- `MavlinkPublisher` follows the `VehicleState` frame contract
  ([#160](https://github.com/wboayue/stable-flight/issues/160)): LOCAL_POSITION_NED `z`/`vz`
  are down-positive (they were up); ATTITUDE reports roll, pitch, and yaw in radians (yaw
  wrapped to [-π, π]) instead of raw quaternion components; VFR_HUD heading is whole
  degrees 0–359 (it was a quaternion component), and ground speed and airspeed are the
  horizontal speed (they were north and east velocity).
- `MavlinkPublisher` streams keep their rate: leftover tick time carries into the next
  period, so the 10 Hz streams no longer run slow when the loop period doesn't divide
  100 ms (about 8.6 Hz at 60 Hz). The heartbeat is sent at 1 Hz (it was 10 Hz), and
  VFR_HUD has its own timer. Every stream sends on the first `publish`, so a ground
  station sees the link immediately ([#163](https://github.com/wboayue/stable-flight/issues/163)).
- Scripted (waypoint) flights no longer crash the loop when the vehicle is far
  from the active waypoint, joystick flights no longer crash after holding yaw
  for a few minutes, and out-of-range stick input or a NaN estimate no longer
  crashes the loop; see the `stable-flight` changelog.
- `SpectrumCommander` clamps its scaled channels to [0, 1]; the configured
  ranges are narrower than the sticks' travel, so values overshot to ~1.2.
- A failure to register the SIGINT handler is now returned from
  `ShutdownSignal::install` instead of panicking on a background thread. The
  Windows console handler no longer uses a `static mut` raw pointer.
- MAVLink `VFR_HUD` throttle now reports the stabilizer's thrust as 0–100 %
  for every command kind. It previously read the command's throttle and
  truncated it to 0 or 100, and reported 0 (with a `println!` every send) for
  waypoint commands. **Breaking:** the public helper
  `MavlinkPublisher::send_vfr_hud` now takes `&ControlOutput` instead of
  `&Command`.
- `MavlinkPublisher` no longer panics the flight loop on a failed
  send/write/flush or a pre-epoch system clock. Failures are logged at `warn`
  and the tick continues. Logging is rate-limited per publisher: the first
  failure is reported, then one per 250 consecutive failures with a count of
  those suppressed. A success resets the count.

[unreleased]: https://github.com/wboayue/stable-flight/compare/main...HEAD
