# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `ScriptCommander::anchored` flies a script relative to where the vehicle starts (#494): `x`,
  `y`, and altitude become meters north, east, and up of its first position, so one script flies
  the same route at any spawn point or field. `ScriptCommander::origin_m` reports the start.

- A script's eighth field is a maneuver flown on reaching a `plane` row's waypoint, before its
  hold (#342): `roll:<deg/s>`, `loop:<m>` (negative: outside), and `doublet_roll`,
  `doublet_pitch`, `doublet_yaw` (`:<deg/s>[:<half width s>]`).
  `ScriptCommander::with_maneuver_config` sets their thrusts, path hold, and recovery.

- Every recorder column set records guidance's wind estimate (#473): `wind_north_mps`,
  `wind_east_mps`, `wind_sd_north_mps`, `wind_sd_east_mps`, NaN without one.
- `Columns::header` (#473): the full header `CsvRecorder` writes, `HEADER` by default, so a
  set that wraps another can join both.
- `cli::parse_param`: a `--param NAME=VALUE` argument as a name and value (a clap
  `value_parser`; `ParamArgError` when it doesn't parse), shared by the simulator examples
  that set params before takeoff (#417).

- `joystick_events` example: every gilrs event as CSV, and each gamepad's mapping source and
  native axis and button codes; `--raw` without gilrs's default filters (#446).

- `recorder::QuadPlaneColumns`: a lift+cruise VTOL's hover, transitions, and plane flight:
  both sides' moments and thrust, the cruise output, every lift rotor, the pusher, the
  surfaces, the setpoint, a blend's lift share, and the flight mode (#433).

- `recorder::CsvRecorder`: a `TelemetryPublisher` writing one CSV row per tick (`wall_s`, then a
  `Columns` set) to a file or any writer, or nothing without one; write failures are ignored.
  Column sets `CopterColumns` and `TailsitterColumns` (#337); both include horizontal position
  and ground velocity (`north_m`, `east_m`, `vel_north_mps`, `vel_east_mps`). A
  `Setpoint::Blend` records as `sp_kind` `blend`, with its hover attitude (#391).

- `ScriptCommander` reads an optional fifth CSV field per waypoint, `copter` or `plane`: the
  regime of the leg to it, requested as the leg begins (#124). `ScriptCommander::from_route`
  takes waypoints with their regimes.
- `ScriptCommander` reads two more optional CSV fields per waypoint (#470): `heading`, in
  degrees from north (`WaypointCommand::heading_rad`), and `hold_s`, seconds to keep
  commanding the waypoint once reached (or before the route's land), stopped on it. An empty
  optional field, `regime` included, takes its default.
- `ScriptCommander` reaches each waypoint within its leg regime's altitude band (copter 0.8 m,
  plane 10 m, as PX4), set with `with_altitude_acceptance(regime, altitude_m)` (#359).

### Changed

- Commanders request a regime (#124): `Extreme3DCommander` D-pad Left asks for copter and
  Right for plane (both attitude sticks), Up keeps the regime with rate sticks;
  `SpektrumCommander`'s mode switch asks for plane at one end (an unread switch is copter).

- `SpektrumCommander` and `Extreme3DCommander` no longer fill `AttitudeCommand::mode`/
  `RateCommand::mode` (removed from core, #181); the Spektrum mode switch is read into channel
  `[5]` but not mapped.

- **Breaking:** `RealTimeRunner::run_with`'s hook also gets the `RunStats` so far
  (`FnMut(&mut FlightLoop, &RunStats)`), so a caller can react to a stall between ticks
  (#298). Add a `_` parameter to existing closures.

## [0.1.0] - 2026-10-10

First release.

### Added

- **`RealTimeRunner`:** paces a core `FlightLoop` at a fixed rate on the wall clock, resyncs
  after a stall instead of replaying missed ticks, and returns `RunStats` (counting overruns); `run_with` adds a hook called with the loop before each tick (param writes,
  config swaps). **`ShutdownSignal`:** stops it cleanly on Ctrl-C or SIGTERM (or a manual trigger).
- **Commanders:**
  - `Extreme3DCommander`: Logitech Extreme 3D joystick; attitude/rate modes on the hat, arm
    and disarm buttons.
  - `SpektrumCommander`: Spektrum transmitter via the WS3000T USB dongle; signed stick axes,
    arm on the gear switch.
  - `ScriptCommander`: CSV waypoint scripts (`x,y,altitude,cruise_speed`) over the core
    `WaypointFollower`; loop, hold, or land at the end; `load_waypoints` for other tools.

  Fallible constructors return typed errors (`CommanderError`, `ScriptError`,
  `InvalidFrequency`).
- No telemetry publisher of its own: SITL telemetry goes over the native link
  (`stable-flight-link`), and `stable-flight-bridge` translates it to MAVLink.
- Re-exports the core crate as `stable_flight_sitl::stable_flight`.
- Examples: `extreme_3d_commander` (joystick smoke test), `spektrum_commander` (raw dongle
  values next to the command, for checking the mapping).

[unreleased]: https://github.com/wboayue/stable-flight/compare/stable-flight-sitl-v0.1.0...HEAD
[0.1.0]: https://github.com/wboayue/stable-flight/releases/tag/stable-flight-sitl-v0.1.0
