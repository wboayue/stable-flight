# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Not yet released. Until the first release this section summarizes what the crate provides,
not each change; per-change entries start after it.

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
    `WaypointFollower`; loop or hold at the end; `load_waypoints` for other tools.

  Fallible constructors return typed errors (`CommanderError`, `ScriptError`,
  `InvalidFrequency`).
- No telemetry publisher of its own: SITL telemetry goes over the native link
  (`stable-flight-link`), and `stable-flight-bridge` translates it to MAVLink.
- Re-exports the core crate as `stable_flight_sitl::stable_flight`.
- Examples: `extreme_3d_commander` (joystick smoke test), `spektrum_commander` (raw dongle
  values next to the command, for checking the mapping).

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight-sitl
