# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Not yet released. Until the first release this section summarizes what the crate provides,
not each change; per-change entries start after it.

### Added

- **`RealTimeRunner`:** paces a core `FlightLoop` at a fixed rate on the wall clock and returns
  `RunStats`; `run_with` adds a hook called with the loop before each tick (param writes,
  config swaps). **`ShutdownSignal`:** stops it cleanly on Ctrl-C (or a manual trigger).
- **Commanders:**
  - `Extreme3DCommander`: Logitech Extreme 3D joystick; attitude/rate modes on the hat, arm
    and disarm buttons.
  - `SpektrumCommander`: Spektrum transmitter via a USB dongle.
  - `ScriptCommander`: CSV waypoint scripts (`x,y,altitude,cruise_speed`) over the core
    `WaypointFollower`; loop or hold at the end; `load_waypoints` for other tools.

  Fallible constructors return typed errors (`CommanderError`, `ScriptError`,
  `InvalidFrequency`, `MavlinkError`).
- **Telemetry:** `MavlinkPublisher` streams `HEARTBEAT`, `ATTITUDE`, `LOCAL_POSITION_NED`,
  `GLOBAL_POSITION_INT`, `HOME_POSITION`, `EXTENDED_SYS_STATE`, and `VFR_HUD` to a ground
  station (e.g. QGroundControl) at fixed rates, with throttled error logging. The heartbeat
  reports the arm state and the command kind as standard mode flags; global and home
  position project local position around a configurable `GeodeticOrigin`, with home latched
  at arming; the landed state comes from the arm state and height above home.
- Re-exports the core crate as `stable_flight_sitl::stable_flight`.
- Examples: `extreme_3d_commander` (joystick smoke test) and `publish_telemetry` (MAVLink demo).

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight-sitl
