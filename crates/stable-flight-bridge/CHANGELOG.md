# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Not yet released. Until the first release this section summarizes what the crate provides,
not each change; per-change entries start after it.

### Added

- **Bridge binary:** native link (UDP, `--link-bind`/`--link-peer`) ⇄ MAVLink v2
  (`--gcs`, any `mavlink::connect` address), presenting the flight controller as system 1,
  component 1.
- **Heartbeat:** `HEARTBEAT` at 1 Hz from the flight controller's (arm state and mode flags, built
  by `stable-flight-mavlink`), only while it is alive.
- **Params:** `PARAM_REQUEST_LIST` and `PARAM_SET` forwarded; the flight controller's params
  and write echoes sent as `REAL32` `PARAM_VALUE`s with their index and count;
  `PARAM_REQUEST_READ` served from a cache, refilled whenever the flight controller appears.
- **Loop:** reads both sides in bounded batches each pass; sleeps 1 ms under traffic and backs
  off to 8 ms while idle. Failures are logged at most once a second per side.
- **State:** `ATTITUDE`, `LOCAL_POSITION_NED`, `GLOBAL_POSITION_INT`, and `VFR_HUD` at 10 Hz
  from the flight controller's latest state; `HOME_POSITION` and `EXTENDED_SYS_STATE` at 1 Hz
  and on change (payloads, home, and landed state from `stable-flight-mavlink`). Global position
  and home projected around `--origin` (or none with `--no-origin`); throttle from telemetry
  records.
- **Arming:** `MAV_CMD_COMPONENT_ARM_DISARM` forwarded as `Control::Arm`, acked once the flight
  controller's heartbeat confirms it (`FAILED` after 1 s).
- **Link heartbeat:** `Control::Heartbeat` to the flight controller at 5 Hz while the ground
  station is alive, so losing it triggers the flight controller's failsafe.
- **QGC connect sequence:** empty `MISSION_COUNT` per mission type, `AUTOPILOT_VERSION` and
  `COMPONENT_METADATA` on request, `UNSUPPORTED` for other commands.
- **Param metadata:** descriptions, units, groups, and ranges from the core's param tables,
  served to QGC as component metadata (plain JSON over HTTP at CRC-versioned paths;
  `--http-bind`, `--http-url`, `--no-metadata`).

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight-bridge
