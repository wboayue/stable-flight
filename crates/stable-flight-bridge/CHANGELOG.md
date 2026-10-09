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
- **QGC connect sequence:** empty `MISSION_COUNT` per mission type, `AUTOPILOT_VERSION` on
  request, `UNSUPPORTED` for other commands.

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight-bridge
