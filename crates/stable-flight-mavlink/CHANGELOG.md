# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2026-10-30

### Added

- `vtol_state(FlightMode)` (#333): a VTOL's `MAV_VTOL_STATE` (`MC`, `FW`, and the
  transitions), as PX4 maps it.

### Changed

- **Breaking:** `extended_sys_state` takes the `vtol_state` to report (#333);
  `MAV_VTOL_STATE_UNDEFINED` for a vehicle that isn't a VTOL.
- `heartbeat` maps `ControlMode::AbsoluteRate` to `GUIDED` (offboard rates, neither manual
  nor stabilized) (#124).
- `vfr_hud` reports `VehicleState::airspeed_mps` as airspeed when present (#181); ground speed
  without it (`None`, NaN, or negative).

## [0.1.0] - 2026-10-10

First release.

### Added

- **`heartbeat`:** `HEARTBEAT` for a generic autopilot of a given `MavType`, reporting the arm
  state and the core `ControlMode` as standard `base_mode` flags (land reads as auto, like
  waypoints).
- **State builders:** `attitude`, `local_position_ned`, `global_position_int`, `home_position`,
  `vfr_hud`, and `extended_sys_state`, with the core's frames (north/east/up, FRD rates)
  converted to MAVLink's (NED) and fixed-point units; NaN reports MAVLink's "unknown" where it
  has one.
- **`saturation`:** one `NAMED_VALUE_INT` (0/1) per core `Saturation` flag (`SAT_TILT`, …),
  each its own chartable entry in a ground station.
- **Geodetic origin:** `GeodeticOrigin`, `DEFAULT_ORIGIN`, and `FlatEarth`, a flat-earth
  projection of local position for global position and home; `FlatEarth::new` rejects an
  out-of-range origin with `InvalidOrigin`.
- **`HomeTracker`, `LandDetector`, and `GroundTracker`:** home latched at each arming, the
  landed state from the arm state, height above home, and land mode (`LANDING`), and both together with change flags.
- Builders return payloads (`*_DATA`) and take vehicle identity as a parameter.
- **`mavlink` re-export**, so callers use the version the builders return.
- **`testing` feature:** `testing::MockConnection`, a scripted `MavConnection` (receives in
  order, then `WouldBlock`; sends recorded, optionally failing) for tests.

[unreleased]: https://github.com/wboayue/stable-flight/compare/stable-flight-mavlink-v0.2.0...HEAD
[0.2.0]: https://github.com/wboayue/stable-flight/releases/tag/stable-flight-mavlink-v0.2.0
[0.1.0]: https://github.com/wboayue/stable-flight/releases/tag/stable-flight-mavlink-v0.1.0
