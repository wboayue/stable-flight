# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Not yet released. Until the first release this section summarizes what the crate provides,
not each change; per-change entries start after it.

### Added

- **`heartbeat`:** `HEARTBEAT` for a generic autopilot of a given `MavType`, reporting the arm
  state and the core `ControlMode` as standard `base_mode` flags.
- Builders return payloads (`*_DATA`) and take vehicle identity as a parameter.
- **`mavlink` re-export**, so callers use the version the builders return.

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight-mavlink
