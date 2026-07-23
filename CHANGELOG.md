# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `plot_flight` binary renders any waypoint CSV (`flights/*.csv`) as a static 3D
  flight path — horizontal `x`/`y` plane, altitude on the vertical axis, segments
  colored by cruise speed. Emits SVG or PNG (chosen by output extension), with
  `--pitch`/`--yaw` flags to adjust the 3D camera angle. Gated behind the optional
  `plot` feature so the default library build stays lean:
  `cargo run --features plot --bin plot_flight -- flights/figure_eight.csv out.svg`.
- `commanders::script::load_waypoints` public function loads and parses a waypoint
  CSV into a `Vec<WaypointCommand>` without building a `ScriptCommander`.
- `ScriptCommander` loads a CSV waypoint script (`x,y,altitude,cruise_speed`, in
  meters and m/s) from `flights/`, then follows and loops the waypoints by a
  configurable horizontal acceptance radius. Adds the public `from_waypoints`
  constructor, `with_acceptance_radius`, and a typed `ScriptError`.
- `generate_figure_eight` example that writes `flights/figure_eight.csv` from a
  Gerono lemniscate sized for a consumer-drone envelope (~60 m × ~30 m, altitude
  sweeping 15–40 m).
- `flights/figure_eight.csv` scripted figure-eight flight path.
- `WaypointCommand` now derives `Clone` and `Copy`.
- Package metadata (`description`, `authors`, `license`, `repository`) in
  `Cargo.toml`.

### Changed

- **Breaking:** `ScriptCommander::new` now returns `Result<Self, ScriptError>`
  and reads the given CSV file, instead of being infallible and returning a
  single fixed waypoint.
- The `copter-auto-realflight` example flies the scripted figure-eight via
  `ScriptCommander` rather than the joystick commander. Each RealFlight example
  is now self-contained (no shared wiring module).
- Upgraded `mavlink` requirement from 0.16.1 to 0.17.1. `mavlink::connect` now
  returns a concrete connection that `MavlinkPublisher` boxes internally.
- Upgraded `uom` requirement from 0.37.0 to 0.38.0, together with
  `realflight-bridge` 1.0.0 → 1.1.0 so both share the same `uom` version.

### Removed

- **Breaking:** the public `ScriptCommander::path` field; the commander now owns
  an internal waypoint list.
- Unused `rand` dependency.

[unreleased]: https://github.com/wboayue/stable-flight-sitl/compare/main...HEAD
