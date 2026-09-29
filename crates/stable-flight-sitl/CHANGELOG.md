# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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

- **Breaking:** the empty `stabilizers::vtol` module. It held only a reference
  link; no VTOL stabilizer exists yet.
- **Breaking:** the public `ScriptCommander::path` field; the commander now owns
  an internal waypoint list.
- Unused `rand` dependency.

### Fixed

- `MavlinkPublisher` and `FileLogger` no longer panic the flight loop on a failed
  send/write/flush or a pre-epoch system clock. Failures are logged at `warn`
  and the tick continues. Logging is rate-limited per publisher: the first
  failure is reported, then one per 250 consecutive failures with a count of
  those suppressed. A success resets the count.

[unreleased]: https://github.com/wboayue/stable-flight/compare/main...HEAD
