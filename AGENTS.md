# AGENTS.md

Guidance for everyone (humans + AI assistants) working in the Stable Flight repo. `architecture.md` covers design rationale.

## Quick facts

- Cargo workspace (virtual manifest at the root, `resolver = "3"`), Rust **edition 2024**, MSRV **1.89** (`rust-version`, set by nalgebra and uuid),
  version `0.1.0`. Shared package metadata and common dependency versions live in the root
  `[workspace.package]` / `[workspace.dependencies]`; crates inherit them with `.workspace = true`.
- Crates:
  - `crates/stable-flight` — the flight-control core: pipeline traits and data types
    (`lib.rs`), `FlightLoop` (one tick), `stabilizers`, `mixers`. No simulator, I/O, clock, or
    thread dependencies, and it is `no_std` — keep it that way (see Coding style).
  - `crates/stable-flight-link` — the `no_std` FC ⇄ companion wire protocol: `Uplink`/`Downlink`
    messages, versioned COBS frames (`Message::encode`/`decode`), the byte-level `Link` (`LinkRx` + `LinkTx`),
    `SendMessage`/`RecvMessage` (encode+send, receive+decode), `router::UplinkRouter` (reads the
    link), `publisher::LinkPublisher` (writes it), `params::ParamServer` (serves core `Params`
    between ticks), `fc::FcLink` (all three wired in one call; `wire_linked` adds a `commander::LinkCommander`), a `std`-feature `UdpLink`, and a
    `testing`-feature `loopback`. Depends only on the core (`serde` feature); other transports
    (USB-CDC) live with their board.
  - `crates/stable-flight-mavlink` — MAVLink message builders: pure functions from core values to
    `mavlink` payloads (`heartbeat`, `state.rs`: attitude, positions, home, VFR_HUD, landed state;
    `saturation.rs`: `Saturation` flags as `NAMED_VALUE_INT`s), plus what they need beyond one `VehicleState` (`geo.rs`: `GeodeticOrigin`/`FlatEarth`;
    `HomeTracker`; `LandDetector`; `GroundTracker` combining them), used by the bridge. Depends
    on core and `mavlink` only.
  - `crates/stable-flight-bridge` — the companion-computer binary: native link (UDP today) ⇄
    MAVLink v2 for ground stations, presenting the FC as sysid 1 / compid 1. Pure `Bridge` (`bridge.rs`:
    FC and GCS liveness, link heartbeat, addressing, routing, the `UNSUPPORTED` fallback for
    `COMMAND_LONG`) over services: `ParamService` (`params.rs`, with the cache),
    `ConnectService` (`connect.rs`: heartbeat, `AUTOPILOT_VERSION`, `COMPONENT_METADATA`,
    missions), `CommandService` (`commands.rs`: arm/disarm and `NAV_LAND`, acked on the FC's heartbeat),
    `SetpointService` (`setpoints.rs`: `MANUAL_CONTROL` → attitude, `SET_POSITION_TARGET_LOCAL_NED`
    → waypoint, `SET_ATTITUDE_TARGET` → absolute attitude), and
    `StateService` (`state.rs`: attitude, positions, home, landed state, `VFR_HUD`, saturation flags, paced by
    `Interval`, `interval.rs`), which queue into an `Outbox` (`outbox.rs`); payload builders in `encode.rs`. `Pump` (`pump.rs`) runs one
    I/O pass (read both sides, route, send) over generic link halves and a boxed GCS connection;
    `IdleBackoff` (`idle.rs`) paces passes (1 ms under traffic, up to 8 ms idle); `main.rs` only
    sets up (`--origin`, `--offboard-speed` parsing in `cli.rs`). Param metadata for QGC: `metadata.rs` (pure: core
    tables → `parameters.json`/`component_general.json`, CRCs), served by `http.rs` (minimal std
    HTTP server thread); `ConnectService` answers `COMPONENT_METADATA`. Params, state, and commands slices of #154; depends on link (`std`), the core (`param-metadata`), `stable-flight-mavlink`, and `mavlink`, not on sitl.
  - `crates/stable-flight-sitl` — the simulator-independent SITL harness: `commanders`,
    `runner` (`RealTimeRunner`, wall-clock pacing of a `FlightLoop`), `shutdown`
    (`ShutdownSignal`). Depends on the core; no telemetry publisher of its own (SITL telemetry goes over the link). Non-simulator examples live in `crates/stable-flight-sitl/examples/*.rs`.
  - `crates/stable-flight-sitl-realflight` — the RealFlight backend: `RealFlightVehicle`,
    `ActuatorMapping`, `airframes` (tuned gains, mixer, and mapping per model), and the
    `copter-*-realflight` and `capture-state-realflight` examples, all serving params and telemetry over the native link (`FcLink`) for the bridge, plus `exchange-latency-realflight` (times the RealFlight round trip alone). Normal deps: core, `realflight-bridge`, `log`; sitl and link (`std`) are dev-dependencies only (CI checks sitl).
    One `stable-flight-sitl-<simulator>` crate per backend.
  - `crates/stable-flight-tools` — unpublished (`publish = false`) dev binaries: `plot_flight`
    (3D flight-path visualizer), `generate_figure_eight` (waypoint-script generator), and `link_pilot` (plays the companion over the native link for `copter-companion-realflight`).
- `firmware/` is excluded from the workspace; each board is its own nested workspace (own
  target, profiles, committed `Cargo.lock`) — see [`firmware/ark-fpv`](firmware/ark-fpv/README.md).
  Run cargo from the board's directory (or use the `fw-*` recipes): its `.cargo/config.toml`
  isn't picked up via `--manifest-path`. Root `cargo fmt --all`/`clippy` don't reach it.
- Run every command from the repo root: `cargo run --example <name>` / `--bin <name>` resolve
  across the workspace (add `-p <crate>` to be explicit), and `flights/*.csv` paths are
  relative to the root.
- Public modules: `stable-flight` — `commanders` (`WaypointFollower`, `TouchdownDetector`, `FailsafeCommander`/`FailsafeConfig`, `LinkedCommander`, `Fallback`, `HoldCommander`), `controllers` (PID loops, `Gains`), `flight_loop`, `guidance` (`CopterGuidance`), `mixers`, `params` (`Params`, `ParamTable`: name → `f32` view over the configs; `params::metadata` (feature `param-metadata`): `ParamMeta`, `ParamMetadata`, descriptions/units/groups/ranges for ground stations), `stabilizers`, `timing` (`Every`), `tuning` (`RelayTuner`/`RelayConfig`: relay-feedback ultimate point; `gains_from_ultimate` with `TuningRule`/`ControllerForm`; `Tunable` + `TuningStatus`; `autotune`: `Autotune` supervisor over any `CopterTunable` stabilizer, `StagePlan`, `Envelope`, `AutotuneEvent`; #149), `testing` (feature: doubles + `conformance`) (traits/types, the `Vec2`/`Vec3` aliases, `GRAVITY`, `euler_angles`, `wrap_angle`, `ControlMode` (`ControlInput::mode()`), `FailsafeState`, and the `nalgebra` re-export at the crate root; feature `serde` derives `Serialize`/`Deserialize` on the pipeline data); `stable-flight-link` — `commander` (`LinkCommander`), `fc` (`FcLink`, `FcLinkConfig`, `FcPoller`), `loopback` (feature `testing`), `params` (`ParamServer`), `publisher` (`LinkPublisher`, `LinkPublisherConfig`), `router` (`UplinkRouter`, `RouteStats`), `udp` (feature `std`: `UdpLink`, `DEFAULT_FC_ADDR`/`DEFAULT_BRIDGE_ADDR`), with `Uplink`, `Control`, `Downlink`, `Output`, `Record`, `ParamRequest`, `ParamValue`, `Message`, `EncodeError`, `DecodeError`, `Link` (`LinkRx` + `LinkTx`, blanket-implemented), `SendMessage`/`RecvMessage` + `LinkError` (blanket-implemented), `PROTOCOL_VERSION`, `MAX_FRAME_LEN`, and the `heapless`/`stable_flight` re-exports at the root; `stable-flight-mavlink` — the builders (`heartbeat`, `attitude`, `local_position_ned`, `global_position_int`, `home_position`, `extended_sys_state`, `vfr_hud`, `saturation` + `SATURATION_FLAGS`), `GeodeticOrigin`, `DEFAULT_ORIGIN`, `FlatEarth`, `InvalidOrigin`, `HomeTracker`, `LandDetector`, `GroundTracker` (`GroundChanges`), and the `mavlink` re-export at the root, `testing` (feature: `MockConnection`); `stable-flight-sitl` — `commanders`, `runner`, `shutdown` (`RealTimeRunner` and `ShutdownSignal` also at the root); `stable-flight-sitl-realflight` — `airframes`, with `RealFlightVehicle`, `RealFlightConfiguration`, `ActuatorMapping`, `MappingError`, `vehicle_state`, `SLOW_EXCHANGE`, and the `realflight_bridge` re-export at the root. sitl re-exports only the core crate itself (`stable_flight_sitl::stable_flight`), not its items: import core items from `stable_flight`.
- Key external crates: `realflight-bridge` (RealFlight I/O, backend crate only), `nalgebra` (core only, `libm`; core aliases `Vec2`/`Vec3` and re-exports it as `stable_flight::nalgebra` — other crates use the re-export), `libm` (`no_std` float math, core), `serde` (core `serde` feature, link), `postcard` + `heapless` + `cobs` (link only), `crc` (link, bridge), `serde_json` (bridge), `mavlink` (`stable-flight-mavlink`, bridge; `emit-extensions` on, so struct literals need v2 extension fields), `gilrs` (joystick), `thiserror`, `signal-hook`. Dev: `approx`, `assert2`, `clap` (examples parse args with it). `stable-flight-tools` adds `plotters` and `clap`.
- `assets/` holds repo images (GitHub social preview options, 1280×640 PNG); not packaged into any crate.
- Untracked generated artifacts: `target/`, `tarpaulin-report.html`, `firmware/*/*.bin`.
- The root `Cargo.lock` is committed (reproducible CI, stable cache key); CI builds with `--locked`,
  so commit lockfile changes with the manifest change that causes them. Dependabot batches weekly
  minor/patch bumps into one PR; Dependabot security updates open their own PRs.
- A dependency bump that needs a newer compiler raises `rust-version` in `[workspace.package]` and
  the MSRV mentions (here, root README prerequisites) in the same PR; check the floor with
  `cargo +<version> check --workspace --all-features --all-targets --locked`.
- Paths below are prefixed with their crate (`core:` = `crates/stable-flight/`, `sitl:` =
  `crates/stable-flight-sitl/`, `sitl-realflight:` = `crates/stable-flight-sitl-realflight/`)
  where it matters.
- Outstanding work lives in GitHub issues. Every follow-up a review or investigation raises
  that isn't fixed in the same PR gets an issue (with its source PR); a pre-existing bug found
  while verifying gets a `bug` issue with repro steps. Labels: `needs-flight` (needs a
  simulator/transmitter check), `later` (waits for a real need). Milestones group issues by
  goal: `tailsitter-sitl`, `ark-fpv` (closed: `quad-sitl`, `publish-0.1`, which shipped 0.1.0);
  put a shared issue in the earliest milestone it unblocks; one that unblocks none gets no milestone.
  The PR that completes an issue says `Closes #n`. GitHub closes on `closes #n` anywhere in the body
  ("Closes #n once published" too), so reference an issue the PR doesn't finish without that word. The finished migration and refactor plans
  are in git history (`plans/`, removed after #61).
- The repo was renamed `stable-flight-sitl` → `stable-flight`; the crate keeps the
  `stable-flight-sitl` name. Old GitHub URLs redirect.

## Commands

```bash
cargo build --workspace                         # or: just build
cargo test --workspace --all-features           # or: just test
cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight  # joystick + RealFlight (--host, --commander extreme3d|spektrum), params + telemetry over the native link (--link-bind/--link-peer) for the bridge; or: just fly-manual
cargo run -p stable-flight-bridge               # native link ⇄ MAVLink for QGC (--link-bind/--link-peer/--gcs/--origin/--http-bind/--http-url/--offboard-speed); or: just bridge
cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight    # scripted figure-eight (script arg, --host, --end hold|loop|land); or: just fly-auto
cargo run -p stable-flight-sitl-realflight --example capture-state-realflight  # joystick flight + raw SimulatorState → CSV; or: just capture
cargo run -p stable-flight-sitl-realflight --example exchange-latency-realflight  # times RealFlight exchanges alone (--host, or --local on the RealFlight machine)
cargo run --example extreme_3d_commander        # joystick smoke test
cargo run --example spektrum_commander          # Spektrum (WS3000T dongle) mapping check: raw values + command
cargo run --bin plot_flight -- flights/figure_eight.csv out.svg  # 3D path viz (SVG or PNG); or: just plot …
cargo run --bin generate_figure_eight           # (re)write flights/figure_eight.csv; or: just gen-figure-eight
cargo run -p stable-flight-sitl-realflight --example copter-companion-realflight  # flown from the link, holds when it goes quiet; or: just fly-companion
cargo run --bin link_pilot                      # plays the companion for fly-companion (instead of the bridge); or: just link-pilot
cargo run -p stable-flight-sitl-realflight --example copter-autotune-realflight  # relay autotune of the rate loops (script arg, --host, --rate-p-scale, --stages rate|angle|all, --record), prints the gains, lands; or: just fly-autotune
cargo tarpaulin -o html                         # or: just cover  (writes tarpaulin-report.html; settings and exclusions in .tarpaulin.toml)
cargo fmt --all
cargo clippy --workspace --all-targets --all-features -- -D warnings  # or: just lint (adds rustdoc -D warnings)
just embedded                                   # no_std check: core (plain, `testing`, `serde`, `param-metadata`) and link for thumbv7em-none-eabihf, then fw-build
just fw-build                                   # ARK FPV firmware release build (fw-size: cargo size; fw-flash: DFU)
```

The flight pipeline has no CLI and there is no `cargo bench` target — don't suggest
`cargo run -- --commander …` or `cargo bench`. The host binaries are `stable-flight-bridge` and, in `stable-flight-tools`, `plot_flight`,
`generate_figure_eight`, and `link_pilot`; their unit tests run under `cargo test --workspace`. Add `-p <crate>` to scope any command to one crate.

## Architecture

The control system is a pipeline of traits sequenced by the core `FlightLoop` and paced by the sitl `RealTimeRunner`: `Commander`, `Vehicle` (`StateSource` + `Actuator`, blanket-implemented — implement those two, never `Vehicle` directly), `Guidance`, `Stabilizer`, `Mixer`, and `TelemetryPublisher`. Their signatures and contracts live in core `src/lib.rs` rustdoc; read it before implementing one.

`TelemetryRecord<'a>` (defined in `lib.rs`) borrows one tick's nominal `dt`, `command`, guidance `setpoint`, `state`, stabilizer `control` output, and mixer `actuators`, plus the tick's `Saturation` flags (merged from `Guidance`/`Stabilizer`/`Mixer::saturation()`, which default to none) and `failsafe` state (`Commander::failsafe()`, default none); the loop assembles it after `actuate`.

Trait methods **do not return `Result`** today. Invalid configuration panics at construction or in `run`; per-tick failures are not modeled. If you add fallible operations, introduce a typed error with `thiserror` rather than panicking inside the loop.

`Command` is `{ arm, input }`: `arm` gates the motors ([arm gate](architecture.md#flight-stabilization-loop)), and `input` is a `ControlInput`: `Attitude(AttitudeCommand)`, `Rate(RateCommand)`, `Waypoint(WaypointCommand)`, `AbsoluteAttitude(AbsoluteAttitudeCommand)`, or `Land(LandCommand)`. `ActuatorCommand` carries generic slots, `motors: [f32; 8]` ([0, 1]) and `servos: [f32; 8]` ([-1, 1]): the mixer sets each slot's meaning, the backend mapping its channel.

### Concrete implementations

| Trait              | Impls                                                                | Crate |
|--------------------|----------------------------------------------------------------------|-------|
| `Commander`        | `WaypointFollower<W: AsRef<[WaypointCommand]>>`; `FailsafeCommander<P: LinkedCommander, F: Fallback>`; `HoldCommander` (a `Fallback`) | core  |
| `Commander`        | `Extreme3DCommander`, `SpektrumCommander`, `ScriptCommander`         | sitl  |
| `Vehicle` (`StateSource` + `Actuator`) | `ComposedVehicle<S, A>`; `testing::NullVehicle` (`testing` feature) | core |
| `Vehicle` (`StateSource` + `Actuator`) | `RealFlightVehicle<T: RealFlightBridge>` | sitl-realflight |
| `Guidance`         | `CopterGuidance`                                                     | core  |
| `Stabilizer`       | `CopterStabilizer`                                                   | core  |
| `Mixer`            | `QuadCopterMixer`; `TailSitterMixer` (placeholder, outputs zeros)    | core  |
| `TelemetryPublisher` | `(A, B)` fan-out; `testing::RecordingTelemetry<N>` (`testing` feature) | core |
| `TelemetryPublisher` | `LinkPublisher` (from `fc::FcLink`; MAVLink is the bridge's job)     | link  |
| `Commander` | `LinkCommander` (from `FcLink::wire_linked`; a `LinkedCommander`) | link |

`CopterGuidance` (outer loop: waypoints → attitude setpoint, heading hold) uses the `altitude` and `lateral` PID controllers; `CopterStabilizer` (inner loop) uses `attitude`, `yaw`, and `body_rate`. All live in the core `controllers` module, with `Gains`. Controllers hold only state; each composite owns one config and implements the opt-in `Configurable` trait (validated `set_config`), which `FlightLoop::set_guidance_config`/`set_stabilizer_config` reach between ticks (rules: `architecture.md#config-ownership`). `CopterStabilizer` also hosts autotune stages (`tuning::Tunable`: `start_tuning`/`tuning`/`stop_tuning` on a `TunedLoop`, a `tuning::RelayTuner` replacing one loop's output), which `FlightLoop` forwards between ticks (it hands out no `&mut` to components), and `tuning::autotune::Autotune` sequences them (rules: `architecture.md#autotune`); `FlightLoop::armed()` reports the last tick's arm flag. `Setpoint` is `Attitude(AttitudeSetpoint)` or `Rate(RateSetpoint)`.

`ScriptCommander::new(path) -> Result<Self, ScriptError>` (sitl) loads a CSV waypoint file
(`x,y,altitude,cruise_speed`) from `flights/` into the core `WaypointFollower` (see its rustdoc). It is geometry-agnostic — the figure-eight shape lives in
`flights/figure_eight.csv`, produced by the `generate_figure_eight` binary
(`crates/stable-flight-tools/src/bin/generate_figure_eight.rs`). The
RealFlight examples are intentionally standalone (each inlines its full pipeline wiring),
so some setup is duplicated between them by design — readability over DRY. Gains and channel
mappings are the exception: they come from the backend's `airframes`, not the example; so is
the link plumbing, which comes from `stable_flight_link::fc::FcLink`.

### Control loop (`FlightLoop::step`, core `src/flight_loop.rs`)

Tick ordering (commander sees the previous tick's state; guidance and stabilizer the fresh estimate; telemetry after `actuate`) and `FlightLoop::prime` are specified in [`architecture.md#flight-stabilization-loop`](architecture.md#flight-stabilization-loop). Call `prime` once before the first `step`. `FlightLoop` has no clock; the sitl `RealTimeRunner` + `ShutdownSignal` pace and stop it (timing rules: `architecture.md#real-time-loop-management`; usage: sitl README).

## Coding style

- Match surrounding code first; the rules below resolve ties.
- `cargo fmt --all` (4-space indent, trailing commas where idiomatic).
- snake_case files/modules, PascalCase types, SCREAMING_SNAKE_CASE constants. Pull magic numbers into named constants when they appear more than once or carry units.
- Prefer early returns and small helpers; keep flight-math functions readable, not arbitrarily short. If a PID step is clearest as one 30-line function, leave it.
- Core is `no_std`: use `core::`, not `std::` (tests may use std). `f32` methods `core` lacks (`powf`, `sin`, `sqrt`, …) go through `libm`.
- Use `Result<T, E>` with `thiserror` enums for new fallible APIs. Avoid panics in library code outside of clearly documented preconditions (e.g. `run`'s frequency assert).
- **No range or finiteness `assert!`s on per-tick inputs.** Saturate, treat NaN as neutral, and wrap angles, per `architecture.md#input-policy-no-panics-in-the-tick`, using the core's `math` helpers. Asserts are only for construction-time config and `dt > 0`.
- **No `.unwrap()` or `.expect()` in `src/`.** They turn recoverable errors into panics that crash the flight loop. Propagate with `?`, return a typed error, or — only when a value is provably non-`None` from local context — use `unwrap_or_else(|| unreachable!("..."))` with a comment explaining the invariant. `.expect()` is fine in `examples/` and `tests/` where a crash is the acceptable failure mode.
- **Encode units in names.** A field, parameter, or constant holding a physical quantity as a bare `f32` (or `Vec3`) carries a unit suffix: `_m`, `_mps`, `_mps2`, `_rad`, `_radps`, `_deg`, `_degps`, `_s`, `_hz`, `_n` (e.g. `altitude_m`, `cruise_speed_mps`, `max_angles_rad`). Use SI (radians, not degrees) unless an external interface dictates otherwise; convert at the boundary. Exempt: normalized/dimensionless values (throttle, actuator commands, normalized tilt and rate commands, gains), `max_integral` windup limits (unit varies by loop; each field documents it), and `dt`, which is always seconds. No `uom` or other unit-typed wrappers.
- Do vector math with `nalgebra` (the core's `Vec2`/`Vec3` aliases, `UnitQuaternion<f32>`). Don't reinvent. nalgebra's `*` on two vectors is not element-wise — use `component_mul`. Read Euler angles with `stable_flight::euler_angles`, not `UnitQuaternion::euler_angles` (NaN-safe, clamped at ±90° pitch, less flash). Wrap angles to [-π, π) with `stable_flight::wrap_angle` rather than a local copy.
- Watch firmware size in core: fixed-size math only (no dynamic matrices or LU/SVD-style decompositions), no `Display`/`Debug` formatting on target, and measure flash with a `thumbv7em-none-eabihf` release build (`opt-level = "s"`, LTO) rather than guessing — `libm` trig, not the vector library, dominates. `just fw-size` gives the real board number.
- Logging via `log` + `env_logger`. Reserve `info!` for lifecycle events; use `debug!` inside the loop.
- Use `&` and slices to avoid allocations in the hot loop. No `Box`/`Vec` per tick.

## Testing

- Unit tests sit next to the code in `mod tests;` files (e.g. core `src/flight_loop/tests.rs`, `src/stabilizers/copter/tests.rs`, `src/mixers/quad/tests.rs`; sitl `src/runner/tests.rs`, `src/commanders/script/tests.rs`; sitl-realflight `src/vehicle/tests.rs`).
- Run `cargo test --workspace --all-targets` locally; integration-style checks that need simulator interaction live in `examples/`.
- Name tests after the behaviour they protect (e.g. `maintains_pitch_hold`), not the function under test.
- Use `assert2::assert!` / `check!` and `approx::assert_relative_eq!` for floats.
- Table-driven tests are the default for control math; one row per scenario.
- For pipeline doubles, use the core `testing` feature (`stable_flight::testing::{NullVehicle, ConstantCommander, RecordingTelemetry}`, plus `testing::conformance` for backend frame checks; enable it in `[dev-dependencies]`) and recover them after a run with `FlightLoop::into_parts()`. Don't add per-crate `Mock*` pipeline types. For the flight controller's side of the link, in tests and examples alike, use `stable_flight_link::fc::FcLink` rather than rewiring the queues, router, server, and publisher. For a MAVLink connection, use `stable_flight_mavlink::testing::MockConnection` (`testing` feature): scripted receives, recorded sends. Mock other traits by hand (see `MockRealFlightBridge` in sitl-realflight `src/vehicle/tests.rs`) — `mockall` is not a dependency.
- A test that guards a fix must fail without it: check by reverting the fix (or mutating the
  guarded value) locally, and say so in the PR. Example: the SIGTERM test was killed by the signal
  without #227's handler.
- Keep time-based table rows clear of float rounding at thresholds: `dt` accumulates in `f32`, so
  three 0.1 s ticks may not reach 0.3. Use, e.g., a 0.35 s timeout with 0.1 s ticks.
- Tests that raise process-wide signals get an integration-test binary of their own
  (`tests/*.rs`), one signal per binary.
- `proptest` and `criterion` are **not** in `Cargo.toml`. Add them deliberately, with a benchmark or property test in the same PR, if you reach for them.
- Coverage: target **90%+ overall** (the Coveralls number for `main`, measured on Linux CI; 85.7% at #285; a local macOS `just cover` reads about 1 point higher). A PR should not lower it, and one that adds library code covers it at 90%+; examples and firmware are excluded (`.tarpaulin.toml`), so the target applies to tested code. `just cover` → review `tarpaulin-report.html`. Refresh after non-trivial control-loop changes. The `coverage` workflow (on push to `main`, or by hand) runs the same tarpaulin and uploads to Coveralls (the README badge).

## Simulator setup

Each backend crate's README owns its setup and usage (RealFlight:
[`crates/stable-flight-sitl-realflight/README.md`](crates/stable-flight-sitl-realflight/README.md)).
A backend must meet the frame contract (`VehicleState` rustdoc) and timing contract (`Vehicle`
rustdoc), and cover every conformance scenario with `stable_flight::conformance_tests!` (core
`testing` feature). Known gaps stay `#[ignore]`d with a reason that links a GitHub issue.

## Flight checks

PRs that change flight behavior, the link, or the bridge end with a RealFlight check. Example
usage: [sitl-realflight README](crates/stable-flight-sitl-realflight/README.md).

- **Setups:**
  - `just fly-companion` + `just bridge` + QGC: the vehicle is flown from the link, and the
    bridge speaks MAVLink to QGC.
  - `just fly-companion` + `just link-pilot`: the pilot plays the companion on the bridge's port,
    so run it instead of the bridge.
  - Offboard MAVLink scripts (e.g. pymavlink) need the bridge's ground-station port (UDP 14550),
    so close QGC first. Run them with `uv run --no-project --with pymavlink python script.py` so
    nothing installs globally.
- **Start order:** bridge and example first, then QGC. Under an open QGC, a vehicle that
  disappears and comes back (bridge or example restart) can leave QGC's virtual joystick silent
  while Arm still works. Restart QGC.
- **Signals to read:**
  - The companion example logs each failsafe handover ("link quiet: holding position" or
    "link quiet: landing", "link back: holding until the sticks move", "link back: flying
    from the link"). Time them against the event you caused; the failsafe timeout is 1 s.
  - QGC's throttle and saturation (`SAT_*`) displays: see the
    [bridge README](crates/stable-flight-bridge/README.md)'s `VFR_HUD` and `NAMED_VALUE_INT` rows.
  - QGC's traffic to the bridge: `nettop -p <bridge pid> -J bytes_in` on the `udp4 *:` socket.
    Heartbeat only is about 21 B/s; the joystick streaming is about 600 B/s.
- **Stopping:** stop examples with Ctrl-C or SIGTERM (`ShutdownSignal` handles both), so
  `RealFlightVehicle` hands RC back to RealFlight. A kill without cleanup leaves "Preexisting
  controller reference" on the next run and truncates capture CSVs.
- **RealFlight disconnects:** the flight examples reconnect on their own (backoff: the
  `RealFlightVehicle` rustdoc) and log "Reconnected to RealFlight". To test it, restart the
  RealFlight proxy mid-flight. `capture-state-realflight` doesn't reconnect.

## Commits & PRs

- Small, imperative commit subjects mirroring existing history (e.g. `Use f32 instead of Duration... (#18)`). Reference issues/PRs with `(#id)`.
- Include simulator context for flight-behaviour changes (model, mode, what you flew).
- PRs need a summary of the scenario tested, reproduction/validation steps (commands, configs), and screenshots or telemetry snippets when visual feedback changes.
- **Merge method follows the branch's commits.** Squash by default: the PR lands as one commit titled `<PR title> (#id)` with the PR body as its message, and the branch commits (review fixes, WIP) are discarded. Rebase-merge when every branch commit is a clean, self-contained step worth keeping in `main` (each builds and passes tests); use a merge commit only for a long-lived branch whose history should stay grouped. Keep the PR title release-note quality either way, and write the PR body as the commit message it becomes (summary first; the template's checklist and flight-check sections are fine to keep).
- `main` is guarded by the repository ruleset `main` (not classic branch protection, whose API reports none; list it with `gh api repos/wboayue/stable-flight/rulesets`): changes land through PRs, no force-push or deletion, and the `test`, `lint`, and `embedded` checks are required. Rewriting published history means disabling the ruleset briefly; do it only on the owner's explicit go and re-enable it right after.
- PRs must be green on the `test` GitHub Actions workflow (`test`, `lint`, and `embedded` jobs) before they can merge; the PR shows `UNSTABLE`/pending until it finishes. Before merging right after a push, confirm a run exists for the PR's head commit and completed (`gh run list --commit $(gh pr view <n> --json headRefOid -q .headRefOid)`): `gh pr checks --watch` returns at once with "no checks reported" when GitHub hasn't registered the checks yet.
- CI triggers only on PRs whose base is `main`. In a stack of PRs (each based on the previous branch), only the bottom one gets checks. To merge the bottom PR of a stack:
  1. Record the parent's tip SHA, then **retarget the child to `main` first** (`gh pr edit <child> --base main`). `gh pr merge --delete-branch` on a parent *closes* any PR still based on it, and GitHub won't reopen a closed PR once its head has been force-pushed.
  2. `gh pr merge <parent> --squash --delete-branch`.
  3. From the top of the stack, `git rebase --update-refs --onto main <old-parent-sha>`, then `git push --force-with-lease` every stacked branch.
- A new published crate inherits `[workspace.package]`, gets its own `README.md` (absolute GitHub links; crates.io doesn't resolve repo-relative ones) and `CHANGELOG.md`, and symlinks `LICENSE` (`ln -s ../../LICENSE`) so it ships in the package. Check with `cargo package -p <crate> --list`.
- **Keep docs in sync with the code, in the same PR.** Renamed trait, changed a method signature, added/removed a component or crate, retired a CLI/example, swapped a default — update the doc that owns the fact. Stale docs mislead both humans and AI; treat them as part of the change, not a follow-up. Each fact has one home; other docs link to it rather than restating it:
  - crate `README.md` — that crate's purpose, usage, features;
  - root `README.md` — project overview and **Status**, crate index, boards, supported simulators, quick start;
  - `AGENTS.md` — commands, coding/testing rules, PR checklist;
  - `CONTRIBUTING.md` — the entry point for outside contributors; links here for the rules;
  - `SECURITY.md` — supported versions and how to report a vulnerability;
  - `.github/` templates — what an issue or PR must include (they point at the rules here);
  - `architecture.md` — crate boundaries, dependency rules, traits, and the rationale behind
    the frame and timing contracts;
  - core rustdoc — the frame contract (`VehicleState`) and timing contract (`Vehicle`) themselves.
- **Keep the project status current.** When a milestone closes or what actually flies changes (firmware flies, a new simulator or airframe flies, the tailsitter flies, a crate is published), update every status claim in the same PR, or the milestone's last one: the root README **Status** paragraph and its board, simulator, and test-model tables; the board's firmware README status; the scope line in `SECURITY.md`; and the GitHub repo description (`gh repo edit --description`). Claim only what has flown; plans say "planned" and link their milestone.
- **Maintain each published crate's `CHANGELOG.md`** (`crates/<crate>/CHANGELOG.md`; unpublished `stable-flight-tools` has none) ([Keep a Changelog](https://keepachangelog.com/en/1.1.0/)). **Until a crate's first release**, `[Unreleased]` is a feature summary of what the crate provides, not a change log: a PR that adds, removes, or reshapes a feature edits the summary, and nothing is marked **Breaking** (no one has a prior version). **After the first release**, every user-facing change adds a bullet under `## [Unreleased]` in the same PR, grouped by `Added`/`Changed`/`Deprecated`/`Removed`/`Fixed`/`Security`; mark breaking changes **Breaking**. Skip internal-only churn (refactors, test/CI tweaks). At release, rename `[Unreleased]` to `## [x.y.z] - YYYY-MM-DD` (at the first release, delete the summary note; the summary becomes that entry), add a fresh empty `[Unreleased]`, and update the links: tag each crate `<crate>-vx.y.z` (e.g. `stable-flight-v0.1.0`), point `[Unreleased]` at `compare/<crate>-vx.y.z...HEAD`, and add a `[x.y.z]` link pointing at `releases/tag/<crate>-vx.y.z`. All six crates are on crates.io since 0.1.0; a new crate starts with the pre-release summary (its links point at the crate's commit history until its first tag).
- **Releasing:** (1) `cargo publish --workspace --dry-run --locked`; (2) a `Release x.y.z` PR with the changelog moves; (3) after it merges, annotated tags `<crate>-vx.y.z` on the merge commit, pushed; (4) one GitHub release per tag, its notes the crate's changelog section, `stable-flight` marked latest (`gh release create <tag> --verify-tag --notes-file … --latest=false` for the rest); (5) `cargo publish --workspace --locked`, which uploads in dependency order and waits for each to index. crates.io rate-limits *new* crate names: on a 429, publish the rest with `cargo publish -p <crate> --locked` after the time it gives. Once the link crate is released, any message shape change bumps `PROTOCOL_VERSION`.

## Workflow

1. Review every plan before implementing it, and include the review in the plan; review the
   finished PR the same way before it merges (fixes from the review go in the same PR, and
   anything deferred gets an issue). The lenses:
   - **Duplication:** logic, constants, or helpers that repeat in the change or already exist elsewhere (check core `math` and sibling crates before writing a helper).
   - **Single responsibility:** each type or function does one job; separate I/O, scheduling, and pure mapping/math.
   - **Composability:** dependencies are injected rather than hard-coded (connections, addresses, clocks), configuration is explicit, and the public surface is no wider than callers need.
   - **Performance**, when the code runs in the hot path (anything `FlightLoop::step` calls each tick — commander, state source, guidance, stabilizer, mixer, actuator, telemetry — or firmware): per-tick allocations, blocking I/O, redundant trig or `libm` calls, and flash size (see Coding style).
2. Before designing around external software (QGC, other MAVLink peers, simulator clients),
   verify the behavior you rely on in its source, not from memory, and record the findings with
   file:line in the issue or PR. Examples: QGC's connect and metadata handling (#206 comment),
   and its arming and `MANUAL_CONTROL` (#234 PR).
3. Read the relevant trait in core `src/lib.rs` and its existing impls before adding a new one. Hardware-reusable code (control math, mixing, sequencing) goes in the core crate; anything touching an OS, clock, file, or network goes in sitl; anything specific to one simulator goes in its `stable-flight-sitl-<simulator>` backend crate. Firmware stays thin: a board crate only wires hardware to the pipeline (clocks, RTIC tasks, peripheral setup); logic it needs (sensor parsing, ESC protocol encoding, estimation) goes in core or another `no_std` crate under `crates/`, unit-tested on the host (`embedded-hal-mock` for drivers) so it counts toward coverage. Firmware itself is excluded from coverage; on-target tests (`embedded-test`) and HIL (#243) check it instead.
4. Wire new components into a `FlightLoop` + `RealTimeRunner` the way sitl-realflight `examples/copter-manual-realflight.rs` does — that's the canonical reference.
5. Add tests alongside the code; cover the boundary/error case, not just the happy path.
   A new `ParamTable` also gets a `ParamMetadata` table in the same file and a row in core
   `src/params/metadata/tests.rs`, so QGC describes its params and their ranges are checked
   against `validate`.
6. `cargo fmt --all && just lint && cargo test --workspace --all-features` before sending a PR.
7. If you change trait surface, module or crate layout, or runnable examples, update the owning docs (see Commits & PRs) in the same PR.
8. To compare against `main` (e.g. test counts), use a throwaway worktree — `git worktree add <dir> main` with its own `CARGO_TARGET_DIR` — not a `git stash` round-trip; stashing staged renames plus untracked files can fail to restore.
9. Pass PR bodies to `gh pr create`/`edit` with `--body-file`; inline heredocs inside `$(…)` break on apostrophes.

## Pre-PR checklist

- [ ] Builds clean (`cargo build --workspace`) and tests pass (`cargo test --workspace --all-features`).
- [ ] Core still builds `no_std` and the firmware still links (`just embedded`).
- [ ] `cargo fmt --all` clean; `just lint` (clippy + rustdoc, `-D warnings`) passes — CI fails on any warning.
- [ ] New fallible APIs return `Result` with `thiserror` types; no new panics in library code.
- [ ] No `.unwrap()` or `.expect()` introduced under `src/` (examples/tests OK).
- [ ] New PID/quaternion/transform logic is unit-tested with table cases.
- [ ] Public API has Rustdoc with at least one example.
- [ ] Hot-path code allocates nothing per tick.
- [ ] Coverage refreshed if control-loop logic changed; overall stays at or moves toward 90%+.
- [ ] Owning docs (crate/root `README.md`, `AGENTS.md`, `architecture.md`) reflect any trait, module, crate, or example changes.
- [ ] If this PR changes what flies or closes a milestone, the status claims match (see Keep the project status current).
- [ ] Each affected published crate's `CHANGELOG.md` reflects any user-facing change (pre-release: the feature summary; after: an `[Unreleased]` entry).
