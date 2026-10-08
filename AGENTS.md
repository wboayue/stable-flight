# AGENTS.md

Guidance for everyone (humans + AI assistants) working in the Stable Flight repo. `architecture.md` covers design rationale.

## Quick facts

- Cargo workspace (virtual manifest at the root, `resolver = "3"`), Rust **edition 2024**,
  version `0.1.0`. Shared package metadata and common dependency versions live in the root
  `[workspace.package]` / `[workspace.dependencies]`; crates inherit them with `.workspace = true`.
- Crates:
  - `crates/stable-flight` — the flight-control core: pipeline traits and data types
    (`lib.rs`), `FlightLoop` (one tick), `stabilizers`, `mixers`. No simulator, I/O, clock, or
    thread dependencies, and it is `no_std` — keep it that way (see Coding style).
  - `crates/stable-flight-sitl` — the simulator-independent SITL harness: `commanders`,
    `telemetry`, `runner` (`RealTimeRunner`, wall-clock pacing of a `FlightLoop`), `shutdown`
    (`ShutdownSignal`). Depends on the core. Non-simulator examples live in `crates/stable-flight-sitl/examples/*.rs`.
  - `crates/stable-flight-sitl-realflight` — the RealFlight backend: `RealFlightVehicle`,
    `ActuatorMapping`, `airframes` (tuned gains, mixer, and mapping per model), and the
    `copter-*-realflight` examples. Normal deps: core, `realflight-bridge`, `log`; sitl is a dev-dependency only (CI checks).
    One `stable-flight-sitl-<simulator>` crate per backend.
  - `crates/stable-flight-tools` — unpublished (`publish = false`) dev binaries: `plot_flight`
    (3D flight-path visualizer) and `generate_figure_eight` (waypoint-script generator).
- `firmware/` is excluded from the workspace; each board is its own nested workspace (own
  target, profiles, committed `Cargo.lock`) — see [`firmware/ark-fpv`](firmware/ark-fpv/README.md).
  Run cargo from the board's directory (or use the `fw-*` recipes): its `.cargo/config.toml`
  isn't picked up via `--manifest-path`. Root `cargo fmt --all`/`clippy` don't reach it.
- Run every command from the repo root: `cargo run --example <name>` / `--bin <name>` resolve
  across the workspace (add `-p <crate>` to be explicit), and `flights/*.csv` paths are
  relative to the root.
- Public modules: `stable-flight` — `commanders` (`WaypointFollower`), `controllers` (PID loops, `Gains`), `flight_loop`, `guidance` (`CopterGuidance`), `mixers`, `stabilizers`, `testing` (feature: doubles + `conformance`) (traits/types, the `Vec2`/`Vec3` aliases, `GRAVITY`, `euler_angles`, and the `nalgebra` re-export at the crate root); `stable-flight-sitl` — `commanders`, `runner`, `shutdown`, `telemetry` (`RealTimeRunner` and `ShutdownSignal` also at the root); `stable-flight-sitl-realflight` — `airframes`, with `RealFlightVehicle`, `RealFlightConfiguration`, `ActuatorMapping`, `vehicle_state`, and the `realflight_bridge` re-export at the root. sitl re-exports only the core crate itself (`stable_flight_sitl::stable_flight`), not its items: import core items from `stable_flight`.
- Key external crates: `realflight-bridge` (RealFlight I/O, backend crate only), `nalgebra` (core only, `libm`; core aliases `Vec2`/`Vec3` and re-exports it as `stable_flight::nalgebra` — other crates use the re-export), `libm` (`no_std` float math, core), `mavlink`, `gilrs` (joystick), `thiserror`, `signal-hook`. Dev: `approx`, `assert2`, `clap` (examples parse args with it). `stable-flight-tools` adds `plotters` and `clap`.
- Untracked generated artifacts: `target/`, root `Cargo.lock`, `tarpaulin-report.html`, `firmware/*/*.bin`.
- Paths below are prefixed with their crate (`core:` = `crates/stable-flight/`, `sitl:` =
  `crates/stable-flight-sitl/`, `sitl-realflight:` = `crates/stable-flight-sitl-realflight/`)
  where it matters.
- Outstanding work lives in GitHub issues. Every follow-up a review or investigation raises
  that isn't fixed in the same PR gets an issue (with its source PR); a pre-existing bug found
  while verifying gets a `bug` issue with repro steps. Labels: `needs-flight` (needs a
  simulator/transmitter check), `later` (waits for a real need). Milestones group issues by
  goal: `quad-sitl`, `tailsitter-sitl`, `ark-fpv`, `publish-0.1` (must land before publishing
  core; #93); put a shared issue in the earliest milestone it unblocks.
  The PR that completes an issue says `Closes #n`. The finished migration and refactor plans
  are in git history (`plans/`, removed after #61).
- The repo was renamed `stable-flight-sitl` → `stable-flight`; the crate keeps the
  `stable-flight-sitl` name. Old GitHub URLs redirect.

## Commands

```bash
cargo build --workspace                         # or: just build
cargo test --workspace --all-features           # or: just test
cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight  # joystick + RealFlight (--host); or: just fly-manual
cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight    # scripted figure-eight (script arg, --host); or: just fly-auto
cargo run -p stable-flight-sitl-realflight --example capture-state-realflight  # joystick flight + raw SimulatorState → CSV; or: just capture
cargo run --example extreme_3d_commander        # joystick smoke test
cargo run --example publish_telemetry           # MAVLink demo
cargo run --bin plot_flight -- flights/figure_eight.csv out.svg  # 3D path viz (SVG or PNG); or: just plot …
cargo run --bin generate_figure_eight           # (re)write flights/figure_eight.csv; or: just gen-figure-eight
cargo tarpaulin --workspace -o html             # or: just cover  (writes tarpaulin-report.html)
cargo fmt --all
cargo clippy --workspace --all-targets --all-features -- -D warnings  # or: just lint (adds rustdoc -D warnings)
just embedded                                   # no_std check: core for thumbv7em-none-eabihf, with and without `testing`, then fw-build
just fw-build                                   # ARK FPV firmware release build (fw-size: cargo size; fw-flash: DFU)
```

The flight pipeline has no CLI and there is no `cargo bench` target — don't suggest
`cargo run -- --commander …` or `cargo bench`. The only host binaries are `plot_flight` and
`generate_figure_eight` in `stable-flight-tools`; their unit tests run under
`cargo test --workspace`. Add `-p <crate>` to scope any command to one crate.

## Architecture

The control system is a six-trait pipeline sequenced by the core `FlightLoop` and paced by the sitl `RealTimeRunner`. Traits and methods (from `crates/stable-flight/src/lib.rs`):

```rust
pub trait Commander {
    fn next_command(&mut self, dt: f32, current_state: &VehicleState) -> Command;
}

pub trait StateEstimator {
    fn estimate_state(&mut self, dt: Duration) -> VehicleState;
}

pub trait Actuator {
    fn actuate(&mut self, actuator_command: &ActuatorCommand);
}

// Blanket-implemented: implement StateEstimator + Actuator, never Vehicle directly.
pub trait Vehicle: StateEstimator + Actuator {}
impl<T: StateEstimator + Actuator> Vehicle for T {}

pub trait Guidance {
    fn guide(&mut self, dt: f32, command: &Command, state: &VehicleState) -> Setpoint;
    fn prime(&mut self, state: &VehicleState);
}

pub trait Stabilizer {
    fn stabilize(&mut self, dt: f32, setpoint: &Setpoint, state: &VehicleState) -> ControlOutput;
    fn prime(&mut self, state: &VehicleState);
}

pub trait Mixer {
    fn mix(&self, controls: &ControlOutput) -> ActuatorCommand;
}

pub trait TelemetryPublisher {
    fn publish(&mut self, record: &TelemetryRecord);
}
```

`TelemetryRecord<'a>` (defined in `lib.rs`) borrows one tick's nominal `dt`, `command`, guidance `setpoint`, `state`, stabilizer `control` output, and mixer `actuators`; the loop assembles it after `actuate`.

Trait methods **do not return `Result`** today. Invalid configuration panics at construction or in `run`; per-tick failures are not modeled. If you add fallible operations, introduce a typed error with `thiserror` rather than panicking inside the loop.

`Command` is an enum: `Attitude(AttitudeCommand)`, `Rate(RateCommand)`, `Waypoint(WaypointCommand)`. `ActuatorCommand` carries `motors: [f32; 8]` plus fixed-wing surfaces (`rudder`/`elevator`/`aileron`/`throttle`/`elevon_*`).

### Concrete implementations

| Trait              | Impls                                                                | Crate |
|--------------------|----------------------------------------------------------------------|-------|
| `Commander`        | `WaypointFollower<W: AsRef<[WaypointCommand]>>`                      | core  |
| `Commander`        | `Extreme3DCommander`, `SpektrumCommander`, `ScriptCommander`         | sitl  |
| `Vehicle` (`StateEstimator` + `Actuator`) | `ComposedVehicle<E, A>`; `testing::NullVehicle` (`testing` feature) | core |
| `Vehicle` (`StateEstimator` + `Actuator`) | `RealFlightVehicle<T: RealFlightBridge>` | sitl-realflight |
| `Guidance`         | `CopterGuidance`                                                     | core  |
| `Stabilizer`       | `CopterStabilizer`                                                   | core  |
| `Mixer`            | `QuadCopterMixer`; `TailSitterMixer` (placeholder, outputs zeros)    | core  |
| `TelemetryPublisher` | `(A, B)` fan-out; `testing::RecordingTelemetry<N>` (`testing` feature) | core |
| `TelemetryPublisher` | `MavlinkPublisher`                                                 | sitl  |

`CopterGuidance` (outer loop: waypoints → attitude setpoint, heading hold) uses the `altitude` and `lateral` PID controllers; `CopterStabilizer` (inner loop) uses `attitude`, `yaw`, and `body_rate`. All live in the core `controllers` module, with `Gains`. `Setpoint` is `Attitude(AttitudeSetpoint)` or `Rate(RateSetpoint)`.

`ScriptCommander::new(path) -> Result<Self, ScriptError>` (sitl) loads a CSV waypoint file
(`x,y,altitude,cruise_speed`) from `flights/` into the core `WaypointFollower` (see its rustdoc). It is geometry-agnostic — the figure-eight shape lives in
`flights/figure_eight.csv`, produced by the `generate_figure_eight` binary
(`crates/stable-flight-tools/src/bin/generate_figure_eight.rs`). The
RealFlight examples are intentionally standalone (each inlines its full pipeline wiring),
so some setup is duplicated between them by design — readability over DRY. Gains and channel
mappings are the exception: they come from the backend's `airframes`, not the example.

### Control loop (`FlightLoop::step`, core `src/flight_loop.rs`)

```rust
let dt_secs = dt.as_secs_f32();
let command = self.commander.next_command(dt_secs, &self.state); // previous tick's state
self.state = self.vehicle.estimate_state(dt);
let setpoint = self.guidance.guide(dt_secs, &command, &self.state);
let control = self.stabilizer.stabilize(dt_secs, &setpoint, &self.state);
let actuators = self.mixer.mix(&control);
self.vehicle.actuate(&actuators);
self.telemetry.publish(&TelemetryRecord { dt, command: &command, setpoint: &setpoint, state: &self.state, control: &control, actuators: &actuators });
```

`FlightLoop::prime()` takes the initial estimate with `Duration::ZERO` and primes guidance, then the stabilizer; call it once before the first `step`. `FlightLoop` has no clock; the sitl `RealTimeRunner` + `ShutdownSignal` pace and stop it (timing rules: `architecture.md#real-time-loop-management`; usage: sitl README).

## Coding style

- Match surrounding code first; the rules below resolve ties.
- `cargo fmt --all` (4-space indent, trailing commas where idiomatic).
- snake_case files/modules, PascalCase types, SCREAMING_SNAKE_CASE constants. Pull magic numbers into named constants when they appear more than once or carry units.
- Prefer early returns and small helpers; keep flight-math functions readable, not arbitrarily short. If a PID step is clearest as one 30-line function, leave it.
- Core is `no_std`: use `core::`, not `std::` (tests may use std). `f32` methods `core` lacks (`powf`, `sin`, `sqrt`, …) go through `libm`.
- Use `Result<T, E>` with `thiserror` enums for new fallible APIs. Avoid panics in library code outside of clearly documented preconditions (e.g. `run`'s frequency assert).
- **No range or finiteness `assert!`s on per-tick inputs.** Saturate, treat NaN as neutral, and wrap angles, per `architecture.md#input-policy-no-panics-in-the-tick`, using the core's `math` helpers. Asserts are only for construction-time config and `dt > 0`.
- **No `.unwrap()` or `.expect()` in `src/`.** They turn recoverable errors into panics that crash the flight loop. Propagate with `?`, return a typed error, or — only when a value is provably non-`None` from local context — use `unwrap_or_else(|| unreachable!("..."))` with a comment explaining the invariant. `.expect()` is fine in `examples/` and `tests/` where a crash is the acceptable failure mode.
- **Encode units in names.** A field, parameter, or constant holding a physical quantity as a bare `f32` (or `Vec3`) carries a unit suffix: `_m`, `_mps`, `_mps2`, `_rad`, `_radps`, `_deg`, `_s`, `_hz`, `_n` (e.g. `altitude_m`, `cruise_speed_mps`, `max_tilt_rad`). Use SI (radians, not degrees) unless an external interface dictates otherwise; convert at the boundary. Exempt: normalized/dimensionless values (throttle, actuator commands, gains) and `dt`, which is always seconds. No `uom` or other unit-typed wrappers.
- Do vector math with `nalgebra` (the core's `Vec2`/`Vec3` aliases, `UnitQuaternion<f32>`). Don't reinvent. nalgebra's `*` on two vectors is not element-wise — use `component_mul`. Read Euler angles with `stable_flight::euler_angles`, not `UnitQuaternion::euler_angles` (NaN-safe, clamped at ±90° pitch, less flash).
- Watch firmware size in core: fixed-size math only (no dynamic matrices or LU/SVD-style decompositions), no `Display`/`Debug` formatting on target, and measure flash with a `thumbv7em-none-eabihf` release build (`opt-level = "s"`, LTO) rather than guessing — `libm` trig, not the vector library, dominates. `just fw-size` gives the real board number.
- Logging via `log` + `env_logger`. Reserve `info!` for lifecycle events; use `debug!` inside the loop.
- Use `&` and slices to avoid allocations in the hot loop. No `Box`/`Vec` per tick.

## Testing

- Unit tests sit next to the code in `mod tests;` files (e.g. core `src/flight_loop/tests.rs`, `src/stabilizers/copter/tests.rs`, `src/mixers/quad/tests.rs`; sitl `src/runner/tests.rs`, `src/commanders/script/tests.rs`; sitl-realflight `src/vehicle/tests.rs`).
- Run `cargo test --workspace --all-targets` locally; integration-style checks that need simulator interaction live in `examples/`.
- Name tests after the behaviour they protect (e.g. `maintains_pitch_hold`), not the function under test.
- Use `assert2::assert!` / `check!` and `approx::assert_relative_eq!` for floats.
- Table-driven tests are the default for control math; one row per scenario.
- For pipeline doubles, use the core `testing` feature (`stable_flight::testing::{NullVehicle, ConstantCommander, RecordingTelemetry}`, plus `testing::conformance` for backend frame checks; enable it in `[dev-dependencies]`) and recover them after a run with `FlightLoop::into_parts()`. Don't add per-crate `Mock*` pipeline types. Mock other traits by hand (see `MockRealFlightBridge` in sitl-realflight `src/vehicle/tests.rs`) — `mockall` is not a dependency.
- `proptest` and `criterion` are **not** in `Cargo.toml`. Add them deliberately, with a benchmark or property test in the same PR, if you reach for them.
- Coverage: `just cover` → review `tarpaulin-report.html`. Refresh after non-trivial control-loop changes.

## Simulator setup

Each backend crate's README owns its setup and usage (RealFlight:
[`crates/stable-flight-sitl-realflight/README.md`](crates/stable-flight-sitl-realflight/README.md)).
A backend must meet the frame contract (`VehicleState` rustdoc) and timing contract (`Vehicle`
rustdoc), and cover every conformance scenario with `stable_flight::conformance_tests!` (core
`testing` feature). Known gaps stay `#[ignore]`d with a reason that links a GitHub issue.

## Commits & PRs

- Small, imperative commit subjects mirroring existing history (e.g. `Use f32 instead of Duration... (#18)`). Reference issues/PRs with `(#id)`.
- Include simulator context for flight-behaviour changes (model, mode, what you flew).
- PRs need a summary of the scenario tested, reproduction/validation steps (commands, configs), and screenshots or telemetry snippets when visual feedback changes.
- **Merges are squash-only** — merge commits and rebase are disabled on GitHub, so a PR lands as a single commit titled `<PR title> (#id)`. Keep the PR title release-note quality; intermediate branch commits are discarded.
- PRs must be green on the `test` GitHub Actions workflow (`test`, `lint`, and `embedded` jobs) before they can merge; the PR shows `UNSTABLE`/pending until it finishes.
- CI triggers only on PRs whose base is `main`. In a stack of PRs (each based on the previous branch), only the bottom one gets checks. To merge the bottom PR of a stack:
  1. Record the parent's tip SHA, then **retarget the child to `main` first** (`gh pr edit <child> --base main`). `gh pr merge --delete-branch` on a parent *closes* any PR still based on it, and GitHub won't reopen a closed PR once its head has been force-pushed.
  2. `gh pr merge <parent> --squash --delete-branch`.
  3. From the top of the stack, `git rebase --update-refs --onto main <old-parent-sha>`, then `git push --force-with-lease` every stacked branch.
- A new published crate inherits `[workspace.package]`, gets its own `README.md` (absolute GitHub links; crates.io doesn't resolve repo-relative ones) and `CHANGELOG.md`, and symlinks `LICENSE` (`ln -s ../../LICENSE`) so it ships in the package. Check with `cargo package -p <crate> --list`.
- **Keep docs in sync with the code, in the same PR.** Renamed trait, changed a method signature, added/removed a component or crate, retired a CLI/example, swapped a default — update the doc that owns the fact. Stale docs mislead both humans and AI; treat them as part of the change, not a follow-up. Each fact has one home; other docs link to it rather than restating it:
  - crate `README.md` — that crate's purpose, usage, features;
  - root `README.md` — project overview, crate index, supported simulators, quick start;
  - `AGENTS.md` — commands, coding/testing rules, PR checklist;
  - `architecture.md` — crate boundaries, dependency rules, traits, and the rationale behind
    the frame and timing contracts;
  - core rustdoc — the frame contract (`VehicleState`) and timing contract (`Vehicle`) themselves.
- **Maintain each published crate's `CHANGELOG.md`** (`crates/<crate>/CHANGELOG.md`; unpublished `stable-flight-tools` has none) ([Keep a Changelog](https://keepachangelog.com/en/1.1.0/)). Every user-facing change adds a bullet under `## [Unreleased]` in the same PR, grouped by `Added`/`Changed`/`Deprecated`/`Removed`/`Fixed`/`Security`; mark breaking changes **Breaking**. Skip internal-only churn (refactors, test/CI tweaks). At release, rename `[Unreleased]` to `## [x.y.z] - YYYY-MM-DD`, add a fresh empty `[Unreleased]`, and update the compare links.

## Workflow

1. Review every plan before implementing it, and include the review in the plan:
   - **Duplication:** logic, constants, or helpers that repeat in the change or already exist elsewhere (check core `math` and sibling crates before writing a helper).
   - **Single responsibility:** each type or function does one job; separate I/O, scheduling, and pure mapping/math.
   - **Composability:** dependencies are injected rather than hard-coded (connections, addresses, clocks), configuration is explicit, and the public surface is no wider than callers need.
   - **Performance**, when the code runs in the hot path (anything `FlightLoop::step` calls each tick — commander, estimator, guidance, stabilizer, mixer, actuator, telemetry — or firmware): per-tick allocations, blocking I/O, redundant trig or `libm` calls, and flash size (see Coding style).
2. Read the relevant trait in core `src/lib.rs` and its existing impls before adding a new one. Hardware-reusable code (control math, mixing, sequencing) goes in the core crate; anything touching an OS, clock, file, or network goes in sitl; anything specific to one simulator goes in its `stable-flight-sitl-<simulator>` backend crate.
3. Wire new components into a `FlightLoop` + `RealTimeRunner` the way sitl-realflight `examples/copter-manual-realflight.rs` does — that's the canonical reference.
4. Add tests alongside the code; cover the boundary/error case, not just the happy path.
5. `cargo fmt --all && just lint && cargo test --workspace --all-features` before sending a PR.
6. If you change trait surface, module or crate layout, or runnable examples, update the owning docs (see Commits & PRs) in the same PR.
7. To compare against `main` (e.g. test counts), use a throwaway worktree — `git worktree add <dir> main` with its own `CARGO_TARGET_DIR` — not a `git stash` round-trip; stashing staged renames plus untracked files can fail to restore.
8. Pass PR bodies to `gh pr create`/`edit` with `--body-file`; inline heredocs inside `$(…)` break on apostrophes.

## Pre-PR checklist

- [ ] Builds clean (`cargo build --workspace`) and tests pass (`cargo test --workspace --all-features`).
- [ ] Core still builds `no_std` and the firmware still links (`just embedded`).
- [ ] `cargo fmt --all` clean; `just lint` (clippy + rustdoc, `-D warnings`) passes — CI fails on any warning.
- [ ] New fallible APIs return `Result` with `thiserror` types; no new panics in library code.
- [ ] No `.unwrap()` or `.expect()` introduced under `src/` (examples/tests OK).
- [ ] New PID/quaternion/transform logic is unit-tested with table cases.
- [ ] Public API has Rustdoc with at least one example.
- [ ] Hot-path code allocates nothing per tick.
- [ ] Coverage refreshed if control-loop logic changed.
- [ ] Owning docs (crate/root `README.md`, `AGENTS.md`, `architecture.md`) reflect any trait, module, crate, or example changes.
- [ ] Each affected published crate's `CHANGELOG.md` has an `[Unreleased]` entry for any user-facing change.
