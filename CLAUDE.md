# CLAUDE.md

Guidance for everyone (humans + AI assistants) working in the Stable Flight repo. `architecture.md` covers design rationale.

## Quick facts

- Cargo workspace (virtual manifest at the root, `resolver = "3"`), Rust **edition 2024**,
  version `0.1.0`. Shared package metadata and common dependency versions live in the root
  `[workspace.package]` / `[workspace.dependencies]`; crates inherit them with `.workspace = true`.
- Crates:
  - `crates/stable-flight-sitl` — the library: the flight-control pipeline plus the RealFlight
    SITL harness. Examples live in `crates/stable-flight-sitl/examples/*.rs`.
  - `crates/stable-flight-tools` — unpublished (`publish = false`) dev binaries: `plot_flight`
    (3D flight-path visualizer) and `generate_figure_eight` (waypoint-script generator).
- `firmware/` is excluded from the workspace; boards will get their own nested workspaces.
- Run every command from the repo root: `cargo run --example <name>` / `--bin <name>` resolve
  across the workspace, and `flights/*.csv` paths are relative to the root.
- `stable-flight-sitl` public modules: `commanders`, `mixers`, `orchestrator`, `simulators`, `stabilizers`, `telemetry`.
- Key external crates: `realflight-bridge` (sim I/O), `robomath` (`Vec2`/`Vec3`/`Quaternion`), `uom` (units), `mavlink`, `gilrs` (joystick), `thiserror`, `signal-hook`. Dev: `approx`, `assert2`, `clap` (examples parse args with it). `stable-flight-tools` adds `plotters` and `clap`.
- Untracked generated artifacts: `target/`, `Cargo.lock`, `tarpaulin-report.html`.
- Unless a path says otherwise, `src/…` and `examples/…` below are relative to
  `crates/stable-flight-sitl/`.
- Plans live in `plans/` (tracked). `plans/workspace-migration.md` tracks the split into
  core / sitl / backend crates; `plans/architecture-refactor.md` tracks trait-surface cleanups
  that land first (its PR 5 is superseded by the migration). Read both before touching trait
  surface or crate layout.
- The repo was renamed `stable-flight-sitl` → `stable-flight`; the crate keeps the
  `stable-flight-sitl` name. Old GitHub URLs redirect.

## Commands

```bash
cargo build --workspace                         # or: just build
cargo test --workspace --all-features           # or: just test
cargo run --example copter-manual-realflight    # joystick + RealFlight (--host to override)
cargo run --example copter-auto-realflight      # scripted figure-eight + RealFlight (script arg, --host)
cargo run --example extreme_3d_commander        # joystick smoke test
cargo run --example publish_telemetry           # MAVLink demo
cargo run --bin plot_flight -- flights/figure_eight.csv out.svg  # 3D path viz (SVG or PNG); or: just plot …
cargo run --bin generate_figure_eight           # (re)write flights/figure_eight.csv; or: just gen-figure-eight
cargo tarpaulin --workspace -o html             # or: just cover  (writes tarpaulin-report.html)
cargo fmt --all
cargo clippy --workspace --all-targets
```

The flight pipeline has no CLI and there is no `cargo bench` target — don't suggest
`cargo run -- --commander …` or `cargo bench`. The only binaries are `plot_flight` and
`generate_figure_eight` in `stable-flight-tools`; their unit tests run under
`cargo test --workspace`. Add `-p <crate>` to scope any command to one crate.

## Architecture

The control system is a five-trait pipeline coordinated by `FlightOrchestrator`. Traits and methods (from `crates/stable-flight-sitl/src/lib.rs`):

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

pub trait Stabilizer {
    fn stabilize(&mut self, dt: f32, command: &Command, state: &VehicleState) -> ControlOutput;
    fn prime(&mut self, state: &VehicleState);
}

pub trait Mixer {
    fn mix(&self, controls: &ControlOutput) -> ActuatorCommand;
}

pub trait TelemetryPublisher {
    fn publish(&mut self, dt: Duration, command: &Command, telemetry: &VehicleState);
}
```

Trait methods **do not return `Result`** today. Invalid configuration panics at construction or in `run`; per-tick failures are not modeled. If you add fallible operations, introduce a typed error with `thiserror` rather than panicking inside the loop.

`Command` is an enum: `Attitude(AttitudeCommand)`, `Rate(RateCommand)`, `Waypoint(WaypointCommand)`. `ActuatorCommand` carries `motors: [f32; 8]` plus fixed-wing surfaces (`rudder`/`elevator`/`aileron`/`throttle`/`elevon_*`).

### Concrete implementations

| Trait              | Impls                                                                |
|--------------------|----------------------------------------------------------------------|
| `Commander`        | `Extreme3DCommander`, `SpektrumCommander`, `ScriptCommander`         |
| `Vehicle` (`StateEstimator` + `Actuator`) | `RealFlightVehicle<T: RealFlightBridge>`, `NullVehicle` |
| `Stabilizer`       | `CopterStabilizer`                                                   |
| `Mixer`            | `QuadCopterMixer`; `TailSitterMixer` (placeholder, outputs zeros)    |
| `TelemetryPublisher` | `MavlinkPublisher`, `FileLogger`                                   |

`CopterStabilizer` is composed of PID controllers in `stabilizers::controllers::{attitude, body_rate, yaw, altitude, lateral}`.

`ScriptCommander::new(path) -> Result<Self, ScriptError>` loads a CSV waypoint file
(`x,y,altitude,cruise_speed`) from `flights/`, then follows and loops the waypoints by
horizontal acceptance radius. It is geometry-agnostic — the figure-eight shape lives in
`flights/figure_eight.csv`, produced by the `generate_figure_eight` binary
(`crates/stable-flight-tools/src/bin/generate_figure_eight.rs`). The
RealFlight examples are intentionally standalone (each inlines its full pipeline wiring),
so some setup is duplicated between them by design — readability over DRY.

### Control loop (from `src/orchestrator.rs`)

```rust
let command = self.commander.next_command(dt, &state);
state = self.vehicle.estimate_state(Duration::from_secs_f32(dt));
let control = self.stabilizer.stabilize(dt, &command, &state);
let actuators = self.mixer.mix(&control);
self.vehicle.actuate(&actuators);
self.telemetry.publish(Duration::from_secs_f32(dt), &command, &state);
```

`orchestrator.run(target_frequency: f64)` runs the loop and panics on non-positive frequency. SIGINT (Unix) / Ctrl-C (Windows) triggers graceful shutdown.

## Coding style

- Match surrounding code first; the rules below resolve ties.
- `cargo fmt --all` (4-space indent, trailing commas where idiomatic).
- snake_case files/modules, PascalCase types, SCREAMING_SNAKE_CASE constants. Pull magic numbers into named constants when they appear more than once or carry units.
- Prefer early returns and small helpers; keep flight-math functions readable, not arbitrarily short. If a PID step is clearest as one 30-line function, leave it.
- Use `Result<T, E>` with `thiserror` enums for new fallible APIs. Avoid panics in library code outside of clearly documented preconditions (e.g. `run`'s frequency assert).
- **No `.unwrap()` or `.expect()` in `src/`.** They turn recoverable errors into panics that crash the flight loop. Propagate with `?`, return a typed error, or — only when a value is provably non-`None` from local context — use `unwrap_or_else(|| unreachable!("..."))` with a comment explaining the invariant. `.expect()` is fine in `examples/` and `tests/` where a crash is the acceptable failure mode.
- Express physical quantities with `uom` and vector math with `robomath`. Don't reinvent.
- Logging via `log` + `env_logger`. Reserve `info!` for lifecycle events; use `debug!` inside the loop.
- Use `&` and slices to avoid allocations in the hot loop. No `Box`/`Vec` per tick.

## Testing

- Unit tests sit next to the code in `mod tests;` files (e.g. `src/stabilizers/copter/tests.rs`, `src/mixers/quad/tests.rs`, `src/orchestrator/tests.rs`, `src/simulators/realflight/tests.rs`, `src/commanders/tests.rs`).
- Run `cargo test --workspace --all-targets` locally; integration-style checks that need simulator interaction live in `examples/`.
- Name tests after the behaviour they protect (e.g. `maintains_pitch_hold`), not the function under test.
- Use `assert2::assert!` / `check!` and `approx::assert_relative_eq!` for floats.
- Table-driven tests are the default for control math; one row per scenario.
- Mock traits by hand (see `MockRealFlightBridge` in `src/simulators/realflight/tests.rs`) — `mockall` is not a dependency.
- `proptest` and `criterion` are **not** in `Cargo.toml`. Add them deliberately, with a benchmark or property test in the same PR, if you reach for them.
- Coverage: `just cover` → review `tarpaulin-report.html`. Refresh after non-trivial control-loop changes.

## Simulator setup

RealFlight runs on Windows; on macOS use a Windows VM. Enable **Settings → Physics → Quality → RealFlight Link**.

```rust
use realflight_bridge::RealFlightRemoteBridge;
use stable_flight_sitl::simulators::RealFlightVehicle;
use stable_flight_sitl::simulators::realflight::{ActuatorMapping, RealFlightConfiguration};

let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
    bridge: RealFlightRemoteBridge::new("10.211.55.3:8080")
        .expect("Failed to create bridge"),
    mapping: ActuatorMapping {
        motors: [Some(2), Some(0), Some(1), Some(3), None, None, None, None],
        ..Default::default()
    },
});
```

`RealFlightLocalBridge` is available when running on the same host as RealFlight. Test models: **QuadCopterX** (multirotor) and **eXplora VTOL** (tailsitter); more in the ArduPilot SITL_Models repo.

## Commits & PRs

- Small, imperative commit subjects mirroring existing history (e.g. `Use f32 instead of Duration... (#18)`). Reference issues/PRs with `(#id)`.
- Include simulator context for flight-behaviour changes (model, mode, what you flew).
- PRs need a summary of the scenario tested, reproduction/validation steps (commands, configs), and screenshots or telemetry snippets when visual feedback changes.
- **Merges are squash-only** — merge commits and rebase are disabled on GitHub, so a PR lands as a single commit titled `<PR title> (#id)`. Keep the PR title release-note quality; intermediate branch commits are discarded.
- PRs must be green on the `test` GitHub Actions workflow (a `build` job running the test suite) before they can merge; the PR shows `UNSTABLE`/pending until it finishes.
- CI triggers only on PRs whose base is `main`. In a stack of PRs (each based on the previous branch), only the bottom one gets checks. Because merges are squash-only, rebase each child onto `main` (`git rebase --onto main <old-parent-branch>`) after its parent merges, then retarget it to `main` so CI runs.
- A new published crate inherits `[workspace.package]`, gets its own `README.md` (absolute GitHub links; crates.io doesn't resolve repo-relative ones) and `CHANGELOG.md`, and symlinks `LICENSE` (`ln -s ../../LICENSE`) so it ships in the package. Check with `cargo package -p <crate> --list`.
- **Keep docs in sync with the code, in the same PR.** Renamed trait, changed a method signature, added/removed a component or crate, retired a CLI/example, swapped a default — update the doc that owns the fact. Stale docs mislead both humans and AI; treat them as part of the change, not a follow-up. Each fact has one home; other docs link to it rather than restating it:
  - crate `README.md` — that crate's purpose, usage, features;
  - root `README.md` — project overview, crate index, supported simulators, quick start;
  - `CLAUDE.md` — commands, coding/testing rules, PR checklist;
  - `architecture.md` — crate boundaries, dependency rules, traits, frame and timing contracts.
- **Maintain each published crate's `CHANGELOG.md`** (`crates/<crate>/CHANGELOG.md`; unpublished `stable-flight-tools` has none) ([Keep a Changelog](https://keepachangelog.com/en/1.1.0/)). Every user-facing change adds a bullet under `## [Unreleased]` in the same PR, grouped by `Added`/`Changed`/`Deprecated`/`Removed`/`Fixed`/`Security`; mark breaking changes **Breaking**. Skip internal-only churn (refactors, test/CI tweaks). At release, rename `[Unreleased]` to `## [x.y.z] - YYYY-MM-DD`, add a fresh empty `[Unreleased]`, and update the compare links.

## Workflow

1. Read the relevant trait in `src/lib.rs` and its existing impls before adding a new one.
2. Wire new components through the orchestrator the way `examples/copter-manual-realflight.rs` does — that's the canonical reference.
3. Add tests alongside the code; cover the boundary/error case, not just the happy path.
4. `cargo fmt --all && cargo clippy --workspace --all-targets && cargo test --workspace --all-features` before sending a PR.
5. If you change trait surface, module or crate layout, or runnable examples, update the owning docs (see Commits & PRs) in the same PR.
6. To compare against `main` (test counts, clippy output), use a throwaway worktree — `git worktree add <dir> main` with its own `CARGO_TARGET_DIR` — not a `git stash` round-trip; stashing staged renames plus untracked files can fail to restore.
7. Pass PR bodies to `gh pr create`/`edit` with `--body-file`; inline heredocs inside `$(…)` break on apostrophes.

## Pre-PR checklist

- [ ] Builds clean (`cargo build --workspace`) and tests pass (`cargo test --workspace --all-features`).
- [ ] `cargo fmt --all` clean; `cargo clippy --workspace --all-targets` adds no new warnings. The tree carries pre-existing warnings (deprecated `let_assert`, unused variables in controllers/tests), so diff against `main` rather than expecting zero.
- [ ] New fallible APIs return `Result` with `thiserror` types; no new panics in library code.
- [ ] No `.unwrap()` or `.expect()` introduced under `src/` (examples/tests OK).
- [ ] New PID/quaternion/transform logic is unit-tested with table cases.
- [ ] Public API has Rustdoc with at least one example.
- [ ] Hot-path code allocates nothing per tick.
- [ ] Coverage refreshed if control-loop logic changed.
- [ ] Owning docs (crate/root `README.md`, `CLAUDE.md`, `architecture.md`) reflect any trait, module, crate, or example changes.
- [ ] Each affected published crate's `CHANGELOG.md` has an `[Unreleased]` entry for any user-facing change.
