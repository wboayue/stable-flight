# Workspace migration plan

Turn `stable-flight-sitl` into a `stable-flight` monorepo. A `no_std` core crate holds the
flight-control code. A backend-agnostic SITL harness consumes it, with one crate per simulator
backend (RealFlight first). Board firmware, added later, is another consumer.

## Status

Legend: ✅ merged · 🔍 in review · ⬜ not started · ⏭ superseded. Update a row when its PR
opens (🔍 #n) and again when it merges (✅ #n). Do it in the PR itself where possible;
otherwise do it in the next PR.

| Step | Status |
|---|---|
| Phase 0 — prep | ✅ (step 3, the optional local dir rename, not done) |
| PR 1 — Convert to workspace | ✅ #43 |
| architecture-refactor PRs 1–4, 6 (see that plan) | ✅ #44, #50, #46; 🔍 #47–#48 (stacked) |
| PR 2 — Extract `stable-flight` core crate | ⬜ (blocked on #47) |
| PR 3 — Split RealFlight into its own backend crate | ⬜ |
| PR 4 — Make core `no_std` | ⬜ |
| architecture-refactor PR 7 — split `Stabilizer` | ⬜ (before publishing core) |

## Target layout

```
stable-flight/                       # GitHub: wboayue/stable-flight (renamed from stable-flight-sitl)
  Cargo.toml                         # virtual manifest; members = crates/*, exclude = firmware/*
  Cargo.lock
  justfile  CLAUDE.md  README.md  architecture.md
  .github/workflows/test.yaml
  flights/                           # waypoint CSVs (shared; paths stay relative to repo root)
  plans/
  crates/
    stable-flight/                   # no_std core: types, traits, controllers, stabilizers, mixers,
      CHANGELOG.md  README.md        #   FlightLoop::step, WaypointFollower, ComposedVehicle,
                                     #   tuple TelemetryPublisher; `testing` feature: test doubles
    stable-flight-sitl/              # std harness: runners, ShutdownSignal, commanders (joystick, script
      CHANGELOG.md  README.md        #   loader), telemetry (MAVLink, file), conformance kit
    stable-flight-sitl-realflight/   # RealFlightVehicle, ActuatorMapping, airframes; deps: core only
      CHANGELOG.md  README.md        #   (sitl is a dev-dependency for examples + conformance)
    stable-flight-tools/             # publish = false: plot_flight, generate_figure_eight
    # later: stable-flight-sitl-<backend>/ per simulator (Gazebo, JSBSim, …)
  firmware/                          # later: one nested workspace per board (not part of this plan)
```

Dependency direction (arrows point at the dependency):

```
tools ──► sitl ──► core ◄── sitl-realflight ◄┄┄ (dev) sitl
                    ▲
          firmware ─┘
```

Firmware, when added, lives in nested workspaces because Cargo config is resolved per directory,
per-package targets are nightly-only, and firmware needs its own profiles (`panic = "abort"`,
size opt) and lockfile (HAL churn).

## Decisions

| Question | Decision | Why |
|---|---|---|
| Host repo | This repo, renamed `stable-flight-sitl` → `stable-flight` | Keeps the PRs, issues, CI, and docs. GitHub redirects the old URL |
| `flights/` | Repo root | Examples/bins resolve `flights/...` from cwd. Running from the root keeps every documented command working |
| Crate names | `stable-flight`, `stable-flight-sitl`, `stable-flight-sitl-<backend>`, `stable-flight-tools` | Free on crates.io (checked 2026-09-28). The shared prefix groups backends on crates.io. Nothing is published yet, so no compatibility shims needed |
| Backends: crates or features | One crate per backend | Each backend brings heavy, backend-specific deps (protobuf/gz-transport, C++ FFI). With features, `--all-features` CI would need every backend's system libs, and feature unification leaks them into every build. Crates also keep each backend's mocks, examples, and tests self-contained |
| Backend dependencies | Core only; sitl as a dev-dependency | Makes a backend reusable with any runner (sitl, HIL, a third-party harness). Only examples and conformance tests need the harness |
| What stays shared | Harness in `stable-flight-sitl` | Runners, shutdown, commanders, telemetry, and conformance don't depend on which simulator is attached |
| Dev tools | Separate `stable-flight-tools` crate, `publish = false` | Plotting/generation aren't harness responsibilities. Drops the `plot`/`tools` features and `plotters`/`clap` from the harness |
| Test doubles | Core `testing` feature | One set of `NullVehicle`/constant commander/recording telemetry for core, sitl, and every backend's tests |
| Airframe tuning | Per-backend `airframes` module | Gains belong to the sim's airframe model, not the example. Examples stay standalone in wiring but stop copying gains |
| `ActuatorMapping` | Stays in the RealFlight crate for now | Promote a generic channel map to the harness when backend 2 shows what's really common |
| Per-crate CHANGELOG | Yes (published crates only) | Crates version and publish independently. `stable-flight-tools` doesn't get one |
| READMEs | Root README = project overview; each crate README = that crate's usage | crates.io renders the crate README |

## Phase 0 — prep (no PR)

1. ✅ Land or park open branches, so the big file move doesn't cause rebase conflicts.
2. ✅ GitHub: rename `wboayue/stable-flight-sitl` → `stable-flight`, then
   `git remote set-url origin git@github.com:wboayue/stable-flight.git`.
3. ⬜ Optional: `mv ~/projects/stable-flight-sitl ~/projects/stable-flight` and move the Claude
   memory dir (`~/.claude/projects/-Users-wboayue-projects-stable-flight-sitl`) to match.

## PR 1 — Convert to workspace (mechanical, no code changes)

Title: `Convert repo to Cargo workspace; split dev tools into their own crate`

- `git mv src examples Cargo.toml CHANGELOG.md crates/stable-flight-sitl/`. Keep `Cargo.lock`,
  `flights/`, `plans/`, `justfile`, `CLAUDE.md`, `architecture.md`, `LICENSE` at the root.
  Copy `README.md` into the crate and slim the root one later.
- `git mv src/bin crates/stable-flight-tools/src/bin`. Tools `Cargo.toml` has
  `publish = false`, depends on `stable-flight-sitl` (path; `plot_flight` uses
  `commanders::script::load_waypoints`), and takes `plotters`/`clap` as plain dependencies.
  Remove the `plot`/`tools` features, the `[[bin]]` tables, and the optional deps from the sitl
  manifest. Bin unit tests now run under plain `cargo test --workspace`.
- Root `Cargo.toml`:
  ```toml
  [workspace]
  resolver = "3"
  members = ["crates/*"]
  exclude = ["firmware"]

  [workspace.package]
  version = "0.1.0"
  edition = "2024"
  authors = ["Wil Boayue <wil@wsbsolutions.com>"]
  license = "MIT"
  repository = "https://github.com/wboayue/stable-flight"

  [workspace.dependencies]
  robomath = { version = "0.1.0", default-features = false }
  uom = { version = "0.38.0", default-features = false, features = ["f32", "si"] }
  log = "0.4.25"
  thiserror = { version = "2.0", default-features = false }
  clap = { version = "4", features = ["derive"] }
  approx = "0.5.1"
  assert2 = "0.4.0"
  ```
  The crates inherit these with `edition.workspace = true`, `robomath.workspace = true`, etc.
  **Caution:** in PR 1, keep the sitl crate's `uom` on its current std/default features
  (`uom = { workspace = true, features = ["std", ...] }` or leave it un-inherited). This PR
  must not change behaviour.
- CI: `cargo test --workspace --all-features`.
- `justfile`: `build`/`test`/`cover` get `--workspace`. `tag` becomes per-crate
  (`stable-flight-sitl-v0.1.0`). Add `plot` / `gen-figure-eight` recipes wrapping
  `cargo run -p stable-flight-tools --bin …`.
- Docs: update `repository`, the crates.io badge, and `blob/main/architecture.md` links in
  `lib.rs`/README. CLAUDE.md gets the new layout, the `-p <crate>` commands (the bins no longer
  need `--features`), and the test-file paths (`crates/stable-flight-sitl/src/...`).
- Verify: `cargo test --workspace --all-features`, `cargo clippy --workspace --all-targets`,
  `cargo run -p stable-flight-sitl --example copter-auto-realflight` from the root (resolves
  `flights/`), and both bins via `-p stable-flight-tools`.

Risk: low. Blame survives through `git log --follow` (pure renames).

**As landed (#43).** Differences from the steps above:
- `Cargo.lock` stays gitignored (it already was). Revisit if the tools binaries need
  reproducible builds.
- `[workspace.dependencies]` has only what's inherited today (`robomath`, `log`,
  `thiserror`, `clap`, `approx`, `assert2`) at their current default features. The
  `no_std` feature settings (`default-features = false`, `uom`) move to PR 4.
- `cargo run --example …` / `--bin …` resolve from the root without `-p`, so documented
  commands barely changed. The justfile gained `plot`, `gen-figure-eight`, and a
  per-crate `tag CRATE VERSION`.
- The root README became the project overview with a crate table, and the component
  walkthrough moved to the crate README. `LICENSE` is symlinked into the crate so it
  ships in the package.
- Regenerating `flights/figure_eight.csv` changed only its comment header.

## PR 2 — Extract `stable-flight` core crate (still std)

Title: `Extract flight-control core into stable-flight crate`

Moves into `crates/stable-flight/src/`:

| From (sitl) | To (core) |
|---|---|
| `lib.rs` types: `Command`, `AttitudeCommand`, `RateCommand`, `WaypointCommand`, `ControlOutput`, `VehicleState`, `ActuatorCommand` | `lib.rs` |
| Traits: `Commander`, `Vehicle`, `StateEstimator`, `Actuator`, `Stabilizer`, `Mixer`, `TelemetryPublisher` | `lib.rs` |
| `stabilizers/` (copter, controllers, `Gains`, tests, controllers README) | `stabilizers/` |
| `mixers/` (quad, tailsitter, tests) | `mixers/` |
| Per-tick sequencing from `FlightOrchestrator::run` | `flight_loop.rs`: `FlightLoop::step(dt)` + `prime` (was `set_initial_state` before #46) |
| Waypoint-following logic from `ScriptCommander` | `commanders/waypoint.rs`: `WaypointFollower` |
| `NullVehicle` (from `simulators/mod.rs`) | `testing` module |

Stays in sitl: `commanders/` (gilrs joysticks, CSV loading), `simulators/`, `telemetry/`
(mavlink, file), runners, examples.

**Split `FlightOrchestrator` by responsibility.** Today it registers signal handlers (inside
`new()`), paces on the wall clock, sequences the tick, and reports loop metrics.
- Core `FlightLoop` sequences one tick only: commander → estimate → stabilize → mix → actuate
  → telemetry. No clock, no threads, no signals. Every runner calls it.
- Sitl `ShutdownSignal`: owns SIGINT/Ctrl-C registration and exposes `is_set()`. Build it
  explicitly and pass it to the runner, so construction no longer has side effects and tests
  can trip it by hand. A future lockstep runner reuses it instead of copying the handler.
- Sitl `RealTimeRunner::run(&mut FlightLoop, frequency, &ShutdownSignal)`: wall-clock pacing
  plus the frequency metrics. Keeps the non-positive-frequency assert.

**Split `ScriptCommander` by responsibility.** Today it both loads the CSV (std) and follows
waypoints (pure math), so waypoint flight couldn't run on firmware.
- Core `WaypointFollower<W: AsRef<[WaypointCommand]>>` implements `Commander`: acceptance
  radius, advance, loop. Firmware can use a `[WaypointCommand; N]` or `&'static [_]`; sitl
  uses a `Vec`.
- Sitl `ScriptCommander` becomes `load_waypoints` + `WaypointFollower<Vec<_>>`. Keep the
  `new(path)` / `from_waypoints` / `with_acceptance_radius` API as thin wrappers.
- Split the tests: following behaviour goes to core, parsing/IO errors stay in sitl.

**Add composition points to core** (each is small, allocation-free, and table-tested):
- `ComposedVehicle<E: StateEstimator, A: Actuator>` implements `Vehicle`. This is how firmware
  (IMU + estimator, ESC output) and sensor-synthesis SITL (truth → sensors → estimator)
  assemble a vehicle. Depends on architecture-refactor PR 4 settling the
  `StateEstimator`/`Actuator` signatures.
- `impl<A: TelemetryPublisher, B: TelemetryPublisher> TelemetryPublisher for (A, B)`. This
  fans out to several sinks (MAVLink + file log) without `Vec<Box<dyn _>>`. Nest the tuples
  for more than two.

**Core `testing` feature** (off by default): `NullVehicle`, `ConstantCommander`,
`RecordingTelemetry` (fixed-capacity, no alloc). Core tests use it via `cfg(test)`. Sitl and
backend crates enable it in `[dev-dependencies]`. Delete the ad-hoc mocks this replaces
(e.g. in `commanders/tests.rs` and `orchestrator/tests.rs`).

Other points:
- Sitl depends on `stable-flight = { path = "../stable-flight", version = "0.1.0" }` and updates
  imports. No glob re-export. Nothing is published yet, so there's no compatibility shim.
- Split orchestrator tests: step semantics go to core, and runner/shutdown/frequency-assert
  tests stay in sitl.
- Doc tests: `stable_flight_sitl::X` → `stable_flight::X` for the moved items.
- Docs: `architecture.md` gets a crate-boundary section plus the runner / `FlightLoop` /
  `ShutdownSignal` split. The CLAUDE.md control-loop snippet points at `FlightLoop::step`, and
  the impl table notes which crate each impl lives in. Add a core CHANGELOG (`Added`: initial
  extraction, `WaypointFollower`, `ComposedVehicle`, tuple telemetry). The sitl CHANGELOG gets
  **Breaking**: types/traits moved to `stable-flight`; `FlightOrchestrator` replaced by
  `FlightLoop` + `RealTimeRunner` + `ShutdownSignal`.

Risk: medium. Import churn is wide, and the orchestrator and `ScriptCommander` splits are real
refactors. Keep the existing tests as the behavioural oracle, so they pass with only import
and location changes. If the PR gets too big, split it into 2a (move + `FlightLoop`), 2b
(`ShutdownSignal`/runner, `WaypointFollower`), and 2c (`ComposedVehicle`, tuple telemetry,
`testing`).

## PR 3 — Split RealFlight into its own backend crate

Title: `Move RealFlight backend into stable-flight-sitl-realflight`

- `git mv` `simulators/realflight.rs` + `realflight/tests.rs` (with `MockRealFlightBridge`) →
  `crates/stable-flight-sitl-realflight/src/`. `realflight-bridge` moves with it.
- **Dependencies:** `[dependencies]` holds `stable-flight`, `realflight-bridge`, and `log` only.
  `stable-flight-sitl` and `stable-flight` with `features = ["testing"]` are
  `[dev-dependencies]`. CI checks this: `cargo tree -p stable-flight-sitl-realflight -e normal`
  must not list `stable-flight-sitl`.
- Move `copter-manual-realflight` and `copter-auto-realflight` into the backend crate's
  `examples/`. `extreme_3d_commander` and `publish_telemetry` stay in sitl (no simulator).
- The `simulators` module goes away (`NullVehicle` already moved to core `testing` in PR 2).
  This supersedes architecture-refactor PR 5.
- **Airframes.** Add `airframes::quadcopter_x() -> QuadCopterXAirframe { stabilizer:
  CopterStabilizerConfig, mapping: ActuatorMapping }`. It's tuned for the ArduPilot SITL_Models
  QuadCopterX in RealFlight. Both examples use it: today `copter-manual` inlines custom
  rate/attitude gains while `copter-auto` uses `CopterStabilizerConfig::default()` for the same
  model. Pick one tuned set, and note the choice in the PR with a flight note. Examples still
  inline their wiring (commander, mixer, telemetry, runner), per CLAUDE.md. Update that rule to
  say "gains come from the backend's `airframes`".
- **Frame contract.** `VehicleState`'s conventions are currently implicit RealFlight ones:
  `position.z` is altitude ASL, up-positive; the quaternion is world→body; the angles are
  radians; and the world axes are whatever RealFlight uses. Document the contract (world frame,
  body frame, units, sign of each angle and rate) on `VehicleState` in core. Each backend
  converts at its boundary.
- **Timing contract.** Document on `Vehicle` what a backend must guarantee each tick: which
  call talks to the sim, and how old the state from `estimate_state` is relative to the last
  `actuate`. Today `RealFlightVehicle::actuate` does the `exchange_data` round-trip and
  `estimate_state` returns the cached result, so the stabilizer sees state from the previous
  exchange. Make this explicit in the RealFlight rustdoc and add a test that pins it (mock
  bridge counts exchanges per tick).
- **Conformance kit.** Add `stable_flight_sitl::conformance`, behind a `conformance` feature so
  test support stays out of the harness's default API. It's a table of physical scenarios
  (level, 30° right roll, 15° nose up, heading east, climbing, yaw-right rate) with their
  expected `VehicleState` signs and values. Each backend feeds in its native fixture for every
  row. The RealFlight crate runs it against `MockRealFlightBridge` fixtures, using the
  dev-dependency with `features = ["conformance"]`. This catches frame bugs before a backend
  flies.
- `justfile`: `fly-manual` / `fly-auto` recipes wrapping
  `cargo run -p stable-flight-sitl-realflight --example …`.
- Docs: CLAUDE.md "Simulator setup" becomes a per-backend section, and the impl table gets a
  crate column. README gets a "Supported simulators" table. architecture.md documents the
  backend boundary, the dependency rule, and the frame and timing contracts.
- CHANGELOG: the sitl crate gets **Breaking** (`RealFlightVehicle` moved). The new crate gets
  `Added`.

Risk: low–medium. Code moves are mechanical. Writing down the frame contract and unifying the
gains may surface a latent sign bug or a tuning regression. That's the point, but fix any
behaviour change in its own PR with a flight note.

## PR 4 — Make core `no_std`

Title: `Make stable-flight no_std`

- `#![cfg_attr(not(test), no_std)]` (same pattern as robomath). Tests stay on host.
- `std::time::Duration` → `core::time::Duration`. `std::f32::consts` → `core::f32::consts`.
- `yaw.rs:196` `f32::rem_euclid` is std-only. Replace with a small `libm`-based helper and
  add a table test for negative/wrap angles. Route any other std-only float methods
  (`sqrt`, `sin`, `powi`, …) through `libm`. The embedded CI build flags anything missed.
  `WaypointFollower`'s distance math is covered by the same rule.
- `uom` via the workspace entry (`default-features = false, features = ["f32","si"]`). The sitl
  crate turns on `std` for itself.
- `log` works as-is in `no_std`. `TailsitterMixer::mix` logs `info!` per tick. Downgrade it to
  `debug!` (CLAUDE.md rule) while in here.
- Audit for `Vec`/`String`/`format!`. There should be none (hot-path rule already bans them).
  The `testing` feature has to stay `no_std` too (`RecordingTelemetry` is fixed-capacity).
- CI: add a job that runs `rustup target add thumbv7em-none-eabihf` and then
  `cargo build -p stable-flight --target thumbv7em-none-eabihf` and
  `cargo build -p stable-flight --features testing --target thumbv7em-none-eabihf`. This is
  what keeps the core embeddable.
- Optional: decide whether `VehicleState` keeps `uom` quantities or goes to plain SI `f32`
  (the stabilizer only reads `.value`). Separate PR if yes (**Breaking**).

Risk: low–medium (`rem_euclid` replacement is the only math change; covered by tests).

## Later (out of scope)

- First board under `firmware/<board>/`: its own `[workspace]`, `Cargo.lock`,
  `.cargo/config.toml` (target, runner), and `publish = false`. Depends on
  `../../crates/stable-flight` by path, with a CI job that builds it via `--manifest-path`. It
  assembles its vehicle with `ComposedVehicle`.
- **Second backend** (e.g. Gazebo or JSBSim). It will likely need:
  - A `LockstepRunner` in sitl, next to `RealTimeRunner`. `dt` comes from sim time, and the
    controller steps the sim instead of sleeping. It reuses `FlightLoop` and `ShutdownSignal`,
    so only pacing is new.
  - Possibly a timestamp on `VehicleState`. `Vehicle::estimate_state(dt)` takes the elapsed
    time as input, but a lockstep sim reports its own.
  - Promoting `ActuatorMapping` to a shared channel map.
  Design these against the real second backend, not speculatively.
- **Sensor-synthesis path.** Backends expose truth state today. Add IMU/GNSS/baro synthesis
  with a noise model from truth, composed through `ComposedVehicle`, so a real estimator
  (fusion-nav) runs in the SITL loop, the same as on hardware.
- **Unify joystick commanders.** `Extreme3DCommander` and `SpectrumCommander` duplicate gilrs
  setup, `read_channels`, and `scale_value`. Replace them with one `GamepadCommander` plus a
  per-device channel map.
- **Split MAVLink encoding from transport.** Put `VehicleState` → `MavMessage` in a `no_std`
  module so firmware can reuse it (the `mavlink` crate supports `no_std`), and keep UDP in sitl.
- `fusion-nav` adapter (`impl StateEstimator`, nalgebra → robomath) behind an optional
  `fusion-nav` feature in core. fusion-nav stays in its own repo.
- `crates/stable-flight-drivers` once a second board shares drivers.
- Publish `stable-flight` 0.1.0, then `stable-flight-sitl`, then `stable-flight-sitl-realflight`
  (dependency order).

## Docs ownership

With several crates, the rule to keep README/CLAUDE.md/architecture.md in sync gets expensive
unless each fact has one home:

| Doc | Owns |
|---|---|
| Crate `README.md` | That crate's purpose, usage example, features |
| Root `README.md` | Project overview, crate index with one line each, supported simulators, quick start |
| `CLAUDE.md` | Commands, coding/testing rules, PR checklist |
| `architecture.md` | Crate boundaries, dependency rule, traits, frame and timing contracts |

Other docs link to the owner instead of restating it. Update CLAUDE.md's sync rule to match
in PR 1.

## Interaction with `plans/architecture-refactor.md`

That plan's PRs 4 (`Vehicle` vs `StateEstimator`/`Actuator`), 6 (decouple telemetry from
`Command`), and 7 (split `Stabilizer`) change the trait surface that becomes the core's public
API. Its PR 5 (rename `simulators/` → `vehicles/`) is superseded by workspace PR 3: the module
disappears and backends become crates. Suggested order:

1. ✅ Workspace PR 1 (mechanical move first, so everything else lands in the final layout).
2. 🔍 Refactor PRs 1–4 and 6 (cheap, and they shape the core API). Refactor PR 4 has to land
   before `ComposedVehicle` (workspace PR 2): it fixes the `StateEstimator`/`Actuator`
   signatures that `ComposedVehicle` composes, and `Vehicle` is the backend contract.
3. ⬜ Workspace PRs 2–4.
4. ⬜ Refactor PR 7 before publishing `stable-flight` 0.1.0.

## Checklist per PR

- [ ] `cargo fmt --all`, `cargo clippy --workspace --all-targets`, `cargo test --workspace --all-features`
- [ ] Embedded build job green, with and without `testing` (PR 4+)
- [ ] Conformance kit green for every backend (PR 3+)
- [ ] No backend crate has `stable-flight-sitl` as a normal dependency (PR 3+)
- [ ] Docs updated in the owning file (see Docs ownership)
- [ ] CHANGELOG entry in each affected published crate
