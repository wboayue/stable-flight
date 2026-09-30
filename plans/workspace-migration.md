# Workspace migration plan

Turn `stable-flight-sitl` into a `stable-flight` monorepo. A `no_std` core crate holds the
flight-control code. A backend-agnostic SITL harness consumes it, with one crate per simulator
backend (RealFlight first). Board firmware, added later, is another consumer.

## Maintaining this plan

This file is the running record of the migration and of everything its reviews turn up.
Update it in the same PR as the work; it's part of the change, not a follow-up.

1. **Status:** mark a row 🔍 #n when its PR opens and ✅ #n when it merges (in that PR, or
   the next one if it's already open). Add a row for any unplanned PR (e.g. a bug fix found
   along the way).
2. **As implemented:** when a PR lands differently from its section below, add a short
   "as implemented" / "after review" note to that section: what changed, what was decided,
   and why.
3. **Follow-ups:** every follow-up a review or investigation raises and that isn't fixed in
   the same PR gets a row in the [Follow-up backlog](#follow-up-backlog) (the item, its
   target PR, and its source). Don't leave follow-ups only in PR descriptions, review
   comments, or chat. The PR that completes one deletes its row and mentions it in its own
   note.
4. **Found bugs:** a pre-existing bug found while verifying goes in the backlog, or in
   Status if it's being fixed now, with how to reproduce it.

## Status

Legend: ✅ merged · 🔍 in review · ⬜ not started · ⏭ superseded. Update a row when its PR
opens (🔍 #n) and again when it merges (✅ #n). Do it in the PR itself where possible;
otherwise do it in the next PR.

| Step | Status |
|---|---|
| Phase 0 — prep | ✅ (step 3, the optional local dir rename, not done) |
| PR 1 — Convert to workspace | ✅ #43 |
| architecture-refactor PRs 1–4, 6 (see that plan) | ✅ #44, #50, #46, #47, #48 |
| PR 2a — core crate + `FlightLoop` | ✅ #52 |
| PR 2b — `ShutdownSignal` / `RealTimeRunner`, `WaypointFollower` | ✅ #53 |
| PR 2c — `ComposedVehicle`, tuple telemetry, `testing` feature | ✅ #54 |
| Fix — attitude-controller panic in waypoint mode (found in 2b) | ✅ #55 |
| Fix — unwrapped heading target → yaw panic in attitude mode (found in #55 review) | ✅ #56 |
| Fix — remaining per-tick panics (mixer, attitude-command, body-rate, altitude, rate-mode; Spektrum clamp) | ✅ #57 |
| PR 3 — Split RealFlight into its own backend crate | 🔍 #PRNUM |
| Fix — RealFlight frame conversions: double radians, down-positive vertical (found in PR 3) | ⬜ |
| PR 4 — Make core `no_std` | ⬜ |
| architecture-refactor PR 7 — split `Stabilizer` | ⬜ (before publishing core) |

## Follow-up backlog

Open follow-ups from PR reviews and investigations, collected in one place (see
[Maintaining this plan](#maintaining-this-plan)). Add a row when one is raised; delete it in
the PR that completes it. The per-PR notes further down keep the history.

| Follow-up | Target | Source |
|---|---|---|
| RealFlight frame bugs. `update_estimate` runs `to_radians()` on uom `.value`, which is already radians, so roll/pitch/heading/body rates come out ~57× too small. `velocity_world_w`/`acceleration_world_az` are down-positive but the contract (and the altitude controller) want up. Reproduce: `cargo test -p stable-flight-sitl-realflight -- --ignored`. Fix both, un-ignore the `conforms_*` tests, and retune/flight-check (gains were tuned against the bug). | own PR (Status row) | PR 3 |
| Unify QuadCopterX gains: `copter-auto` still flies `CopterStabilizerConfig::default()`, not `airframes::quadcopter_x().stabilizer`. Do it with the frame fix, since both sets were tuned against the bug. | frame-fix PR | PR 3 |
| RealFlight conformance fixtures are synthetic. The quaternion assumes a standard ZYX world→body rotation, and the signs come from the `realflight-bridge` field docs. Capture a live `SimulatorState` per scenario (hover level, bank right, nose up, face east, climb, yaw right) and use those. | frame-fix PR | PR 3 |
| `SpectrumCommander` maps roll/pitch/yaw to [0, 1] centred at 0.5; `AttitudeCommand` needs [-1, 1] centred at 0 (constant bank at centre stick). Needs the transmitter to verify each axis's sign. | own PR | #57 |
| Upstream failsafe: estimator-health checks, then a land or disarm commander. The Input Policy is last-resort only (NaN thrust → motors off). | first board / Later | #57 review |
| `BodyRateController`: one non-finite axis zeroes all three moments; handle axes separately. | own PR | #57 review |
| Unify derivative-gap handling (flag in attitude/body-rate, `Option` in yaw) into a shared `DerivativeState`; make `BodyRateController::last_error` private; drop the first-tick-after-construction derivative kick in attitude/body-rate. | refactor PR 7 | #57 review |
| Make saturation observable (debug log or a telemetry flag/counter) for tuning. | refactor PR 7 | #57 review |
| Yaw: controller state carries across mode switches, `self.heading` goes stale in waypoint mode, and guidance needs an absolute-heading path. | refactor PR 7 | #56 review |
| Heading slew rate is per tick (`HEADING_STEP_PER_TICK`); make it `dt`-based. | own PR or refactor PR 7 | #56 review |
| Document the `VehicleState::heading` range (RealFlight gives [0, 2π); controllers accept any). | own PR | #56 review |
| Lateral: `accel / g` scaled by `max_angles` is a simplified tilt model; `LateralControllerConfig::max_rate` is unused. | refactor PR 7 | #55 |
| `no_std` blockers beyond the plan list: `powf` in `fit_to_thrust_curve`, `std::f32::consts` imports, `rem_euclid` in `math::wrap_angle`. | PR 4 | #56/#57 reviews |
| `TailSitterMixer` logs `info!` every tick; `ConsoleTelemetryPublisher` logs `info!` every tick. | PR 4 / own PR | #52, #48 reviews |
| `RealTimeRunner` replays missed ticks back-to-back after a stall; resync instead and count overruns in `RunStats`. | own PR or lockstep runner | #53 review |
| `Runner` trait, once a second runner exists. | Later | #53 review |
| `WaypointFollower` runtime path updates (`AsMut`/`set_waypoints`) for a mission uploader. | Later | #53 review |
| Sensor synthesis needs a wrapper like `Sensed<V: Vehicle, S>`; `ComposedVehicle` can't give the estimator the actuator's truth. | Later | #54 review |
| `ComposedVehicle` `estimator_mut`/`actuator_mut`. | first board | #54 review |
| Measured timestamp/overrun field on `TelemetryRecord`, so rate-bucketed publishers don't drift. | Later | #48 review |
| `FileLogger::publish` writes 10 heartbeats with 1 s sleeps per tick (a stub); a failed packet write after the timestamp truncates the `.tlog` frame. | own PR | #45/#50 reviews |
| `MavlinkPublisher::new`, `FileLogger::new`, and the joystick commanders' `Default` impls `.expect()` at construction; return `Result`. `MavlinkPublisher` is untestable against a failing connection until then. | own PR | #50 review |
| `stabilizers::low_pass_filter_vec3` is public but unused. | own PR | #44 review |
| CLAUDE.md's trait block and impl table repeat `lib.rs` / architecture.md; point to them instead. | docs PR | #44, #46, #47 reviews |
| `Commander`/`Mixer` combinators; keep `TelemetryPublisher` impls opt-in (no blanket impl) so the tuple impl stays coherent. | Later | #54 review |

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
- `ComposedVehicle<E: StateEstimator, A: Actuator>` implements `StateEstimator` and `Actuator`
  by delegation, and gets `Vehicle` from the blanket impl (#47). Implementing `Vehicle`
  directly is no longer possible. This is how firmware (IMU + estimator, ESC output) and
  sensor-synthesis SITL (truth → sensors → estimator) assemble a vehicle. The #47 review
  confirmed this compiles with no coherence conflict.
- `impl<A: TelemetryPublisher, B: TelemetryPublisher> TelemetryPublisher for (A, B)`. This
  fans out to several sinks (MAVLink + file log) without `Vec<Box<dyn _>>`. Nest the tuples
  for more than two.

**Core `testing` feature** (off by default): `NullVehicle`, `ConstantCommander`,
`RecordingTelemetry` (fixed-capacity, no alloc). Core tests use it via `cfg(test)`. Sitl and
backend crates enable it in `[dev-dependencies]`. Delete the ad-hoc mocks this replaces
(since 2b: the `Mock*` types in sitl `runner/tests.rs`; `commanders/tests.rs` and
`orchestrator/tests.rs` were deleted in 2b).

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

**Split into 2a/2b/2c.** 2a as implemented:
- Moved the types and traits (sitl `lib.rs` → core `lib.rs`; git sees a copy, so use
  `git blame -C -C` or `git log --follow -C` for history), `stabilizers/`, and
  `mixers/` to `crates/stable-flight`.
- New core `flight_loop.rs`: `FlightLoop::{new, prime, step, state}`, with sequence-log
  tests of call order and what each stage sees.
- `FlightOrchestrator` keeps its public API and wraps a `FlightLoop`. Its signal handling,
  pacing, and metrics move to 2b.
- `step` takes `dt: Duration` and passes `dt.as_secs_f32()` to the commander and stabilizer.
  Before, they got `1.0 / frequency` directly: identical at 250 Hz, and it differs only in
  the last float bits for periods that aren't a whole number of nanoseconds.
- Kept in sitl for now: `NullVehicle` (moves to core `testing` in 2c) and
  `commanders/tests.rs` (tests the `Commander` trait with a mock; revisit in 2b with
  `WaypointFollower`).
- Docs: new core README (pipeline, stabilizer, and mixer usage moved from the sitl README),
  root crate table, architecture.md crate boundaries and diagram, CLAUDE.md crate layout,
  control-loop snippet, and impl table with a crate column. Also a core CHANGELOG and a
  **Breaking** sitl entry.
- After review:
  - `uom` is now a `[workspace.dependencies]` entry.
  - The tick-ordering contract lives in architecture.md, and the core README and
    `FlightOrchestrator` docs link to it.
  - The `FlightLoop` struct is unbounded; the bounds live on its `impl`.
  - sitl re-exports the core crate as `stable_flight_sitl::stable_flight`, so users don't
    get two copies. Items aren't re-exported.
  - Dropped an internal-only CHANGELOG line.
  - Review follow-ups:
    - 2c: `FlightLoop::into_parts()` (or accessors) so tests and runners can recover
      components, and the `testing` feature replacing the duplicated orchestrator and
      flight-loop mocks.
    - 2b: the `commanders/tests.rs` trait-only tests, and signal setup in
      `FlightOrchestrator::new`.
    - PR 4: `TailSitterMixer` logs `info!` per tick.
  - Skipped a debug check for `step()` before `prime()`: it's documented, and a
    side-effect-free `new()` matters more.

2b as implemented:
- `FlightOrchestrator` and the `orchestrator` module are removed (**Breaking**) and
  replaced by sitl `ShutdownSignal` (`shutdown.rs`) and `RealTimeRunner` (`runner.rs`),
  re-exported at the sitl root.
- `ShutdownSignal`:
  - `new()` is unconnected; `install()` wires SIGINT and returns `io::Result`. `trigger()`
    and `is_set()` are the rest of the API, and clones share the flag.
  - Unix uses `signal_hook::flag::register`, with no thread.
  - Windows keeps `SetConsoleCtrlHandler` (Ctrl-C and Ctrl-Break), since signal-hook's
    Windows docs say not every Ctrl-C becomes SIGINT. A `OnceLock` replaces the
    `static mut` pointer, and every install shares one process-wide flag.
  - The Windows path was compile- and clippy-checked with
    `--target x86_64-pc-windows-msvc`, but not run.
- `RealTimeRunner`:
  - `new(hz)` returns `Result<_, InvalidFrequency>` instead of asserting in `run`
    (CLAUDE.md prefers `Result` for new fallible APIs), and also rejects NaN/∞.
  - `run(&mut FlightLoop, &ShutdownSignal)` primes, checks the signal before each tick,
    sleeps to accumulated deadlines, and returns `RunStats { ticks, elapsed }`.
  - It still primes inside `run` so callers can't forget.
- Core `commanders::WaypointFollower` plus `EmptyWaypoints` and `DEFAULT_ACCEPTANCE_RADIUS`.
  `ScriptCommander` is now CSV loading plus a `WaypointFollower<Vec<_>>`, with the same API.
  Following tests moved to core (table-style, including array, `&'static`, and `Vec`
  storage); sitl keeps parse/IO tests and one delegation test.
- Deleted sitl `commanders/tests.rs`: 5 of its 7 tests only exercised a mock defined in the
  test file. The other 2 also checked the `AttitudeCommand`/`WaypointCommand` defaults; after
  review, `AttitudeCommand` and `RateCommand` derive `Default` (same all-zero values), so no
  hand-written default is left untested.
- Runner tests cover invalid frequencies, the period, an already-set signal (primes, zero
  ticks), and stopping on a trigger with the nominal `dt`. The orchestrator's `MockVehicle`
  duplicate went with the orchestrator tests; `NullVehicle` still moves to core `testing`
  in 2c.
- Verified Unix Ctrl-C end to end with a throwaway example: SIGINT → clean exit and log
  lines, at 249.9 Hz for a 250 Hz target.
- After review:
  - `RealTimeRunner::new` uses `Duration::try_from_secs_f64` and also rejects rates whose
    period overflows (1e-30 Hz used to panic) or rounds to zero (1e10 Hz used to spin with
    `dt = 0`).
  - A second Ctrl-C terminates the process: `register_conditional_shutdown` on Unix, and
    falling through to the default handler on Windows.
  - Windows install failures cache and report the original OS error.
  - `is_set`/`trigger` use Acquire/Release.
  - `ticks.is_multiple_of`.
  - `ScriptCommander::follower()` accessor and `From<EmptyWaypoints> for ScriptError`.
  - Follower tests derive distances from `DEFAULT_ACCEPTANCE_RADIUS`.
  - Docs de-duplicated: architecture.md owns the timing rules, the sitl README owns the run
    snippet, and the core README documents `WaypointFollower`.
  - Review follow-ups:
    - Own PR or with the lockstep runner: resync the schedule after a stall instead of
      replaying missed ticks back-to-back, and count overruns in `RunStats`. This isn't a
      regression; the orchestrator did the same.
    - Later: a `Runner` trait once a second runner exists; a primed-state guard if more
      runners need priming; runtime path updates for `WaypointFollower` (`AsMut` or
      `set_waypoints`) once a mission uploader exists.
    - 2c: replace the new `runner/tests.rs` mocks with the `testing` feature.
- **Found while verifying (fixed in its own PR, see Status):** `AttitudeController::control`
  (`stabilizers/controllers/attitude.rs:118-127`) `assert!`s that desired roll and pitch
  are in [-1, 1]. In waypoint mode with the vehicle far from the target (e.g. a
  `NullVehicle` at the origin and `flights/figure_eight.csv`), the lateral controller asks
  for pitch ≈ -4.6, and the flight loop panics. That contradicts "no panics in the loop".
  Clamp (or saturate the lateral output) instead of asserting; do it in its own PR with
  table tests, before hardware.
  - As fixed:
    - Root cause: `LateralController::control` returned an unbounded `accel / g` as a
      "normalized" angle and never applied its configured `max_integral`.
    - Both controllers now saturate through `stabilizers::saturate_unit` (NaN → 0): the
      lateral output at the source, and the attitude input defensively.
    - The lateral integral is clamped.
    - Table tests for both controllers, plus a `CopterStabilizer` regression test that
      reproduces the original panic without the fix and checks the tilt points toward the
      target.
    - After review:
      - Finite-error guards keep a NaN input from sticking in the lateral and attitude
        integrals; NaN survives `clamp`.
      - `LateralController::with_config` validates `max_integral`.
      - `saturate_unit` is `pub(crate)`, with a `saturate_unit_vec2` form, and its doc
        states the producer/consumer contract.
    - Review follow-ups (own PRs), the same class of in-loop panic:
      - The attitude-mode heading target is never wrapped (`self.heading += yaw * …`), so
        sustained yaw input trips yaw.rs's ±2π assert. **Fixed in its own PR (see
        Status):**
        - The heading is wrapped each tick through a new `wrap_angle` (which `angle_diff`
          now uses).
        - The yaw controller's range assert is replaced by wrapping, and a non-finite
          target returns a zero rate without touching its state.
        - The 0.0001 step is named `HEADING_STEP_PER_TICK`.
        - A regression test runs 200k ticks of full yaw both ways; on `main` it fails with
          the original panic.
        - After review:
          - `wrap_angle` moved to `stabilizers/mod.rs` next to `saturate_unit`, so there's a
            single place for PR 4's `libm` swap and a neutral import for PR 7.
          - A NaN heading target re-latches from a valid estimate; before, heading hold was
            silently lost.
          - The yaw controller clears its previous error on a non-finite tick (no derivative
            kick).
          - Tests cover a heading primed above π matching its wrapped equivalent, the NaN
            re-latch, and the derivative kick; the new tests fail when their fix lines are
            removed.
          - The stale `%` `angle_diff` in the controllers README was corrected, and the
            `angle_diff` tests merged into a table.
        - Review follow-ups:
          - Own PR: attitude returns zero rates on non-finite input (like yaw), and document
            the NaN contract once (**done in the remaining-panics PR**); document the
            `VehicleState::heading` range (still open).
          - Own PR or refactor PR 7: make the heading slew rate depend on `dt`, not
            `HEADING_STEP_PER_TICK`.
          - Refactor PR 7: yaw state carries across mode switches, `self.heading` goes stale
            in waypoint mode, and guidance needs an absolute-heading path.
      - `process_attitude_command` asserts its inputs are in range, and the Spektrum
        commander's `scale_value` is unclamped.
      - `BodyRateController` panics on non-finite estimated rates.
      - Optional: a rate-limited log on non-finite controller input.
    - The figure-eight + `NullVehicle` run that crashed now runs at 250 Hz.
    - Still open (not in scope): `max_angles`-scaled `accel / g` is a simplified tilt
      model, and `LateralControllerConfig::max_rate` is unused. Revisit with refactor PR 7
      (guidance/inner-loop split).
- **Remaining per-tick panics, fixed together (see Status):**
  - `QuadCopterMixer::mix` range asserts. Rate-mode throttle was never clamped, and NaN
    moments fail every range check.
  - `process_attitude_command` range asserts, reachable through the Spektrum commander's
    unclamped channels.
  - `BodyRateController` non-finite asserts.
  - Fixes:
    - Saturation helpers (`saturate_unit`/`_vec2`, new `saturate_throttle`, and
      `wrap_angle`) moved to a crate-private core `math` module, so the mixer doesn't
      import from `stabilizers`. The module doc states the input policy once, and
      architecture.md "Input Policy" owns it for readers.
    - Attitude and body-rate return zero output on a non-finite error, with a private
      `resync_derivative` flag, so the first finite tick after a gap has no derivative
      kick. `BodyRateController::last_error` stays `pub`, so there's no API change.
    - Spektrum `scale_value` is clamped.
    - Panic-expecting tests are rewritten as "saturates like its in-range equivalent"
      tables. New stabilizer regression tests reproduce the original mixer/attitude and
      body-rate panics when run against `main`.
  - After review:
    - `AltitudeController` returns hover thrust on a non-finite error and keeps NaN out of
      its integral. Before, a NaN altitude estimate stuck there and, once the mixer
      saturated NaN thrust to 0, cut the motors for good.
    - Rate-mode NaN commands hold zero rate (per-axis `saturate_unit`; `clamp` passed NaN).
    - `LateralController` returns level on a non-finite error (∞ used to mean full tilt).
    - `AttitudeController`/`YawController::with_config` validate their clamp limits.
    - The `math` module doc now points at architecture.md, which owns the Input Policy
      and now states that NaN thrust → 0 is a last resort and that failsafes belong
      upstream. README and rustdoc paths fixed; CHANGELOG no longer overclaims.
    - New tests for each (altitude table, and stabilizer NaN-altitude and NaN-rate
      regressions) fail when their fix lines are removed.
    - Remaining review follow-ups moved to the Follow-up backlog.
  - **Found, not fixed (own PR, needs the transmitter):** `SpectrumCommander` maps
    roll/pitch/yaw to [0, 1] centred at 0.5, but `AttitudeCommand` expects [-1, 1] centred
    at 0, so centred sticks command a constant bank. It needs `2·x − 1` (and a check of
    each axis's sign) validated on the real transmitter.

2c as implemented:
- Core `composition` module:
  - `ComposedVehicle<E, A>` (delegating `StateEstimator`/`Actuator`, plus `estimator()`,
    `actuator()`, `into_parts()`; re-exported at the root).
  - `impl TelemetryPublisher for (A, B)`.
- `FlightLoop::into_parts()`, the #52 review follow-up.
- Core `testing` module behind `#[cfg(any(test, feature = "testing"))]`, with the
  `testing` feature off by default:
  - `NullVehicle` moved from sitl. It now counts estimates and actuations and keeps the
    last command; `new`/`with_state`. **Breaking** in sitl.
  - `ConstantCommander`.
  - `RecordingTelemetry<const N: usize>`: a fixed ring of `RecordedTick` (owned copies of
    `TelemetryRecord`, possible because #48 made the command/control types `Copy`), plus
    `total`/`last`/`iter`.
  - All allocation-free.
- sitl enables `testing` only in `[dev-dependencies]`. `cargo tree -e normal` confirms
  normal builds don't get it.
- sitl `runner/tests.rs` mocks are deleted. It uses the real `CopterStabilizer` +
  `QuadCopterMixer` with core doubles and reads them back via `into_parts()`.
- `flight_loop/tests.rs` keeps its shared-log `Recording*` doubles, because they test
  cross-component call order, which a per-component double can't show.
- New test code uses `assert2::assert!(let …)` instead of the deprecated `let_assert!`.
- Docs: core README Composition and Testing sections, architecture.md, and CLAUDE.md
  (impl table, and a testing rule to use core doubles rather than per-crate `Mock*`
  pipeline types).
- After review:
  - CI gained a `cargo build --workspace` step (default features), so library code that
    accidentally uses the dev-only `testing` feature fails CI.
  - `impl TelemetryPublisher for ()` is the no-op sink; it replaces the ad-hoc `Quiet` in the
    `FlightLoop` example.
  - `RecordedTick` defers its field docs to `TelemetryRecord` and gains `as_record()` for
    replaying.
  - `NullVehicle` counts are `u64`, matching `RunStats::ticks`.
  - `RecordingTelemetry` documents its inline size (~200 B per tick).
  - Rustdoc examples for `into_parts` and the fan-out, written without `testing` so they
    also run with the feature off.
  - `FlightLoop` tests rename the local `LogTelemetry`, and the `into_parts` test uses the
    real stabilizer and mixer.
  - Doc facts de-duplicated.
  - The `composition` module doc now says `ComposedVehicle` doesn't fit estimators that
    need the actuator's data; the sensor-synthesis bullet in "Later" is updated.
  - Review follow-ups:
    - First board: `estimator_mut`/`actuator_mut`.
    - Later: `Commander`/`Mixer` combinators against a real need. Keep publishers opt-in (no
      blanket `TelemetryPublisher` impl), so the tuple impl stays coherent.
    - Refactor PR 7: update `runner/tests.rs`, which uses the real `CopterStabilizer`.

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
  call talks to the sim, and how old the state from `StateEstimator::estimate_state` is
  relative to the last `Actuator::actuate`. #47 already documents RealFlight's caching and the
  `dt == Duration::ZERO` first-call convention in rustdoc, so this PR formalizes it as the
  cross-backend contract and adds the pinning test. Today `RealFlightVehicle::actuate` does the `exchange_data` round-trip and
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

As implemented (#PRNUM):
- Crate `stable-flight-sitl-realflight`: `vehicle.rs` (+ `vehicle/tests.rs`) is private, with
  `RealFlightVehicle`, `RealFlightConfiguration`, and `ActuatorMapping` at the root and
  `realflight_bridge` re-exported. Normal deps are core, `realflight-bridge`, `robomath`, and
  `log`. `robomath` is needed because `VehicleState` and the controller configs are built from
  its vectors. CI's `cargo tree` check covers every `stable-flight-sitl-*` crate.
- **Frame contract** writing surfaced two RealFlight bugs (Status row + backlog), not fixed
  here per the risk note. (1) Angles and rates are converted to radians twice. (2) Vertical
  velocity/acceleration are down-positive. Their conformance tests are `#[ignore]`d with
  the reason; `conforms_level` and the quaternion checks pass. Attitude-mode angles and all
  body rates are ~57× too small in flight today. Waypoint-mode lateral/attitude use the
  quaternion and are unaffected; the altitude D term has the wrong sign.
- **Gains not unified.** Both sets were tuned against those bugs, so picking one now is a
  tuning change with no fix behind it. `airframes::quadcopter_x()` carries `copter-manual`'s
  gains plus the mapping; `copter-auto` takes only the mapping and keeps the defaults.
  Unify in the frame-fix PR.
- **Conformance kit:** scenarios are `const Scenario`s (physical truth in degrees and
  north/east/up, so a backend builds its native fixture from them). `check` returns a
  `ConformanceError` listing every field off contract; the attitude quaternion is checked
  through its `roll()/pitch()/yaw()`. RealFlight has one `conforms_*` test per scenario, so
  known failures can be ignored one at a time.
- **Timing:** the mock bridge counts exchanges and stamps each state. The test pins one
  exchange to prime plus one per tick, and that tick *n* stabilizes on exchange *n*'s state.
- `ActuatorMapping` gained `Debug`/`Clone`/`PartialEq`.

## PR 4 — Make core `no_std`

Title: `Make stable-flight no_std`

- `#![cfg_attr(not(test), no_std)]` (same pattern as robomath). Tests stay on host.
- `std::time::Duration` → `core::time::Duration`. `std::f32::consts` → `core::f32::consts`.
- `f32::rem_euclid` (now only in `yaw.rs`'s `wrap_angle`) is std-only. Replace with a small `libm`-based helper and
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
  - Possibly a timestamp on `VehicleState`. `StateEstimator::estimate_state(dt)` takes the elapsed
    time as input, but a lockstep sim reports its own.
  - Promoting `ActuatorMapping` to a shared channel map.
  Design these against the real second backend, not speculatively.
- **Sensor-synthesis path.** Backends expose truth state today. Add IMU/GNSS/baro synthesis
  with a noise model from truth, so a real estimator (fusion-nav) runs in the SITL loop, the
  same as on hardware. `ComposedVehicle` doesn't fit, because its halves can't see each other
  while the truth comes from the actuator's simulator exchange. Likely a wrapper like
  `Sensed<V: Vehicle, S>` that post-processes a full truth vehicle's estimate through the
  sensor model and estimator (#54 review).
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
2. ✅ Refactor PRs 1–4 and 6 (cheap, and they shape the core API). Refactor PR 4 has to land
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
