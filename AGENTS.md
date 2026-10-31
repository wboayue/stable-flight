# AGENTS.md

Rules and commands for everyone (humans + AI assistants) working in this repo. Design rationale
lives in `architecture.md`; each crate's README owns its usage.

## Layout

- Cargo workspace (`crates/*`, `resolver = "3"`), Rust **edition 2024**, MSRV **1.89**
  (`rust-version`, set by nalgebra and uuid). Shared metadata and common dependency versions live
  in the root `[workspace.package]`/`[workspace.dependencies]`; crates inherit them with
  `.workspace = true`.
- `firmware/<board>` is excluded from the workspace: each board is its own nested workspace
  (own target, profiles, committed `Cargo.lock`). Run cargo from the board directory or use the
  `fw-*` recipes; root `fmt`/`clippy` don't reach it.
- Run every command from the repo root: examples and binaries resolve across the workspace, and
  `flights/*.csv` paths are root-relative.
- `docs/tuning/` holds the tuning guides, one per airframe type. A tuning change to an
  airframe updates its guide in the same PR. `docs/glossary.md` defines the control terms
  code and docs use.
- Untracked: `target/`, `tarpaulin-report.html`, `firmware/*/*.bin`, `sandbox/` and `tasks/` (local notes).
- `Cargo.lock` is committed and CI builds `--locked`: commit lockfile changes with the manifest
  change that causes them. A dependency that needs a newer compiler raises `rust-version` and the
  root README prerequisites in the same PR; CI's `msrv` job checks the workspace on that
  toolchain (locally: `cargo +<version> check --workspace --all-features --all-targets --locked`).

| Crate | Role | Key dependencies |
|---|---|---|
| `stable-flight` (core) | `no_std` flight control: pipeline traits and types (`lib.rs`), `FlightLoop`, `commanders`, `composition`, `controllers`, `estimation` (`wind`), `guidance`, `stabilizers`, `mixers`, `params` (+ `params::metadata`, feature `param-metadata`), `timing`, `tuning` (autotune), `view`, `testing` (feature) | `nalgebra` (`libm`), `libm`, `thiserror`, `log`; `serde` (feature) |
| `stable-flight-link` | `no_std` FC ⇄ companion protocol: messages, COBS frames, router, publisher, `ParamServer`, `FcLink` (wires them), `UdpLink` (`std`), `loopback` (`testing`) | core (`serde`), `postcard`, `heapless`, `cobs`, `crc` |
| `stable-flight-mavlink` | Pure builders from core values to MAVLink payloads, plus origin, home, land, and ground trackers; `testing::MockConnection` | core, `mavlink` (`emit-extensions`: struct literals need v2 extension fields) |
| `stable-flight-bridge` | Companion binary: native link ⇄ MAVLink v2 for ground stations (FC as sysid 1/compid 1); pure `Bridge` services, `Pump` for I/O, `http` for QGC param metadata | link (`std`), core (`param-metadata`), `stable-flight-mavlink`, `mavlink`, `crc`, `serde_json`, `clap` |
| `stable-flight-sitl` | Simulator-independent SITL: commanders (joystick, Spektrum, script), `RealTimeRunner`, `TickRate`, `ShutdownSignal`, `recorder` | core, `gilrs`, `signal-hook` |
| `stable-flight-sitl-realflight` | RealFlight backend: `RealFlightVehicle`, `ActuatorMapping`, `airframes`, `capture`, and the RealFlight examples | core, `realflight-bridge`; sitl and link as dev-dependencies |
| `stable-flight-sitl-xplane` | X-Plane backend: `XPlaneVehicle`, `LocalOrigin`, `ActuatorMapping`, `airframes`, `sweep`, `capture`, and the X-Plane examples | core, `xplane-bridge`; sitl as a dev-dependency |
| `stable-flight-sim` | Unpublished `no_std` deterministic simulator: copter, plane, tailsitter, and quadplane models (lockstep `StateSource` + `Actuator`), wind, perturbations, `airframes`; closed-loop tests in `tests/` (wiring in `tests/common`) | core, `libm`, `thiserror` |
| `stable-flight-tools` | Unpublished dev binaries: `plot_flight`, `generate_figure_eight`, `link_pilot`, `check_frames` | core, sitl, link, sitl-realflight, sitl-xplane, `plotters`, `clap` |

- Other crates use core's `nalgebra` re-export (`stable_flight::nalgebra`) and `Vec2`/`Vec3`
  aliases. sitl re-exports only the core crate (`stable_flight_sitl::stable_flight`); import core
  items from `stable_flight`.
- Hardware-reusable logic (control math, mixing, sequencing, sensor parsing, estimation) goes in
  core or another `no_std` crate, unit-tested on the host. Anything touching an OS, clock, file,
  or network goes in sitl; anything specific to one simulator goes in its
  `stable-flight-sitl-<simulator>` crate. Board crates only wire hardware to the pipeline.
- Outstanding work lives in GitHub issues. Labels: `needs-flight` (needs a simulator or
  transmitter check), `later` (waits for a real need), `bug`, and `pr:<name>` (issues that land
  as one PR; the label's description says what it covers). Milestones group issues by goal;
  put a shared issue in the earliest milestone it unblocks.
- Issues follow the task template (`.github/ISSUE_TEMPLATE/task.yml`): problem first, scope
  with what's out, and a **Done when** checklist of outcomes checkable by a command, test, or
  flight. The PR that ticks every box says `Closes #n`. Umbrella issues list their children
  plus the end-to-end outcome.

## Commands

```bash
just build | just test | just lint            # build; cargo test --workspace --all-features; clippy + rustdoc, -D warnings
cargo fmt --all
just embedded                                 # no_std check on thumbv7em-none-eabihf: core (plain, testing, serde, param-metadata), link (plain, testing), sim; then fw-build
just fw-build | fw-size | fw-bin | fw-flash   # ARK FPV firmware: release build, flash/RAM size, raw binary, USB DFU flash
just cover                                    # tarpaulin → tarpaulin-report.html (.tarpaulin.toml)
just bridge                                   # native link ⇄ MAVLink for QGC (--link-bind/--link-peer/--gcs/--origin/--http-bind/--http-url/--offboard-speed/--vehicle-type)
just fly-manual | fly-auto | fly-autotune      # RealFlight QuadCopterX: joystick, waypoint script, autotune (--airframe titan-cobra)
just fly-companion | link-pilot                # copter flown from the link; link_pilot plays the companion (or just bridge + QGC)
just fly-tailsitter | fly-tailsitter-auto | fly-tailsitter-companion  # RealFlight eXplora: joystick, script, link
just fly-quadplane-auto                        # RealFlight Titan Cobra: script
just fly-xplane | fly-xplane-auto | fly-xplane-companion | fly-xplane-autotune  # X-Plane Alia 250: joystick, script, link, hover autotune
just capture | capture-xplane | xplane-sweep   # raw state → CSV (RealFlight, X-Plane); X-Plane engine sweep
just plot <csv> <out> | just gen-figure-eight # plot_flight (a waypoint script; --record: a recorder CSV as time series), generate_figure_eight
just tag <crate> <version> | just versions    # push a <crate>-v<version> tag; list tags
just todo-check                               # TODO(#n) markers whose issue is closed (needs gh)
cargo test -p stable-flight-sim               # closed-loop flights on the in-repo simulator (crates/stable-flight-sim/tests)
cargo run -p stable-flight-sitl-realflight --example channel-sweep-realflight -- --motors 0,1 --servos 2,3  # drive raw channels, record a capture
cargo run -p stable-flight-sitl-realflight --example exchange-latency-realflight                           # time RealFlight exchanges alone
cargo run -p stable-flight-sitl-realflight --example surface-probe-realflight                             # quadplane surface directions in flight (Cobra)
cargo run --bin check_frames -- capture.csv   # capture vs the frame contract (--pitch-deg: body rotation, 90 for the eXplora; --sim xplane for X-Plane captures)
cargo run --example extreme_3d_commander      # joystick smoke test (spektrum_commander: Spektrum mapping)
cargo run --example joystick_events           # every gilrs event as CSV, gamepad codes (--raw: without gilrs's filters)
```

Add `-p <crate>` to scope any command. The flight pipeline has no CLI and there is no `cargo
bench` target. `just --list` shows each recipe's flags; flight scripts are `flights/*.csv`, each
saying in its header what it flies; the backend READMEs give each example's usage.

## Architecture

`FlightLoop` (core) runs one tick of a pipeline of traits; sitl's `RealTimeRunner` paces it.
Traits and their contracts are in core `src/lib.rs` rustdoc: read it before implementing one.

- `Commander` → `Command { arm, regime, input }`. `arm` gates the motors; `regime` requests a
  `FlightRegime` (guidance owns the flown `FlightMode`); `input` is a `ControlInput`:
  `Attitude`, `Rate`, `Waypoint`, `AbsoluteAttitude`, `Land`, or `AbsoluteRate`.
- `Vehicle` = `StateSource` + `Actuator` (blanket-implemented: implement those two, never
  `Vehicle`). `VehicleState` rustdoc holds the frame contract; `Vehicle` rustdoc the timing
  contract.
- `Guidance` → `Setpoint` (copter: `Attitude`, `Rate`, `BodyRate`; plane: `Plane`,
  `PlaneRate`; the variant picks the controller); `Stabilizer` → `ControlOutput` (thrust,
  moments); `Mixer` → `ActuatorCommand` (`motors: [f32; 8]` in [0, 1], `servos: [f32; 8]` in
  [-1, 1]; the mixer sets each slot's meaning, the backend mapping its channel).
- `TelemetryPublisher` gets a `TelemetryRecord` after `actuate`: the tick's `dt`, command,
  setpoint, state, control, actuators, merged `Saturation`, failsafe state, and flight mode.
- Tick order, `prime` (call once before the first `step`), and the arm gate:
  `architecture.md#flight-stabilization-loop`. Timing: `architecture.md#real-time-loop-management`.
- Components own one config each and implement `Configurable` (validated `set_config`), reached
  between ticks through `FlightLoop::set_guidance_config`/`set_stabilizer_config`
  (`architecture.md#config-ownership`). `CopterStabilizer` hosts autotune probes
  (`tuning::Tunable`), which `FlightLoop` forwards; `tuning::autotune::Autotune` runs them
  (`architecture.md#autotune`).
- Trait methods don't return `Result`: invalid configuration panics at construction, and
  per-tick failures aren't modeled. A new fallible operation gets a `thiserror` type.

| Trait | Implementations | Crate |
|---|---|---|
| `Commander` | `WaypointFollower`, `FailsafeCommander<P: LinkedCommander, F: Fallback>`, `HoldCommander` (a `Fallback`) | core |
| `Commander` | `Extreme3DCommander`, `SpektrumCommander`, `ScriptCommander` | sitl |
| `Commander` | `LinkCommander` (a `LinkedCommander`, from `FcLink::wire_linked`) | link |
| `StateSource` + `Actuator` | `ComposedVehicle<S, A>`; `testing::NullVehicle` | core |
| `StateSource` + `Actuator` | `RealFlightVehicle<T: RealFlightBridge>` | sitl-realflight |
| `StateSource` + `Actuator` | `XPlaneVehicle<B: DatarefSource + DatarefSink>` | sitl-xplane |
| `Guidance` | `CopterGuidance` (altitude/lateral loops, heading hold, `ThrustLimits`); `PlaneGuidance` (FBWA sticks, acro, offboard); `TailsitterGuidance` (hover + plane under a `Transition`); `QuadPlaneGuidance` (hover + plane under a `LiftTransition`, blended through it); `View<G>` | core |
| `Stabilizer` | `CopterStabilizer` (attitude, yaw, body-rate loops); `PlaneStabilizer` (attitude, turn coordination, airspeed-scaled rate loop); `TailsitterStabilizer` (each setpoint on its side); `QuadPlaneStabilizer` (lift rotors and cruise side, both in a `Setpoint::Blend`); `View<S>` | core |
| `Mixer` | `TableMixer<N>` (presets `quad_x`, `hex_x`, `tri`, `tailsitter_2p`, …) | core |
| `TelemetryPublisher` | `(A, B)` fan-out; `testing::RecordingTelemetry<N>` | core |
| `TelemetryPublisher` | `LinkPublisher` (from `FcLink`; MAVLink is the bridge's job) | link |

- `View<T>` runs any guidance or stabilizer in a constant rotated body frame (state in, moments
  back out); `View::tailsitter` flies nose-up hover on the copter controllers.
- RealFlight examples are standalone (each inlines its pipeline wiring; duplication by design).
  Gains, mixer, and channel mapping come from `airframes`; link plumbing from `FcLink`.
  `copter-manual-realflight` is the reference wiring for a new component.

## Coding style

- Match surrounding code first; the rules below resolve ties. `cargo fmt --all`.
- snake_case files/modules, PascalCase types, SCREAMING_SNAKE_CASE constants. Name a magic number
  when it repeats or carries units.
- Prefer early returns and small helpers; keep flight math readable, not arbitrarily short.
- Core is `no_std`: `core::`, not `std::` (tests may use std); `f32` math core lacks (`powf`,
  `sin`, `sqrt`, …) goes through `libm`.
- New fallible APIs return `Result` with `thiserror` enums. No panics in library code outside
  documented preconditions.
- **No range or finiteness `assert!`s on per-tick inputs:** saturate, treat NaN as neutral, wrap
  angles (`architecture.md#input-policy-no-panics-in-the-tick`, core `math` helpers). Asserts are
  for construction-time config and `dt > 0` only.
- **No `.unwrap()`/`.expect()` in `src/`.** Propagate with `?` or a typed error; only for a value
  provably present from local context, `unwrap_or_else(|| unreachable!("..."))` with a comment
  on the invariant. `.expect()` is fine in `examples/` and `tests/`.
- **Units in names:** a bare `f32`/`Vec3` physical quantity carries a suffix (`_m`, `_mps`,
  `_mps2`, `_rad`, `_radps`, `_deg`, `_degps`, `_s`, `_hz`, `_n`). SI internally; convert at
  external boundaries. Exempt: normalized values (throttle, actuator commands, gains),
  `max_integral` limits (each field documents its unit), and `dt` (always seconds). No
  unit-typed wrappers.
- Vector math with `nalgebra` (`Vec2`/`Vec3`, `UnitQuaternion<f32>`); `*` on two vectors isn't
  element-wise (use `component_mul`). Euler angles via `stable_flight::euler_angles` (NaN-safe,
  clamped at ±90°), angle wrapping via `stable_flight::wrap_angle`.
- Firmware size: fixed-size math only (no dynamic matrices or decompositions), no
  `Display`/`Debug` formatting on target; measure with `just fw-size` rather than guess (`libm`
  trig dominates). CI's `embedded` job fails past `FLASH_BUDGET_BYTES` (`test.yaml`); a PR
  that grows the firmware on purpose raises it.
- **Comments and docs state the current code**, not how it got there:
  - Rustdoc is the contract: behavior, units, invariants.
  - Inline comments give the non-obvious *why* in present tense: physics, invariants, a
    measured fact.
  - Cite where evidence or a spec lives, one trailing reference: `(#n)` for flight data behind a
    tuned value. The comment states the claim and the reference backs it ("reads the 10 ms lag as
    30 ms (#551)", never a bare `(#n)`); rustdoc reads complete without it, since docs.rs readers
    may never follow it. Cite the issue holding the measurements, not the PR that landed them (the
    PR only when it alone holds them); a fact other code or docs lean on goes in `docs/findings/`
    and is cited by path. For ArduPilot or PX4, the module and method or parameter (ArduPilot's
    `AP_TECS` and `Q_WP_SPD`, PX4's `VT_B_DEC_MSS`), never a line number: upstream lines drift,
    names last. No narrative ("raised from", "previously", "after #n", "interim").
  - Open work is `TODO(#n)` in code comments (`just todo-check` flags ones whose issue
    closed); markdown links the issue.
  - `architecture.md` gives rationale, including briefly why rejected alternatives were
    rejected. A regression test may name the issue it guards; its name states the behavior.
  - History lives in commits, PR bodies, and CHANGELOGs.
- Logging via `log`: `info!` for lifecycle events, `debug!` inside the loop.
- Hot path (anything `FlightLoop::step` calls, and firmware): no allocation per tick, no blocking
  I/O.

### Readability

Write for a competent Rust developer who isn't a controls expert, reviewing the diff by hand.

- **Plain words before math.** A controller's or estimator's rustdoc says what it does
  physically and why, then the law: "pushes the nose down harder the further airspeed falls
  below trim", then the equation. Terms of art link the
  [glossary](docs/glossary.md). A PR that adds a term, changes what one means in code, or
  removes its last use updates the glossary in the same PR.
- **Names over symbols.** `roll_error_rad`, `pitch_rate_integral`, not `e_phi`, `i_q`. Textbook
  symbols only in an equation comment that maps them to the code's names.
- **Signs and frames at the point of use.** Where a value is computed or crosses a boundary,
  state its frame and positive direction (body or NED, nose up).
- **Name the intermediate steps.** Flight math is a sequence of `let`s with unit suffixes, one
  idea each; don't chain more than a few terms without naming the result.
- **Tuned values explain themselves.** Each gain or limit says what raising it does
  ("higher: tighter hold, oscillates past 0.3"), its unit, and its evidence `(#n)`.
- **Plain over clever** in new code: a function before a generic, a generic before a macro; a
  new trait only for a second impl or a test double.

### Locality

Related code sits together, so a reviewer reads one behavior in one place.

- **One concept, one module.** A component's config, defaults, `ParamTable` and metadata,
  constructor, tick method, and helpers live in its module. A change touching many files for one
  behavior suggests the behavior is split across modules that should be one.
- **Top-down order in a file:** public type and its rustdoc, config, `impl` (entry points
  first), then private helpers in call order, each below its first caller.
- **Constants next to their user.** A constant used once is defined beside that use; a shared one
  goes in the module that owns the concept, never a catch-all `constants.rs`.
- **Tests mirror the source:** in its order, with test helpers next to the tests using them.
- **Reviewable diffs:** moves, renames, and reformatting in their own commit, apart from behavior
  changes.

## Testing

- Unit tests sit next to the code in `mod tests;` files (e.g. core `src/flight_loop/tests.rs`,
  `src/mixers/table/tests.rs`; sitl-realflight `src/vehicle/tests.rs`). Simulator-dependent
  checks are examples, not tests; closed-loop checks that need no realism fly `stable-flight-sim`
  in its `tests/`.
- **Closed-loop behavior** (loops, guidance, commanders, failsafe, mixer limits) is tested on
  `stable-flight-sim`, wired through `tests/common` (`pipeline`, `hover`, `probe`, `autotune`,
  `fly`). Reproduce a suspected flight bug there before filing it, put the measured repro in the
  issue, and land the fix's regression test there. A behavior the sim shows but the fix waits on
  stays `#[ignore = "... (#n)"]`. `FlightLoop` exposes no telemetry mid-run: read per-tick
  values in `fly`'s callback, recorded ticks after `into_parts()`, or the link's downlink.
- **Sim test bands** come from a cited external target (ArduPilot, PX4: module and method or parameter) or a measured
  value with its margin stated beside the constant, never from the thresholds of the code under
  test. A band that only a tuning change can meet comes with that tuning, in `airframes`, from
  the sim autotune where it covers the loop.
- When a band or a tuning trades climbing against sinking, bias toward climbing: a climb costs
  height, a sink can end in the ground (#343).
- Name tests after the behavior they protect (`maintains_pitch_hold`), not the function.
- Table-driven tests are the default for control math, one row per scenario. Use `assert2`
  (`assert!`/`check!`) and `approx::assert_relative_eq!` for floats.
- Doubles: core `testing` feature (`NullVehicle`, `ConstantCommander`, `RecordingTelemetry`,
  `conformance`), recovered with `FlightLoop::into_parts()`; no per-crate `Mock*` pipeline types.
  The FC side of the link is `FcLink`, in tests and examples alike. A MAVLink connection is
  `stable_flight_mavlink::testing::MockConnection`. Mock other traits by hand (e.g.
  `MockRealFlightBridge`); `mockall` isn't a dependency.
- A change that keeps existing airframes flying as before while changing a unit or a scale
  (#448's pusher range) pins their old actuator commands in a test.
- A test that guards a fix must fail without it: revert or mutate the fix locally, and say so in
  the PR. Commit or `git add` the fix before mutating it: `git checkout -- <file>` restores the
  index, so it wipes an unstaged fix along with the mutation. A check whose threshold reads the
  mutated config can fail for the wrong reason (a sink limit of 0 turns "not sinking past the
  limit" into "not sinking"): name the failures that are the intended ones. A `check!` inside a
  loop over rows panics at the end of that iteration, so only the first failing row reports:
  confirm each row fails on its own (e.g. run it first).
- Keep time-based rows clear of `f32` rounding at thresholds (three 0.1 s ticks may not reach
  0.3 s: use a 0.35 s timeout).
- A test that raises a process-wide signal gets its own `tests/*.rs` binary, one signal each.
- `proptest` and `criterion` aren't dependencies; add one only with a test or benchmark using it.
- A new `ParamTable` gets a `ParamMetadata` table in the same file and a row in core
  `src/params/metadata/tests.rs`, which checks its ranges against `validate`.
- Coverage target: **90%+** overall (Coveralls on `main`, Linux CI; local macOS reads about a
  point higher). A PR doesn't lower it, and new library code is covered at 90%+. Examples and
  firmware are excluded (`.tarpaulin.toml`).

## Simulators and flight checks

- Each backend crate's README owns its setup and usage. A backend meets the frame and timing
  contracts and covers every conformance scenario with `stable_flight::conformance_tests!`;
  known gaps stay `#[ignore]`d with a reason linking an issue.
- PRs that change flight behavior, the link, or the bridge end with a RealFlight check.
- **Before any simulator run, or reading one's CSV or log, follow
  [`.claude/skills/flight-check`](.claude/skills/flight-check/SKILL.md):** scripted runs,
  tick-rate screening, comparable runs, stopping, and per-simulator setup. The measured facts
  behind it are in [`docs/findings/`](docs/findings/README.md).
- Confirm with whoever is at the simulator which model is loaded before every run; never
  assume it from the last run or a go.
- A setting that depends on a model gap listed in the `stable-flight-sim` README's **Limits**
  needs a RealFlight flight; the sim passing it isn't enough.

## Commits, PRs, and releases

- Small, imperative commit subjects, referencing issues as `(#id)`. Flight-behavior changes
  include simulator context (model, mode, what was flown).
- A PR body states the scenario tested and how to reproduce it (commands, configs), with
  telemetry or screenshots when behavior is visible. It becomes the squash commit message:
  summary first.
- PR bodies and issue comments name external software's behavior and file ("as ArduPilot's
  `AP_TECS` does"), not line numbers: those date as upstream moves. Code comments cite the
  same way (**Coding style**).
- Squash-merge a single-change PR (`<PR title> (#id)`). Rebase-merge a PR with follow-up commits
  (fixes found during implementation or review) to keep that history: each commit a
  self-contained step that builds and passes, subject referencing the issue. Merge commits only
  for long-lived branches.
- The PR that completes an issue says `Closes #n`; GitHub closes on that phrase anywhere in the
  body, so don't use it for issues the PR doesn't finish.
- Problems found during implementation or review get follow-up commits in the same PR, not new
  issues. File an issue instead when the fix significantly grows the PR's scope or needs a
  flight the PR didn't already need; an unfixed pre-existing bug gets a `bug` issue with repro
  steps.
- `main` is protected by the repository ruleset `main` (`gh api repos/wboayue/stable-flight/rulesets`):
  PRs only, no force-push or deletion, and the `test`, `lint`, `msrv`, and `embedded` jobs of the `test`
  workflow must pass. Rewriting published history needs the ruleset off briefly, on the owner's
  explicit go only.
- Merging, including the bottom of a PR stack (CI runs only on PRs based on `main`):
  [`.claude/skills/merge-pr`](.claude/skills/merge-pr/SKILL.md).
- Pass PR bodies to `gh pr create`/`edit` with `--body-file` (inline heredocs in `$(…)` break on
  apostrophes).
- **Docs follow the code in the same PR.** Each fact has one home; others link to it:
  - crate `README.md`: that crate's purpose, usage, and features;
  - root `README.md`: overview, **Status**, crate index, boards, simulators, quick start;
  - `AGENTS.md`: layout, commands, rules, checklist;
  - `CONTRIBUTING.md`: entry point for outside contributors, linking here;
  - `SECURITY.md`: supported versions and reporting;
  - `.github/` templates: what an issue or PR must include;
  - `architecture.md`: crate boundaries, dependency rules, and the rationale for the contracts;
  - `docs/glossary.md`: control and airframe terms, one definition each;
  - `docs/tuning/`: how to tune an airframe type (order, knobs and params, symptoms, a worked
    example); the airframe code holds the values and the flights behind them;
  - `docs/findings/`: measured simulator and flight facts, one per file, with their data;
  - `.claude/skills/`: procedures run on demand (flight checks, merging, releasing);
  - core rustdoc: the frame contract (`VehicleState`) and timing contract (`Vehicle`).
- **Status claims match what has flown.** When that changes (a new airframe, simulator, board,
  or published crate), update in the same PR: the root README **Status** and its tables, the
  board's firmware README, `SECURITY.md`'s scope line, and the repo description
  (`gh repo edit --description`). Plans say "planned" and link their milestone.
- **Changelogs:** each published crate has `CHANGELOG.md` ([Keep a
  Changelog](https://keepachangelog.com/en/1.1.0/); `stable-flight-tools` has none). Every
  user-facing change adds a bullet under `[Unreleased]` (`Added`/`Changed`/`Deprecated`/
  `Removed`/`Fixed`/`Security`, each heading once: add to an existing one), breaking changes
  marked **Breaking** (an API, CLI, or wire change users must act on) or **Breaking
  (behavior):** (unchanged code and
  params fly differently: a new default, filter, or control law); skip internal churn. A new
  crate's `[Unreleased]` is a feature summary until its first release, with no **Breaking**
  marks. Once a link release is out, any message shape change bumps `PROTOCOL_VERSION`.
- **New published crate:** inherits `[workspace.package]`, has its own `README.md` (absolute
  GitHub links; crates.io doesn't resolve relative ones) and `CHANGELOG.md`, and symlinks
  `LICENSE` (`ln -s ../../LICENSE`). Check with `cargo package -p <crate> --list`.
- **Releasing:** [`.claude/skills/release`](.claude/skills/release/SKILL.md).

## Workflow

1. Review every plan before implementing it, and include the review in the plan; review the
   finished PR the same way before it merges. Fixes go in the same PR as follow-up commits;
   deferred items get issues (**Commits, PRs, and releases**). The lenses:
   - **Duplication:** logic, constants, or helpers repeated in the change or already elsewhere
     (check core `math` and sibling crates first).
   - **Single responsibility:** one job per type or function; I/O, scheduling, and pure
     math/mapping kept apart.
   - **Composability:** dependencies injected (connections, addresses, clocks), configuration
     explicit, public surface no wider than callers need.
   - **Performance** in the hot path: per-tick allocation, blocking I/O, redundant trig or `libm`
     calls, flash size.
   - **Readability:** could a developer who isn't a controls expert follow the change from its
     rustdoc and names alone? Related code adjacent, terms in the glossary, signs and frames
     stated (**Readability**, **Locality** under **Coding style**).
2. Before designing around external software (QGC, MAVLink peers, simulator clients, other
   flight stacks), verify its behavior in its source, not from memory, and record where it
   lives (module and function or parameter, no line numbers) in the issue or PR. This repo is
   MIT: from GPL code (ArduPilot) cite and describe the behavior in your own words, then
   implement from that description; never copy or translate its code.
   Do the same by default with permissive code (PX4 BSD-3-Clause, QGC Apache-2.0).
3. Read the relevant trait in core `src/lib.rs` and its existing impls before adding one.
4. Add tests with the code, covering the boundary and error cases, not just the happy path.
5. Run `cargo fmt --all && just lint && cargo test --workspace --all-features` before a PR.
6. Compare against `main` in a throwaway worktree (`git worktree add <dir> main`, own
   `CARGO_TARGET_DIR`), not a `git stash` round-trip.
7. Before planning a fix to flight behavior, measure the candidates (settings and designs) on
   the sim with a throwaway `#[ignore]` sweep test in a scratch worktree, and put the table in
   the plan. Rejected designs go in the PR with their numbers.
8. Tune a new airframe's loops on a sim model sized from it (X-Plane's `.acf`, RealFlight's
   `.RFX`, or its measurements, plus probe flights), then confirm in RealFlight or X-Plane: trial
   there confounds coupled loops and ends in crashes (the Alia's hover from a model, #351; its
   plane side, #392). An `.RFX` is a zip; its `.rfvehicle` holds the mass, motors, props, and
   surfaces. The RealFlight models' RFX files are available (ArduPilot
   [SITL_Models](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models)
   for the Titan Cobra; the eXplora and QuadCopterX from the maintainer).

## Pre-PR checklist

- [ ] `cargo build --workspace` and `cargo test --workspace --all-features` pass.
- [ ] `just embedded` passes (core `no_std`, firmware links).
- [ ] `cargo fmt --all` clean; `just lint` passes (CI fails on any warning).
- [ ] New fallible APIs return `thiserror` types; no new panics, `.unwrap()`, or `.expect()` under `src/`.
- [ ] New control math (PID, quaternion, transforms) has table-driven tests.
- [ ] Public API has rustdoc with at least one example.
- [ ] Hot-path code allocates nothing per tick.
- [ ] Coverage stays at or moves toward 90%+.
- [ ] Owning docs, status claims, and affected changelogs reflect the change; comments state the current code.
- [ ] New or changed control terms are in `docs/glossary.md`.
