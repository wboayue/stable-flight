# Architecture refactor plan

Series of PRs addressing the issues surfaced in the architecture review: dead
traits, misleading names, over-broad trait scopes, and one outright stale doc
claim (VTOL stabilizer). Ordered cheapest → riskiest so each PR is independently
reviewable, revertable, and stops the bleeding on the worst items first.

Each PR ships with updates to the docs that own the changed facts (see CLAUDE.md,
"Keep docs in sync").

Since the workspace migration (#43), source paths below (`src/…`, `examples/…`) are
relative to `crates/stable-flight-sitl/`. Line numbers are as of the original review.
Sequencing against the workspace migration lives in `plans/workspace-migration.md`.

**Status legend:** ✅ merged · 🔍 in review · ⬜ not started · ⏭ superseded. Update a
row when its PR opens (🔍 #n) and again when it merges (✅ #n). Do it in the PR itself
where possible; otherwise do it in the next PR (for example the rebase of the next
stacked branch).

## PR sequence at a glance

| # | Title                                         | Scope             | Risk   | Status |
|---|-----------------------------------------------|-------------------|--------|--------|
| 1 | Stop claiming the VTOL stabilizer exists      | Docs + 1 stub     | None   | ✅ #44 |
| 2 | Replace panicking telemetry writes            | Bug fix           | Low    | ✅ #50 (replaced auto-closed #45) |
| 3 | Rename leaky method names                     | Mechanical rename | Low    | 🔍 #46 |
| 4 | Resolve `Vehicle` vs `StateEstimator`/`Actuator` | Trait surface  | Low    | 🔍 #47 |
| 5 | Rename `simulators/` module to `vehicles/`    | Mechanical move   | Low    | ⏭ workspace PR 3 |
| 6 | Decouple `TelemetryPublisher` from `Command`  | Trait surface     | Medium | 🔍 #48 |
| 7 | Split `Stabilizer` into guidance + inner loop | Architecture      | High   | ⬜ |

PRs 3, 4 and 6 are stacked (#46 → #47 → #48). Merge them in order, using CLAUDE.md's
stacked-PR procedure: retarget the child to `main` before merging its parent.

---

## PR 1 — Stop claiming the VTOL stabilizer exists

**Problem.** `src/stabilizers/vtol.rs` is a one-line URL-comment stub. CLAUDE.md
and architecture.md both reference a "VTOL stabilizer" as if it were a real
component.

**Change.**
- Either delete `src/stabilizers/vtol.rs` entirely, or replace its contents
  with a `// TODO: tailsitter stabilizer — see ...` and a `#![allow(unused)]`
  if needed to compile.
- Drop "VTOL stabilizer in `stabilizers::vtol`" from CLAUDE.md's impl table.
- Drop the same line from architecture.md's polymorphism section.
- README is already silent on this — verify.

**Risk.** None. No callers.

**Decision.** Delete the stub vs. keep as TODO marker. Recommend delete — the
ArduPilot URL in the comment is preserved in git history and `TailsitterMixer`
already signals VTOL ambition.

**As implemented (#44).** Deleted. The crate README also claimed the module existed and
was fixed too. CHANGELOG: **Breaking** removal of the empty module.
After review: the "planned" note lives only in architecture.md, linking the eXplora VTOL
model. `TailSitterMixer` (correct casing) is labelled a placeholder in CLAUDE.md and
architecture.md, since it returns zeroed actuator commands.
**Review follow-ups (not in #44):** CLAUDE.md's impl table duplicates architecture.md's
impl list; keep one and link to it. `stabilizers::low_pass_filter_vec3` is public but
unused, so delete it or move it to a filters module.

---

## PR 2 — Replace panicking telemetry writes

**Problem.** CLAUDE.md's "no `.unwrap()`/`.expect()` in `src/`" rule catches
real bugs already in tree: every telemetry tick can crash the flight loop on
a transient I/O error.

Confirmed offenders:
- `src/telemetry/mavlink.rs:64` — `.send(...).expect("Failed to send message")`
  inside `send_telemetry`. UDP send fails → process dies mid-flight.
- `src/telemetry/file_logger.rs:37, 88, 93, 105` — `write_all` / `flush`
  calls each `.expect(...)` per tick. Disk full or fd closed → process dies.
- `src/telemetry/file_logger.rs:82` — `SystemTime::duration_since` with
  `.expect("System time error")`. Clock skew (NTP step) panics the loop.

**Change.**
- `TelemetryPublisher` is fire-and-forget today; it doesn't return `Result`.
  Two options:
  1. **Log and continue.** Replace each `.expect(...)` with a `match`/`if let
     Err` that emits `log::warn!` and returns. Zero API churn.
  2. **Surface errors.** Change the trait to return `Result`. Bigger blast
     radius, conflicts with PR 6's signature redesign.
- Recommend option 1 for this PR; defer "should telemetry return Result?" to
  PR 6, where the signature is already being touched.
- Add a rate-limited log guard so a persistent error (e.g. log file deleted)
  doesn't spam at 250 Hz. A simple "log once every N ticks" counter on the
  publisher struct is sufficient.

**Out of scope.** The `Default` impls in `commanders/extreme_3d.rs:348` and
`commanders/spektrum.rs:21` panic because `Default::default()` can't return
`Result`. That's a separate question — either remove the `Default` impls or
keep them as documented "construction may panic" sugar. Address with the
relevant component, not here.

**Risk.** Low. Pure error-path change. Existing tests stay green.

**As implemented (#50, originally #45).** Option 1. The crate-private
`telemetry::throttle::ErrorThrottle` is count-based. It reports the first failure of an
episode, then one per 250 consecutive failures with a count of those suppressed, and a
success ends the episode. That's roughly one line every 5 s for a dead MAVLink link, which
sends about 50 messages/s. The throttle only decides; each publisher logs, so the log
target is the publisher's module. `FileLogger` writes each frame through a single
`write_frame` helper. One throttle per publisher; table-tested. A pre-epoch clock records timestamp 0. Constructor
`.expect()`s (`MavlinkPublisher::new`, `FileLogger::new`) are still out of scope.
**Review follow-ups (not in #50):** a failed packet write after a successful timestamp
write leaves a truncated `.tlog` frame. `MavlinkPublisher` can't be tested against a
failing connection until its constructor stops hard-coding `connect(...).expect`.
**Found while here, not fixed:** `FileLogger::publish` writes 10 heartbeats with a 1 s
`sleep` between each on every tick, stalling the loop about 10 s per tick. It's unused
today; fix or remove it before anyone wires it in.

---

## PR 3 — Rename leaky method names

Three mechanical renames. One PR, fully `cargo fix`-able with find/replace.

| Trait              | Old                  | New              | Why |
|--------------------|----------------------|------------------|-----|
| `Commander`        | `next`               | `next_command`   | Return type isn't obvious from `next`. |
| `Stabilizer`       | `set_initial_state`  | `prime`          | The method primes integrators/filters, not "sets initial state." |
| `TelemetryPublisher` | `send_telemetry`   | `publish`        | `telemetry.send_telemetry(...)` stutters. |

**Change.**
- Update trait defs in `src/lib.rs`.
- Update all impls (`CopterStabilizer`, `Extreme3DCommander`, `SpektrumCommander`,
  `ScriptCommander`, `MavlinkPublisher`, `FileLogger`).
- Update orchestrator call sites in `src/orchestrator.rs`.
- Update examples in `examples/*.rs`.
- Update Rustdoc snippets and the architecture.md mermaid diagram.

**Risk.** Low. Public API breaks for any downstream user, but the crate is
pre-1.0 (`0.1.0`) and no published consumers are known. Bump to `0.2.0` and
note in the next release.

**Decision.** `prime` vs `reset` for `set_initial_state`. Recommend `prime` —
the method warms internal estimators from a known good state rather than
clearing them; `reset` would imply "back to zero."

**As implemented (#46).** `prime`. `prime` also got a rustdoc explaining what it does.
The version stays `0.1.0` until release; the CHANGELOG marks the renames **Breaking**.

---

## PR 4 — Resolve `Vehicle` vs `StateEstimator` / `Actuator`

**Problem.** `StateEstimator` and `Actuator` are declared in `src/lib.rs:564,585`
but nothing implements or consumes them. `Vehicle` carries both methods directly.
Dead traits are confusing — readers wonder which is the real interface.

**Two viable options.**

1. **Delete the dead traits.** Simplest. Lose the conceptual hint that sensing
   and actuation are separable. Fine if no near-term plan to split them.
2. **Make `Vehicle: StateEstimator + Actuator` via supertraits.** Keeps
   `Vehicle` as the orchestrator-facing trait but lets future hardware impls
   compose two drivers. `StateEstimator::estimate_state` currently takes
   `&self`; change to `&mut self` to match `Vehicle::estimate_state` (or vice
   versa — pick one consistent shape).

**Recommendation.** Option 2. The split is real on hardware (IMU thread vs
ESC output thread), and Stable Flight SITL exists explicitly to ease the
hardware transition. Cheap to do now, expensive to do once impls multiply.

**Change.**
- Reconcile `&self` vs `&mut self` on the two `estimate_state` signatures.
- Declare `pub trait Vehicle: StateEstimator + Actuator {}` with a blanket
  impl, or split `Vehicle`'s methods out and have it be a marker trait.
- Update `RealFlightVehicle` to impl both supertraits.
- No orchestrator change — `Vehicle` is still the bound it requires.

**Risk.** Low. One impl. Examples don't reference these traits directly.

**As implemented (#47).** Option 2: `pub trait Vehicle: StateEstimator + Actuator {}` with
a blanket impl, and `estimate_state` takes `&mut self`. `RealFlightVehicle`, `NullVehicle`,
and the orchestrator test mock implement the two halves. Callers on concrete types
must now import `StateEstimator`/`Actuator` (one doctest needed it). Adds a rustdoc
example on `Vehicle`.

---

## PR 5 — Rename `simulators/` to `vehicles/`

**Problem.** The module is named for the only current impl ("simulators")
rather than the role ("vehicles/platforms"). When hardware impls land they'll
live in the wrong place.

**Change.**
- `git mv src/simulators src/vehicles`
- Rename `src/lib.rs` `pub mod simulators;` → `pub mod vehicles;`
- Update all `use stable_flight_sitl::simulators::...` to `vehicles::...` in
  `examples/*.rs` and inline Rustdoc.
- Update CLAUDE.md "Public modules" list, simulator setup snippet path,
  architecture.md polymorphism list.
- Update README simulator setup snippet.

**Risk.** Low. Mechanical. Re-export `pub use crate::vehicles as simulators`
for one release if external consumers are a concern; otherwise just break.

**Decision.** `vehicles/` vs `platforms/`. Recommend `vehicles/` — matches
the trait name and the domain vocabulary in the rest of the crate.

**Note.** Can ship with PR 4, but separable. Keep separate for cleaner diffs.

**Superseded.** Workspace-migration PR 3 moves RealFlight into its own backend crate, and
the `simulators` module goes away. History also shows this module was already renamed
`vehicles` → `simulators` once (84693ea).

---

## PR 6 — Decouple `TelemetryPublisher` from `Command`

**Problem.** `publish(dt, command, state)` (post-PR 3) couples telemetry to
the `Command` enum. Every new command variant churns telemetry impls. Some
telemetry payloads care about control output, actuator commands, or loop
timing — none of which are reachable today without widening the signature
further.

**Change.**
- Introduce `pub struct TelemetryRecord<'a>` carrying `dt`, `command`, `state`,
  and ideally `control_output` and `actuator_command` references.
- Trait becomes `fn publish(&mut self, record: &TelemetryRecord);`.
- Orchestrator assembles the record once per tick after `actuate`.
- Update `MavlinkPublisher` and `FileLogger`.

**Risk.** Medium. Two impls churn, plus orchestrator. No flight-behavior
change.

**Decision.** Lifetimes on `TelemetryRecord` — borrow everything (`&'a`) or
copy small fields and borrow the heavy ones. Recommend borrow everything;
the record never outlives the tick.

**As implemented (#48).** It borrows everything except `dt` (`Duration` is `Copy`) and
derives `Copy`. Fields: `dt`, `command`, `state`, `control`, `actuators`.
`publish` stays fire-and-forget (no `Result`). The loop can't act on a telemetry
failure, and publishers handle their own errors since #50. The trait docs say so.

---

## PR 7 — Split `Stabilizer` into guidance + inner loop

**Problem.** `Stabilizer::stabilize` handles three command kinds (`Attitude`,
`Rate`, `Waypoint`) by matching at the top of `CopterStabilizer::stabilize`
(see `src/stabilizers/copter.rs:174`). That mixes guidance (`Waypoint` →
`AttitudeCommand`) with stabilization (`Attitude`/`Rate` → `ControlOutput`).
Conventionally these run at different rates (≈50 Hz vs 250 Hz). Today they're
collapsed.

**Change.**
- Define `pub trait Guidance { fn step(&mut self, dt: f32, command: &Command, state: &VehicleState) -> ManualCommand; }`
  where `ManualCommand` is `Attitude(...)` or `Rate(...)` only.
- Rename the existing `Stabilizer` trait to `InnerLoop` (or keep
  `Stabilizer` — the name now fits its actual job): takes a `ManualCommand`
  and produces `ControlOutput`.
- Optionally rename the umbrella concept the orchestrator depends on to
  `FlightController` and have it own a `Guidance` + a `Stabilizer`.
- Split `Command` into `MissionCommand` (Waypoint) and `ManualCommand`
  (Attitude/Rate). The orchestrator's `Commander` produces `Command`
  (an outer union); `Guidance` consumes it and emits `ManualCommand` for
  the inner loop.
- Decide guidance rate. Phase 1: run guidance every tick (same as today).
  Phase 2: introduce a divider (e.g., guidance every 5th tick at 50 Hz when
  the loop runs at 250 Hz). Don't conflate the two phases.
- Update `CopterStabilizer` to split into `CopterGuidance` + the existing
  inner-loop cascade.

**Risk.** High. Touches flight-control semantics. Gate on:
- All existing example flights produce identical telemetry pre/post
  (run both, diff `MavlinkPublisher` output).
- Unit tests in `stabilizers::copter::tests` pass without modification.
- New unit tests covering `CopterGuidance` in isolation.

**Decision.** Whether to do guidance rate division in this PR or defer to a
PR 7.5. Recommend defer — first ship the structural split with no behavior
change, then add the divider in a follow-up where the diff is easy to read.

---

## Out of scope (for now)

- Async / `Arc<Mutex<_>>` for cross-thread telemetry. Re-evaluate once
  hardware lands.
- `no_std` migration. Now scheduled as workspace-migration PR 4 (core crate only).
- Replacing PID with LQR/MPC. Independent of trait shape.
- Sensor fusion / Kalman filtering for `estimate_state`. Lives behind the
  `StateEstimator` trait surface (post-PR 4) and can be added without
  further refactor.
