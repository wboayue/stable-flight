# Architecture refactor plan

Series of PRs addressing the issues surfaced in the architecture review: dead
traits, misleading names, over-broad trait scopes, and one outright stale doc
claim (VTOL stabilizer). Ordered cheapest → riskiest so each PR is independently
reviewable, revertable, and stops the bleeding on the worst items first.

Each PR ships with synchronized updates to `README.md`, `CLAUDE.md`, and
`architecture.md` per the in-sync rule.

## PR sequence at a glance

| # | Title                                         | Scope             | Risk   |
|---|-----------------------------------------------|-------------------|--------|
| 1 | Stop claiming the VTOL stabilizer exists      | Docs + 1 stub     | None   |
| 2 | Replace panicking telemetry writes            | Bug fix           | Low    |
| 3 | Rename leaky method names                     | Mechanical rename | Low    |
| 4 | Resolve `Vehicle` vs `StateEstimator`/`Actuator` | Trait surface  | Low    |
| 5 | Rename `simulators/` module to `vehicles/`    | Mechanical move   | Low    |
| 6 | Decouple `TelemetryPublisher` from `Command`  | Trait surface     | Medium |
| 7 | Split `Stabilizer` into guidance + inner loop | Architecture      | High   |

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
- `no_std` migration. Re-evaluate when targeting the Matek H753.
- Replacing PID with LQR/MPC. Independent of trait shape.
- Sensor fusion / Kalman filtering for `estimate_state`. Lives behind the
  `StateEstimator` trait surface (post-PR 4) and can be added without
  further refactor.
