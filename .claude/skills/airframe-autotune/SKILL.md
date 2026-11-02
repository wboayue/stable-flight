---
name: airframe-autotune
description: Tune an airframe's loops with autotune in RealFlight or X-Plane and commit the gains in stable-flight. Use for a hover or plane-side autotune issue on an airframe (baseline, design runs, median gains, confirm, laps against main, docs), and when a design run fails on a simulator but passes on the sim.
---

# Airframe autotune

Getting from an airframe's hand tune to committed autotuned gains. How each flight is flown
and screened is [flight-check](../flight-check/SKILL.md); confirm the loaded model before every
run, and fly only on the go. The plane design and copter searches are in
[architecture.md](../../../architecture.md#autotune); past runs: #535 (eXplora), #536 (Titan
Cobra), #537 (X-Plane Alia), #540 (Alia hover).

## Wiring

- The example per simulator and side: `copter-autotune-<sim>`, `plane-autotune-<sim>`
  (`--airframe`, `--stages`, `--step-test`, `--param`, `--record`). A new airframe or
  simulator gets its arm there, following the existing ones.
- A plane tune script, `flights/<airframe>_plane_tune.csv`: a level leg at the trim airspeed
  into the hold, every plane leg over a safe area (water at Oceano), and room for the hold's
  return radius. Fly the hold at least 25% above the wing's slowest airspeed.
- The airframe's overrides (step size, overshoot, clip allowance) come from its
  `stable-flight-sim` autotune test, each constant citing it. Tune on the sim first
  (AGENTS.md **Workflow**); the simulator confirms.

## Runs

1. **Baseline:** `--stages none --step-test` on the hand tune. Its step tests are the table's
   left column.
2. **Design runs:** three runs with `--step-test --record`, reset before each. Screen each for
   tick rate before reading its gains. Note each run's fits (damping, stiffness) beside its
   gains: the spread says how far one run is from repeatable.
3. **Pick:** each gain's median over the runs. Say which run each axis's set comes from; when
   the medians mix runs within an axis, flag it. Decline a set with a bad step test (a rate loop
   overshooting past a third, never settling) even if it's the median.
4. **Commit** the gains to `airframes` with a comment: the fit that sets them, the step tests,
   the issue. Pin them in the airframe test (the X-Plane `alia250` test lists what's its own
   against the sim's).
5. **Confirm:** `--stages none --step-test` on the committed gains. Runs vary: report this
   run, not only the design run's own step tests.
6. **Laps:** the airframe's waypoint laps (`flights/<airframe>_*lap*.csv`) on the branch and
   on `main` (build `main` in a throwaway worktree with its own `CARGO_TARGET_DIR`). Compare
   peak sink in plane against `PGUID_SINK_MAX` + 0.25, the dip after the handoff into plane,
   tracking error, and touchdown distance. A lap limit an earlier issue set (#547's descent)
   must still hold.

A batch go covers the batch: baseline, three design runs, or the confirm run plus laps.

## Judging

- The issue's bar is usually "at least as good as the hand tune". Report each loop's
  overshoot / rise / settle / final, hand against tuned, and name each number that got worse;
  the maintainer decides whether it's accepted. Accepted costs go in the PR body and the
  tuning guide.
- When the trade is climbing against sinking, bias toward climbing (#343).

## When a design run fails

A failed stage ends the run; read the judge's pairs in the log (`RUST_LOG=info,stable_flight_sitl::autotune=debug`).

1. Find what the simulator's airframe does that the sim's doesn't, from a measured quantity in
   the recorder CSV. For example, the X-Plane Alia held 0.13–0.27 opposite aileron in a 20° bank
   (#537), and the Cobra rolled into its 30° banks (#635).
2. In a scratch worktree, scale the matching sim model coefficient until the sim shows the
   same quantity, and check that the sim design then fails the same way (`c_roll_r` ×2 for the Alia).
3. Sweep the fix on that sim in a throwaway `#[ignore]` test, and table the options for the
   plan. Land the scaled variant as the regression test, mutation-checked.
4. Rerun the design runs on the fix.

## Docs and close

- `docs/tuning/<type>.md`: a worked example with the script, the fits' spread, the gains,
  the hand-against-tuned step-test table, and the laps.
- The crate CHANGELOG: **Breaking (behavior):** for the gains; core's for any design fix.
- Post the design, step-test, and lap tables on the issue; the PR body links the comment.
- The PR closes the issue; tick it on its umbrella after merge.

The watchdog and lap metrics are session scripts until
[#645](https://github.com/wboayue/stable-flight/issues/645).
