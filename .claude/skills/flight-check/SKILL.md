---
name: flight-check
description: Fly, screen, and compare RealFlight or X-Plane runs in stable-flight. Use before any simulator flight (a PR's flight check, a tuning or test flight, a capture) and when analyzing a recorder CSV or run log from one.
---

# Flight check

How to fly simulator runs that compare row against row, and how to read them. Simulator setup
is in each backend's README (`crates/stable-flight-sitl-realflight`, `crates/stable-flight-sitl-xplane`);
measured facts behind these rules are in [`docs/findings/`](../../../docs/findings/README.md).

Read the file for the simulator before a run:

- [realflight.md](realflight.md): fields, tick rate, stopping, disconnects.
- [xplane.md](xplane.md): reset, Oceano, plane legs, watchdog.
- [comparable-runs.md](comparable-runs.md): one knob per run, tick-rate screening, explaining a trace.

## Rules

- **Launch only on the go** of whoever is at the simulator: a run takes over their sim. A
  batch go covers back-to-back runs.
- **Scripted over manual.** Fly from a waypoint script with `--record` (the `*-auto-*`
  examples on `flights/*.csv`, `plane-probe-xplane`), started and stopped from the command line.
  When no script covers the check, add one to `flights/` first; each script's header comment says
  what it flies (`head -5 flights/*.csv`). A script flies the same route every run and needs no
  pilot at the sticks.
- **Scripts are start-relative:** (0,0) at height 0 is the spawn, so they fly at any field
  (#494).
- **A hold starts only once its waypoint is reached (within 3 m).** A waypoint the vehicle
  can't reach (pinned at the tilt cap, belly into the wind) stalls the script: in wind, climb
  and hold at headings that hold (eXplora: a wingtip into the wind,
  [finding](../../../docs/findings/realflight-wind.md)), and stop runs from a watchdog.
- **Joystick only for what needs one:** stick modes (FBWA, acro), the commanders themselves,
  and handling. Sticks confound VTOL flights: a stick's throttle flies a quadplane's pusher in
  plane and its lift rotors in copter, so a back transition hands full throttle to the lift
  (#392), and the D-pad doesn't hold a plane request under X-Plane (#446).
- **Sim model gaps:** a setting that depends on a gap in the `stable-flight-sim` README's
  **Limits** needs a RealFlight flight; the sim passing it isn't enough.

## A run

1. **Build release once** in the worktree under test:
   `cargo build --release -p stable-flight-sitl-<sim> --example <example>`.
2. **Launch the binary itself in the background**, one run per command, logging from the
   environment (without `RUST_LOG` the examples print nothing: no mode changes, no exit
   summary):
   `RUST_LOG=info ./target/release/examples/<example> <script> --end land --record <scratch>/run-N.csv > <scratch>/run-N.log 2>&1 &`.
   Don't chain `cd … &&` before the `&`: `$!` becomes a subshell and SIGTERM misses the example.
3. **Watchdog** on the recorder CSV: stop on landing (disarmed below 0.5 m after the climb
   target, or after a back transition), drift past a radius, a floor on plane legs, or a timeout.
4. **Stop with SIGTERM** (or Ctrl-C), never a kill: `ShutdownSignal` lets the vehicle hand
   control back. Then read the exit lines (`Ran N ticks at X Hz`, `ticks skipped`, the
   `host slow` total).
5. **Screen the tick rate** before reading anything else ([comparable-runs.md](comparable-runs.md#tick-rate)).
6. **Report** each run as a row against the earlier ones; ask the person at the simulator to
   confirm what only they can see (the sim view, QGC).

`--end land` keeps the example running on the ground: stop it before the next run.

## Link setups

- `just fly-companion` + `just bridge` + QGC, or `just fly-companion` + `just link-pilot`
  (instead of the bridge). Offboard MAVLink scripts use the bridge's UDP 14550 (close QGC
  first) and run with `uv run --no-project --with pymavlink python script.py`.
- **Start order:** bridge and example first, then QGC. If the vehicle disappears and returns
  under an open QGC, its virtual joystick can go silent while Arm still works: restart QGC.
- **Signals:** the companion example logs each failsafe handover (failsafe timeout 1 s). QGC's
  throttle and `SAT_*` displays: the bridge README's `VFR_HUD` and `NAMED_VALUE_INT` rows.
  QGC → bridge traffic: `nettop -p <bridge pid> -J bytes_in` (heartbeat only ≈ 21 B/s, joystick
  ≈ 600 B/s). `--record` on the tailsitter examples logs setpoints, rates, moments, and
  actuators per tick (columns: `stable_flight_sitl::recorder::TailsitterColumns`).
