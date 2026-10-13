# stable-flight-sim

A deterministic rigid-body multirotor simulator for closed-loop tests of the Stable Flight
pipeline in `cargo test` and CI. Unpublished, `no_std`, fixed-size math only.

The goal is plausible, repeatable dynamics, not fidelity:
[RealFlight](../stable-flight-sitl-realflight) covers realism. The rationale is in
[architecture.md](../../architecture.md#in-repo-simulator).

## Usage

`SimVehicle` is a `StateSource` + `Actuator`, so a `FlightLoop` flies it like any backend.
`airframes::quad_x()` pairs a model with the mixer and configs that fly it:

```rust
use std::time::Duration;
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::testing::{ConstantCommander, RecordingTelemetry};
use stable_flight::FlightLoop;
use stable_flight_sim::{SimVehicle, Wind, airframes};

let airframe = airframes::quad_x();
let vehicle = SimVehicle::new(airframe.model, &airframe.mixer)?.with_wind(Wind::default());
let mut flight_loop = FlightLoop::new(
    ConstantCommander::default(),
    vehicle,
    CopterGuidance::new(airframe.guidance),
    CopterStabilizer::new(airframe.stabilizer),
    airframe.mixer,
    RecordingTelemetry::<64>::new(),
);
flight_loop.prime();
flight_loop.step(Duration::from_millis(4));
```

```bash
cargo test -p stable-flight-sim   # unit tests, conformance, and the closed-loop flights below (~7 s)
```

## Closed-loop tests

Each file in `tests/` flies the full pipeline on `airframes::quad_x()` (wiring in
`tests/common`):

| Test | Asserts |
|---|---|
| `hover` | Altitude, position, and heading hold in calm air; on the waypoint in steady wind; return after a gust; a hold over the start completes at its altitude |
| `step_response` | Every loop (roll/pitch/yaw rate, roll/pitch angle, heading), both ways, against ArduPilot's and PX4's autotune targets: overshoot ≤ 5%, rate bounce-back ≤ 10%, rate rise ≤ 0.14 s (yaw 0.2 s), settled, clipping within the step plan's allowance |
| `waypoint` | A square route: each leg near its line and altitude at cruise speed; holds the last corner |
| `land` | Climb, land on a waypoint (one leg out, or over the start), disarm, rest on the ground |
| `failsafe` | Over the link (`FcLink` + loopback): silence hands over to `HoldCommander` within the timeout, which holds in place armed; the link takes back over after recovery |
| `saturation` | Each limit (tilt caps, altitude thrust, moments, mixer collective, mixer near idle) sets its `Saturation` flags, and a hover sets none |
| `autotune` | `StagePlan::copter_search` from low rate gains tunes every stage to `quad_x`'s gains within 25%; from high gains it doesn't yet (`#[ignore]`, #361) |
| `determinism` | A windy, gusty route twice gives bit-identical states |

## Timing

Lockstep, no wall clock. `actuate` stores the motor commands; `state(dt)` integrates them over
`dt` in steps of at most `MultirotorModel::max_step_s` and returns the result, so tick *n* sees tick *n − 1*'s
command. The priming call (`dt` zero) returns the current state without stepping. State is
perfect (no noise or estimator), and the same inputs give the same flight bit for bit.

## Model

| Part | Model |
|---|---|
| Rigid body | NED world, FRD body; mass, diagonal inertia, gravity; Euler's equations with the gyroscopic term; semi-implicit Euler, steps of at most `max_step_s` (1 ms for `quad_x`) |
| Motors | First-order lag on each command; thrust `max_thrust_n · command²`; yaw drag torque ∝ thrust |
| Geometry | From the mixer's rows: a roll or pitch factor of 1 acts at `moment_arm_m`, a yaw factor of ±1 gives `yaw_torque_per_thrust_m`, so torques always match what the mixer expects. Motor-only mixers (servo rows are rejected) |
| Aerodynamics | Linear drag on the velocity through the air; linear rotational damping |
| Wind | Steady wind plus an optional "1 − cos" `Gust`, a function of simulated time only |
| Ground | Plane at altitude 0. Contact stops sinking; resting (thrust below weight) holds the body level, stops rotation, and applies Coulomb friction |

`airframes::quad_x()`: 1 kg, 250 mm motor to motor, 6 N per motor (thrust-to-weight about
2.4), motor time constant 30 ms. Roll and pitch gains come from the sim autotune, yaw and
heading are hand-tuned on the sim, guidance uses the core defaults, and the hover thrust is
derived from the model.

## Limits

Out of scope: blade-element props, ground effect, battery sag, sensor noise and latency,
terrain, visuals, real-time pacing, and multiple vehicles. Landing levels the body instantly
rather than modeling a tip-over. Servos (tilt rotors, control surfaces) aren't modeled.
