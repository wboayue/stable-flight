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

Each file in `tests/` flies the full pipeline on `airframes::quad_x()` unless noted (wiring
and the step-response spec in `tests/common`):

| Test | Asserts |
|---|---|
| `hover` | Altitude, position, and heading hold in calm air; on the waypoint in steady wind; return after a gust; a hold over the start completes at its altitude |
| `step_response` | Every loop (roll/pitch/yaw rate, roll/pitch angle, heading), both ways, against ArduPilot's and PX4's autotune targets: overshoot ≤ 5%, rate bounce-back ≤ 10%, rate rise ≤ 0.14 s (yaw 0.2 s), settled, clipping within the step plan's allowance. `heavy_quad` and `racer` meet the same targets on their tuned loops (roll/pitch rate and angle) |
| `waypoint` | A square route: each leg near its line and altitude at cruise speed; holds the last corner |
| `land` | Climb, land on a waypoint (one leg out, or over the start), disarm, rest on the ground |
| `failsafe` | Over the link (`FcLink` + loopback): silence hands over to `HoldCommander` within the timeout, which holds in place armed; the link takes back over after recovery |
| `saturation` | Each limit (tilt caps, altitude thrust, moments, mixer collective, mixer near idle) sets its `Saturation` flags, and a hover sets none |
| `autotune` | `StagePlan::copter_search` from low, a little high (×1.5), and far too high (×3, steps clip) rate gains tunes every stage to `quad_x`'s gains within 25% |
| `robustness` | `StagePlan::copter_search` on every airframe × perturbation set (clean, 10 and 25 ms delay, gyro noise with a 90 Hz tone, CG offset, weak motor, all combined) × start (core's default gains; ×3 the airframe's rate P and D): every stage `Tuned`, the result meets the `step_response` targets on that same plant, within a simulated-time budget. Clean rows and 10 ms delay from the low start pass; the rest stay ignored on what blocks them: gyro noise ([#373](https://github.com/wboayue/stable-flight/issues/373)), trim moments ([#374](https://github.com/wboayue/stable-flight/issues/374)), a high start under delay ([#375](https://github.com/wboayue/stable-flight/issues/375)), 25 ms delay ([#376](https://github.com/wboayue/stable-flight/issues/376)) |
| `determinism` | A windy, gusty route twice gives bit-identical states, clean and with every perturbation on (one seed) |

## Timing

Lockstep, no wall clock. `actuate` stores the motor commands; `state(dt)` integrates them over
`dt` in steps of at most `MultirotorModel::max_step_s` and returns the result, so tick *n* sees tick *n − 1*'s
command, plus `Perturbations::actuator_delay_s` when set. The priming call (`dt` zero) returns
the current state without stepping. State is exact (no estimator) apart from the gyro noise
below, and the same inputs and seed give the same flight bit for bit.

## Model

| Part | Model |
|---|---|
| Rigid body | NED world, FRD body; mass, diagonal inertia, gravity; Euler's equations with the gyroscopic term; semi-implicit Euler, steps of at most `max_step_s` (1 ms for `quad_x`) |
| Motors | First-order lag on each command; thrust `max_thrust_n · command²`; yaw drag torque ∝ thrust |
| Geometry | From the mixer's rows: a roll or pitch factor of 1 acts at `moment_arm_m`, a yaw factor of ±1 gives `yaw_torque_per_thrust_m`, so torques always match what the mixer expects. Motor-only mixers (servo rows are rejected) |
| Aerodynamics | Linear drag on the velocity through the air; linear rotational damping |
| Wind | Steady wind plus an optional "1 − cos" `Gust`, a function of simulated time only |
| Ground | Plane at altitude 0. Contact stops sinking; resting (thrust below weight) holds the body level, stops rotation, and applies Coulomb friction |

`Perturbations` (`SimVehicle::with_perturbations`, all off by default) bring the things that
most often break a tune on hardware. Each is a range to sweep, so a method that passes on them
isn't fitted to one clean plant:

| Perturbation | Model |
|---|---|
| `actuator_delay_s` | Motor commands applied this long after `actuate`, to the integration step; up to `MAX_ACTUATOR_DELAY_S` (50 ms) |
| `gyro` | Gaussian white noise at `rms_radps` per axis from a seeded generator, plus an optional `Vibration` tone, on the reported body rates only. Sampled at the tick, so a tone above half the loop rate aliases |
| `disturbance_moment_nm` | A constant FRD body moment (CG offset) |
| `motor_thrust_scale` | Each motor's thrust as a fraction of the model's (mismatch); its yaw torque follows |

Airframes span the plausible range of multirotors, so a method checked on all of them isn't
fitted to one. Each is an X quad whose guidance uses the core defaults, with the hover thrust
derived from the model; roll and pitch gains come from the sim autotune.

| Airframe | Mass | Motor to motor | Thrust-to-weight | Motor lag | Source |
|---|---|---|---|---|---|
| `quad_x()` | 1 kg | 250 mm | 2.4 | 30 ms | Plausible 5-inch with a camera; yaw and heading hand-tuned on the step responses |
| `heavy_quad()` | 3 kg | 350 mm | 2 | 50 ms | Mass and diagonal from ArduPilot SITL's default frame; yaw and heading `quad_x`'s |
| `racer()` | 0.8 kg | 250 mm | 8 | 15 ms | Mass and diagonal from ArduPilot SITL's freestyle model; yaw and heading `quad_x`'s |

`heavy_quad` and `racer` take ArduPilot's ring inertia (half the mass on the motor ring); their
thrust-to-weight and motor lag are chosen to span the range (sources in `airframes` rustdoc).

## Limits

Out of scope: blade-element props, ground effect, battery sag, estimator and filter models
(attitude, position, and velocity are exact; only the gyro is noisy), terrain, visuals,
real-time pacing, and multiple vehicles. Landing levels the body instantly
rather than modeling a tip-over. Servos (tilt rotors, control surfaces) aren't modeled.
