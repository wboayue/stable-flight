# stable-flight-sim

A deterministic rigid-body multirotor, airplane, tailsitter, and quadplane simulator for closed-loop
tests of the Stable Flight pipeline in `cargo test` and CI. Unpublished, `no_std`, fixed-size math only.

The goal is plausible, repeatable dynamics, not fidelity:
[RealFlight](../stable-flight-sitl-realflight) covers realism. The rationale is in
[architecture.md](../../architecture.md#in-repo-simulator).

## Usage

`SimCopter`, `SimPlane`, `SimTailsitter`, and `SimQuadPlane` are `StateSource` + `Actuator`, so a `FlightLoop` flies them like
any backend. `airframes::quad_x()` pairs a model with the mixer and configs that fly it:

```rust
use std::time::Duration;
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::testing::{ConstantCommander, RecordingTelemetry};
use stable_flight::FlightLoop;
use stable_flight_sim::{SimCopter, Wind, airframes};

let airframe = airframes::quad_x();
let vehicle = SimCopter::new(airframe.model, &airframe.mixer)?.with_wind(Wind::default());
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

`airframes::plane()` does the same for `SimPlane` with `PlaneGuidance` and `PlaneStabilizer`,
plus a `Start` in level flight (`SimPlane::with_start`); it doesn't take off from the ground.
`airframes::tailsitter()` pairs `SimTailsitter` with `TailsitterGuidance` and
`TailsitterStabilizer`; it starts resting on its tail and takes off nose up.
`airframes::quadplane()` pairs `SimQuadPlane` with `QuadPlaneGuidance`, `QuadPlaneStabilizer`,
and `TableMixer::quadplane()`; it starts on its gear and takes off on its lift rotors, or in
cruise from its `start` (`SimQuadPlane::with_start`).

```bash
cargo test -p stable-flight-sim   # unit tests, conformance, and the closed-loop flights below (~30 s)
```

## Closed-loop tests

Each file in `tests/` flies the full pipeline on `airframes::quad_x()` unless noted (`plane_*`
on `airframes::plane()`, `tailsitter_*` on `airframes::tailsitter()`, `quadplane_*` on
`airframes::quadplane()`, `alia` on `airframes::alia250_hover()`); wiring and the copter and plane
step-response specs are in `tests/common`:

| Test | Asserts |
|---|---|
| `hover` | Altitude, position, and heading hold in calm air; on the waypoint in steady wind; return after a gust; a hold over the start completes at its altitude |
| `step_response` | Every loop (roll/pitch/yaw rate, roll/pitch angle, heading), both ways, against ArduPilot's and PX4's autotune targets: overshoot ≤ 5%, rate bounce-back ≤ 10%, rate rise ≤ 0.14 s (yaw 0.2 s), settled, clipping within the step plan's allowance. `heavy_quad` and `racer` meet the same targets on their tuned loops (roll/pitch rate and angle) |
| `waypoint` | A square route: each leg near its line and altitude at cruise speed; holds the last corner |
| `land` | Climb, land on a waypoint (one leg out, or over the start), disarm, rest on the ground |
| `failsafe` | Over the link (`FcLink` + loopback): silence hands over to `HoldCommander` within the timeout, which holds in place armed; the link takes back over after recovery |
| `saturation` | Each limit (tilt caps, altitude thrust, moments, mixer collective, mixer near idle) sets its `Saturation` flags, and a hover sets none |
| `autotune` | `StagePlan::copter_search` from low, a little high (×1.5), and far too high (×3, steps clip) rate gains tunes every stage to `quad_x`'s gains within 25% |
| `robustness` | `StagePlan::copter_search` on every airframe × perturbation set (clean, 10 and 25 ms delay, gyro noise with a 90 Hz tone, CG offset, weak motor, all combined) × start (core's default gains; ×3 the airframe's rate P and D): every stage `Tuned`, the result meets the `step_response` targets on that same plant, within a simulated-time budget. Clean rows (but `heavy_quad` from the high start) and 10 ms delay from the low start pass; the rest stay ignored on what blocks them: the seeded D ([#378](https://github.com/wboayue/stable-flight/issues/378)), gyro noise ([#373](https://github.com/wboayue/stable-flight/issues/373)), trim moments ([#374](https://github.com/wboayue/stable-flight/issues/374)), 25 ms delay ([#376](https://github.com/wboayue/stable-flight/issues/376)), a later loop ringing on its start gains ([#379](https://github.com/wboayue/stable-flight/issues/379)) |
| `plane_level` | The line north at 100 m and 15 m/s in calm air, a 5 m/s headwind, and a 5 m/s crosswind: altitude within ±1 m and airspeed within ±1 m/s (ArduPilot's autotest bands) for 30 s after settling, on the line |
| `plane_step` | Roll ±30° and pitch ±10° steps in absolute attitude mode from level flight: rise within PX4's 2.2 × the attitude time constant (`fw_att_control_params.c:56,71`), overshoot within the copter spec's 5%, settled in 4 s, surfaces unclipped past the copter angle steps' allowance |
| `plane_turn` | A loiter by guidance and a held 30° and 45° bank: sideslip within 2.5° (measured + margin); the loiter also within 5 m of its circle and ±1 m of its altitude (ArduPilot's band) |
| `plane_stall` | A zoom at zero throttle on a held pitch-up rate, wings level and rolling, until past the stall: every state finite, body rates bounded, flying again within 2 s of the entry's end, at most 60 m lost (wings level, the held pitch-up carries the tail slide over the back), off the ground (measured + margin) |
| `maneuver` | A `ManeuverRunner` from the line on `airframes::plane()` and the tailsitter's plane side: a 180°/s roll both ways turns a full turn before its timeout and loses at most the measured height (pure roll rate lets the lift tip over); a 10 m and 25 m loop spans its radius within 15%; a 1 rad/s roll and pitch doublet's first half peaks as a held step does at cruise thrust and settles by its pause's end; each recovers within 2 s and flies the line again (measured + margin) |
| `tailsitter_hover` | Takes off from its tail and holds a nose-up hover: within the copter's calm-air bands; near the waypoint in a steady 3 m/s wind on the belly or along the span; back after a 5 m/s gust either way (measured + margin) |
| `tailsitter_transition` | Against PX4's VTOL limits (`vtol_att_control_params.yaml`), calm air: from a hover at 30 m, forward to plane flight within 15 s (`VT_TRANS_TIMEOUT`), then the line held within `plane_level`'s bands; back to a hover ahead on the line within 10 s (`VT_B_TRANS_DUR`), then held within the copter's calm-air bands; a forward transition ended by a `Copter` request, the timeout (`TransitionTimeout`), or an 8 m/s span gust past the roll limit (`RollError`); `Plane` requested below `min_height_m` refused (`BelowMinAltitude`) until it climbs past. Each loses at most 20 m (`VT_QC_T_ALT_LOSS`, until 5 s after). The back transition climbs at most 10.5 m (measured 9.5 m), hands over not sinking past the sink guard's limit, and keeps thrust above zero until 5 s after the handover; ramps at thrust 0.3 and 0.5, which stall nose-high, still hand over within those limits on the sink guard ([#343](https://github.com/wboayue/stable-flight/issues/343)). A fast ramp that arrives climbing keeps the hover's integrals through the handoff ([#326](https://github.com/wboayue/stable-flight/issues/326)) |
| `quadplane_hover` | Takes off from its gear on the lift rotors and holds a hover within the copter's calm-air bands, the pusher stopped |
| `quadplane_transition` | Against PX4's VTOL limits, as `tailsitter_transition`, calm air: from a hover at 30 m, forward to plane flight within 15 s (measured 8.2 s), then the line held within `plane_level`'s bands; back to a hover where it stops within 10 s (measured 7.3 s), then held within the copter's calm-air bands, and so are hover points 120 m ahead of and behind it, flown to after the handover; a pusher too weak for the airspeed gate times out (`TransitionTimeout`); the link going silent mid-transition hands over to `HoldCommander`, back on the lift rotors, which holds where it engaged. Each loses at most 20 m (`VT_QC_T_ALT_LOSS`, until 5 s after) |
| `quadplane_waypoints` | Copter legs on the lift rotors from a hover at 30 m, each within PX4's 20 m altitude loss (`VT_QC_T_ALT_LOSS`) and then held within the copter's calm-air position and altitude bands: in calm air 100 m ahead, behind, and to the side, uncapped and at ArduPilot's 5 m/s `Q_WP_SPD`; in an 11 m/s headwind a takeoff and hold, and legs upwind and downwind at 5 m/s ([#427](https://github.com/wboayue/stable-flight/issues/427)) |
| `alia` | The Alia 250, a full-size multirotor, on the autotune plan scaled to it (`tests/common/large.rs`: 0.2 rad/s rate and 5° angle steps, a 30 s settle): every stage `Tuned` from rate P 1 and 3 to the airframe's gains within 25%; every loop's steps (yaw rate 0.2 rad/s, heading 29°) within the step spec's overshoot, bounce-back, settling, and clipping bands (PX4's rise time aside: the motors alone lag 0.17 s); the calm-air hover bands; back after `hover`'s gust (measured + margin) |
| `alia_plane` | `alia250()` in plane flight from 300 m at 45 m/s: level flight within ±5 m and 1.5 m/s of climb rate for 2 minutes, and a climb asked for at 40 m/s that the pitch limit can't make alone climbing at least 1 m/s on full thrust ([#454](https://github.com/wboayue/stable-flight/issues/454)); PX4's attitude step spec on 20° roll and 5° pitch steps (roll's overshoot measured, [#458](https://github.com/wboayue/stable-flight/issues/458)); ±30 m altitude steps and a turn entry within measured bands; entering plane flight from a forward transition within PX4's 20 m (`VT_QC_T_ALT_LOSS`), and within 30 m at 1.5× the pitch rate P and 2× the TECS damping ([#392](https://github.com/wboayue/stable-flight/issues/392)) |
| `determinism` | A windy, gusty route twice gives bit-identical states, clean and with every perturbation on (one seed); the airplane's line, the tailsitter's takeoff, transition to plane flight, and back, and the quadplane's takeoff to a hover through the same wind too |

## Timing

Lockstep, no wall clock. `actuate` stores the actuator commands; `state(dt)` integrates them
over `dt` in steps of at most the model's `max_step_s` and returns the result, so tick *n* sees tick *n − 1*'s
command, plus `Perturbations::actuator_delay_s` when set. The priming call (`dt` zero) returns
the current state without stepping. State is exact (no estimator) apart from the gyro noise
below, and the same inputs and seed give the same flight bit for bit.

## Model

Shared by every vehicle:

| Part | Model |
|---|---|
| Rigid body | NED world, FRD body; mass, diagonal inertia, gravity; Euler's equations with the gyroscopic term; semi-implicit Euler, steps of at most `max_step_s` (1 ms for every airframe) |
| Wind | Steady wind plus an optional "1 − cos" `Gust`, a function of simulated time only |
| Ground | Plane at altitude 0. Contact stops sinking; resting (thrust and lift below weight) holds the body level (a tailsitter nose up on its tail), heading kept, stops rotation, and applies Coulomb friction |

`SimCopter` (`MultirotorModel`):

| Part | Model |
|---|---|
| Motors | First-order lag on each command; thrust `max_thrust_n · command^thrust_exponent` (2 when the command sets prop speed, 2/3 when it sets motor power); yaw drag torque ∝ thrust |
| Geometry | From the mixer's rows: a roll or pitch factor of 1 acts at `moment_arm_m`, a yaw factor of ±1 gives `yaw_torque_per_thrust_m`, so torques always match what the mixer expects. Motor-only mixers (servo rows are rejected) |
| Aerodynamics | Linear drag on the velocity through the air; linear rotational damping |

`SimPlane` (`FixedWingModel`, `AeroCoefficients`):

| Part | Model |
|---|---|
| Aerodynamics | Stability derivatives (Beard and McLain, *Small Unmanned Aircraft*, ch. 4) on the air-relative velocity: lift linear in α, blended by a sigmoid at the stall into a flat plate's, so it falls off past it; parasitic plus induced drag, blended the same way into a flat plate's `2·sin²α`; side force from sideslip and rudder; roll, pitch (static stability on `sin α`), and yaw moments from α, β, the body rates (damping), and the surfaces. Bounded at any α, broadside and backward included |
| Surfaces | First-order lag on each servo; the mixer's servo rows say which of the aileron, elevator, and rudder axes each moves (an axis's deflection is the servos' least-squares fit to the row factors, so elevons work too), times `max_deflection_rad` |
| Prop | The motor rows, through the motor lag, thrust `max_thrust_n · command²` along body `x`; no torque, no falloff with airspeed |

`SimTailsitter` (`TailsitterModel`: a `FixedWingModel` plus the motors and their wash) is
`SimPlane`'s physics with:

| Part | Model |
|---|---|
| Motors | Each motor's thrust along body `x`; a mixer yaw factor of ±1 acts at `motor_arm_m` from the centerline (differential thrust); `max_thrust_n` per motor |
| Prop wash | `PropWash`: actuator-disc momentum theory along the thrust axis, `surface_fraction` of it over the surfaces only, so the elevons work in a hover. Nose first, the far wake's `thrust / disc area` of added dynamic pressure; sinking tail first, the static jet less the sink, so the elevons lose authority as it sinks (none at the jet's speed; no vortex-ring flow) |
| Frame | Airplane FRD body: hovering nose up is 90° of pitch, level in `View::tailsitter`; it rests on its tail |

`SimQuadPlane` (`QuadPlaneModel`: a `FixedWingModel` plus `LiftRotors`; the pusher is the mixer's one `MixRow::pusher` row) is
`SimPlane`'s physics with:

| Part | Model |
|---|---|
| Lift rotors | `SimCopter`'s motors along body `−z`: lag, `max_thrust_n · command²`, roll, pitch, and yaw torque from the mixer's motor rows (all but the pusher) |
| Pusher | The mixer's `MixRow::pusher` row (exactly one, else `ModelError::PusherCount`): `SimPlane`'s prop along body `x` (the `FixedWingModel`'s thrust and lag) |
| Lift-rotor drag | In-plane H-force: linear drag on the air's velocity in the body `x`–`y` plane, `drag_n_per_mps` at full lift thrust, scaled by the thrust, so none with the rotors stopped. Booms and stopped rotors are in the wing's `c_drag_0` |
| Frame | Airplane FRD body, which is also the hover's: it hovers level and rests level on its gear |

`Perturbations` (`with_perturbations` on any vehicle, all off by default) bring the things that
most often break a tune on hardware. Each is a range to sweep, so a method that passes on them
isn't fitted to one clean plant:

| Perturbation | Model |
|---|---|
| `actuator_delay_s` | Motor and servo commands applied this long after `actuate`, to the integration step; up to `MAX_ACTUATOR_DELAY_S` (50 ms) |
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
| `alia250_hover()` | 2,857 kg | 5.2 m × 6.1 m | 1.34 | 170 ms | X-Plane's Alia 250 (`.acf`, `engine-sweep-xplane`); yaw and heading hand-tuned on its yaw steps; not in `all()` (below) |

`heavy_quad` and `racer` take ArduPilot's ring inertia (half the mass on the motor ring); their
thrust-to-weight and motor lag are chosen to span the range (sources in `airframes` rustdoc).

`plane()` is ArduPilot SITL's default airplane (2 kg, 1.88 m span, 0.45 m² wing, its
coefficients and ±20° surfaces) on `TableMixer::plane()`, with the Aerosonde's inertia scaled
to it, ArduPilot SITL's default servo lag (0.14 s), and a prop of static thrust-to-weight 1.2.
It flies core's plane stabilizer and guidance defaults at a 15 m/s trim airspeed, except
for what the model gives: each rate loop's feedforward from its rate damping and the cruise
thrust from its level-flight drag; pitch rate I is tuned to the plane step spec (sources in
`airframes` rustdoc).

`quadplane()` is ArduPilot SITL's quadplane (`plane()`'s wing and aerodynamics with an X quad,
4.5 kg, lift rotors 350 mm apart across the diagonal) on `TableMixer::quadplane()`: lift rotors
on motors 0–3 in `quad_x` order, the pusher on motor 4, the ailerons, elevator, and rudder on
servos 0–2.
Inertia is `plane()`'s Aerosonde scaling at 4.5 kg; the lift rotors take `heavy_quad`'s
thrust-to-weight 2, lag, and yaw torque, the pusher `plane()`'s thrust-to-weight 1.2. It hovers
on core's copter configs with roll and pitch gains from the sim autotune, and flies plane on
`plane()`'s gains with the cruise thrust for its mass on the pusher. Transitions fly core's
defaults. Copter flight hands nose-down pitch past 3° to the pusher (`ForwardThrust`, gain 1.2,
as ArduPilot's `Q_FWD_THR_GAIN`), so the wing isn't flown at negative α.
`alia250_hover()` is the BETA Alia 250 as X-Plane 12 flies it in copter mode, 2,857 kg, sized from
its `.acf` and an `engine-sweep-xplane` run rather than from the real aircraft
([#351](https://github.com/wboayue/stable-flight/issues/351)): thrust going with
command^(2/3) (X-Plane's electric throttle sets power), 0.17 s motor lag, the effective roll
and pitch inertias the sweep's moment pulses measured, and `quad_x` with yaw reversed at
thrust factor 0.8 and yaw ratio 1. Its roll and pitch gains come from the sim autotune on a
plan scaled to it, its yaw and heading from its yaw steps, and its lateral loop is slowed for
its slow tilt; `stable-flight-sitl-xplane`'s `airframes::alia250()` flies these in X-Plane.

`alia250()` is the whole Alia 250, a `QuadPlaneAirframe`: `alia250_hover()`'s lift rotors and
gains, and a wing and pusher fitted to X-Plane's level flight, pulses, and full-power climb
([#392](https://github.com/wboayue/stable-flight/issues/392); the fit table is in its rustdoc).
Its pusher takes the model's thrust law for a power-set prop: linear in command, 21 N lost per
m/s of airspeed. It trims as X-Plane flies within 1.1° of angle of attack, 0.11 of elevator,
and 0.07 of pusher (41–48 m/s); its roll damping comes from steady rolls under held aileron.
Its plane side is tuned on this model ([#392](https://github.com/wboayue/stable-flight/issues/392)):
pitch rate I and P against the wing's strong pitch stability, rate feedforward from the model,
roll asking no more than its ailerons give, a 15° pitch-up limit below the stall, the
transition gate at 40 m/s where the wing holds level, and total energy control with the
model's full-thrust climb and least-thrust sink (1.8 and 3.5 m/s at 45 m/s).
`stable-flight-sitl-xplane`'s `airframes::alia250()` flies the same plane side. Its
lateral-directional terms are still borrowed ([#458](https://github.com/wboayue/stable-flight/issues/458)).

`tailsitter()` is ArduPilot SITL's tailsitter (`plane-tailsitter`: the default airplane with
1.5× its thrust) on two motors 25 cm either side of the centerline and
`TableMixer::tailsitter_2p()`, with two 10-inch props washing the elevons. The hover side flies
the eXplora's RealFlight gains; the plane side is `plane()`'s on the hover thrust curve,
without rudder feedforward (yaw is differential thrust). Its 30° hover tilt and plane pitch
limits sit below both handover angles, so the transition tests fly ramps past them
([#388](https://github.com/wboayue/stable-flight/issues/388)). Transitions fly core's
defaults but for the airspeed gate, the wing's 10 m/s minimum.

## Limits

The Alia leaves out X-Plane's nose-up pitch at level thrust (2.2 °/s², which X-Plane's rate
I trims), its tail's weathervaning in sideways flight, and its wing in hover; its drag is a
guess from its size.

Out of scope: blade-element props, ground effect, battery sag, estimator and filter models
(attitude, position, and velocity are exact; only the gyro is noisy), terrain, visuals,
real-time pacing, and multiple vehicles. Landing levels the body instantly
rather than modeling a tip-over. Multirotor servos (tilt rotors) aren't modeled.

The airplane leaves out flaps, accuracy at high α (past the stall it is only plausible),
prop torque and thrust falloff with airspeed, and takeoff: it starts in the air (`Start`),
and on the ground it rests rather than rolling.

The tailsitter leaves out the props' reaction torque under differential thrust (the mixer
doesn't compensate it either), the wash's lift, drag, and damping on the wing, each prop
washing only its own elevon, and ground effect. It keeps the airplane's sideslip
derivatives, tail included, so wind along the span in a hover (90° of sideslip) yaws it
harder than a flying wing: about 4 m/s of air along the span, from wind or from flying
sideways, outgrows its differential thrust
([#387](https://github.com/wboayue/stable-flight/issues/387)). Its hover tests stay on the
belly axis, and its transitions fly in calm air. Its elevons keep pitch authority at low
airspeed and low thrust that the RealFlight eXplora lacks: a back ramp at thrust 0.6 that the
sim flies cleanly wallows nose-high on the eXplora
([#399](https://github.com/wboayue/stable-flight/issues/399)).

The quadplane leaves out the lift rotors' thrust change in axial and edgewise flow, their
wash over the wing and tail, interference between rotor and wing, and the pusher's wash over
the tail. The air from above in a climb meets the wing and tail broadside (α −90°), so its
pitch moment noses the quadplane up as it climbs out; core's default rate gains don't hold it.
Without forward thrust, copter flight at speed fights the wing: nose down it sinks, and tail
first at 10 m/s it flips ([#427](https://github.com/wboayue/stable-flight/issues/427)).
