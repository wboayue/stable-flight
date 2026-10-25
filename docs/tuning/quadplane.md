# Quadplane tuning guide

Tunes a lift+cruise VTOL (quadplane) flown with `QuadPlaneGuidance` and
`QuadPlaneStabilizer`:
- **Hover:** the copter controllers, on the lift rotors.
- **Wing-borne flight:** the plane controllers, on the surfaces and pusher.
- **Between them:** a `LiftTransition`, both sides at once.

The reference airframes are the simulator's
[`airframes::quadplane()`](../../crates/stable-flight-sim/src/airframes/quadplane.rs), checked
in `crates/stable-flight-sim/tests/quadplane_transition.rs`, and RealFlight's Titan Cobra VTOL,
[`airframes::titan_cobra()`](../../crates/stable-flight-sitl-realflight/src/airframes.rs),
whose rustdoc holds its values and the flights behind them
([#433](https://github.com/wboayue/stable-flight/issues/433)).

## What makes a quadplane different

The vehicle stays level, so the two sides never fight over the same actuators. A transition
doesn't hand control from one controller to the other. Instead it trades the lift: the lift
rotors' thrust and moments are weighted by a lift share, and the wing flies in full.

- **Forward:** the lift rotors hold the hover while the pusher builds airspeed, then hand
  the lift to the wing over a fixed time.
- **Back:** the pusher is cut, and the lift rotors take the lift back while drag slows it.
- **Copter flight at speed:** tilted nose down, the wing at negative α pushes it down, and
  tail first it can flip ([#427](https://github.com/wboayue/stable-flight/issues/427)).
  Forward thrust hands the nose-down pitch to the pusher (below). Waypoint legs turn the nose
  along the leg first (`GUID_YAW_SLEW`, 60°/s; 0 holds the heading), so the pusher can help.
  Waypoint and offboard position flight slow while the nose is more than 20° off the
  waypoint, as ArduPilot's quadplane does: `OFFNOSE_SPD` (15 m/s) just past 20°, a third of it
  (5 m/s) from 160°, tail first. Uncut, the Cobra flipped at 11–16 m/s backward, pitching up
  past its lift rotors' authority
  ([#437](https://github.com/wboayue/stable-flight/issues/437)). Sticks and land flight
  aren't cut: fly sticks slow tail first.
- **Heading in wind:** the fin swings the nose into the wind, and the lift rotors' yaw can't
  hold it off: in the sim, 5 m/s from behind turned it around. When the heading stays more
  than `GUID_YAW_YLD_ERR` (30°) off its target without yawing toward it for `GUID_YAW_YLD_S`
  (1 s), waypoint flight yields the heading to the nose until the next waypoint. A crosswind
  takeoff then climbs as fast as into a headwind, within 1.4 s, instead of 8 s slower
  ([#431](https://github.com/wboayue/stable-flight/issues/431)).

## Order

1. **Hover.** Tune the lift rotors as a copter: the copter rate and angle loops (`STAB_*`),
   with `just fly-autotune` or by hand, then altitude and position (`GUID_*`). The surfaces
   stay neutral in a hover. A CG off the lift rotors' center needs a steady pitch moment:
   give the pitch rate loop enough I to hold it (I · max integral above the trim).
   Fast uncapped copter legs overshoot on the linear position loop, which starts braking
   only `2 · v` out: set `GUID_LAT_BRK` below the deceleration the airframe reaches nose up
   (fly an uncapped leg with `--record`, read the speed's fall after the stop starts), with
   margin for the tilt to come round.
   The hover wind estimate (`GUID_WND_*`) is off until its drag coefficients are fitted:
   the steps are the tailsitter guide's
   [wind estimate](tailsitter.md#wind-estimate), flown with the pusher off.
2. **Surface directions.** Before the first plane flight, check each surface pushes the way
   the mixer expects. On the ground with no airflow they move nothing; in copter flight at
   speed they do, while the lift rotors hold attitude against them
   (`surface-probe-realflight`). A reversed one is `reversed_servos` in the backend mapping;
   a surface whose neutral sits off center is a servo trim.
3. **Plane.** Tune the wing as an airplane (`PSTAB_*`, `PGUID_*`). Set `PGUID_CRUISE` to the
   pusher thrust that holds level flight at trim airspeed, as a share of the full pusher: the
   pusher spans its whole range, whatever the lift rotors' thrust factor
   ([#448](https://github.com/wboayue/stable-flight/issues/448)). Then the total energy
   control below.
4. **Transitions.** Tune these last, from a settled hover at least `TRAN_MIN_HGT` up. Set
   `TRAN_FW_MIN_HGT` below the plane legs, with room under it for a back transition's loss
   (1.9 m at a 3.5 m/s sink on the sim quadplane): plane flight below it falls back to a
   hover where copter flight takes over.

**Worked example (Alia 250 in X-Plane, hover only, #351):** a full-size aircraft, so the
small-copter defaults don't carry over; measure first, then tune on a sim model of it.
- **Measure:** `just xplane-sweep` on the runway: the yaw sign of each diagonal (reversed
  from `quad_x`: `TableMixer::with_reversed_yaw`), the liftoff throttle (0.648), the thrust
  curve from the step after liftoff (X-Plane's electric throttle sets power, thrust goes as
  its 2/3 power: `STAB_THR_EXPO` 1.5), the motor lag (0.17 s), and each axis's response to a
  moment pulse (the effective inertias).
- **Mixer headroom:** at thrust factor 0.9 a full moment moved each motor 0.1, too little to
  follow a step; 0.8 doubles it with the hover still under the ceiling. At the default yaw
  ratio 2.5, a full yaw clipped one diagonal and nearly stopped the other, and the mixer gave
  up collective: the Alia sank 53 m at full throttle. Yaw ratio 1 stops it, at the cost of
  holding heading at a few m/s sideways ([#440](https://github.com/wboayue/stable-flight/issues/440)).
- **Rate and angle:** the sim autotune on a model sized from the sweep, with the probes
  scaled to the aircraft (0.2 rad/s rate steps, 5° angle steps, a 30 s settle;
  `stable-flight-sim` `tests/common/large.rs`): rate P 1.52 roll, 2.57 pitch, no D; angle P
  1.03. Rate I 0.2 trims X-Plane's nose-up pitch at level thrust.
- **Yaw and heading:** the quad values limit-cycled at ±1 moment; yaw rate P 5 without I and
  heading P 0.8 step 29° with 2% overshoot.
- **Position:** core's lateral loop swayed into the tilt cap after a 5° step;
  `GUID_LAT_POS_P` 0.2 and `GUID_LAT_VEL_P` 0.4, `_I` 0.05 hold a hover through a gust and fly waypoint legs to within
  lat/lon resolution.

**Worked example (Alia 250 plane side, on the sim, #392):** the same route for the wing: fit a
sim model to X-Plane's level flight, pulses, roll holds (`plane-probe-xplane`), and full-power
climb, then tune on the model and pin the X-Plane airframe to it.
- **Pitch I against pitch stability:** a stable wing (`Cm_α` −1.48) pushes back on any pitch
  off trim, so a small rate I can't hold an attitude: px4xplane's 0.045 left trim 2° short and
  held a 5° step at 27%. TECS's integrators hid it in waypoint flight, as slow pitch behind
  turn dips and descent overshoot. `PSTAB_PIT_I` 0.5 and `PSTAB_PIT_P` 1 rise a 5° step in
  0.85 s; the rate feedforward comes from the model (`rate_feedforward`).
- **The gate against the wing:** with a pitch loop that tracks, a transition handed over below
  the speed the wing holds level at (37.5 against 40 m/s) let TECS pull it into a stall: 296 m
  lost. `TRAN_ASPD_MIN` at `PGUID_ASPD_MIN` (40 m/s) enters within 10–14 m; a higher gate waits
  longer on the lift rotors (drag-limited, more pusher doesn't help).
- **Pitch up below the stall:** at 24° (`PGUID_PIT_UP`, px4xplane's) a sinking entry held full
  elevator and fell into a stall cycle to the ground once TECS damping or pitch P rose 1.5×.
  15° holds with twice either.
- **Roll within the ailerons:** full aileron gives about 17°/s (`Cl_p` from the roll holds);
  asking 70°/s clipped them and overshot 15%. `PSTAB_RMAX_X` 15°/s and `PSTAB_TC_X` 0.6 s rise
  a 20° bank in 1.2 s.

**Worked example (Titan Cobra in RealFlight):**
- **Hover:** nose-heavy on its lift rotors, a level hover holds front motors at 0.60 and rear
  at 0.52, a 0.37 nose-up moment; pitch rate I 0.5 holds it. The autotune raised roll rate P
  (steps 0.85 → 0.95 of the step); pitch rate steps stop near 0.90 on any gains
  ([#438](https://github.com/wboayue/stable-flight/issues/438)).
- **Copter legs:** an uncapped 300 m leg (`flights/cobra_out_back.csv`) reaches 16.7 m/s
  and brakes at about 3 m/s² nose up; the linear loop overshot it 15–20 m.
  `GUID_LAT_BRK` 1.5 stops a sim model fitted to its coast and top speed within 1.9 m
  ([#488](https://github.com/wboayue/stable-flight/issues/488)). Judge such legs only on
  runs that hold 245 ticks/s while braking: at RealFlight's 120 the shaped legs
  (`GUID_LAT_ACC` 1) overshot 16–36 m, and at 250 stopped within 1.3 m.
- **Surfaces:** ArduPilot's parameter file reverses none; the probe confirmed each axis at
  12–16 m/s. Both tail panels trim +0.2 (`SERVO2_TRIM`, `SERVO4_TRIM` 1600).
- **Plane and transitions:** with ArduPilot's airspeeds, trim pitch, and cruise throttle, and
  `TRAN_ASPD_MIN` 11 (its `ARSPD_FBW_MIN`), the first lap flew: forward in 9 s gaining
  2.6 m, the lap at 27–32 m and 13–15 m/s, back within 2.6 m. The back transition ended on
  `TRAN_VT_TIMEOUT` still at 6.7 m/s, slowed by drag only.

## Plane altitude and airspeed (total energy control)

Waypoint flight holds altitude and airspeed together
([#454](https://github.com/wboayue/stable-flight/issues/454)): thrust for the total energy,
pitch for its balance. Its knobs are ArduPilot's `TECS_*`, under `PGUID_`:

| Param | Field | Set it to |
|---|---|---|
| `PGUID_CLMB_MAX` | `plane.tecs.max_climb_mps` | the steady climb at full thrust and trim airspeed |
| `PGUID_SINK_MIN` | `plane.tecs.min_sink_mps` | the steady sink at least thrust (`PGUID_THR_MIN`) and trim airspeed |
| `PGUID_ASPD_MIN`, `_MAX` | `plane.tecs.min_airspeed_mps`, … | the stabilizer's `PSTAB_ASPD_MIN`, `_MAX` |
| `PGUID_TCONST` | `plane.tecs.time_constant_s` | 5 s; longer for a slow, heavy aircraft |
| `PGUID_PTCH_DAMP`, `PGUID_THR_DAMP`, `PGUID_INTEG` | … | ArduPilot's 0.3, 0.5, 0.3 |
| `PGUID_PTCH_INTEG` | `plane.tecs.pitch_integrator_gain` | 0.3 (ArduPilot's `TECS_INTEG_GAIN`); lower if descents sink past `PGUID_SINK_MAX` |
| `PGUID_SPDWEIGHT` | `plane.tecs.speed_weight` | 1; toward 0 to hold height before speed, 2 the reverse |
| `PGUID_RLL2THR` | `plane.tecs.roll_compensation` | 10; more if turns lose airspeed |

The climb and sink set where full and least thrust sit, so measure them: fly a leg at full
thrust and one at least, or read them off a fitted model (the Alia's
`alia250_energy_limits_are_the_models`, 1.8 and 3.5 m/s at 45 m/s). Symptoms:

- **Climbs stall short at full thrust:** `PGUID_CLMB_MAX` above what the aircraft makes.
- **Overshoots altitude on steps:** raise `PGUID_PTCH_DAMP`, or `PGUID_TCONST`. A large one
  that doesn't change with them is the pitch loop lagging its setpoint (`PSTAB_*`): on the sim
  Alia a 30 m descent overshoots 20 m while pitch trails by several degrees.
- **Slow below `PGUID_ASPD_MIN` with full thrust:** underspeed has latched (below 0.9 of it);
  it clears back at `PGUID_ASPD_MIN`.
- **Sinks past `PGUID_SINK_MAX` on a descending leg:** catching up height and airspeed
  together (a leg faster than the entry), the pitch integral winds the nose down past the
  limit. Lower `PGUID_PTCH_INTEG`: the Alia's 0.15 sinks 4.7 m/s on a leg 5 m/s past its entry,
  against 5.4 m/s at 0.3 (6 m/s in X-Plane); 0.1 starts to drift entering turns
  ([#503](https://github.com/wboayue/stable-flight/issues/503)).
- **Dives entering plane flight:** the forward transition hands over below the speed the wing
  holds level at, so it trades height for speed: the sim Alia hands over at 37.5 m/s against
  its 40 and loses 41 m. Raise `TRAN_ASPD_MIN` toward `PGUID_ASPD_MIN`.

## Transition knobs

| Param | What it does | Raise it when | Lower it when |
|---|---|---|---|
| `TRAN_ASPD_MIN` | Airspeed that starts the lift ramp | The wing sinks as the ramp begins | It waits long on the lift rotors |
| `TRAN_FW_RAMP` | Seconds the lift rotors take to hand over | The handover sinks or pitches | It's slow and the wing is clearly flying |
| `TRAN_FW_THR` | Most pusher thrust going forward | It times out short of `TRAN_ASPD_MIN` | It overspeeds or balloons |
| `TRAN_PSH_SLEW` | Fastest the pusher's thrust rises | The wait is long | The pitch jerks as the pusher spins up |
| `TRAN_TIMEOUT` | Longest wait on airspeed before it reverts | A slow pusher needs longer | |
| `TRAN_VT_RAMP` | Seconds the lift rotors take the lift back | | It sinks as the wing slows |
| `TRAN_VT_SPD` | Forward speed that ends the back transition | | The copter takes over still fast |
| `TRAN_VT_TIMEOUT` | Longest back transition | Drag slows it slowly | |
| `TRAN_VT_DEC` | Deceleration the back transition pitches up to hold | It ends on `TRAN_VT_TIMEOUT` still fast | The wing balloons as the nose comes up |
| `TRAN_VT_DEC_I` | How fast that pitch-up grows (0: off) | It reaches its pitch late | The pitch overshoots and the climb surges |
| `TRAN_ROLL_MAX` | Bank that cuts a transition short | Gusts end transitions | |
| `TRAN_HANDOFF` | Seconds the pusher thrust blends at the handoff to plane | Thrust steps at the handoff | |

Going forward the pusher flies the plane guidance's thrust for the command, up to
`TRAN_FW_THR`. On a waypoint that is the airspeed loop; on sticks it is the throttle stick,
so push the throttle up to transition. Pitch is the plane guidance's, held to ArduPilot's
transition limits (0 below 3 m/s of ground speed, ±3° waiting, ±8° on the ramp). On a
waypoint it flies toward the altitude the transition holds, not the waypoint's: the wing
enters plane flight level and loaded, and a climb or descent starts there (#491).

Without a valid airspeed the lift ramp never starts, and the transition times out to copter.

Going back the pusher stops and the nose comes up while it slows less than `TRAN_VT_DEC`
(PX4's `VT_B_DEC_MSS`, 2 m/s², and `VT_B_DEC_I`, 0.1), at most about 17°. A clean wing barely
slows on drag alone: at pitch 0 the X-Plane Alia reached its 28 s `TRAN_VT_TIMEOUT` at
31 m/s, and the hover braking from there pitched it to 57° and sank 11 m (#496). With the
pitch-up it ends on speed after 22 s from 45 m/s, climbing 9 m, and hands over at 5 m/s
(the sim Alia: 21 s, 4.9 m).

## Forward thrust

On a waypoint or land, copter flight holds nose-down pitch to `FWD_PIT_LIM` and gives the
pusher `FWD_THR_GAIN · tan(pitch)` the hover asked for, as ArduPilot's `Q_FWD_THR_GAIN` and
`Q_FWD_PIT_LIM`. Piloted sticks keep their tilt. It's off until `FWD_THR_GAIN` is set.

| Param | What it does | Raise it when | Lower it when |
|---|---|---|---|
| `FWD_THR_GAIN` | Pusher thrust per tan of the hover's nose-down pitch (0: off) | It can't hold position or fly upwind in wind | Copter legs overspeed and balloon |
| `FWD_PIT_LIM` | Most nose-down pitch with forward thrust on | | It sinks at speed on copter legs |

Start `FWD_THR_GAIN` at the weight over the pusher's most thrust (ArduPilot's rule), then raise
it until it holds in the strongest wind it must fly in. Under the 27° waypoint tilt cap the
pusher reaches only `FWD_THR_GAIN · tan 27°`, so the rule alone can leave it short.

**Worked example (sim quadplane):** the rule gives 0.83 (4.5 kg, pusher 1.2× weight); at 0.75
it holds in an 11 m/s headwind but flies upwind at 0.6 m/s instead of 5. 1.8 (0.9 of
ArduPilot's default 2) gets there but balloons 16 m on an uncapped 100 m leg. 1.08 flies the
11 m/s leg at 5 m/s and balloons 4 m at most
([#427](https://github.com/wboayue/stable-flight/issues/427)).

**Worked example (Titan Cobra in RealFlight):** on an uncapped 100 m copter leg ahead, gain 0
pitched 31–33° nose down at 19 m/s, lost 3.5–4.7 m and ballooned 6 m braking; gain 1.08 held
the nose within 7°, lost 0.5 m and ballooned 1.2–1.4 m, at 13 m/s (two runs each,
[#433](https://github.com/wboayue/stable-flight/issues/433)). PX4's limit is 20 m
(`VT_QC_T_ALT_LOSS`).
