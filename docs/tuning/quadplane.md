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
  Forward thrust hands the nose-down pitch to the pusher (below). Tail first, keep legs slow:
  the Cobra flipped at 11–16 m/s backward, pitching up past its lift rotors' authority
  ([#437](https://github.com/wboayue/stable-flight/issues/437)).

## Order

1. **Hover.** Tune the lift rotors as a copter: the copter rate and angle loops (`STAB_*`),
   with `just fly-autotune` or by hand, then altitude and position (`GUID_*`). The surfaces
   stay neutral in a hover. A CG off the lift rotors' center needs a steady pitch moment:
   give the pitch rate loop enough I to hold it (I · max integral above the trim).
2. **Surface directions.** Before the first plane flight, check each surface pushes the way
   the mixer expects. On the ground with no airflow they move nothing; in copter flight at
   speed they do, while the lift rotors hold attitude against them
   (`surface-probe-realflight`). A reversed one is `reversed_servos` in the backend mapping;
   a surface whose neutral sits off center is a servo trim.
3. **Plane.** Tune the wing as an airplane (`PSTAB_*`, `PGUID_*`). Set `PGUID_CRUISE` to the
   pusher thrust that holds level flight at trim airspeed.
4. **Transitions.** Tune these last, from a settled hover at least `TRAN_MIN_HGT` up.

**Worked example (Titan Cobra in RealFlight):**
- **Hover:** nose-heavy on its lift rotors, a level hover holds front motors at 0.60 and rear
  at 0.52, a 0.37 nose-up moment; pitch rate I 0.5 holds it. The autotune raised roll rate P
  (steps 0.85 → 0.95 of the step); pitch rate steps stop near 0.90 on any gains
  ([#438](https://github.com/wboayue/stable-flight/issues/438)).
- **Surfaces:** ArduPilot's parameter file reverses none; the probe confirmed each axis at
  12–16 m/s. Both tail panels trim +0.2 (`SERVO2_TRIM`, `SERVO4_TRIM` 1600).
- **Plane and transitions:** with ArduPilot's airspeeds, trim pitch, and cruise throttle, and
  `TRAN_ASPD_MIN` 11 (its `ARSPD_FBW_MIN`), the first lap flew: forward in 9 s gaining
  2.6 m, the lap at 27–32 m and 13–15 m/s, back within 2.6 m. The back transition ended on
  `TRAN_VT_TIMEOUT` still at 6.7 m/s, slowed by drag only.

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
| `TRAN_ROLL_MAX` | Bank that cuts a transition short | Gusts end transitions | |
| `TRAN_HANDOFF` | Seconds the pusher thrust blends at the handoff to plane | Thrust steps at the handoff | |

Going forward the pusher flies the plane guidance's thrust for the command, up to
`TRAN_FW_THR`. On a waypoint that is the airspeed loop; on sticks it is the throttle stick,
so push the throttle up to transition. Pitch is the plane guidance's, held to ArduPilot's
transition limits (0 below 3 m/s of ground speed, ±3° waiting, ±8° on the ramp).

Without a valid airspeed the lift ramp never starts, and the transition times out to copter.

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

**Worked example (sim quadplane):** the rule gives 0.83 (4.5 kg, pusher 1.2× weight): it
holds in an 11 m/s headwind but flies upwind at 0.6 m/s instead of 5. ArduPilot's default 2
gets there but balloons 16 m on an uncapped 100 m leg. 1.2 flies the 11 m/s leg at 5 m/s and
balloons 4 m at most
([#427](https://github.com/wboayue/stable-flight/issues/427)).

**Worked example (Titan Cobra in RealFlight):** on an uncapped 100 m copter leg ahead, gain 0
pitched 31–33° nose down at 19 m/s, lost 3.5–4.7 m and ballooned 6 m braking; gain 1.2 held
the nose within 7°, lost 0.5 m and ballooned 1.2–1.4 m, at 13 m/s (two runs each,
[#433](https://github.com/wboayue/stable-flight/issues/433)). PX4's limit is 20 m
(`VT_QC_T_ALT_LOSS`).
