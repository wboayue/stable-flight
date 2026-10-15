# Quadplane tuning guide

Tunes a lift+cruise VTOL (quadplane) flown with `QuadPlaneGuidance` and
`QuadPlaneStabilizer`:
- **Hover:** the copter controllers, on the lift rotors.
- **Wing-borne flight:** the plane controllers, on the surfaces and pusher.
- **Between them:** a `LiftTransition`, both sides at once.

The reference airframe is the simulator's
[`airframes::quadplane()`](../../crates/stable-flight-sim/src/airframes/quadplane.rs), checked
in `crates/stable-flight-sim/tests/quadplane_transition.rs`. No quadplane has flown in a
simulator backend yet, so this guide covers the knobs and the order. Worked numbers from a
flight come when one has flown.

## What makes a quadplane different

The vehicle stays level, so the two sides never fight over the same actuators. A transition
doesn't hand control from one controller to the other. Instead it trades the lift: the lift
rotors' thrust and moments are weighted by a lift share, and the wing flies in full.

- **Forward:** the lift rotors hold the hover while the pusher builds airspeed, then hand
  the lift to the wing over a fixed time.
- **Back:** the pusher is cut, and the lift rotors take the lift back while drag slows it.
- **Copter flight at speed:** nose down, the wing pushes down and sinks it
  ([#427](https://github.com/wboayue/stable-flight/issues/427)). Keep copter legs short and
  slow.

## Order

1. **Hover.** Tune the lift rotors as a copter: the copter rate and angle loops (`STAB_*`),
   with `just fly-autotune` or by hand, then altitude and position (`GUID_*`). The surfaces
   stay neutral in a hover.
2. **Plane.** Tune the wing as an airplane (`PSTAB_*`, `PGUID_*`). Set `PGUID_CRUISE` to the
   pusher thrust that holds level flight at trim airspeed.
3. **Transitions.** Tune these last, from a settled hover at least `TRAN_MIN_HGT` up.

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
