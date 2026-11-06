# The eXplora's wash pitches it nose down, harder past the stall nose first

**Measured:** 2026-11-05, replaying RealFlight's eXplora back-transition runs (Eli Field,
`flights/explora_back_ramp.csv` at `TRAN_VT_THR` 0.60–0.75, recorded at 250 Hz) through the sim
eXplora's aerodynamics: each tick's airspeed, rates, elevons, and motors in, the pitch moment
out, against the moment RealFlight's pitch acceleration takes (`Iy · q̇`, `q̇` over ±6 ticks).
**Data:** #399, #661.

The gap (RealFlight's moment less the sim's, at the sim's inertia), by phase:

| Phase | Ticks | Gap, N·m | Wash pressure × S·c |
|---|---|---|---|
| Cruise (`Plane`) | 17014 | +0.05 | 1.2 |
| Hover, below 1.5 m/s | 28469 | −0.71 | 3.3 |
| Back ramp, attached | 1368 | −0.47 | 0.6 |
| Back ramp, stalled, motors below 0.6 | 475 | −1.17 | 1.0 |
| Back ramp, stalled, motors above 0.6 | 184 | −3.27 | 3.1 |

- **Hover:** the gap follows the wash. Settled in a hover (below 1.5 m/s and 0.3 m/s of climb,
  0.1 rad/s of pitch rate, 0.83–0.95 thrust), RealFlight holds 0.14 of nose-up elevon (median
  of 319 ticks over five runs; middle half 0.04–0.26). #539's calm hover holds about 0.11.
- **Stalled, nose first:** the gap grows with thrust beyond the hover's share. A wash moment
  on the hover's share plus one scaled by the stall's blend and the free stream's share of the
  pressure over the surfaces, `q / (q + wash)`, fits every stalled phase (−0.145 and −3.3 of the
  surfaces' wash pressure). A moment on the hover's share alone fits neither the hover nor the
  stall.
- **Attached:** the gap grows with α on the free stream, not the wash (the same `gap / q·S·c` at
  low and high thrust): −0.03 at 3–6°, −0.09 at 6–9°, −0.16 at 9–15°, 0 at cruise's 0–3°. Neither
  wash term covers it, and a nose-down pitch break from 2° lifts the cruise pitch stiffness past
  RealFlight's (114/s² against 68.8–93.4, #587).

In the sim, `PropWash::pitch_moment` −0.13 and `stall_pitch_moment` −2.0 (nose first only:
tail-first sinks are past the stall too, and the stall term there stopped the soft tune's
descent swing) fly the back ramps as RealFlight's did (`tests/tailsitter_transition.rs`):

| `TRAN_VT_THR` | RealFlight | Sim: elevons, most | Sim: sink, most |
|---|---|---|---|
| 0.60 | stalled, elevons full | 1.00 | 6.1 m/s |
| 0.65 | stalled, elevons full | 1.00 | 5.7 m/s |
| 0.70 | clean, elevons 0.53, sinking 0.4 m/s | 0.55 | 0.40 m/s |
| 0.75 | clean, elevons 0.23 | 0.15 | 0 |

The hover holds 0.10 of elevon. A wash moment on the hover's share large enough to stall the
0.65 ramp alone (−0.3) holds 0.23, clips the tuned pitch rate steps past the autotune's limit,
and stops the soft tune's descent swing ([descent swing](explora-descent-swing.md)): with the
trim near `explora_vtol`'s `max_integral` 0.28 the rate I pins there, where RealFlight's
0.14 leaves it room.

**So:** the sim eXplora's hover trim and back-ramp stall come from the wash's pitch; its
attached high-α pitch moment still runs nose up of RealFlight's, so a back ramp's attached
phase holds less elevon than RealFlight's.
