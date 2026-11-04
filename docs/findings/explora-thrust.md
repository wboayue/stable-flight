# The eXplora hovers at thrust 0.88 and cruises at 0.86; its prop loses thrust faster at part throttle

**Measured:** 2026-11-03, RealFlight, eXplora VTOL, Eli Field, calm,
`flights/explora_back_ramp.csv` runs 1–5 (#399), and `flights/explora_loops.csv` runs 2–3
(#630). **Data:** #651.

Thrust setpoints (`sp_thrust`), the median over each steady leg (|climb| < 0.3 m/s). The
recorder's `thrust` column is after the stabilizer's squared thrust curve, `sp²`: 0.863 cruise
reads 0.745 there, which is where an earlier "level thrust 0.76" came from (#342).

| Leg | Setpoint |
|---|---|
| Hover near still (1.1 m/s), 30 and 80 m | 0.88 (0.879 → 0.893 over 60 s) |
| Hover leg along the span at 3 m/s, 30 m | 0.91 |
| Level plane flight, 14.0–14.2 m/s, 80 m | 0.863 (0.875 → 0.859 over the leg) |

#123's joystick hops hovered at 0.88–0.89 too: `explora_vtol`'s `hover_thrust` 0.84 came from
a lower window, not a RealFlight change.

**Takeoff** at setpoint 1.0 (motors 0.9) climbs at 3.0 m/s² over its first 0.8 s, T/W 1.31
against 1/0.88² = 1.29 for thrust linear in motor command (1.67 if quadratic).

**Back ramps** (sink guard on, from 14 m/s at 80 m), height gained by the handover, its speed
and climb, the most elevon:

| `TRAN_VT_THR` | Gain | Handover | Climb | Elevons |
|---|---|---|---|---|
| 0.75 | +8.9 m | 2.5 m/s | +1.9 m/s | 0.23 |
| 0.70 | +7.4 m | 1.4 m/s | −0.4 m/s | 0.53 |
| 0.65 | stalled: nose stops at 59°, falls to 45° | | | 1.0 |
| 0.60 | stalled at 54° for 9.4 s | | | 1.0 |

The 0.75 ramp loses about 1 m of total energy (height plus speed) by the handover. Its pitch
and speed follow a prop that loses thrust with airspeed at the same rate at any throttle until
30° of pitch; past it, at part throttle and 5–10 m/s, the climb falls off sooner (peak
4.6 m/s against 5.3 m/s on that model). A fixed-pitch prop whose zero-thrust speed scales
with its rpm reproduces it.

**Centered rate sticks** at cruise thrust 0.85, stepped down from the line: 0.010 loses
0.30 m/s in 10 s, 0.018 loses 0.58 m/s, about 31 m/s per unit of setpoint.

**So:** the sim eXplora's hover and cruise are matched to these setpoints, and its prop's
zero-thrust speed falls with rpm (`ThrustFalloff::PitchSpeed`), in
`crates/stable-flight-sim/src/airframes/explora.rs`; compare thrust in setpoints, not the
recorder's `thrust` column.
