# The eXplora's elevon trim falls with airspeed as V^-0.74

**Measured:** 2026-11-01, RealFlight, eXplora VTOL, Eli Field, calm,
`flights/explora_loops.csv`, runs 2 and 3. **Data:** #648.

In steady level plane flight (|roll| < 3°, |q| < 0.001 rad/s, |climb| < 0.3 m/s; about 20k rows
over 13.4–19 m/s), the elevon command that holds pitch (`moment_y`) goes as V^-0.74, with no
thrust term (−0.08 per unit). In the four 10 s pauses, at constant thrust, it goes as V^-1.0.
The rate loop's airspeed scaling assumes V^-2, so a speed loss leaves its integral to unwind the
difference, pitching the nose up.

How far depends on the thrust step into the pause. Stepping down 0.018 of thrust setpoint, the
pitch rose 0.21° in both runs (11.7° per unit). Stepping down 0.010, it rose 0.06–0.07°, then
fell 0.18° from the 2° bank the pause began in. RealFlight's pauses start 6 s after a corner,
with thrust still settling, so they step down less than a settled line does.

The sim eXplora's trim command stays flat with airspeed: its trim moment, like its elevons'
authority, grows as V². On the same gains it drifts 22.6° per unit of thrust step.

**So:** compare centered-sticks drift per unit of thrust step, not in degrees
(`explora_centered_rate_sticks_hold_pitch`, `crates/stable-flight-sim/tests/maneuver.rs`).
