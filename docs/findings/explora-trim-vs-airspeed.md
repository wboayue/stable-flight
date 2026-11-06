# The eXplora's elevon trim falls with airspeed as V^-0.74

**Measured:** 2026-11-01, RealFlight, eXplora VTOL, Eli Field, calm,
`flights/explora_loops.csv`, runs 2 and 3; 2026-11-05, the same,
`flights/explora_plane_tune_18.csv`. **Data:** #648.

In steady level plane flight (|roll| < 3°, |q| < 0.001 rad/s, |climb| < 0.3 m/s; about 20k rows
over 13.4–19 m/s), the elevon command that holds pitch (`moment_y`) goes as V^-0.74, with no
thrust term (−0.08 per unit). In the four 10 s pauses, at constant thrust, it goes as V^-1.0.
The rate loop's airspeed scaling assumes V^-2, so a speed loss leaves its integral to unwind the
difference, pitching the nose up.

How far depends on the airspeed the pause loses to its thrust step. Stepping down 0.018 of
thrust setpoint, it lost 0.58 m/s and the pitch rose 0.21° in both runs (0.36° per m/s).
Stepping down 0.010, it lost 0.30 m/s and rose 0.06–0.07°, then fell 0.18° from the 2° bank the
pause began in. RealFlight's pauses start 6 s after a corner,
with thrust still settling, so they step down less than a settled line does.

RealFlight's plane autotune fits at 14 and 18 m/s split it (#648; 18 m/s runs 1–2, trimmed
there): the fitted trim moment falls from 0.089 to 0.074 (V^-0.73), and the elevons' pitch
control grows from 80–82 to 123 rad/s² (V^1.65, slower than dynamic pressure). So the moment
the elevons hold grows only about as V^0.9: the trim moment, not the elevons' authority, departs
from V².

The sim eXplora's trim moment grows as V² below trim airspeed and as V¹ above it
(`c_pitch_0_knee`): its fitted trim falls from 0.090 to 0.073 (V^-0.82). Its centered sticks,
losing speed below trim, drift 1.04° per m/s lost (#666).

**So:** compare centered-sticks drift per m/s of airspeed lost, not in degrees
(`explora_centered_rate_sticks_hold_pitch`, `crates/stable-flight-sim/tests/maneuver.rs`).
