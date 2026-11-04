# The eXplora's autotuned hover descends at 2 m/s as cleanly as at 1.5; at 2.5 its arrival lags

**Measured:** 2026-11-03, RealFlight, eXplora on `explora_vtol`'s autotuned hover (#539), Eli
Field, 250 Hz through each phase read (runs with a host-slow stretch in one left out), with
`--param GUID_DESC_MAX=<sink>` and `--record`. **Data:** #653.

Worst |`view_pitch` − `sp_pitch`| by phase of a hover descent, three runs per row:

- **start:** the first 3 m of sink (calm and wind) or the first 5 s (back transition);
- **arrive:** the last 3 m, braking onto the waypoint at `GUID_DESC_ACC`;
- **mid:** between, sinking at the limit.

| Route | Sink, m/s | Start | Mid | Arrive | Elevons, most mid |
|---|---|---|---|---|---|
| calm, `explora_descent.csv` 60 → 20 m | 1.5 | 3.9–8.0° | 0.3–0.4° | 0.3–1.6° | 0.15–0.16 |
| | **2.0** | 2.2–2.9° | 0.3–0.7° | 0.5–0.7° | 0.16–0.21 |
| | 2.5 | 2.3–6.6° | 0.6–1.1° | 0.9–2.5° | 0.18–0.28 |
| 5 km/h wind, `explora_descent_wind.csv` 60 → 20 m | 1.5 | 1.1–3.5° | 1.3–2.0° | 0.4–4.8° | 0.39–0.67 |
| | **2.0** | 0.6–1.1° | 1.3–1.8° | 1.2–6.4° | 0.63–0.71 |
| back transition, `explora_back_ramp.csv` 80 → 30 m (calm) | 1.5 (1 run, #399) | 7.7° | 0.2° | 6.5° | 0.21 |
| | **2.0** | 7.1–7.5° | 0.2° | 3.2–6.7° | 0.25–0.28 |
| | 3.0 (4 runs, #399) | 2.5–6.9° | 0.6–1.3° | 1.6–6.4° | 0.40–0.54 |

Mid-descent the nose tracks within 2° at every rate and route, the elevons short of 0.75. The
start and arrive spikes are setpoint steps the pitch loop lags, the same size at 1.5 and
2.0 m/s, and the only places the elevons reach 1.0, at either rate:

- **calm start:** the climb to 60 m reversing into the descent;
- **calm arrive:** the brake onto 20 m, which grows with the rate: 2.5 m/s reached 2.3–2.5° in
  two of three runs against 1.5's 1.6° at most;
- **wind:** a gust letting go of the 27° tilt cap steps the setpoint 7–14° in 0.5 s. At 5 km/h
  the hover sits at that cap for 6–82% of a descent at either rate and drifts downwind while
  it does (2.3–9.4 m);
- **back transition:** the hover leaves its hold at 80 m with a 25° step, and reaches 30 m as
  the next leg starts (the 6.2–6.7° arrive peaks at 30.5 m, at both rates).

The wind route holds 60 m for 20 s first: the wind estimate takes no samples sinking or
climbing faster than 1 m/s (`WindConfig::max_axial_speed_mps`), so on `explora_descent.csv`,
with no hold at the top, it settled within `GUID_WND_HDG_SD` in time to turn a wingtip into
the wind in 2 of 6 runs; the other 4 drifted belly first past 15 m before their descent.

**So:** `explora_vtol` descends at 2 m/s; 2.5 m/s is the step whose calm arrival lags past
1.5 m/s's.
