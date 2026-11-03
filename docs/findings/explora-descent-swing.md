# The eXplora's hover descent swings on a soft pitch tune, not from a vortex ring

**Measured:** 2026-11-03, RealFlight, eXplora, Eli Field, wind off,
`flights/explora_descent.csv` (60 → 20 m over the start) with `--param GUID_DESC_MAX=<sink>`
and `--record`, 250 Hz throughout. **Data:** #647.

On `explora_vtol`'s autotuned hover (#539), the nose holds from 1.5 to 3 m/s of sink. With
only the pitch loops set back to the hand tune #414 flew (`STAB_RATE_PIT_P=0.3`,
`STAB_RATE_PIT_I=0.5`, `STAB_RATE_PIT_D=0`, `STAB_ATT_PIT_P=4`), it swings from 2.5 m/s, as
#414's ±20° did on its code (3030c23, flown again here: −22.2…+17.7°, 2.51 s, elevons 0.77).
View pitch through the descent:

| Hover | Sink, m/s | View pitch | sd | Period | Elevons, most |
|---|---|---|---|---|---|
| autotuned | 1.5 | −3.5…−2.4° | 0.3° | | 0.16 |
| autotuned | 2.0 | −4.5…−2.4° | 0.6° | | 0.21 |
| autotuned | 2.5 | −5.3…−2.0° | 1.0° | | 0.30 |
| autotuned (3 runs) | 3.0 | −4.8…−1.4° | 0.4–0.9° | | 0.27–0.37 |
| soft pitch | 2.5 | −15.0…+6.3° | 6.2° | 2.51 s | 0.53 |
| soft pitch (2 runs) | 3.0 | −19.5…+12.4° | 8.4° | 3.05 s | 0.73 |

Not a vortex ring: the specific force along the thrust axis (`view_force_z_mps2`) follows the
thrust command lagged about 0.12 s, leaving 0.05 m/s² unexplained at 3 m/s on the autotuned
hover, so RealFlight shows no lost thrust and no buffeting. The swing is the pitch loop losing
the elevons' wash as the sink grows. #399's descents after a back transition (80 → 30 m at
3 m/s, `flights/explora_back_ramp.csv`) track their pitch setpoint within 2.7° too; their large
angles are the setpoint braking leftover speed, not a swing.

The sim's momentum-theory wash left the soft tune at 0.2° and 1.1° (2.5 and 3 m/s);
`PropWash::sink_loss` 1.4 gives 5.3° (2.53 s) and 7.9°, and the autotuned hover 0.2° at 3 m/s.
At 1.6 the autotuned hover swings at 3 m/s (6.7°), which RealFlight's doesn't.

**So:** `explora()` flies `sink_loss` 1.4, and its descent rows (`tests/explora.rs`) hold the
sim to these numbers; a descent limit is a property of the pitch tune, so re-check it on the
sim and in RealFlight after a hover retune.
