# Comparable runs

## One knob per run

- Change one knob per run, fly the same script with `--record` (transitions:
  `flights/explora_transition_lap.csv --end land`), and report each run as a row against the
  earlier ones.
- To tell whether a behavior predates a change, fly the parent commit in a throwaway worktree.
- Before crediting a fix with a flight behavior, fly main on the same route too, with the
  heading relative to the wind held and reported
  ([finding](../../../docs/findings/heading-decides-roll-cycle.md)).
- The same settings can hand a back transition over climbing or sinking
  ([finding](../../../docs/findings/back-transition-handover-varies.md)): fly several runs and
  compare runs with a matched handover. A behavior that shows in only some runs needs 3 or more
  per arm.
- `--end land` starts the landing inside the follower's 0.8 m altitude acceptance, so a dip
  below the last waypoint shows only with `--end hold` (#403).

## Tick rate

- Screen each run's `RUST_LOG=info` log with `grep "host slow"`: the runner logs each stretch
  under 225 Hz (over 0.5 s, until back over 240 Hz) with its start, rate, and length in seconds
  since priming, about the recorder's `wall_s`, and the stretches' total at exit.
- Around the event a run measures, count the `wall_s` rows per wall-clock second too, and drop
  a run below about 245 Hz there ([finding](../../../docs/findings/low-tick-rate-slows-clocks.md)).
- A release build skips less but doesn't stop it; RealFlight's own pacing is the larger cause
  ([realflight.md](realflight.md)). The runner drops time lost to a stall over 100 ms, so heavy
  `ticks skipped` in the exit log slows the clocks the same way.

## Explaining a trace

Before suspecting a bug, replay the recorder CSV through the real component (state rebuilt
from the view angles, nominal `dt` per tick) and compare its output with the recorded
setpoint; the hover guidance reproduces it exactly (#405). Plot a run with
`just plot --record <csv> <out.png>` (time series and a ground track; `--from`/`--to` in
seconds).
