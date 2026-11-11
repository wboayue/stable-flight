Part of the [architecture overview](../../architecture.md).

## Design search

**Design search** (`StagePlan::copter_design_search`, the copter default;
[#312](https://github.com/wboayue/stable-flight/issues/312)). A search from the start gains
spends most of each stage settling before each step: on the sim, a rate stage's 50 s holds about 5 s of
stepping, an angle stage's 61 s about 24 s (182–627 s per row); QuadCopterX in RealFlight
takes about 9 min. The design search fits each axis and designs its gains, then searches
each loop from its design, so the search starts near its answer.

### Identify

* *Identify* (`tuning::IdentifyProbe`, in each `Method::Design` stage): a shrinking square
  wave of 0.7 rad/s added to the rate setpoint, every loop closed (PX4's excitation:
  `mc_autotune_attitude_control`'s `getIdentificationSignal`, `MC_AT_SYSID_AMP`, added to
  the rate setpoint by `mc_att_control`), and a least-squares fit of the axis's body
  rate to its moment command: a second-order model with a delay (PX4's `ArxRls<2,2,1>`), the
  delay the shortest within 1% of the least residual among those whose moment pushes the
  axis the right way (a positive moment-weight sum). In RealFlight the fits explain only
  35–72% of the rate change, so noise picks among near-tied delays: 10 of 67 recorded
  QuadCopterX fits read under 20 rad/s² (negative in 7); replayed with the filter, 5 did
  (#529). A wider tie (5%) left 1 but took 1-tick delays in 6, which design too hot a loop:
  one rang and failed its stage. A fitted pole at or below 0 is a lag under a tick, which
  the delay absorbs, and designs as zero lag (D 0): in RealFlight both copters' poles
  scatter from −0.2 to 0.2 on noise (Titan Cobra delay 9–10 ticks, QuadCopterX 3–4), and
  half the fits, failing as no lag, ended Cobra design searches after three in a row.
  Replayed, all 26 recorded fits design rate P within 0.47–0.54 on the Cobra and 0.61–0.84
  on QuadCopterX, against 0.48–0.54 and 0.69–1.10 from the 11 that had a lag (#563). PX4
  designs from the fitted coefficients with no lag step (`pid_design::computePidGmvc`), so
  a near-zero pole gives it a near-zero D the same way. An angle stage's fit runs on the
  rate loop under it. A 4°
  wave on the angle setpoint was rejected: its sustained part scales with the start's angle
  P, and each edge kicks the angle D into a one-tick moment spike clipped at 1, which
  RealFlight delivers in part. On the Titan Cobra's start (angle P 3, rate P 0.59) the
  spikes were most of roll's excitation and fit 11 rad/s² against pitch's 22–24, so the
  design rang; without angle D, roll didn't fit at all. On the rate setpoint roll fits 23
  against pitch's 27 (#561). The
  probe records the window and fits it when the excitation ends; the least-squares
  fit takes the rate's change on the last rate and the last change, since fitting the last
  two rates side by side loses the lag in `f32` (up to 1.6 ms off, #516), and recursive least
  squares hadn't converged in 3 s. On the sim, 3 s per axis fit the same
  model from the low, own, and ×3 starts, recovered each airframe's motor lag (29.8, 49.1,
  14.9 ms against 30, 50, 15), and found the actuator delay (1, 3, and 6–7 ticks clean, at
  10 ms, at 25 ms). The fit reads the raw rate and moment: a low-pass on both (PX4's
  autotune prefilters alike) kept the sim's gyro tone out of the rate terms, four poles at
  40 Hz passing 28 of 30 noise-row runs over 5 noise seeds, but on RealFlight's QuadCopterX,
  whose dynamics aren't the model's, every cutoff tried (40–80 Hz, two to eight poles) read
  the 5–13 ms lag as 10–270 ms, and the designed rate P fell to half (#373).
* *Noisy windows* (`IdentifyProbe`, #551, #603): in closed loop the controller feeds gyro
  noise into the moment, so the least-squares fit reads that noise as the airframe: with the
  sim's 0.02 rad/s of white noise alone it read every airframe's lag as 0, and with its
  90 Hz tone it designed rate P from 0.23 to 16 against 0.67–1.90. So the probe fits by least
  squares first and gates on what it leaves unexplained, the residual's RMS per tick: past
  0.017 rad/s the window is noisy. RealFlight's windows leave 0.002–0.012 (eXplora and
  QuadCopterX, 8 windows), the sim eXplora with 0.01 rad/s of gyro noise 0.012, and the sim's
  noise rows 0.024–0.100. A clean window leaves under 0.001 whatever its gains: a gate on the
  rate's own noise (the RMS of its second difference) read a clean `racer` at 1.75× its
  gains, ringing, as 0.074 against RealFlight's 0.020 at most.

  A noisy window goes on to 8 s at twice the amplitude (`IdentifyConfig::noisy`,
  1.4 rad/s for the copter), then takes the integrated fit. The recorded moment goes through
  each lag on a grid (0, then 1–500 ms in 5% steps, refined between the best point's
  neighbors) and each delay in half ticks (the zero-order hold leaves a moment's first tick
  partly felt), and the rate is fitted to it with a gain and a constant, both sides through a
  0.1 s leaky integrator, the measured rate only the target. Integrated, the target's noise
  is mostly the current tick's, which no moment commanded before it carries. In the per-tick
  form the rate's change holds the last tick's noise too, which the moment one tick on
  answered (table). Gyro noise fills the residual, every delay within 0.05% of the least, so
  the least wins outright, where the least squares' 1% tie took the shortest.
  Under the sim's noise with 10 ms of delay (true 3 ticks; quiet least squares
  reads 3), over 5 seeds on roll and pitch, the read delay, and the rate P designed against
  the noise-free plant's:

  | fit, window | `quad_x` | `heavy_quad` | `racer` | rate P against noise-free |
  |---|---|---|---|---|
  | projected moment, per tick, 1% tie (rejected) | 1 | 1 | 1 | 2.0–2.9× |
  | the same, least residual | 2–10 | 8–10 | 3–7 | 0.33–1.39× |
  | integrated, 0.7 rad/s, 3 s | 4–8 | 4–7 | 3–4 | 0.51–1.29× |
  | integrated, 1.4 rad/s, 3 s | 3–5 | 4–5 | 3–4 | 0.75–1.40× |
  | integrated, 0.7 rad/s, 8 s | 3–5 | 4–5 | 3–4 | 0.81–1.57× |
  | recorded moment per tick, extended to 1.4 rad/s, 8 s | 1–8 | 3–5 | 3 | 0.57–2.29× |
  | integrated, extended to 1.4 rad/s, 8 s | 3–4 | 4 | 3–4 | 0.88–1.08× |

  Noise-only rows (true 1 tick) read 1–2. The extension costs 5 s per noisy fit, and the
  fit about 11 ms on a desktop in the window's last tick, against 0.1 ms for the least
  squares; the window (26 KB) holds 8.2 s.

  The gate stays: on a quiet window the least squares is exact, and the integrated fit reads
  a delay-dominated axis short. On RealFlight's eXplora roll window it read 4½ ticks and
  98 ms of lag against the least squares' 9 and 2 ms, and designed rate P 1.48 against
  RealFlight's 0.94 (pitch 3.16 against 1.69, QuadCopterX 15–35% over).

  A window past 20% saturated never takes the integrated fit: the ×3 starts ring at the rate
  loop's ±1 for 510–629 of 750 ticks, and a clipped moment isn't the one the airframe felt.
  A saturated window that is quiet takes the least squares, exact there. One that is also
  noisy has no usable fit (the least squares designed `quad_x`'s rate P 8.1 against 1.9),
  so the probe aborts (`IdentifyAbort::Saturated`) and the design
  stage lowers its start, every gain ÷ 1.6, and fits again at once, up to eight times (the
  search's cap, below). The refit doesn't wait for the settle: gains a cut below ones that
  rang seldom settle. On the rows over the time budget (the trims, and `heavy_quad`), every
  wait ran to the 10 s timeout, where an identification starts anyway, so each 3 s fit cost
  13 s. At ÷ 1.6 the ×3 starts under noise stop
  saturating after 4 cuts (`quad_x`, `heavy_quad`) or 5 (`racer`), against 6–8 at ÷ 1.3,
  and their robustness rows take 303–334 s against the 350 s budget, where waiting at ÷ 1.3
  took up to 510 s (#673). Rejected: refitting at once at ÷ 1.3 or ÷ 2, which missed the
  step spec on 4 and 3 of the 15 ×3 noise rows over seeds 1–5 (÷ 1.6: none); ÷ 1.6 still
  waiting (366–419 s); and a budget raised to the slowest ×3 row, which would hide the wait.

  Rejected (#551, #603): the moment projected on the excitation (10 past offsets, six slow
  lags of it, and a constant), which drops the noise-driven moment, most of the window's
  fast content, so its residual didn't tell the delays apart (table); fitting the projected
  rate as the target too (`quad_x` 2–10 ticks at 3 s, 1–10 at 8 s); both sides prefiltered
  at 20–200 ms on the projected moment (1 tick); the recorded moment per tick, not integrated
  (table; `quad_x`'s noise row on seed 2 then fails its roll angle design); the integrated
  fit at 0.4 s (some 2 ticks,
  P to 1.58×) or 1 s; a design for the lowest P among the tied delays (0.3–0.6× the
  noise-free P); the longest delay of a near-tie (eXplora roll P 0.71, pitch 2.06–2.42); the
  delay from the least-squares fit (roll lag 97 ms); a low-pass on both sides of the
  least-squares fit (PX4's prefilter, above); instrumental variables on the lagged
  excitation (erratic, P 0.2–23); one moment term (worse); and a free weight on each of two
  ticks instead of half-tick delays (fitted noise, opposite in sign). A real airframe whose
  vibration crosses the gate takes the integrated fit, which reads a delay-dominated axis
  short (the eXplora's roll, above).

### Design

* *Design* (`tuning::design_rate`, `design_angle_p`, `COPTER_LOOP_SHAPING`): rate D/P at the
  fitted lag, so the PD's zero cancels the motor lag; rate P for a 65° phase margin on the
  model's frequency response, its delay and the stabilizer's gyro filters included
  (`GyroFilters`: the low-pass, and the notch when on; left out, the low-pass took all but
  2° of `quad_x`'s margin, #373); rate I at `rate_i_per_p` of P;
  angle P the
  largest in the angle search's range that keeps a 65° margin on the angle loop around the
  designed rate loop, less 10%, the rate loop's D on its measurement and P alone on its
  setpoint. A fit whose moment pushes the axis the wrong way has no design: its phase
  starts half a turn from an integrator's, and through the gyro filter drifts across the
  margin, which would read as a crossover. The angle loop's model leaves the rate I out: its zero sits
  about 45× below the angle crossover, and around a fit with a pole just outside the unit
  circle it flipped the design between P 9 and none on near-identical fits (#374). The 65°
  margin met the step spec on every clean, 10 ms, and 25 ms row of `quad_x`, `heavy_quad`,
  and `racer`, from both starts (60° and 55° each missed on some). A fitted integrator past
  1 (an unstable pole from a ringing start, whose phase meets no crossover) is held at 1. Angle P by margin rather than by model steps: both put it at the
  range's top on every row, and the steps cost about 19k simulated controller ticks between
  two flight ticks, the margin up to 18 sweeps of the grid, about 7.5k frequency points
  (#517). The design runs on a log-spaced grid from 0.1 rad/s to Nyquist, each crossing
  refined by bisection. A design stage commits its gains without steps, once the vehicle
  holds the settle about the loop's axis on them (0.5 s within the settle timeout); a design
  that doesn't, or breaches the envelope, goes back and the axis is fitted again, as an
  unusable fit is. In RealFlight, fits reading 8 and 19 rad/s² against 25–71 designed pitch
  rate P 3.93 and 2.30, which rang at 0.6 rad/s on pitch and kept the stages after them
  from settling (#518). Only the loop's axis counts, since the other may still ring on its
  start gains. A ceiling on the designed P can't tell those apart: healthy sim designs reach
  P 4.66 (`quad_x` clean), and 31× a low start.
* *Full-size multirotor* (`StagePlan::large_copter_design_search`,
  `LARGE_COPTER_LOOP_SHAPING`): rate D 0, rate P for the 65° margin through the motor lag,
  the start's rate I kept, then the large-copter search. Cancelling the lag puts the
  crossover where the delay and gyro filters leave the margin, about 50 rad/s on every sim
  copter: 1.25–2.5× 1/lag on `quad_x`, `heavy_quad`, and `racer`, 8.5× on the Alia 250's
  0.17 s lag. The fit is exact there (lag 0.170 s, roll 1.74 and pitch 1.04 rad/s² on any
  window), so the design is right on the clean model: rate P 28.6 and 47.7 with D 4.9 and
  8.1 meet the large steps' spec. But the search after it, its P capped at 3, lowers P and
  keeps D, and the Alia never settles; and under gyro noise that D clips every loop and the
  run loses altitude. Without D, from rate P 1 and 3, the design gives P 1.65 and 2.75 and
  the search ends at 1.49 and 2.48 (hand tune 1.52, 2.57) in about 13 min of sim time,
  against 11–15 for the search alone. Rejected: the search's rate P range raised to 100 (P
  climbs to 69, 1212 s); the designed P capped at 3 with D scaled (pitch angle design fails,
  2 misses); D at a quarter or half of the lag-cancelling D (5.8% overshoot; clipping); the
  design alone, without the search (its D clips every loop under gyro noise) (#572).

### Search

* *Search* (`SearchPlan::copter`, after every loop is designed): the step search from the
  design, P bracketed from it in either direction, D as designed. Designing everything first keeps a loop still on its start gains from ringing through
  another's steps: `racer`'s pitch on its high start with 25 ms of delay (±2.8 rad/s, #518).
  D stays fixed while P moves, even though D/P then no longer matches the fitted lag: the
  fixed D is what lets overshoot bound P. Rejected: D scaled with every P tried, so D/P stays
  the lag. Larger P then brings larger D, and most searches end at the top of the range (2.7
  after the back-off). Of the 49 robustness tests (48 rows and the noise-seeds test, 72 runs
  in all), 24 passed, against 48 of 49 on fixed D (the 49th, `racer` at 25 ms, is ignored). 22
  runs ended `NotSettled`, 18 of them on pitch rate. `heavy_quad`'s noise rows missed the
  spec 4 times each in 443–763 s, against none in 305–338 s. Also rejected: D/P kept only
  when the search lowers a start above `max`. No robustness row or noise seed reaches that
  path (0 of 72 runs changed), since every design on those plants sits under `max` (#683).
* The design search searches yaw rate before roll and pitch: their settles leave yaw out,
  so a yaw start that rings spoils their steps instead. On `racer` with 25 ms of delay,
  core's default yaw gains ring, the design's rate gains overshoot by 0–23% from one step
  to the next (0% from a still hover), and the rate searches passed nothing. A search's
  start that doesn't settle within the settle timeout is lowered by the bracket's factor
  until it does, at most eight times, the cap a rate design's saturated fits share (80 s on
  ringing gains; `racer` needs one); a tighter settle (0.05 or 0.03 rad/s) didn't help and
  cost the clean rows their time budget (#552, #598).
* Every copter step may clip for 10 ticks (`COPTER_MAX_SATURATED_S`), and roll and pitch
  are searched and step-tested at 5° (`COPTER_TILT_STEP`): at the tuned angle P 9 a
  step asks for more than full moment for its first ticks, a limit rather than a bad loop
  (#310). On the sim's tuned `quad_x`, 5° steps clip for 5 ticks (6–10 under its trim rows)
  and 10° steps for 13 (`heavy_quad` 15). After a QuadCopterX autotune in RealFlight, the
  10° step test was `Invalid` on all 8 runs, and 3 of 16 angle searches kept their start
  after points whose every step clipped. Swept over the angle allowance, the robustness
  table passed 21 rows at 5 ticks, 22 at 6 and 8 (`quad_x`'s three trim rows gained,
  `heavy_quad`'s weak motor or CG offset lost), and 24 at 10 and 15 (nothing lost). 15 ticks
  would also pass 10° steps but spends 60 ms of a 200 ms rise at the limit. The sim spec's
  clip band reads the same allowance, so the RealFlight check is what judges it. With the
  rate allowance, #518 measured `quad_x`'s angle P held at 9 with 10 ms of delay and its
  steps clipping; on the gyro-filtered rate loop with rate I, those rows pass.
* Under gyro noise, a search's verify settles up to 1.14 times slower than its baseline
  while overshooting within its margin, so `Acceptance` allows 1.2: at 1.1, `quad_x`'s yaw
  and pitch rate searches kept their start as worse on 12 of its 30 noisy runs over 5
  seeds, and every ratio from 1.2 up tunes the same rows with none lost (#672). The sim
  spec judges each step under noise on the mean of 4, ArduPilot's
  `AUTOTUNE_SUCCESS_COUNT`: ArduPilot keeps a gain only once its passing twitches
  outnumber its failing ones by 4 (`AC_AutoTune`), so it never judges on one, though the
  spec's mean is looser (one step can overshoot if the others make up for it). One
  0.5 rad/s yaw step reads 0.6–7.1% overshoot for gains whose mean is 3.5%. Tightening the search instead (a 0.04
  or 0.045 overshoot limit, a 0.85 or 0.8 back-off, 3 verify pairs) moved the misses
  between seeds and cost other rows a clip or rise target or the time budget; judging each
  direction on its own lowered P on the CG-offset rows until their rate steps missed the
  rise target.
* Yaw rate steps are filtered at 10 Hz, as ArduPilot filters a yaw twitch
  (`AUTOTUNE_Y_FILT_FREQ`), and each yaw bracket point must pass 3 pairs in a row
  (`confirm`; not more verify pairs), toward ArduPilot's 4 (`AUTOTUNE_SUCCESS_COUNT`). At the roll and pitch filter, 40 Hz, gyro
  noise on a 0.5 rad/s step reads as overshoot and bounce-back: only `racer` yaw P
  0.50–0.55 met the spec on all 5 noise seeds, below which a step doesn't settle in its
  window and above which it bounces back past 10%. The searches landed at 0.47–0.61, and on
  seed 2 a verify that bounced back 11% kept the start (P 1.5, overshooting 15%). At 10 Hz,
  0.40–1.05 meets the spec; on 2 passing pairs, seed 5 still passed P 1.23 and its result
  (P 1.11) overshot 5.5%; on 3, every seed lands within the spec. Rejected: 3 pairs at 40 Hz (P 0.41–0.44 on two seeds, unsettled); 10 Hz
  with a 0.8 back-off (yaw steps then miss the rise target on 5 rows, 4 of them
  `heavy_quad`'s); 4 pairs (`quad_x`'s seed 3 from the ×3 start takes 359 s, past the
  budget); 20 Hz with 3 pairs, which passes too (the clean `racer` at P 0.97, 3.7%
  overshoot at 40 Hz, against 7.4% for 10 Hz's P 1.18) but has no source.
  The rows take 6 s longer on average, at most 32 s (#604).
* Under a trim, a rate step overshoots more one way than the other until the rate I holds
  the whole trim: the angle loop holds the rest through its rate setpoint, which the step
  replaces. The I term takes over with a time constant of P/I, 5 s at `RATE_I_PER_P`.
  After the sim's 15 s hover, the tuned `quad_x` pitch rate steps under the combined
  perturbations overshoot 5.4% one way and 1.2% the other; after 30 s, 3.3% and 3.7%,
  about the same as after 60 s. The search judges a pair on its two steps' means, which
  cancels the bias. The sim spec steps a trimmed plant after a 30 s hover (#675).
* Moving the angle loop's share into the rate I as a rate step starts was rejected. An
  autotune step starts while the angle loop still levels from the last one, asking up to
  0.22 rad/s against a 0.05 rad/s trim share, so the transfer failed 37 of 49 robustness
  tests. Low-passed over 1 s, it lowered `quad_x`'s pitch rate P 18% and left `racer`'s
  steps overshooting the other way (3.5–4.1%).
* The trim rows pass, with the rate I term limited in moment (*Retuning `i` is bumpless* in [config ownership](config.md#config-ownership), #374).
  `racer`'s ×3 start breaches the rate envelope in its first fit with 25 ms of delay: the
  abort is the safe outcome, and the row stays ignored.

### Results

* *Why search rather than trust the design:* on QuadCopterX in RealFlight the fits scatter
  (acceleration −74 to 98 rad/s² on one axis within a run, delay 1–10 ticks, against no
  such spread on the sim), so the design alone left a loop slower than its start or put
  back to it in 4 of 7 runs: roll rate designed at P 0.39–1.04 (0.39 rose in 149 ms, the
  hand tune in 98–119 ms), and angle P at its ceiling overshooting 0.06 in 3 (#518). The
  scatter isn't the host's timing: every fit's 3 s held all 750 ticks, and the bad fits had
  no more late ticks than the good ones. The search keeps only gains whose steps passed, so
  a bad fit costs bracket points rather than a bad tune.
* On the sim (#518's sweep), every clean, 10 ms, and 25 ms row of the three airframes from
  both starts passed in 152–380 s (`robustness`); in the same sweep the search from the
  start gains took 308–693 s on the 12 rows it passes and failed the 6 high starts with
  delay to settle. With the yaw search the passing rows take 202–280 s (#296).
* The design's D (D/P at the fitted lag) holds: on the clean plant from core's defaults,
  `quad_x` and `heavy_quad` rise in 52 and 48 ms against the fastest pairs' 48 and 44 ms
  (`autotune`'s rise tests, within 25%; #531). Rejected (#531): the rate search scaling D
  with P (`racer` 24 ms, but `quad_x` 76 ms and `heavy_quad` 112 ms, and 16 of 42
  `robustness` rows against 21); a search maximum of 6 (16 rows); D at 0.5 or 0.7 of the
  lag, with or without that scaling (12–20 rows, `quad_x` 52–72 ms, `heavy_quad` 64–92 ms);
  and a 55° rate margin (18 rows, `quad_x` 72 ms).
* `racer` flies its gyro low-pass at 75 Hz, Mission Planner's initial `INS_GYRO_FILTER`
  for 5-inch props (`ConfigInitialParams`): at core's 40 Hz the filter's lag is a large
  share of its loop's, and its rate search stopped at P 0.55, rising in 28 ms against the
  fastest pair's 20 ms. At 75 Hz it rises within one 4 ms tick of the fastest pair; 80 Hz
  and up loses its noise row's time budget (#552).

### Rejected

* Rejected, from the same sweep
  ([`autotune_methods`](../../crates/stable-flight-sim/tests/autotune_methods.rs)): PX4's pole
  placement (`pid_design::computePidGmvc`) assumes one tick of delay and found no design on any
  10 or 25 ms row; a gain search on the model passes gains the model can't predict (4 misses,
  or none found); ArduPilot's QuikTune ramp (`VTOL-quicktune.lua`) needs excitation the
  clean sim lacks, then missed the spec on some rows at every slew threshold from 5 to 80
  (too soft at 5, rate P 0.1–0.3; too stiff at 80, `quad_x` P 5.6), couldn't lower a ×3
  start more than its 20%, and took 26–239 s for the rate loops alone. Also rejected (#518):
  the design kept after a check of two step pairs on it (the loop shaping alone; 77–109 s on
  the sim, unreliable in RealFlight as above); refitting until two designs agree within 25%
  (in RealFlight it settled on a pair of bad fits, roll rate P 0.39); verifying each loop
  right after its design (3 of the 6 rows with 25 ms of delay failed to settle); searching
  the rate loops before designing the angle loops on them (6 of 18 rows failed:
  `AngleUnstable`, `NotSettled`); verifying the angle designs instead of searching them
  (angle P stayed at 9, and `quad_x`'s angle steps clipped with 10 ms of delay).
* Rejected: relay feedback (Åström–Hägglund) and its tuning rules, which seeded the search
  from the start gains with D and a P cap (`0.6·Ku`). On QuadCopterX it couldn't measure yaw
  rate within the yaw authority left at hover, and on the angle loops it read an ultimate
  gain near 60 (an integrator around a fast rate loop) whose rule P oscillated once the rate
  setpoint saturated (#295); its D held the rate search's P below what `heavy_quad` and
  `racer` support (#378).
