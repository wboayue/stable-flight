## Autotune

PID autotune tunes a fresh airframe from a bad or unflyable start without a pilot nursing it.
The copter default (`StagePlan::copter_design_search`) fits each rate axis in hover, designs
its gains from the fit, then searches each loop's gain by steps from the design. Relay
feedback and a search from the start gains were rejected (**Design search**, below).

* **Probe and host** (`tuning::Probe`; `Tunable<L>`, implemented by `CopterStabilizer` for
  its `TunedLoop`s and `PlaneStabilizer` for its `PlaneLoop`s): a probe
  excites and measures one loop *inside* the tick on the fresh estimate, since a between-ticks
  driver would add a tick of delay to what it measures. Each tick the host hands the probe the
  loop's setpoint, error, whether its path clipped last tick, and its axis's body rate and last
  moment command (`LoopSignals`), before the loop's controller runs; the probe answers with the
  setpoint the controller runs on instead (a step, the identification's square wave). Every
  controller keeps running. The host owns the loop and the
  handoff; the probe owns the signal and the measurement. The trait is generic over the loop
  type, so a VTOL stabilizer hosts both sides' probes through one contract
  (`Tunable<TunedLoop>` and `Tunable<PlaneLoop>`). A probe flies only while its side flies
  alone: the other side's setpoint or a transition (a quadplane's blend, a tailsitter's ramp)
  interrupts it, since a handoff isn't a loop to measure.
* **Step response** (`tuning::StepProbe`): holds the setpoint at the starting measurement plus
  the step for a fixed window, so steps compare on equal terms, and reports `StepMetrics`
  normalized to the step (overshoot, 10–90% rise, settling, bounce-back after the peak, final
  value) from the low-passed response, plus the time the loop's path was clipped: a clipped step
  measures the limit, not the loop. A rate step is short, since a held rate tilts the vehicle.
  The rate PID's D acts on the measured rate, so a step doesn't kick the moment; a step test
  still allows a few clipped ticks (`StepTestPlan::max_saturated_s`). `StepSummary` keeps means and
  ranges over steps, and overshoot per direction, whose difference shows a trim or CG offset.
* **Sides** (`Supervisor<K: Side>`): the executor below is one state machine for every set of
  loops; a `Side` names its loops, gains, config, methods, envelope, and procedures.
  `Autotune` is the copter's (`CopterSide`), `PlaneAutotune` the plane's (`PlaneSide`). A
  stabilizer whose config holds a side's (`SideConfig`: a VTOL's two) is tuned through it, the
  rest of its config untouched. Rejected: a separate plane supervisor, which would repeat the
  settle, probe, candidate, stall, and recovery logic (about 600 lines) and let the two drift
  on the subtle rules (a breach after a freeze, stale states).
* **Procedures and executor** (`tuning::autotune`, between ticks): each stage's `Method`
  (`StepTest`, `Search`, `Design`) becomes a procedure, a pure state machine fed the stage's start
  and each probe's `Measurement` (or abort), which answers "run this probe" (optionally on
  candidate gains), "finish with these gains", or "fail". It never touches the flight loop, so its
  decisions are tested by feeding it inputs. `Autotune` executes it: it settles before each
  probe, runs it, guards the envelope, and commits. A step clipped too long, or one during which
  the host froze (`Autotune::note_stall`, e.g. a tick over 50 ms after the last by sitl's
  `RunStats::elapsed`), runs again, within a budget of steps in a row (3 clipped, 10 stalled);
  so does a frozen probe's abort, and for `Envelope::recovery_s` after a freeze breaches don't
  count: the vehicle moved while the host saw nothing (RealFlight returned one state for
  0.47 s and the vehicle came back at −19° pitch, #524). The runner's resyncs
  (`RunStats::skipped_ticks`) aren't freezes: at about 223 Hz it falls 100 ms behind every few
  seconds with no tick late by much. While the host runs slow for a stretch
  (`Autotune::set_host_slow`, e.g. sitl's `RunStats::host_slow`), the running probe stops at
  once and counts as stalled, and settling waits without timing out: every tick gets the
  nominal `dt`, so at half rate a step held for its ticks tilts twice as far, and RealFlight's
  slow stretches outlast the settle timeout (#438). A design stage's fit and hold keep running
  through a slow stretch: RealFlight's slow stretches at 222–225 Hz left 1.1–3.3 s between them, too short
  for a settle and a 3 s fit, and a fit with the physics 12% ahead of the ticks designs rate P
  5–13% lower on the sim, which the search climbs from (#524). While the vehicle's state is
  stale (`Autotune::set_state_stale`: a simulator link down awaiting a reconnect, the state
  held while the vehicle flies on unseen), every tick is a freeze: any running probe, fit
  included, stops as stalled, settles and holds wait, and the jump when the state clears is
  excused for `Envelope::recovery_s` from there. Past `Envelope::max_stale_s` (2 s) the stage
  fails `HostFault`: RealFlight's reconnects come 0.5, 1.5, and 3.5 s into an outage, and a
  QuadCopterX unseen for 3.5 s mid-step rolled over on the link's last inputs (#309). Stale
  time sums the nominal `dt`, so a reconnect attempt that blocks its tick (up to the 250 ms
  connect timeout) reads short. The core gets only the verdicts (stalled, slow, stale), so it
  stays clock-free. Candidate gains go in once
  the vehicle has settled on the committed gains and come out when the probe ends, so a
  candidate flies only its step.
* **Gain search** (`Method::Search`): the largest gain of one term whose step pairs meet a
  criterion, assuming the response worsens as the gain grows. A baseline on the start gains, then a
  bracket (`×1.3` per pair) and bisection to 10%, each point one `±` pair judged on its means
  (trim doesn't bias it). The result, backed off 10% (as ArduPilot does), is verified with fresh
  pairs and kept only if it flies no worse than the baseline (`Acceptance`), when the baseline
  met the criterion; otherwise it need only meet the criterion. A too-high start fails the
  criterion yet settles fast, overshooting within the settle band, so holding the result to
  it would keep the high gain (#361). A result that misses the criterion too is still kept if
  it misses by no more than that start, on each metric the criterion reads (overshoot,
  bounce-back, and for an angle loop how far steps end past the step): a verify that just
  misses can still meet the step spec where the failing start doesn't. On a yaw plan at
  40 Hz with 2 pairs under gyro noise, `racer`'s seed 2 verified P 0.61 bouncing back 11%,
  within the spec, against its start's P 1.5 overshooting 15%. Searching on below a failed
  verify met the spec too, but took 20–25 s more, which the ×3 rows (up to 343 s of 350)
  can't spare. No row of the current robustness table reaches the case (#680). A start above the search's maximum isn't flown: it's
  lowered to the maximum and committed at once, so the vehicle settles on it and it stays if
  nothing better passes or the stage fails, since a gain that high can ring once disturbed and
  never settle (#375). A start whose steps keep clipping leaves no baseline rather than ending
  the stage. Each loop's search starts from its design (`SearchPlan::copter`):
  * Rate: D as designed, P searched on overshoot ≤ 0.05 and
    bounce-back ≤ 0.1 (ArduPilot's default aggressiveness). Rate P has a sharp limit
    (QuadCopterX: P 1.14 flew clean, 1.22 overshot 0.18; #298), so a bracket point passes only on
    two pairs in a row (ArduPilot's `ignore_next`). Above the start
    the candidates' moment demand clips before overshoot binds, so the search allows clipped
    ticks rather than treating clipping as failure.
  * Angle: P searched on overshoot and bounce-back
    ≤ 0.05 (ArduPilot's angle-P limit); bounce-back counts even when a disturbance holds the angle
    short of the step (about 8% on QuadCopterX). Overshoot counts from where each step ends
    (`Criterion::Overshoot::settled_past`), and no step may end past the settle band (0.1),
    so a disturbance holding the angle past the step doesn't count either. ArduPilot judges its
    angle-P peak against the target (`AC_AutoTune_Multi::updating_angle_p_up`). Judged that way,
    every Titan Cobra pitch P from 6.5 down to 1.7 failed: each ended 5–7% past the step without
    ringing (bounce-back 0.00–0.01). The search walked P down to 1.3, the position loop then
    swung pitch ±15°, and the next stage timed out (#564). Rate steps keep judging from the
    step: their 0.25 s window can end mid-rise (yaw rises in about 0.6 s), where the end isn't a
    settled value, and judged from it the sim's yaw search took a P that overshot its 1 s
    test.
  * Yaw rate (`SearchPlan::copter_yaw_rate`) isn't designed: the design's yaw P (5–8 on the
    sim's quads) clipped 0.5 rad/s steps for 44–116 ms, and the search from its D
    (0.22–0.25) left steps short of 90% in the window. It's searched from the start as rate
    is, but on 0.5 rad/s steps with D 0 (ArduPilot's `ATC_RAT_YAW_D` and PX4's
    `MC_YAWRATE_D` default to 0). Prop drag gives a quad little yaw authority, so 1 rad/s
    steps clip at any P that rises within PX4's 0.2 s; on the sim they clipped 4–10× as long
    and settled on lower P. A yaw trim (QuadCopterX hovers on 0.40 of yaw moment, #192) takes
    authority from one direction, so the step against it rises slower and the one with it
    overshoots: the pair's means judge it. QuadCopterX has half `quad_x`'s yaw authority, so
    its steps reach only about half the step in the 0.25 s window and its search stops on
    clipping, not on the criterion; on a sim plant fitted to it (RealFlight's turns and steps
    alike) that bound lands on P 1.11–1.15 from every start, overshooting at most 0.03.
    Longer search windows were rejected: at 1 s a start with I as high as P overshoots on the
    I alone and the search falls to P 0.51 (0.5 s does the same on `heavy_quad`). The yaw
    step test watches 1 s instead (`COPTER_YAW_RATE_TEST_STEP`), so the slow rise and settle
    show (#558).

    The yaw I follows P as on roll and pitch (**Sequence**), at a larger share
    (`YAW_RATE_I_PER_P`, 0.3/s): after liftoff it rebuilds the trim in 4.9 s on the sim's
    QuadCopterX yaw, against 7.3 s at 0.2, at the same tuned P; at 0.4–0.5 the tuned steps
    overshoot past the spec (#558). The yaw integral has its own limit
    (`BodyRateControllerConfig::yaw_max_integral`, 0.5 of moment, ArduPilot's yaw IMAX;
    PX4's 0.3, `MC_YR_INT_LIM`, is short of 0.40); a limit of the I × 1 rad held 0.40 only
    from P 2, and the search kept its start on every 0.40-trim row. Keeping the start's I
    instead was rejected: a high I makes small P overshoot on the I alone, the bracket stops
    there, and the result depends on the start (0.35 against 0.91 on `racer`). A start whose
    I builds the trim slowly fails the same way: `racer`'s own yaw gains (P 1.5, I 0.1)
    settle with part of a 0.40 trim on P, and every lower candidate drifts with the trim
    (#296).
  * D isn't searched the way ArduPilot does (raise D until the response bounces back): with the
    rate I in the loop, overshoot decays without falling back, so bounce-back never bounds D.
  * An envelope breach while a candidate flies fails that candidate, not the run: the committed
    gains go back in and get `Envelope::recovery_s` to recover.
* **Sequence:** innermost first, so each stage flies on the gains the earlier ones found. A stage
  tunes only its own terms; the rest keep their gains. No stage searches the rate loops' I,
  which holds trim rather than shaping the response: it follows P, at `RATE_I_PER_P` (0.2/s;
  yaw at a larger share, under *Yaw rate*) of every rate P a copter search or design sets (`SearchPlan::i_per_p`,
  `DesignPlan::rate_i_per_p`; a search's baseline flies the start's I, so from a start off
  the ratio a candidate differs in I as well as P), as ArduPilot's autotune sets I from P
  (`AUTOTUNE_PI_RATIO_FINAL`, 1.0). On the sim robustness table 1.0 failed most rows, 0.1–0.3
  passed every clean and delayed row, and 0.2 the most trim rows; switching between a low I
  during probes and the final I between them (ArduPilot's `AUTOTUNE_PI_RATIO_FOR_TESTING`)
  bumped the I term at each switch and lost the trim rows (#374). The large-aircraft plan keeps
  the start's I: on the Alia (no rate D) it overshot the rate steps by 10%. Heading isn't
  tuned. Before each probe the vehicle must hold level and still (`Envelope::settle`): roll
  and pitch within the tilt of the attitude target guidance holds (`FlightLoop::setpoint`),
  as ArduPilot's level check compares with the desired attitude, a position hold's lean
  included. Measured from level, a hover leaning to hold its position never settles: the
  eXplora in RealFlight leans 3.4° in calm air, its elevons holding pitch in the propwash,
  past the step settle's 2.5° (#539). The vehicle must also stand still over the ground
  (0.2 m/s): our position hold leans further and faster than ArduPilot's (whose lean grows
  0.5° per meter, capped at 10°), and a step started while it leans back toward its position
  measures the return too; under a CG offset the sim `quad_x`'s rate searches kept nothing.
  A lean cap alone (10°) still let those steps start. The envelope still reads tilt from
  level, a safety limit whatever guidance asks. Body rates must be within the settle's rate
  limit, read through the stabilizer's gyro low-pass (raw, the sim's gyro noise held them over the settle, so no stage started; the
  envelope reads the raw rates, so a breach shows on its first tick). The yaw rate counts only
  for yaw rate and heading stages: on `racer` with 25 ms of delay the yaw loop rings at ±0.18
  rad/s on its start gains without tilting the vehicle, which would hold up every roll and
  pitch stage (#518); ArduPilot's level check covers every axis (`AC_AutoTune::currently_level`,
  yaw within `AUTOTUNE_LEVEL_RATE_Y_CD`, 7.5°/s), and widens its limits up to twice as the
  wait grows, which here started about half
  the probes before the vehicle was still and made angle steps clip. An identification that
  can't settle starts at the timeout instead of failing: the fit only gains excitation from
  the ringing. A step can't, so a loop the run hasn't reached yet that rings on its start
  gains would fail the stage; the design search designs every loop before it steps any. A finished stage commits its gains through
  `set_stabilizer_config`.
* **Envelope and outcomes:** tilt, body rate, or altitude loss past the `Envelope` limits, a
  probe abort, or a rate setpoint interrupting an angle stage fails the run, and the config goes
  back to the last kept stage: earlier stages stay, and nothing the failing stage tried
  survives except gains it lowered from above its search's maximum. A design isn't kept until
  its loop's search finishes, so a failure anywhere before then (a design error, a breach on
  the design during its search's baseline) puts that loop's earlier gains back; each other
  stage keeps only its own loop's gains. A search that keeps no result keeps its start only
  if the start's steps met the criterion, else the run's start: a design that never passed
  doesn't outlive its search.
  Disarming before completion reverts to the config at start. The caller owns the trigger and the
  logging (`AutotuneEvent`s); the executor has no clock. The envelope and settle read tilt and
  per-axis rates in the frame the probed loops fly (`Tunable::probe_frame`): a `View`'s, so a
  tailsitter hovering nose up reads level and a roll stage waits on view roll (airplane yaw).
  The stabilizer reports its frame rather than the caller passing it to `Autotune`, so the
  two can't disagree: the frame comes from whichever stabilizer hosts the probes.
* **Robustness:** the claim is that the method, not one tune, transfers to hardware, so the sim
  checks it on a table of plants (airframes × delay, gyro noise, trim moments) from a low and a
  high start, each result held to the published step targets on its own plant
  ([`robustness`](../../crates/stable-flight-sim/README.md#closed-loop-tests)). Rows that fail stay
  ignored on the issue blocking them rather than loosening the targets.
* **Design search** (`StagePlan::copter_design_search`, the copter default): fit, design,
  then search; see [Design search](autotune-design-search.md#design-search) below.
