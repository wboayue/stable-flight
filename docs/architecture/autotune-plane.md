## Plane autotune

* **Plane side** (`PlaneAutotune`): probes on the plane's roll and pitch rate and angle loops,
  within a `PlaneEnvelope` (bank, pitch, rate, sink, and an airspeed band from 0.9 × the
  stabilizer's minimum to its maximum). A breach puts the committed gains back; the host then
  recovers (`PlaneTuneHold::recover`): a VTOL hovers on its rotors, the airplane flies wings
  level.
  * **The hold** (`PlaneTuneHold`): probes fly in attitude hold, as a pilot flies
    ArduPilot's AUTOTUNE in FBWA, the pitch trimmed slowly toward trim airspeed and the thrust
    toward the starting height: without the pitch trim the airplane slowed past its minimum
    within a few steps, and steps from a line held by the energy control disagreed with the
    spec on pitch, since the control answers a pitch step with its own (#533); without the
    thrust trim the Alia 250 sank 0.2 m/s and left the envelope within 3 minutes (#534).
    Between probes, past a radius and flying away, the hold turns back toward its start until
    it heads there or has turned around (a start inside the turn's circle stays abeam), and
    the host holds probes off meanwhile (`Supervisor::set_maneuvering`), so no step starts
    mid-turn.
  * **The settle** waits for the airspeed too (`AirspeedSettle`): within 1 m/s of trim and
    changing by under 0.2 m/s² (low-passed over 1 s) for the settle's hold. A wing in an
    attitude hold rides its [phugoid](../../docs/glossary.md) with its attitude and rates still,
    and steps started on its swing (±1.7 m/s on the airplane) read 0–2% overshoot where the
    same gains overshot 4–5% from steady flight: the check passed gains the spec failed, and
    the Alia's pitch missed on every row until the settle waited (#534).
  * **Steps** (`StepTestPlan::plane`): rate steps 8°/s for 1 s (8° of rotation), attitude
    steps 30° of bank and 10° of pitch, each capped at the stabilizer's limit, since the probe
    replaces the saturated setpoint (the Alia 250's 20° bank limit). Only the surfaces at full
    throw count as a clip: an angle step's rate target at the attitude rate limit is part of
    the loop it measures.
* **Plane method** (`PlaneMethod::Design`, `StagePlan::plane_design`): each axis is fitted
  to a physical model in flight, its gains designed on the model, and the result checked on
  attitude steps. Each method in the comparison excited the aircraft itself, with no pilot
  (#533).
  * **Model** (`WingIdentifyProbe`, `WingModel`): the identification's square wave on the
    rate setpoint, 6 s, fitted to how the surfaces turn the aircraft: a roll rate dies away
    against the wing's damping, and a pitch rate also meets its
    [stiffness](../../docs/glossary.md), which grows with the angle of attack α. The fit is: rate
    acceleration = stiffness × α + damping × rate + control × moment, the moment through a
    fitted surface lag and delay, every second tick (9 KB). FF is damping over control: the
    moment that holds a rate. α is the pitch less the climb angle through the air (climb over
    airspeed): from the ground track, an 8 m/s crosswind halved the eXplora's ground speed
    against its airspeed, its fit lost the stiffness, and all its wind rows failed (#534). A
    stiff wing's α follows its rate, so stiffness and damping trade off in the fit with
    little change in its response.
  * **Design** (`WingDesign`): the spec's attitude step flown on the fitted model, a candidate
    passing when it overshoots by no more than 5%, settles, rises within 2.2τ, and clips no
    longer than a step may. τ is bisected to the quickest at which some P and I on a 16 × 12
    grid pass, raised 15%, and the passing cell farthest from a failing one taken; the moment
    is held 20% short of full throw, and I must hold the fit's trim within half the integral
    limit (the airplane's pitch at I 0.25 couldn't, sat 1.75° off its hold, and its stage
    timed out settling). About 1,350 candidates, four per update. A stiff axis fitted with
    damping at or below 0 is designed undamped, with no FF: the trade-off in the fit carries a
    barely damped pitch past 0 under a high P (the sim eXplora's from 1.63/s at P 0.008 to
    −0.62/s at 0.24; RealFlight's from ×3 gains to −7.6/s), and the design on the floored model
    meets the spec from ×3 to ×5 gains on the sim and from ×3 in RealFlight. Identifying on
    lowered gains was rejected: the bias is already there at the eXplora's own gains, and a
    probe can't tell a start is too high (#618).
  * **Check:** attitude pairs on the candidate, each step held on it first so the rate loop's
    integral holds what it needs (from ×3 gains the committed loop's I term, carried over,
    slowed every second step 3.6 s against 1.1 s). A sag raises I by half, a slow rise P a
    grid step (a slower τ rose slower still: 2.0τ to 3.5τ from 0.56 to 0.85 s), an overshoot
    or clip τ by 15%; eight pairs, then the stage fails. An overshoot that keeps more than
    half of itself to the step's end holds past the target rather than swinging past it, and
    raises I: a wing that rolls into its turns needs a steady aileron in a bank
    the fit from level flight doesn't see, and a slower τ only widens the error that holds it
    (RealFlight's Titan Cobra ended 30° steps 1.09–1.13 past; the sim quadplane at four times
    its yaw-rate roll moment, 1.09, tunes in three pairs). An overshoot that swings back but
    still ends more than half the limit past is part of each, and raises I and τ together: τ
    alone took the swing out and left the offset (the Cobra at 14 m/s, 1.02–1.04; the sim at six
    times, 1.03, #635). Holding or ending past doubles I where a sag raises it by half: the
    offset is a moment only I supplies, and at a sag's rate the X-Plane Alia 250's I climbed
    from 0.01 to 0.85 through all eight pairs and its 20° steps still ended 1.09–1.22 past;
    the sim Alia, its yaw-rate roll moment fitted to X-Plane's held aileron (#458), tunes from
    every start by the seventh, and at 1.5 doesn't (#537).
  * On the sim it tunes the airplane, the eXplora, the Titan Cobra, and the Alia 250 to the
    plane spec (`tests/common/plane_spec.rs`) from their own gains, a third, and three times
    them: 42–60 s on the airplane and eXplora, 70–90 s on the Cobra, 434–512 s on the Alia,
    whose 567 m turns and 1.5 s roll time constant stretch every settle (#534). Model design
    with a speed term in the fit (from the identification or from held pitch steps) met the
    spec no more often than without (15 and 12 of 32 rows against 12) before the settle and
    α fixes, which brought the plain fit to 24 (#534).
  * Rejected (#533):
    * The copter design search (rate P searched on rate steps). A plane's rate steps bound
      nothing: roll passes at any P until a 30° attitude step clips the surfaces, and pitch
      never reaches 90% because the stiffness holds the rate back. Judged on attitude steps
      instead, it met the spec on 14 rows, but 8 of them only after its pitch search breached
      the envelope and the eXplora recovered to a hover. Its FF, one over the fit's static
      gain, folds the stiffness in (plane pitch 0.60 against 0.46).
    * PX4's design (`fw_autotune_attitude_control`, `pid_design`'s `computePidGmvc`) gives
      rate P 0.14–0.2 on every airframe, since it designs on the model scaled to unit gain
      and applies the gain unscaled. Its attitude rule (60° of error gives full output) sets
      τ at 0.9–1.0 s, which misses rise and settling: 4 rows, though in 11 s.
    * ArduPilot's (`AP_AutoTune`, our own steps for the pilot's) raises P until its output's
      slew rate swings past `RATE_SMAX`, then keeps 35%: still 3–11× the hand tunes, so
      30° roll steps clip. Its `AUTOTUNE_LEVEL` pitch τ (0.75 s) misses rise: 4 rows, 4
      recoveries.
    * A step search from the start meets the spec only from the airframe's own gains (10
      rows).
  * The model holds airspeed constant, so it under-asks for pitch I; the check's I raises
    supply the rest. A full grid of P, I, and τ on the model, about 10⁷ model ticks, takes
    seconds on the host, too long between ticks on a flight controller, so the design bisects
    τ and searches P and I at it (#534).
