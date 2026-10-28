# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- `IdentifyProbe` fits only delays whose moment pushes the axis the right way, falling back
  to all when none does (#529). In RealFlight, noise had picked delays with negative moment
  effects that the design can't use: 10 of 67 QuadCopterX fits read under 20 rad/s², 5
  replayed with the filter.
- A design stage's fit and hold run through a slow host (`Autotune::set_host_slow`) instead of
  stopping (#524): on RealFlight they stopped at every dip to 222–225 Hz and failed the stage
  `Stalled`; a fit 12% slow designs rate P 5–13% lower on the sim. `Autotune::note_stall`
  marks a freeze: breaches for `Envelope::recovery_s` after it don't count, and a probe that
  aborts across it runs again.

### Added

- `StagePlan::copter_design_search` (#518): the copter autotune as a step search from a
  loop-shaping design. Roll and pitch rate, then angle, each fitted and designed
  (`Method::Design`, `DesignPlan`), then each loop's P searched by steps from its design
  (`SearchPlan::copter`). On the sim it tunes every copter airframe, clean
  and with 10 and 25 ms of actuator delay, from low and ×3 starts, in 152–380 s of simulated
  time, against the step search's 308–693 s on the rows that search passes.
  `AutotuneAbort::Design` reports a fit the design can't use, and `Outcome::Designed` a
  design stage's gains, which a failure puts back until the loop's search finishes.
  A design flies only once the vehicle holds the settle about its axis on it; one that
  rings (`AutotuneEvent::Unsettled`) or breaches the envelope is fitted again.
  `TunedLoop::inner` and
  `outer` name the rate loop under an angle loop and the angle loop over a rate loop.
- `tuning::design_rate` and `design_angle_p` (`LoopShapingConfig`, preset
  `autotune::COPTER_LOOP_SHAPING`) (#517): rate and angle gains from an identified
  `RateModel` by loop shaping. Rate D/P at the fitted motor lag and P for a 65° phase margin;
  angle P for a 65° margin on the angle loop, within the angle search's range. On the sim the
  designed gains meet the step spec on every copter airframe, clean and with 10 and 25 ms of
  actuator delay. `RateModel::response` gives the model's frequency response.

- `tuning::IdentifyProbe` (`ProbeConfig::Identify`, preset `autotune::COPTER_IDENTIFY`) (#516):
  fits a rate axis's `RateModel` (motor lag, delay, angular acceleration per unit moment) from
  a square wave on its angle loop's setpoint, every loop closed, in `f32` and fixed memory. On
  the sim, 3 s per axis recovers each copter's motor lag within 1% and its actuator delay, from
  low and ×3 starting gains.

- `TableMixer::config()`: the thrust factor and yaw ratio a mixer scales by, so a copter
  wiring can mix a quadplane's lift rotors at its config (#438).
- `Autotune::set_host_slow` (#438): while the host runs slow (its ticks further apart than
  their `dt`), the running probe stops at once and runs again later, counted as stalled, and
  settling waits without timing out. RealFlight runs at about 120 Hz for minute-long stretches,
  where a 63-tick rate step held for its ticks tilted twice as far (an autotune failed on
  `Tilt`) and settles timed out (`NotSettled`).
- `AutotuneEvent::Stalled` reports a probe the host's timing spoiled: stopped by
  `set_host_slow` (no measurement) or finished across a `note_stall` (its untimely
  measurement).

- `LateralControllerConfig::brake_mps2` (`GUID_LAT_BRK`, default 0: off): with shaping off
  (`GUID_LAT_ACC` 0), the position loop slows toward a waypoint along the braking curve at
  this deceleration instead of in proportion to the distance, which starts braking only
  `2 · v` out (#488). On the sim, uncapped 300 m quadplane legs that overshot 12–20 m stop
  within 1.9 m at 1.5 m/s².

- `WaypointFollower::with_wait_for_altitude`/`set_wait_for_altitude` (off by default), for
  wing-borne legs (#451): a waypoint reached or passed outside the altitude acceptance is
  emitted without a velocity until within it, then advanced past, so plane guidance loiters
  around it (PX4's fixed-wing behavior) instead of flying the line through it on and away. On
  the sim, a leg climbing steeper than `airframes::plane()` can stays within 52 m of its
  waypoint until at altitude, where it flew 390 m past it.

- `WaypointFollower::route_end` reports what the follower does after the last waypoint.

- `TecsConfig::pitch_integrator_gain` (`PGUID_PTCH_INTEG`, default 0.3): the pitch loop's
  integral gain, apart from the thrust loop's, as PX4 splits `FW_T_I_GAIN_PIT` and
  `FW_T_THR_INTEG` (#503).

- **Quadplane back transition deceleration** (#496): `LiftTransitionConfig` gains
  `back_decel_mps2` (`TRAN_VT_DEC`, 2 m/s²) and `back_decel_i` (`TRAN_VT_DEC_I`, 0.1),
  PX4's `VT_B_DEC_MSS` and `VT_B_DEC_I`. `QuadPlaneGuidance`'s back transition pitches the
  nose up, on both sides, while it slows less than the target, at most 0.3 rad; gain 0 keeps
  it level as before. On the sim the Alia's back transition from 45 m/s ends on speed in
  21 s, where at pitch 0 it reached its 28 s timeout at 17.7 m/s.

- **VTOL fallback below a plane minimum height** (#450): `TransitionConfig` and
  `LiftTransitionConfig` gain `plane_min_height_m` (`TRAN_FW_MIN_HGT`, off by default, as
  PX4's `VT_FW_MIN_ALT`). Plane flight below it falls back to copter through the back
  transition (a quadchute), reporting `ModeRefusal::BelowPlaneMinAltitude`, and a forward
  transition needs it to start. A quadplane's fallback gives its lift rotors all the lift at
  once, skipping the back ramp (#493). On the sim a plane leg sinking through 20 m hovers at
  most 3.3 m below it, and the Alia diving through it at 53 m/s 2.4 m below (7.0 m with the
  ramp); on `main` it flew into the ground.

- **Scripted maneuvers** (#342): `commanders::maneuver`.
  - A `Maneuver` is up to `MAX_SEGMENTS` (16) `Segment`s, `Copy` and fixed-size:
    `Segment::Rotate` holds a signed body rate about one `Axis` (`RateLaw::Constant`, or
    `RateLaw::Radius`, `V / R` each tick) until an angle has turned or a time has passed, at
    the maneuver's or the cruise `Thrust`; `Segment::Pause` holds zero rate. The angle is the
    attitude's twist about the axis, unwrapped, not Euler angles.
  - Catalog: `Maneuver::roll`, `Maneuver::loop_` (negative radius: outside),
    `Maneuver::doublet` (`+rate`, `−rate`, a `DOUBLET_PAUSE_S` pause, at cruise thrust: a step
    response in flight). `Maneuver::new` refuses out-of-range segments with `ManeuverError`.
  - `ManeuverRunner` flies one as `ControlInput::AbsoluteRate`, then a level
    `AbsoluteAttitude` recovery (`RecoveryConfig`), then hands back; `ManeuverConfig` sets the
    thrusts, the angle timeout, the path hold, and the recovery. `abort` recovers at once.
  - Each `Segment::Rotate` holds a `Path` (#481): a roll its line, a loop its circle, a
    doublet none (`Path::Free`). The runner steers back onto it with pitch and yaw rate, a PD
    on the error across the path (ArduPilot's scripted aerobatics, gains 2 and 2.8), plus
    half the lift a rolling wing loses on a line (`PathConfig`). On the sim, rolls lose at
    most 3.5 m (rates alone: 37 m) and loops 1.1 m (6.4 m).

- `PlaneRateControllerConfig::trim` (`PSTAB_RLL_TRIM`, `PSTAB_PIT_TRIM`, `PSTAB_YAW_TRIM`,
  default 0, #342): a per-axis moment added to the plane rate loop's output, scaled by the
  airspeed scale squared like the feedback (PX4's `TRIM_ROLL`/`_PITCH`/`_YAW`), in attitude
  and rate flight alike, so rate flight at zero rate holds the wing's steady moment.
  **Breaking:** a `PlaneRateControllerConfig` struct literal needs `trim` (or
  `..Default::default()`).

- `CopterGuidanceConfig::wind_heading_min_mps` (`GUID_WND_HDG_SPD`, default 0: off, #470):
  holding a waypoint or landing, a wind at least this fast, its direction known within 15° by
  the hover wind estimate, turns the heading to put the wingtip nearer the heading into the
  wind (the vehicle's `x` axis 80° off where it comes from, 10° short of square), at
  `heading_rate_radps`; a waypoint's
  `heading_rad` wins, and longer legs keep their heading until within
  `waypoint_yaw_min_leg_m` of the waypoint. **Breaking:** a `CopterGuidanceConfig` struct
  literal needs `wind_heading_min_mps` (or `..Default::default()`).

- **Hover wind estimate** (#473): `estimation::wind::{WindEstimator, WindConfig,
  WindEstimate}`.
  - It inverts a per-axis drag model (bluff-body `½ρv²/B` plus momentum `μv`, as ArduPilot's
    and PX4's drag fusion) to air velocity, and runs a two-state linear Kalman filter on world
    wind, each axis's noise scaled by the drag's slope.
  - `CopterGuidance` runs it while airborne, more than `AIRBORNE_HEIGHT_M` (1 m) above where
    it was primed on the ground, on ticks tilted no more than 45°. A tailsitter's hover view
    is its thrust frame. `TailsitterGuidance` and `QuadPlaneGuidance` pause it outside copter
    flight, and the quadplane while its pusher runs.
  - `CopterGuidanceConfig::wind` sets it: `GUID_WND_BCOEF_X/_Y`, `GUID_WND_MCOEF_X/_Y`,
    `GUID_WND_F_NSE`, `GUID_WND_P_NSE`. Off by default, as in both stacks.
  - `Guidance::wind` reports it (default `None`). `View`, `TailsitterGuidance` and
    `QuadPlaneGuidance` forward it, and `FlightLoop` puts it in `TelemetryRecord::wind`
    (and `RecordedTick::wind`).
  - `estimation::wind::specific_force_ned`: what an accelerometer reads, from the state.
  - **Breaking:** `TelemetryRecord` and `CopterGuidanceConfig` struct literals need `wind`.
- **Waypoint heading** (#470): `WaypointCommand::heading_rad`, a heading for copter
  waypoint flight to turn to at `CopterGuidanceConfig::heading_rate_radps` (`GUID_HDG_RATE`,
  default 60°/s, ArduPilot's waypoint yaw rate) instead of along the leg, never yielded; on a
  tailsitter it is the belly's direction. PX4 likewise flies a mission item's yaw over its yaw
  mode. **Breaking:** a `WaypointCommand` struct literal needs `heading_rad` (`None` keeps
  the old behavior), and a `CopterGuidanceConfig` literal `heading_rate_radps` (or
  `..Default::default()`).
- `view::tailsitter_rotation()` (#333): the tailsitter hover view's rotation, which
  `View::tailsitter` flies in, for showing hover attitude elsewhere (the bridge).

- **Waypoint yaw** (#452): copter waypoint flight turns the heading-hold target along each
  leg, from where the vehicle was when the waypoint began to the waypoint, at
  `CopterGuidanceConfig::waypoint_yaw_radps` (`GUID_YAW_SLEW`, default 60°/s, ArduPilot's
  and PX4's; 0 holds the heading), on legs longer than `waypoint_yaw_min_leg_m`
  (`GUID_YAW_MIN_LEG`, default 3 m), as ArduPilot's `WP_YAW_BEHAVIOR` and PX4's
  `MPC_YAW_MODE` face the next waypoint by default. Offboard position targets, flown as
  waypoints, turn too; land flight, sticks, and absolute attitudes' headings are unchanged.
  **Breaking:** on by default.
- **Heading yield** (#431): a waypoint heading stuck more than
  `CopterGuidanceConfig::heading_yield_rad` (`GUID_YAW_YLD_ERR`, default 30°) off its target,
  not yawing toward it, for `heading_yield_s` (`GUID_YAW_YLD_S`, default 1 s; 0: never) is
  yielded: the target follows the vehicle's heading until the waypoint changes, so a
  quadplane's fin turning it into the wind isn't fought. **Breaking:** a
  `CopterGuidanceConfig` struct literal needs the four new fields (or
  `..Default::default()`).
- **Off-nose speed** (#437): `guidance::vtol::OffNoseSpeed` (`OffNoseSpeedConfig`,
  `DEFAULT_OFF_NOSE_SPEED_MPS`) caps a quadplane's waypoint speed in copter flight while the
  nose is more than 20° off the bearing to the waypoint, as ArduPilot's quadplane
  (`get_scaled_wp_speed`): `speed_mps` (`OFFNOSE_SPD`, default 15 m/s; 0: off) divided by 1
  rising to 3 from 20° to 160°. `QuadPlaneGuidance` flies it on waypoints, so offboard
  position targets too. **Breaking:** on by default, and a `QuadPlaneGuidanceConfig` struct
  literal needs `off_nose` (or `..Default::default()`).
- `TableMixer::with_reversed_yaw()` and `MixRow::with_reversed_yaw()`: a preset for a model
  whose props spin the other way (the X-Plane Alia 250 on `quad_x` and `quadplane`, #351,
  #392); the mixer's servo rows keep their yaw.
- `TableMixer::quadplane_vtail()`: `quadplane` with two ruddervators in place of the elevator
  and rudder (pitch alike, yaw opposite ways); an A-tail flies it with its panels swapped (#433).
- **Lift+cruise guidance and stabilizer** (#391): `guidance::vtol::QuadPlaneGuidance`
  (`QuadPlaneGuidanceConfig`) flies a quadplane's hover on `CopterGuidance`, its plane flight
  on `PlaneGuidance`, and the transitions under a `LiftTransition` as a `Setpoint::Blend`
  (`BlendSetpoint`: hover and plane attitudes and the lift share). Forward, the hover side
  holds altitude while the pusher flies the plane guidance's thrust and both sides its pitch,
  within ArduPilot's transition limits; back, the pusher is cut. `stabilizers::vtol::
  QuadPlaneStabilizer` (`QuadPlaneStabilizerConfig`) flies hover setpoints on the lift
  rotors with the surfaces neutral, plane setpoints on the surfaces and pusher
  (`ControlOutput::cruise`), and a blend on both, the lift rotors weighted by their share.
  `LiftTransitionConfig` gains the pusher's `forward_thrust` and `pusher_slew_per_s`,
  `handoff_s`, and `roll_limit_rad` (`TRAN_FW_THR`, `TRAN_PSH_SLEW`, `TRAN_HANDOFF`,
  `TRAN_ROLL_MAX`).
- **Quadplane forward thrust** (#427): `guidance::vtol::ForwardThrust` (`ForwardThrustConfig`,
  `ForwardSplit`), ArduPilot's `Q_FWD_THR_GAIN`/`Q_FWD_PIT_LIM`: in copter flight on a
  waypoint or land, `QuadPlaneGuidance` holds nose-down pitch to the limit (default 3°) and
  flies the pusher on `gain · tan(pitch)` the hover asked for, as a `Setpoint::Blend` at lift
  share 1, so the wing isn't flown at negative α. `QuadPlaneGuidanceConfig::forward`; params
  `FWD_THR_GAIN` (default 0: off) and `FWD_PIT_LIM`, with metadata.
- **Lift+cruise transition** (#391): `guidance::vtol::LiftTransition` (`LiftTransitionConfig`,
  `LiftTransitionStep`), a quadplane's copter ⇄ plane state machine. Forward, the lift rotors
  hold hover until a valid airspeed reaches the gate (default 9 m/s), then their lift share
  ramps to 0 (5 s); without airspeed it times out to copter (15 s waiting). Back, the share ramps to 1
  (3 s), completing below a forward speed (5 m/s) or after 10 s. Params `TRAN_ASPD_MIN`,
  `TRAN_FW_RAMP`, `TRAN_TIMEOUT`, `TRAN_MIN_HGT`, `TRAN_VT_RAMP`, `TRAN_VT_SPD`,
  `TRAN_VT_TIMEOUT`, with metadata.
- **Lift+cruise mixing** (#391): `MixRow::pusher` (a forward motor flying the cruise thrust,
  not desaturated with the lift motors; `MixRow::is_pusher`) and the `TableMixer::quadplane()`
  preset (`quad_x` lift rotors on motors 0–3, pusher on 4, `plane` surfaces on servos 0–2).
  With `ControlOutput::cruise` set, servo rows fly its moments; without it, they fly
  `moments` as before and pushers stay stopped.
- Tailsitter back-transition sink guard (#343): `TransitionConfig::back_sink_limit_mps` and
  `back_sink_gain` (`TRAN_VT_SINK_LIM`, `TRAN_VT_SINK_GN`; default 1 m/s and 0.4 per m/s)
  raise the back ramp's thrust while it sinks, so a ramp too weak to hold the nose up doesn't
  stall and fall. Thrust saturation is flagged while it acts.
- `FlightLoop::vehicle()`, beside `commander()`, `guidance()`, and `stabilizer()` (#348).
- `TableMixer::plane()`: a conventional airplane preset (motor 0; servos 0 ailerons,
  1 elevator, 2 rudder), flown by the simulator's fixed-wing model (#348).
- `TableMixer::rows` and `MixRow::{roll, pitch, yaw}`: read a mixer table back, e.g. for a
  simulator that derives motor torques from it (#347).
- **Plane waypoint navigation** (#124): `PlaneGuidance` flies `Waypoint`s. It follows the
  leg's line with L1 (ArduPilot's lateral acceleration, from the ground velocity so a
  crosswind crab is flown), loiters clockwise around a waypoint with no leg, holds the
  waypoint's altitude with pitch (PI), and its `cruise_speed_mps` as airspeed with throttle
  (PI around cruise; cruise plus a climb share without airspeed). New `PlaneGuidanceConfig`
  fields and `PGUID_*` params.

- **Tailsitter wiring** (#124): `guidance::vtol::TailsitterGuidance` (hover guidance in the
  tailsitter view and `PlaneGuidance` under a `Transition`: the forward ramp on the hover side,
  the back ramp on the plane side, `Land` asks for copter, a roll past the limit cuts a
  transition short, a thrust blend at each handoff, and stick throttle held at cruise after
  the handoff into plane until the stick reaches it) and `stabilizers::vtol::TailsitterStabilizer`
  (each setpoint on its side, the incoming side primed). `Transition::cut_short`;
  `TransitionConfig` gains `forward_thrust`, `back_thrust`, `handoff_s`, `roll_limit_rad`;
  `Setpoint::with_thrust`.
- **Params** for the plane and transition configs (`PSTAB_*`, `PGUID_*`, `TRAN_*`, with
  metadata); the tailsitter composites chain their parts' tables.

- **Plane control** (#124): `stabilizers::plane::PlaneStabilizer` (PX4's fixed-wing
  structure: yaw-free attitude error over a time constant, level-turn feedforward on pitch and
  yaw rate faded past 70° bank, and `controllers::PlaneRateController`, a PI + feedforward rate
  loop scaled by airspeed with its integral held below minimum airspeed) and
  `guidance::plane::PlaneGuidance` (FBWA sticks, acro rate sticks, absolute attitude and rates,
  a wings-level placeholder for waypoints). New setpoints `Setpoint::Plane(PlaneSetpoint)` and
  `Setpoint::PlaneRate(BodyRateSetpoint)`; `PlaneRate` flies at any attitude (rolls, loops).
  `FlightModeReport::plane_only`; `ModeRefusal::RollError`.
- **Absolute rates** (#124): `ControlInput::AbsoluteRate(AbsoluteRateCommand)` (rad/s and
  thrust, a stream; `ControlMode::AbsoluteRate`) for maneuvers and offboard rates.
  `CopterGuidance` flies it as the new `Setpoint::BodyRate(BodyRateSetpoint)`, which
  `CopterStabilizer` tracks at absolute rates saturated to its full-scale rates.

- **`guidance::vtol`** (#181): `Transition`, the copter ⇄ plane state machine a VTOL
  guidance owns. It moves toward the requested `FlightRegime`, ramps a frame-free tilt target
  (the nose's angle from vertical) through `ToPlane`/`ToCopter`, completes on tilt (and a
  minimum airspeed when valid), flags the controller handoff, and refuses below a height
  above the arming point. A forward timeout falls back to copter until copter is requested;
  a back timeout completes. `ThrustHandoff` blends thrust across a handoff. Defaults follow
  ArduPilot's tailsitter (50°/s, 45°) and PX4's 15 s timeout. Not wired into a guidance
  yet (#124).

- **Flight regimes** (#181): `Command::regime` requests a `FlightRegime` (`Copter`, the
  default, or `Plane`) every tick, like `arm`; `Command::with_regime` sets it. Guidance owns
  the flown `FlightMode` (`Copter`, `Plane`, `ToPlane`, `ToCopter`) and reports it with an
  optional `ModeRefusal` as a `FlightModeReport` through `Guidance::flight_mode` (default
  `None`: copter only, recorded as `FlightModeReport::copter_only`, which refuses `Plane` as
  `Unsupported`). `TelemetryRecord::flight_mode` carries it; `View` forwards it.
- **`VehicleState::airspeed_mps`** (`Option<f32>`): speed relative to the air mass, `None`
  without air data; `valid_airspeed_mps` drops a NaN or negative reading. The conformance kit expects ground speed (still air) and passes a backend
  that reports `None`.

- **Autotune probes** (#149, #298): `Probe` excites and measures one loop inside the tick,
  fed the loop's `LoopSignals` and answering with the setpoint the loop runs on:
  `ProbeConfig::Step` (a `StepProbe`: holds a step for a fixed window and reports
  `StepMetrics`: overshoot, rise, settling, bounce-back, final value, clipped time;
  `StepSummary` aggregates them). It reports `ProbeStatus` and, when done, a `Measurement`. The `Tunable` trait (`start_probe`, `probe`,
  `stop_probe`), implemented by `CopterStabilizer` on a `TunedLoop` (roll/pitch/yaw rate,
  roll/pitch angle, or heading), hosts one; the loop is handed back, its I term kept and its
  derivative history cleared, when the probe finishes, on `stop_probe`, or when a rate tick interrupts an angle
  or heading probe. `prime` ends a probe. `FlightLoop` forwards the three calls, and adds
  `armed()`. `TunedLoop::gains`/`gains_mut` read or retune a loop's gains in a
  `CopterStabilizerConfig`.
- **`tuning::autotune::Autotune`** (#149, #298): executes a plan of stages between ticks
  (`StagePlan`: a `TunedLoop` and a `Method`; `Method::StepTest(StepTestPlan)` measures alternating steps and
  keeps the gains, repeating clipped or stalled steps (`Autotune::note_stall`) up to
  `max_invalid` and `max_stalled` in a row (`AutotuneAbort::Invalid`, `Stalled`);
  `Method::Search(SearchPlan)` finds the largest gain of one `Term` whose ± step pairs meet a
  `Criterion`, by bracketing and bisection from the start gains, and keeps it only if its
  verify flies no worse than a baseline on the start gains (`Acceptance`); rate P
  (`SearchPlan::copter_rate`, step overshoot) and angle P (`SearchPlan::copter_angle`, 5°
  steps: `COPTER_ANGLE_SEARCH_STEP`); a
  bracket point passes on `SearchPlan::confirm` pairs in a row; candidate gains fly only
  during their step, and an `Envelope` breach while they fly fails only the candidate (the
  committed gains get `Envelope::recovery_s` to recover); presets
  `COPTER_RATE_STEP`, `COPTER_ANGLE_STEP`, `COPTER_STEP_SETTLE`). Each stage's decisions run as a pure procedure; the executor settles
  before each probe (`Envelope::settle`, or the method's `Settle`), aborts on an `Envelope` breach (back to the last committed config),
  commits each stage's gains through `set_stabilizer_config`, and reverts to the starting
  config on disarm; reports `AutotuneEvent`s for logging (`StageStarted`, `Measured`,
  `Tried`, `StageDone` with its `Outcome`, ...). Works with any `CopterTunable` stabilizer (`Tunable` on `TunedLoop` +
  `Configurable`). `FlightLoop`'s accessors and config setters no longer require the pipeline
  bounds.
- **`mixers::TableMixer`** (#245): one table-driven mixer for multirotors and
  thrust-and-servo frames. Each `MixRow` drives a motor or servo `Slot` with roll, pitch, and
  yaw factors; motors share airmode-style desaturation, servos clamp on their own (both
  reported in `Saturation`). `MixerConfig` (thrust factor, yaw ratio) is set at construction;
  `TableMixer::new` validates a custom table (`MixerError`). Presets: `quad_x`, `quad_plus`,
  `hex_x`, `hex_plus`, `octo_x`, `y6`, `x8`, `tri` (tail tilt servo), `bi` (tilting rotors),
  `tailsitter_2p` (2 motors and 2 elevons, airplane frame, #121).
- **`View`** (#122): runs a `Guidance` or `Stabilizer` in a body frame rotated by a
  constant: the state goes in rotated, a stabilizer's moments come back in the body frame, and
  config, params, and autotune probes pass through. `View::tailsitter` (90° in pitch) flies a
  tailsitter's nose-up hover on the copter controllers, as ArduPilot's tailsitter view does.
- **`VehicleState::rotate_body`:** the state in another body frame (attitude, body rates, Euler
  fields), for a constant mount or view rotation.
- **`ActuatorCommand::SLOTS`:** the number of motor and of servo slots (8).
- **`ThrustLimits`** (#123): `CopterGuidanceConfig::thrust` bounds guidance's thrust while
  flying, all off by default. **Thrust floor:** `min_thrust` (param `GUID_THR_MIN`, default
  0: none) holds every thrust at or above it while flying, so a tailsitter's elevons keep the
  propwash they need to hold the nose up (as ArduPilot's `Q_TAILSIT_MIN_VO`). A piloted
  throttle at or below `IDLE_THROTTLE` (0.05) passes through, so the motors can idle on the
  ground; waypoint and land flight always get it. Raising a thrust flags `thrust` saturation.
  **Sink guard and boost ceiling:** piloted flight sinking faster than `sink_limit_mps`
  (`GUID_SINK_LIM`) gets `sink_gain` (`GUID_SINK_GAIN`) more thrust per m/s past it, since
  direct throttle can't hold a descent rate and a sinking tailsitter loses its elevons; `max_boost_thrust` (`GUID_BOOST_MAX`) stops the attitude-mode
  tilt boost short of full thrust, keeping motor headroom for the moments.
- **`Setpoint::thrust`:** the linear thrust either variant carries.

### Changed

- **Breaking:** the autotune settle (`Settle::rate_radps`) holds the roll and pitch rates
  only: the untuned yaw loop can ring on its start gains without tilting the vehicle (#518).
  An identification starts at the settle timeout instead of failing the stage.
  New `AutotuneAbort::Design`, `Outcome::Designed`, and `AutotuneEvent::Unsettled` variants
  break exhaustive matches.
- **Breaking:** a search that keeps no result keeps its start gains only if their steps met
  the criterion, and otherwise puts back the loop's gains at the run's start: a design it was
  seeded with doesn't outlive it (#518).

- **Breaking:** `LoopSignals` carries the probed axis's `rate_radps` and `last_moment`, which
  a host fills for an identification probe (#516).

- `AutotuneEvent::Measured` reports only probes the host kept time for; a stalled one comes as
  `AutotuneEvent::Stalled` (#438).

- **Breaking:** `TecsConfig::integrator_gain` (`PGUID_INTEG`) sets the thrust loop's integral
  gain only; the pitch loop's is `pitch_integrator_gain` (#503).
- The TECS height demand starts moving at the aircraft's climb rate, within the climb and sink
  limits, as PX4's reference model does, instead of level (#503). Entering plane flight
  sinking, it no longer pitches up to stop the sink and then sinks past `PGUID_SINK_MAX`
  catching up. With `pitch_integrator_gain` 0.15, the X-Plane Alia's sink after a descending
  entry peaks at 5.0–5.1 m/s, against 6.0–6.1 m/s before.
- `PlaneStabilizer` saves its rate integrals when a `Setpoint::PlaneRate` follows another
  setpoint and restores them when a `Setpoint::Plane` follows (#342): what acro or a maneuver
  winds up no longer carries into the attitude flight after it.
- Copter waypoint descents speed up from hover at `CopterGuidanceConfig::descent_accel_mps2`
  (`GUID_DESC_ACC`, default 1 m/s², ArduPilot's `WP_ACC_Z`) instead of stepping to
  `max_descent_mps` (#417): the start no longer cuts thrust at once (on the sim quad, a 0.03
  dip instead of 0.52). A descent reaches 3 m/s 3 s after it starts.
- A copter waypoint descent or land latched while the vehicle climbs starts its altitude
  target at that climb and sheds it at the new `CopterGuidanceConfig::climb_decel_mps2`
  (`GUID_CLIMB_DEC`, default 2.5 m/s², ArduPilot's `Q_PILOT_ACCEL_Z`), instead of holding the
  target while the vehicle climbs past it (#462). A sink still starts it at rest. A tailsitter
  handed over from a back transition no longer cuts thrust to stop its climb: on the sim
  tailsitter at +3.4 m/s with a 1.5 m/s descent limit, it sinks 1.61 m/s instead of 1.98 and
  rises 2.6 m instead of 0.9. **Breaking:** a `CopterGuidanceConfig` struct literal needs
  `climb_decel_mps2` (or `..Default::default()`).
- **Breaking:** `LinkedCommander::sync_regime` (#333): `FailsafeCommander` hands control back
  in the regime the fallback flew (copter, for `HoldCommander`), syncing the primary to it, so
  a plane request from before a link outage doesn't transition the vehicle on the link's
  return. The handover tick also keeps that regime rather than dropping to the default.

- **Breaking:** `PlaneGuidance` flies waypoint altitude and airspeed by total energy control
  (#454), as ArduPilot's and PX4's TECS: thrust for the total energy (height and airspeed
  together), pitch for the balance between them. A climb the pitch limit can't make now runs
  full thrust, where the airspeed loop took its power and the aircraft sank; and the two
  loops no longer limit-cycle through the airframe's coupling (the Alia 250: ±10–15 m in
  X-Plane). New `PlaneGuidanceConfig::tecs` (`TecsConfig`, ArduPilot's defaults): time
  constant, climb and sink limits, damping, integrator gain, speed weight, roll compensation,
  and the least and most airspeed, which also latches underspeed (full thrust, pitch for
  airspeed) below 0.9 of the least (`PGUID_TCONST`, `PGUID_CLMB_MAX`, `PGUID_SINK_MIN`,
  `PGUID_SINK_MAX`, `PGUID_THR_DAMP`, `PGUID_INTEG`, `PGUID_PTCH_DAMP`, `PGUID_SPDWEIGHT`,
  `PGUID_RLL2THR`, `PGUID_ASPD_MIN`, `PGUID_ASPD_MAX`). The airspeed loop's `speed_p`,
  `speed_i`, and `speed_max_integral` (`PGUID_SPD_P`, `_I`, `_IMAX`) are gone; the altitude
  loop (`PGUID_ALT_*`) and `climb_thrust_per_rad` fly only without an airspeed. An airframe
  sets `max_climb_mps` and `min_sink_mps` to its climb at full thrust and sink at least
  thrust, and the airspeeds to its stabilizer's.
- **Breaking:** `TableMixer` pusher rows fly the cruise thrust over their full range, no
  longer scaled by `MixerConfig::thrust_factor` (the lift motors' moment headroom; a pusher
  carries no moments, and ArduPlane's throttle spans 0–100%). For the same cruise thrust the
  pusher runs `1 / thrust_factor` harder; an airframe tuned on the scaled pusher multiplies
  its pusher values (`PGUID_CRUISE`, `TRAN_FW_THR`, `TRAN_PSH_SLEW`, `FWD_THR_GAIN`) by its
  thrust factor to keep its commands (#448).
- **Breaking:** `ControlOutput` has a `cruise` field (`Option<CruiseOutput>`, #391): a
  lift+cruise VTOL's pusher thrust and control-surface moments, flown alongside the lift
  rotors' `thrust` and `moments`. Struct literals add `cruise: None` or
  `..Default::default()`; `ControlOutput::new` sets `None`. `View` rotates the cruise moments
  with the others.
- **Breaking:** `CopterGuidanceConfig` has a `descent_accel_mps2` field (`GUID_DESC_ACC`,
  #445); struct literals add it.
- **Breaking:** `LateralControllerConfig` has a `max_accel_mps2` field (`GUID_LAT_ACC`,
  #418), 0 (off) by default; struct literals add it. Set, the lateral loop's velocity target
  starts at the measured velocity and moves toward the position loop's demand at that rate,
  its change fed forward (PX4 `MPC_ACC_HOR`), so a new hover leg asks for
  `atan(max_accel_mps2 / g)` of tilt at first rather than the tilt for its whole speed.
- **Breaking:** `MixerError::PusherBeforeMotor` (the enum is `#[non_exhaustive]`).
- **Breaking:** `guidance::copter::LAND_LEASH_M` is now `ALTITUDE_LEASH_M`: it leashes waypoint
  descents as well as lands (#400).
- Tailsitter back transitions add thrust while sinking by default (the sink guard, #343): a
  `TransitionConfig` built on `..Default::default()` gets it with no change.
  `back_sink_limit_mps: 0.0` restores the fixed `back_thrust`.
- **Breaking:** `AttitudeSetpoint`, `RateSetpoint`, and `BodyRateSetpoint` have an `idle`
  field, read through `Setpoint::idle()` (#326): struct literals add `idle: false` or
  `..Default::default()`. `CopterGuidance` sets it while a piloted throttle is at or below
  `IDLE_THROTTLE` (or NaN), and `CopterStabilizer` clears its integrals on it instead of on
  zero thrust. A disarmed tick's telemetry setpoint is idle.
- **Breaking:** `AttitudeSetpoint` and `PlaneSetpoint` have a `transition` field (#388):
  struct literals add `transition: false` or `..Default::default()`. A tailsitter ramp sets it,
  and the stabilizer saturates its pitch at ±π/2 instead of `max_angles_rad` or
  `max_pitch_rad` (ArduPilot ramps past its limits the same way).
- **Breaking:** `AutotuneEvent::Lowered`. A search whose start is above its maximum lowers it
  to the maximum and commits it before anything else flies, so the vehicle settles on it and
  it stays if the stage keeps or fails; a gain that high could ring once disturbed and fail
  the stage `NotSettled` (#375).
- **Breaking:** `AutotuneEvent` is `#[non_exhaustive]`: a `match` on it needs a wildcard arm,
  and new events stop being breaking.
- `WaypointFollower` reaches a waypoint only within an altitude band as well as the
  horizontal radius (or past it), as PX4 does: default `DEFAULT_ALTITUDE_ACCEPTANCE_M` 0.8 m,
  `with_altitude_acceptance`/`set_altitude_acceptance` to change it, and
  `PLANE_ALTITUDE_ACCEPTANCE_M` 10 m for wing-borne legs.
- **Breaking:** `LateralController::control` takes `limit`, the acceleration the caller last
  capped; the integral doesn't grow along it. `CopterGuidance` passes its capped tilt request
  (#357).
- `LateralControllerConfig::default()` has a velocity I of 0.4 and `max_integral` 12.5 m (±5
  m/s², the default tilt cap), so a waypoint hold trims a steady wind instead of settling
  downwind (#357).
- `PlaneStabilizerConfig`, `PlaneGuidanceConfig`, and `TransitionConfig` check each field on
  its own (no cross-field rules), so each param's range is fixed; out-of-order airspeeds
  clamp, and a `forward_max_tilt_rad` below `forward_done_rad` leaves the forward transition
  to its timeout.

- `CopterStabilizer` takes its attitude error from the attitude quaternion (#124): the
  shortest tilt of the thrust axis onto the setpoint's (roll and pitch relative to the current
  heading), then the heading error weighted by the cosine of the tilt, as body rates. Near
  level it matches the per-axis Euler difference it replaces; far over (a tailsitter's view
  in transition, view pitch toward ±90°) it stays defined and no longer treats Euler rates as
  body rates.

- **Breaking:** `Guidance::guide` takes the requested `FlightRegime` after `dt` (#181). A
  copter-only guidance ignores it.
- **Breaking:** `Command`, `VehicleState`, `TelemetryRecord`, and `testing::RecordedTick` have
  new fields (`regime`, `airspeed_mps`, `flight_mode`); struct literals need them or
  `..Default::default()`.

- **`CopterStabilizer` clears its integrals at zero thrust** (#123): a setpoint with thrust 0
  (or NaN) zeroes the rate, attitude, and yaw I terms after the tick, so they don't wind up
  against a resting attitude on the ground and kick at liftoff (as ArduPilot does at zero
  throttle).
- **Breaking:** `CopterGuidanceConfig` has a new `thrust` field (`ThrustLimits`, #123); struct
  literals add `..Default::default()` (no limits, the previous behavior).

- **Breaking:** roll and pitch have separate gains in the rate and attitude loops (#289).
  `BodyRateControllerConfig` and `AttitudeControllerConfig` replace `gains` with `roll_gains`
  and `pitch_gains`, and the params `STAB_RATE_P/I/D` and `STAB_ATT_P/I/D` become
  `STAB_RATE_RLL_*`, `STAB_RATE_PIT_*`, `STAB_ATT_RLL_*`, and `STAB_ATT_PIT_*`. To keep a
  shared tune, set both to the old value.

### Removed

- **Breaking:** `AttitudeCommand::mode` and `RateCommand::mode` (unspecified, never read)
  and `WaypointCommand::roll_rad` (never read) (#181). The regime is `Command::regime`.

- **Breaking:** `mixers::QuadCopterMixer` (#245). Use `TableMixer::quad_x()`, which mixes the
  same; its type is `TableMixer<4>`.
- **Breaking:** the `mixers::TailSitterMixer` placeholder (#121). Use
  `TableMixer::tailsitter_2p()`.

### Fixed

- `QuadPlaneGuidance`'s forward transition flies the wing toward the altitude the transition
  holds, not a waypoint's (#491). Toward a lower waypoint it pitched the wing nose down while
  the lift rotors held, entered plane flight sinking and unloaded, and stalled pulling out:
  the sim Alia 250 sank 19–23 m/s, past its 5 m/s `PGUID_SINK_MAX`, and through its 20 m
  `TRAN_FW_MIN_HGT` to the ground. It now sinks at most 4.8 m/s and falls back above the
  floor. PX4 likewise flies the transition at the wing's trim pitch whatever the altitude.

- After a fallback (a forward transition timed out or cut short, or plane flight below its
  minimum height), `TailsitterGuidance` and `QuadPlaneGuidance` fly a waypoint as a hover
  where copter flight took over until `Copter` is requested, instead of chasing it (#453): a quadplane
  timing out toward a plane waypoint 100 km off chased it on its lift rotors at 15 m/s,
  265 m in 20 s. `ToCopter` no longer returns to `Plane` after a fallback.

- `LateralController` with `max_accel_mps2` set (`GUID_LAT_ACC`) brakes onto its target along
  a braking curve at `BRAKING_SHARE` (0.7) of the limit, instead of in proportion to the
  distance under the rate limit (#466): a 260 m quadplane leg on the sim overshot by 193 m
  uncapped and 37 m at 10 m/s at 1 m/s², now under 1 m.

- `CopterGuidance` lands (`ControlInput::Land`) speed up to `land_speed_mps` at
  `descent_accel_mps2`, as waypoint descents do, instead of stepping to it: the step cut the
  RealFlight eXplora's thrust 0.89 → 0.76 in one tick (#476).

- `CopterGuidance` waypoint descents brake onto the waypoint (#445, #403): the altitude
  target descends at up to `max_descent_mps` and slows near the waypoint so
  `descent_accel_mps2` (`GUID_DESC_ACC`, default 1.0 m/s², ArduPilot's `WP_ACC_Z`) stops it
  there, its descent rate fed forward.
- The descent feedforward no longer steps where the waypoint takes over (#445, #403). A hold
  pushed past the leash above its target, as a quadplane transition the wing lifts, stepped
  half of full thrust in one tick, and arrivals sank past the waypoint: 0.52 m from 30 m on
  the sim quad and 1.16 m after a tailsitter's back transition, now 0.11 m and 0.06 m.
- `CopterGuidance` waypoint descents keep their altitude target at most `ALTITUDE_LEASH_M`
  (1 m) below the vehicle (#400). A hover that starts its descent still climbing, as a
  tailsitter does after a back transition, no longer sinks past `max_descent_mps` once the
  climb stops (sim fast ramp: 3.89 → 3.35 m/s against a 3 m/s limit; 4.31 → 3.29 with the
  eXplora's thrust floor).
- `CopterStabilizer` keeps its integrals at zero thrust in flight (#326): the altitude loop
  shooting up past a waypoint no longer drops the yaw trim, and a tailsitter's thrust handoff
  after a back transition no longer steps the moments. A piloted throttle resting just above
  zero (up to `IDLE_THROTTLE`) on the ground now clears them.
- A tailsitter completes its transitions below its stabilizers' limits (#388): the back
  ramp no longer stops at the plane pitch cap (30° by default) and flies to the 15 s timeout,
  nor the forward ramp at the hover tilt cap.
- Autotune's gain search tunes down from a too-high start (#361): the result is held to the
  baseline only when the baseline met the criterion (a too-high rate P settles fast and kept
  itself as `Kept(Worse)`), a start above the search's maximum flies no baseline, and a
  baseline whose steps keep clipping no longer fails the stage `Invalid`.
- A waypoint straight over the vehicle is no longer reached on the ground: `RouteEnd::Land`
  climbs to it before landing (instead of staying armed on the ground), and `RouteEnd::Hold`
  completes at its altitude (#359).
- The attitude error recovers from exactly inverted: a 180° tilt turns about body `x` instead
  of giving no correction (#124).

## [0.1.0] - 2026-10-10

First release.

### Added

- **Pipeline traits and data:** `Commander`, `StateSource` + `Actuator` (blanket
  `Vehicle`), `Guidance`, `Stabilizer`, `Mixer`, `TelemetryPublisher`, and `Configurable`;
  `Command { arm, input }` with `ControlInput` (attitude, rate, waypoint, offboard absolute
  attitude, land; its kind as `ControlMode` via `mode()`), `Setpoint`, `VehicleState`,
  `ControlOutput`, `ActuatorCommand` (generic `motors` and `servos` slots, their meaning set
  by the mixer), and `TelemetryRecord`. Rustdoc specifies the backend
  frame contract (`VehicleState`) and timing contract (`Vehicle`).
- **`FlightLoop`:** runs one tick per `step(dt)` (command → estimate → guide → stabilize →
  mix → actuate → telemetry) with no clock or allocation. `Command::arm` gates the motors and
  re-primes on arming. Guidance and stabilizer configs can be retuned between ticks.
- **Copter guidance** (`CopterGuidance`):
  - waypoint flight: an altitude loop with gravity feedforward and tilt compensation, a
    position → velocity → acceleration lateral cascade capped at the waypoint's cruise speed,
    a waypoint tilt cap, and a descent limit (`max_descent_mps`: the altitude target slews
    down, so a low waypoint descends instead of cutting thrust);
  - land (`LandCommand`): hold position and descend at `land_speed_mps`;
  - piloted attitude mode with heading hold and throttle tilt boost (none when inverted);
  - rate passthrough;
  - absolute attitude (`AbsoluteAttitudeCommand`: angles flown as given, the heading as the
    heading-hold target, thrust not boosted);
  - heading carried across mode switches.
- **Copter stabilizer** (`CopterStabilizer`): cascaded attitude/heading → body-rate PID, with
  a thrust curve; a non-finite gyro axis zeroes only its own moment. **Controllers** (`controllers`): altitude, lateral, attitude, yaw, and body
  rate, with validated configs (`InvalidConfig`).
- **Mixers:** `QuadCopterMixer` with airmode-style desaturation (moments kept, collective gives
  way, no lift at zero thrust); `TailSitterMixer` placeholder.
- **Commanders:** `WaypointFollower` (allocation-free; loops, holds, or lands at the route's
  end, flies legs at cruise speed, advances past a waypoint missed wide; `RouteEnd::Land`
  disarms on touchdown). `TouchdownDetector` decides touchdown during a land from the vertical
  speed (a descent, then stopped for 1 s). `FailsafeCommander` flies a
  link-fed `LinkedCommander` and falls back to a `Fallback` (`HoldCommander`: hold position,
  or finish an armed land the link dropped)
  when it goes stale, returning after a recovery period; the fallback never arms, a disarm is
  always honored, the primary's arm is synced to the vehicle at the handover
  (`LinkedCommander::sync_arm`), and sticks take back an armed vehicle only once they move
  (`takeover_stick_delta`), reporting `FailsafeState::AwaitingSticks` meanwhile; an armed land
  disarms on touchdown, syncing the primary, whichever commander flies it.
  `ControlInput::is_sticks` names stick-style input and `is_stream` input that must keep
  arriving (sticks and absolute attitudes). `Commander::failsafe` reports the state into
  `TelemetryRecord::failsafe` (`FailsafeState`).
- **Params** (`params`): every guidance and stabilizer gain and limit readable, settable,
  and listable by index or by a MAVLink-sized name (`STAB_RATE_P`, `GUID_TILT_MAX`), with
  angles in degrees. Writes go through `set_config`, so an invalid value is rejected. `FlightLoop`
  exposes both components' params. Retuning an integral gain carries the I term over.
- **Timing** (`timing::Every`): dt-driven fixed-rate scheduling for streams slower than the
  loop, without drift.
- **Saturation reporting:** per-tick `Saturation` flags (tilt, thrust, rate-loop moments,
  mixer) from each stage, merged into `TelemetryRecord`.
- **Composition:** `ComposedVehicle` (separate state source + actuator) and tuple fan-out of
  telemetry publishers.
- **`testing` feature:** allocation-free doubles (`NullVehicle`, `ConstantCommander`,
  `RecordingTelemetry`), `conformance` scenarios with `conformance_tests!` for backends, and
  `settled_yaw_rate` for checking rate gains.
- **`param-metadata` feature:** `params::metadata` describes every guidance and stabilizer
  param for ground stations: description, units, category and group, and the valid range
  (`ParamMeta`, `Bound`, `ParamMetadata`), checked against `validate` by tests. Stays `no_std`;
  firmware leaves it off.
- **`serde` feature:** `Serialize`/`Deserialize` on the pipeline data (`Command`,
  `ControlInput`, `Setpoint`, `VehicleState`, `ControlOutput`, `ActuatorCommand`,
  `Saturation`), for wire protocols such as `stable-flight-link`. Stays `no_std`.
- **Math:** `Vec2`/`Vec3`, `GRAVITY`, NaN-safe `euler_angles`, `wrap_angle`, and a `nalgebra`
  re-export.
- **`no_std`**, including the `testing` feature; builds for `thumbv7em-none-eabihf`. The tick
  never panics on bad input (saturate, NaN as neutral), only on `dt <= 0`.

[unreleased]: https://github.com/wboayue/stable-flight/compare/stable-flight-v0.1.0...HEAD
[0.1.0]: https://github.com/wboayue/stable-flight/releases/tag/stable-flight-v0.1.0
