# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `CopterStabilizerConfig::gyro_notch` (`controllers::NotchConfig`; params `STAB_NTCH_HZ`,
  `STAB_NTCH_BW`, `STAB_NTCH_ATT`): a notch on the roll and pitch rates ahead of the gyro
  low-pass, for one vibration tone, in ArduPilot's fixed harmonic notch form (center, width,
  depth). Off by default. `controllers::GyroFilters` is the chain the rate loop flies, and
  `CopterStabilizerConfig::gyro_filters_on` gives an axis's (#597).
- `TecsConfig` fields for the total energy control's fixed time constants and underspeed
  latch, with params and the old values as defaults (#622): `underspeed_share`
  (`PGUID_USPD_SHARE`, 0.9), `height_demand_time_constant_s` (`PGUID_HDEM_TC`, 3 s),
  `airspeed_rate_time_constant_s` (`PGUID_ASPD_RT_TC`, 0.5 s), `kinetic_rate_high_pass_s`
  (`PGUID_KE_HPF_TC`, 1 s), and `guidance::plane::DEFAULT_*` consts for each.
- Quadplane transition pitch and off-nose cut as config fields, with params and the old values
  as defaults (#623): `LiftTransitionConfig::forward_pitch_max_rad` (`TRAN_FW_PIT_MAX`, 3°;
  twice it plus 2° on the lift ramp, as ArduPilot's `Q_TRAN_PIT_MAX`), and
  `OffNoseSpeedConfig::free_rad` (`OFFNOSE_FREE`, 20°), `full_rad` (`OFFNOSE_FULL`, 160°;
  at or below `free_rad`, a step to the whole cut), and `tail_first_divisor` (`OFFNOSE_DIV`,
  3), with `guidance::vtol::DEFAULT_*` consts for each.
- `WindConfig` fields for the wind estimator's fixed limits, with params and the old values
  as defaults (#624): `initial_sd_mps` (`GUID_WND_INIT_SD`, 10 m/s), `gate_sigma`
  (`GUID_WND_GATE`, 5), and `max_axial_speed_mps` (`GUID_WND_AX_SPD`, 1 m/s), with
  `estimation::wind::DEFAULT_*` consts for each.
- `FlightLoop::setpoint`: the setpoint guidance gave the stabilizer on the last tick (#539).
- Plane autotune (#534): `PlaneStabilizer` hosts probes on its roll and pitch rate and angle
  loops (`stabilizers::plane::PlaneLoop`, `PlaneAxisGains`), as do `TailsitterStabilizer` and
  `QuadPlaneStabilizer` on their plane side. `tuning::autotune::PlaneAutotune` runs plane
  stages within a `PlaneEnvelope` (bank, pitch, rate, sink, airspeed band, and a settle on a
  steady airspeed near trim, `AirspeedSettle`): a step test (`StagePlan::plane_step_test`,
  `StepTestPlan::plane`) or a design (`PlaneMethod::Design`, `PlaneDesignPlan`,
  `StagePlan::plane_design`), which fits each axis in flight (`tuning::WingIdentifyProbe`,
  `WingModel`, `ProbeConfig::IdentifyWing`, `Measurement::Wing`), designs its rate gains and
  time constant on the fit (`tuning::WingDesign`, `WingDesignConfig`), and checks them on
  attitude pairs. `commanders::PlaneTuneHold` flies the attitude hold the probes ride on,
  trimming pitch for airspeed and thrust for height, turns back between probes, and recovers
  to a hover or wings level. `AutotuneAbort::is_breach` and `Unverified`,
  `DesignError::NoWingDesign`, `Supervisor::set_maneuvering`, `LoopSignals::alpha_rad`.

### Changed

- **Breaking:** `estimation::wind::INITIAL_SD_MPS` is `DEFAULT_INITIAL_SD_MPS`, the default of
  `WindConfig::initial_sd_mps` (#624).
- **Breaking:** `CopterGuidance`'s fixed limits are `CopterGuidanceConfig` fields, with params
  and the old values as defaults (#621): `stick_yaw_rate_radps` (`GUID_STICK_YAW`, 1 rad/s),
  `idle_throttle` (`GUID_THR_IDLE`, 0.05), `heading_yield_rate_radps` (`GUID_YAW_YLD_RT`,
  0.1 rad/s), `altitude_leash_m` (`GUID_ALT_LEASH`, 1 m),
  `wind_heading_from_wind_rad` (`GUID_WND_HDG_OFF`, 80°), `wind_heading_max_sd_rad`
  (`GUID_WND_HDG_SD`, 15°), `wind_heading_deadband_rad` (`GUID_WND_HDG_DB`, 10°),
  `airborne_height_m` (`GUID_WND_AIR_HT`, 1 m), and `feedforward_hold_band_mps`
  (`GUID_WND_FF_BAND`, 0.3 m/s). `guidance::copter::IDLE_THROTTLE`, `ALTITUDE_LEASH_M`, and
  `AIRBORNE_HEIGHT_M` are now `DEFAULT_IDLE_THROTTLE`, `DEFAULT_ALTITUDE_LEASH_M`, and
  `DEFAULT_AIRBORNE_HEIGHT_M`; a `CopterGuidanceConfig` struct literal needs the new fields
  (or `..Default::default()`).
- **Breaking (behavior):** hover guidance with a drag model (`GUID_WND_BCOEF`/`MCOEF`) feeds
  a leg's drag forward: the push the lateral loop's commanded velocity, past 0.3 m/s, takes
  through the estimated wind. The lateral I term then trims only the wind, and hover legs stop
  on their waypoint instead of overshooting by the drag the I learned on the way: the sim
  eXplora's 25 m legs overshot 1.0–2.5 m, now at most 0.25 m (#609). The RealFlight eXplora
  (`explora_vtol`) has a drag model, so it flies its legs differently.

- **Breaking:** `tuning::Tunable` is generic over the loop type (`Tunable<TunedLoop>` for a
  copter's loops) instead of naming it as `Target`, so a VTOL stabilizer hosts both sides'
  probes; on a VTOL, name the side (`Tunable::<TunedLoop>::probe(&stabilizer)`). The
  supervisor is `Supervisor<K: Side>`, with `Autotune` the copter's
  (`Supervisor<CopterSide>`); `StagePlan`, `AutotuneEvent`, and `AutotuneStatus` take the
  side's types, defaulting to the copter's. `AutotuneAbort` gains `AirspeedLow` and
  `AirspeedHigh` (#534).
- **Breaking:** `tuning::LoopSignals` gains `alpha_rad` (0 outside a wing's pitch axis);
  `ProbeConfig`, `Measurement`, `AutotuneAbort`, and `DesignError` gain variants (#534).
- **Breaking (behavior):** a VTOL stabilizer's hover probe is interrupted when the setpoint
  leaves the hover side (a plane setpoint or a transition's blend), where it flew on before
  (#534).
- **Breaking:** `tuning::design_rate` and `design_angle_p` take the stabilizer's
  `GyroFilters` in place of the gyro cutoff (`GyroFilters::low_pass(cutoff_hz)` for the old
  call), so a design models the notch too; `CopterStabilizerConfig` gains `gyro_notch` (#597).
- **Breaking:** `IdentifyAbort::Saturated`: a window whose moments mostly sit at the rate
  loop's limit while the least squares leaves it noisy is no longer fitted. A design stage
  answers it on a rate loop by lowering its start, every gain ÷ 1.3, and fitting again, up to
  eight times (#551, #598). `IdentifyAbort` is now `#[non_exhaustive]`.
- **Breaking (behavior):** `IdentifyProbe` fits a noisy window in two stages: the moment
  projected on the probe's excitation, then a lag and half-tick delay grid on it, so gyro
  noise in the moment no longer reads as the airframe (the sim's noise rows read every lag as
  0 before). A quiet window (the least-squares fit leaving at most 0.017 rad/s
  unexplained, as on RealFlight's) or one whose moments mostly sit at the rate loop's limit
  keeps the least-squares fit on the recorded moments, unchanged. The probe records its window (13 KB) and fits once at the end (#551).
- **Breaking:** `BodyRateControllerConfig::max_integral` and `yaw_max_integral` (params
  `STAB_RATE_IMAX`, `STAB_RATE_IMAX_Z`) are the most moment the rate I terms hold, in
  normalized moment, not a limit on the integrated error in rad: the rate loop integrates
  `I·e·dt`, as ArduPilot's `AC_PID` and PX4's `RateControl` do. To keep what a config held,
  set the new value to its I gain × the old one; a value above 1 (as a saved 1 rad or 5 rad
  would read) fails validation (#374).
- **Breaking (behavior):** the rate I limits default to 0.5 of moment (ArduPilot's
  `ATC_RAT_*_IMAX`), not 1 rad and 5 rad: at the default gains the roll and pitch I terms
  hold 0.5 of trim, not 0.03; the yaw term holds 0.5, as before at the default yaw I. A change
  of rate I gain keeps the I term, as before, and an I gain of 0 drops it (#374).
- **Breaking (behavior):** the autotune's settle (`Envelope::settle`, a step plan's
  `COPTER_STEP_SETTLE`) reads roll and pitch from the attitude target guidance holds, a
  position hold's lean included, as ArduPilot's level check does
  (`AC_AutoTune::currently_level`), and waits for the vehicle to stand still over the ground
  (under 0.2 m/s); the envelope still reads tilt from level. The eXplora in RealFlight leans
  3.4° in calm air to hold its position, past the step settle's 2.5°, so no step started
  (#539).
- **Breaking (behavior):** `StagePlan::copter_design_search` searches yaw rate right after
  the designs, before the roll and pitch searches (index 4 of 9, not 8): a yaw start that
  rings spoiled their steps (#552).
- **Breaking (behavior):** a search whose start won't settle within
  `Envelope::settle_timeout_s` lowers it by its `factor` and commits it
  (`AutotuneEvent::Lowered`) instead of failing `NotSettled`, again at each timeout, up to
  eight times and down to its `min` (#552, #598).

### Fixed

- `WingDesign` designs a stiff axis (a wing's pitch) fitted with damping at or below 0 as
  undamped, with no feedforward, instead of failing: the sim eXplora's barely damped pitch
  fitted −0.72/s from three times its gains and the plane autotune stopped on
  `NoWingDesign` (#618).
- **Breaking (behavior):** the wind estimator counts each measurement by the drag's slope at
  the air its estimate predicts (or the measured air, if steeper), takes bluff drag on the
  airspeed with the climb counted in, and skips ticks climbing or sinking faster than 1 m/s.
  With a ballistic coefficient set, a wrong estimate in still air now comes back instead of
  staying: the sim tailsitter read −0.60 m/s in calm air after its takeoff, now −0.14 m/s
  (#608). Momentum drag alone estimates as before, apart from the climb skip.
- A rate step stage holds the integral of the angle or heading loop over the stepped rate
  loop: the step tilts the vehicle away from that loop's setpoint, and integrating it wound
  the loop up for the handoff. On the sim's `quad_x` at angle P 2 and I 0.25, the vehicle
  didn't settle within 10 s after a large rate step (#579).

## [0.2.0] - 2026-10-30

### Added

- `Tunable::probe_frame`: the frame a stabilizer's probed loops fly in (identity by default;
  `View` and `TailsitterStabilizer` report their view). `Autotune` reads its envelope and
  settle in it, so it runs on a tailsitter hovering nose up in its view, where it used to
  read 90° of pitch and fail every stage `Tilt` (#321).
- `StagePlan::large_copter_search`, `SearchPlan::large_copter`, `Envelope::large_copter`, and
  `LARGE_COPTER_RATE_STEP`, `LARGE_COPTER_TILT_STEP`, `LARGE_COPTER_MAX_SATURATED_S`: the step
  search for a full-size multirotor, on 0.2 rad/s rate steps over 1.5 s and 5° angle steps
  over 2.5 s, keeping the start's rate I, waiting 30 s to settle. It tunes the Alia 250's sim
  model, where the design search doesn't yet (#351, #572). `StepTestPlan::large_copter` and
  `StagePlan::large_copter_step_test` step-test on the same steps (#540).
- `StepSummary::overshoot_past_end`: each step's peak past where it ended, or past the step
  when it ended short (#564).
- `Autotune::set_state_stale`: while the host holds a stale vehicle state (a simulator link
  down), every tick is a freeze, probes stop as stalled, and settles and holds wait; past
  `Envelope::max_stale_s` (2 s) the stage fails `AutotuneAbort::HostFault`. **Breaking:** the
  new `Envelope` field and `AutotuneAbort` variant (#309).
- `Autotune::abort` ends a run from outside, as a breach would (the config as of the last
  kept stage, the probe stopped, `StageFailed`) (#309).
- `controllers::RATE_I_PER_P`: the copter rate I's share of P (#374).
- `controllers::YAW_RATE_I_PER_P`: the copter yaw rate I's share of P, 0.3 (#558).
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
  a square wave added to its rate setpoint (0.7 rad/s, PX4's `MC_AT_SYSID_AMP`), every loop
  closed, in `f32` and fixed memory; an angle stage's fit runs on the rate loop under it
  (#561). On the sim, 3 s per axis recovers each copter's motor lag within 1% and its actuator
  delay, from low and ×3 starting gains. On the Titan Cobra in RealFlight roll fits 23 rad/s²,
  where a 4° wave on the angle setpoint fit 11 (#561).
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
- `TecsConfig::pitch_integrator_gain` (`PGUID_PTCH_INTEG`, default 0.2): the pitch loop's
  integral gain, apart from the thrust loop's, as PX4 splits `FW_T_I_GAIN_PIT` and
  `FW_T_THR_INTEG` (#503, #505).
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
  **Breaking (behavior):** on by default.
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
  rising to 3 from 20° to 160°. `QuadPlaneGuidance` flies it on waypoints, so offboard position
  targets too. **Breaking (behavior):** on by default. **Breaking:** a `QuadPlaneGuidanceConfig`
  struct literal needs `off_nose` (or `..Default::default()`).
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

- `RateModel::lag_s` reads a fitted pole at or below 0 as a lag under a tick (0 s), and
  `accel_radps2` takes the pole as 0. `design_rate` then designs D 0 rather than failing with
  `DesignError::NotALag`, which now means only a pole at or past 1. In RealFlight half the
  Titan Cobra's and QuadCopterX's fits landed at or below 0, and three in a row failed the
  Cobra's design search (#563).
- **Breaking:** `Criterion::Overshoot` has a `settled_past: Option<f32>`. When it's set,
  overshoot counts from where each step ended (`StepSummary::overshoot_past_end`), and no step
  may end more than that past the step. `SearchPlan::copter_angle` sets it to its settle band
  (0.1), so a disturbance holding an angle past its step, without ringing, no longer walks the
  angle search's P down: it walked the Titan Cobra's pitch P to 1.3 in RealFlight (#564). The
  rate searches keep judging from the step (`None`).
- **Breaking (behavior):** `StagePlan::copter_design_search` searches yaw rate as its ninth
  stage (`SearchPlan::copter_yaw_rate`: P from the start, I at `YAW_RATE_I_PER_P` of P, D kept,
  0.5 rad/s steps watched 0.25 s, `COPTER_YAW_RATE_STEP`), and `StepTestPlan::copter(YawRate)`
  steps 0.5 rad/s watched 1 s (`COPTER_YAW_RATE_TEST_STEP`), so a yaw as slow as QuadCopterX's
  rises and settles in the step test (#296, #558). Yaw rate and heading stages settle on the yaw
  rate too; roll and pitch stages still leave it out. `MAX_STAGES` is 14 (#296).
- **Breaking:** `BodyRateControllerConfig::yaw_max_integral` (param `STAB_RATE_IMAX_Z`,
  default 5 rad) limits the yaw rate integral; `max_integral` limits roll and pitch only. A
  yaw I of 0.3 of P holds a large yaw trim (0.40 on QuadCopterX from P 0.27) (#296, #558).
- **Breaking (behavior):** copter step tests allow 10 clipped ticks on every loop
  (`COPTER_MAX_SATURATED_S`, 0.042 s; angle loops 0.022 s before), and `StepTestPlan::copter`
  steps roll and pitch by 5° instead of 10°: the angle step test measures a tuned angle loop
  instead of failing `Invalid` on clipping (#310).
- **Breaking:** `COPTER_ANGLE_SEARCH_STEP` is `COPTER_TILT_STEP` (5°, roll and pitch, searched
  and step-tested), and `COPTER_ANGLE_STEP` is `COPTER_HEADING_STEP` (10°, heading only)
  (#310).
- **Breaking (behavior):** the copter rate loop's D acts on the measured rate instead of the
  error, so a rate setpoint step has no derivative kick (`D·A/dt` for one tick before); hand
  tunes keep their gains, and their setpoint steps follow P and I alone (#373).
- **Breaking (behavior):** `CopterStabilizerConfig::gyro_cutoff_hz` (param `STAB_GYRO_FLT`,
  default 40 Hz, `DEFAULT_GYRO_CUTOFF_HZ`): the rate loop flies the body rates through a
  two-pole low-pass; 0 turns it off (#373).
- **Breaking:** `design_rate` and `design_angle_p` take the gyro cutoff and model the filter in
  the rate loop, and the rate loop's D on its measurement; a fit whose moment pushes the axis
  the wrong way is `DesignError::NoCrossover` (#373).
- The autotune's settle reads the body rates through the stabilizer's gyro low-pass; the
  envelope still reads them raw (#373). On the sim robustness table `heavy_quad`'s CG offset
  from the high start and weak-motor rows now pass.
- **Breaking (behavior):** the copter roll and pitch rate loops have an I term by default, at
  `RATE_I_PER_P` (0.2/s) of P (0.03), so a steady trim moment (a CG offset, a weak motor) is
  held without waiting on the attitude loop's I (#374). Airframes that set their own rate gains
  keep them.
- `BodyRateController` holds its integral while its moment is clipped in the error's
  direction (it may still shrink), as ArduPilot's `AC_PID` does on the mixer's limit flags;
  the controller sees only its own ±1 clip, not a mixer limit below it (#374).
- **Breaking:** `SearchPlan` has `i_per_p` and `DesignPlan` `rate_i_per_p`: the copter rate
  search and design set I at `RATE_I_PER_P` of each P they set (`None` keeps the start's I).
  `design_angle_p` models the rate loop without its I (#374). On the sim robustness table
  `quad_x`'s weak-motor and high-start CG offset rows and `racer`'s weak-motor rows now pass.
- **Breaking (behavior):** the TECS pitch integral acts on the energy balance rate error, as
  ArduPilot's and PX4's, instead of the balance error (#505): it pulls the nose up once a sink
  passes `PGUID_SINK_MAX`. The kinetic energy rate is high-passed (1 s), so a steady
  acceleration isn't a rate error; the integral fades while pitch is pinned at a limit; the
  pitch demand's airspeed part is no longer held to half the climb's energy rate.
  `QuadPlaneGuidance`'s forward transition passes its pitch limits into the TECS. On the sim
  Alia, a 100 m descent at 50 m/s from a 45 m/s entry sinks 4.5 m/s (4.7 before), the forward
  transition loses 7 m entering plane flight (11 m), and reaches its airspeed gate at 36 s of
  its 45 s timeout, where with the transition's limits passed to the old integral it timed out.
- `IdentifyProbe` fits only delays whose moment pushes the axis the right way, falling back
  to all when none does (#529). In RealFlight, noise had picked delays with negative moment
  effects that the design can't use: 10 of 67 QuadCopterX fits read under 20 rad/s², 5
  replayed with the filter.
- A design stage's fit and hold run through a slow host (`Autotune::set_host_slow`) instead of
  stopping (#524): on RealFlight they stopped at every dip to 222–225 Hz and failed the stage
  `Stalled`; a fit 12% slow designs rate P 5–13% lower on the sim. `Autotune::note_stall`
  marks a freeze: breaches for `Envelope::recovery_s` after it don't count, and a probe that
  aborts across it runs again.
- **Breaking:** the autotune settle (`Settle::rate_radps`) holds the roll and pitch rates
  only: the untuned yaw loop can ring on its start gains without tilting the vehicle (#518).
  An identification starts at the settle timeout instead of failing the stage.
  New `AutotuneAbort::Design`, `Outcome::Designed`, and `AutotuneEvent::Unsettled` variants
  break exhaustive matches.
- **Breaking (behavior):** a search that keeps no result keeps its start gains only if their
  steps met the criterion, and otherwise puts back the loop's gains at the run's start: a design
  it was seeded with doesn't outlive it (#518).
- **Breaking:** `LoopSignals` carries the probed axis's `rate_radps` and `last_moment`, which
  a host fills for an identification probe (#516).
- `AutotuneEvent::Measured` reports only probes the host kept time for; a stalled one comes as
  `AutotuneEvent::Stalled` (#438).
- **Breaking:** `TecsConfig::integrator_gain` (`PGUID_INTEG`) sets the thrust loop's integral
  gain only; the pitch loop's is `pitch_integrator_gain` (#503).
- **Breaking (behavior):** the TECS height demand starts moving at the aircraft's climb rate,
  within the climb and sink limits, as PX4's reference model does, instead of level (#503).
  Entering plane flight sinking, it no longer pitches up to stop the sink and then sinks past
  `PGUID_SINK_MAX` catching up. With `pitch_integrator_gain` 0.15, the X-Plane Alia's sink after
  a descending entry peaks at 5.0–5.1 m/s, against 6.0–6.1 m/s before.
- `PlaneStabilizer` saves its rate integrals when a `Setpoint::PlaneRate` follows another
  setpoint and restores them when a `Setpoint::Plane` follows (#342): what acro or a maneuver
  winds up no longer carries into the attitude flight after it.
- **Breaking (behavior):** copter waypoint descents speed up from hover at
  `CopterGuidanceConfig::descent_accel_mps2` (`GUID_DESC_ACC`, default 1 m/s², ArduPilot's
  `WP_ACC_Z`) instead of stepping to `max_descent_mps` (#417): the start no longer cuts thrust
  at once (on the sim quad, a 0.03 dip instead of 0.52). A descent reaches 3 m/s 3 s after it
  starts.
- **Breaking (behavior):** a copter waypoint descent or land latched while the vehicle climbs
  starts its altitude target at that climb and sheds it at the new
  `CopterGuidanceConfig::climb_decel_mps2` (`GUID_CLIMB_DEC`, default 2.5 m/s², ArduPilot's
  `Q_PILOT_ACCEL_Z`), instead of holding the target while the vehicle climbs past it (#462). A
  sink still starts it at rest. A tailsitter handed over from a back transition no longer cuts
  thrust to stop its climb: on the sim tailsitter at +3.4 m/s with a 1.5 m/s descent limit, it
  sinks 1.61 m/s instead of 1.98 and rises 2.6 m instead of 0.9. **Breaking:** a
  `CopterGuidanceConfig` struct literal needs `climb_decel_mps2` (or `..Default::default()`).
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
- **Breaking (behavior):** `TableMixer` pusher rows fly the cruise thrust over their full range,
  no longer scaled by `MixerConfig::thrust_factor` (the lift motors' moment headroom; a pusher
  carries no moments, and ArduPlane's throttle spans 0–100%). For the same cruise thrust the
  pusher runs `1 / thrust_factor` harder; an airframe tuned on the scaled pusher multiplies its
  pusher values (`PGUID_CRUISE`, `TRAN_FW_THR`, `TRAN_PSH_SLEW`, `FWD_THR_GAIN`) by its thrust
  factor to keep its commands (#448).
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
- **Breaking (behavior):** tailsitter back transitions add thrust while sinking by default (the
  sink guard, #343): a `TransitionConfig` built on `..Default::default()` gets it with no
  change. `back_sink_limit_mps: 0.0` restores the fixed `back_thrust`.
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
- **Breaking (behavior):** `WaypointFollower` reaches a waypoint only within an altitude band as
  well as the horizontal radius (or past it), as PX4 does: default
  `DEFAULT_ALTITUDE_ACCEPTANCE_M` 0.8 m, `with_altitude_acceptance`/`set_altitude_acceptance` to
  change it, and `PLANE_ALTITUDE_ACCEPTANCE_M` 10 m for wing-borne legs.
- **Breaking:** `LateralController::control` takes `limit`, the acceleration the caller last
  capped; the integral doesn't grow along it. `CopterGuidance` passes its capped tilt request
  (#357).
- **Breaking (behavior):** `LateralControllerConfig::default()` has a velocity I of 0.4 and
  `max_integral` 12.5 m (±5 m/s², the default tilt cap), so a waypoint hold trims a steady wind
  instead of settling downwind (#357).
- `PlaneStabilizerConfig`, `PlaneGuidanceConfig`, and `TransitionConfig` check each field on
  its own (no cross-field rules), so each param's range is fixed; out-of-order airspeeds
  clamp, and a `forward_max_tilt_rad` below `forward_done_rad` leaves the forward transition
  to its timeout.
- **Breaking (behavior):** `CopterStabilizer` takes its attitude error from the attitude
  quaternion (#124): the shortest tilt of the thrust axis onto the setpoint's (roll and pitch
  relative to the current heading), then the heading error weighted by the cosine of the tilt,
  as body rates. Near level it matches the per-axis Euler difference it replaces; far over (a
  tailsitter's view in transition, view pitch toward ±90°) it stays defined and no longer treats
  Euler rates as body rates.
- **Breaking:** `Guidance::guide` takes the requested `FlightRegime` after `dt` (#181). A
  copter-only guidance ignores it.
- **Breaking:** `Command`, `VehicleState`, `TelemetryRecord`, and `testing::RecordedTick` have
  new fields (`regime`, `airspeed_mps`, `flight_mode`); struct literals need them or
  `..Default::default()`.
- **Breaking (behavior):** `CopterStabilizer` clears its integrals at zero thrust (#123): a
  setpoint with thrust 0 (or NaN) zeroes the rate, attitude, and yaw I terms after the tick, so
  they don't wind up against a resting attitude on the ground and kick at liftoff (as ArduPilot
  does at zero throttle).
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

[unreleased]: https://github.com/wboayue/stable-flight/compare/stable-flight-v0.2.0...HEAD
[0.2.0]: https://github.com/wboayue/stable-flight/releases/tag/stable-flight-v0.2.0
[0.1.0]: https://github.com/wboayue/stable-flight/releases/tag/stable-flight-v0.1.0
