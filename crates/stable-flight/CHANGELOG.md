# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `commanders::RouteEnd` and `WaypointFollower::with_route_end`: loop the route (default) or
  hold the last waypoint, with `WaypointFollower::is_complete` once it is reached ([#126](https://github.com/wboayue/stable-flight/issues/126)).
- `WaypointFollower` fills a waypoint's empty `velocity_mps` with one along the leg that leads
  to it at its `cruise_speed_mps`, so legs are flown at cruise speed instead of slowing at
  every waypoint; the first and the held last waypoint get none ([#109](https://github.com/wboayue/stable-flight/issues/109)).
- `controllers::LateralTarget` (position, feedforward velocity, speed limit) and
  `LateralEstimate` (position, velocity) ([#109](https://github.com/wboayue/stable-flight/issues/109)).
- `wrap_angle` wraps radians to [-π, π) (rounding can give +π; non-finite input gives
  NaN; `no_std`), exported at the crate root beside `euler_angles`
  ([#166](https://github.com/wboayue/stable-flight/issues/166)).
- Initial release of the flight-control core, extracted from `stable-flight-sitl`:
  the pipeline traits (`Commander`, `StateEstimator`, `Actuator`, `Vehicle`,
  `Guidance`, `Stabilizer`, `Mixer`, `TelemetryPublisher`), their data types
  (`Command` and its variants, `Setpoint` and its variants, `VehicleState`,
  `ControlOutput`, `ActuatorCommand`, `TelemetryRecord`), the `guidance` module
  (`CopterGuidance`), the `stabilizers` module (`CopterStabilizer`), the
  `controllers` module (PID controllers and `Gains`, formerly
  `stabilizers::controllers` and `stabilizers::Gains`), and the `mixers` module (`QuadCopterMixer`,
  placeholder `TailSitterMixer`). Earlier history of these items is in the
  `stable-flight-sitl` changelog.
- `FlightLoop`: owns the six pipeline components and runs one control tick per
  `step(dt)` (commander → estimate → guide → stabilize → mix → actuate →
  telemetry), with `prime()` for the initial `Duration::ZERO` estimate and
  guidance and stabilizer priming. No clock, threads, or signals; runners decide
  when to step. Allocation-free. `into_parts()` returns the six components after
  a run.
- `commanders::WaypointFollower<W: AsRef<[WaypointCommand]>>`: a `Commander` that
  follows and loops a waypoint list by horizontal acceptance radius (default
  `DEFAULT_ACCEPTANCE_RADIUS_M`, 3 m; `with_acceptance_radius` to override).
  Works over an array, `&'static` slice, or `Vec`, and never allocates, so
  waypoint flight can run on firmware. `new` returns `EmptyWaypoints` for an
  empty list.
- `ComposedVehicle<E, A>`: a `Vehicle` built from a separate `StateEstimator`
  and `Actuator` (e.g. IMU/estimator + ESC driver), with `estimator()`,
  `actuator()`, and `into_parts()`.
- `impl TelemetryPublisher for (A, B)` fans each tick out to both publishers
  (nest tuples for more sinks), and `impl TelemetryPublisher for ()` is the
  no-op sink.
- `testing` feature (off by default) with allocation-free test doubles in
  `stable_flight::testing`: `NullVehicle` (fixed state, counts estimates and
  actuations, keeps the last command), `ConstantCommander`, and
  `RecordingTelemetry<N>` (ring of the last `N` ticks as `RecordedTick`s, which
  `as_record()` can replay).
- Backend contracts in rustdoc: `VehicleState` documents its frame contract
  (translation north/east/up, attitude NED world → FRD body, SI units, the sign of
  each angle and body rate), and `Vehicle` documents the per-tick timing contract
  every backend must meet.
- `testing::conformance` (`testing` feature): physical scenarios (level, banked,
  nose up, heading east, banked climb east, climbing, moving northeast, yaw and
  combined rates) with the `VehicleState` each must produce, `Scenario::check`
  (allocation-free `ConformanceError`), and the `conformance_tests!` macro, which
  generates one test per scenario plus a coverage check. docs.rs renders the
  `testing` feature.
- `QuadCopterMixer` derives `Debug`, `Clone`, `Copy`, and `PartialEq`.
- `Guidance` trait (`guide(dt, &Command, &VehicleState) -> Setpoint`, `prime`) and
  `guidance::copter::CopterGuidance` / `CopterGuidanceConfig` (altitude and lateral
  gains): the outer loop, split out of `CopterStabilizer`. It flies waypoints and
  owns the attitude-mode heading-hold target. `Setpoint` is `Attitude(AttitudeSetpoint)`
  (normalized roll/pitch, absolute heading, linear thrust) or `Rate(RateSetpoint)`.
  `TelemetryRecord` and `RecordedTick` gain the tick's `setpoint`.
- `CopterGuidanceConfig::navigation_period_s` (default `DEFAULT_NAVIGATION_PERIOD_S`,
  20 ms): how often the waypoint lateral loop runs; `0.0` runs it every tick.
  `CopterGuidance::new` panics on a negative or non-finite period.
- The crate is `no_std` (including the `testing` feature), so it builds for
  bare-metal targets such as `thumbv7em-none-eabihf`. It depends on `nalgebra`
  without its `std` feature; enable it yourself if you need it.
- **Breaking:** `CopterGuidanceConfig::max_tilt` (default `DEFAULT_MAX_TILT`, 0.6 of
  the stabilizer's maximum tilt) caps the magnitude of the waypoint roll/pitch
  request, so a far waypoint can't ask for full tilt while also climbing hard.
  `CopterGuidance::new` panics outside `(0, 1]`. Piloted attitude flight is not
  capped ([#110](https://github.com/wboayue/stable-flight/issues/110)).
- **Breaking:** `hover_thrust` on `AltitudeControllerConfig` and
  `CopterGuidanceConfig` (default `controllers::DEFAULT_HOVER_THRUST`, 0.5): the
  airframe's level-hover linear thrust, fed forward by the altitude loop.
  `CopterGuidance::new` panics outside `(0, 1]`
  ([#96](https://github.com/wboayue/stable-flight/issues/96)).
- `GRAVITY` (standard gravity, 9.80665 m/s²), used by the altitude and lateral
  controllers (they had their own 9.81) and available to backends.
- `Vec2` and `Vec3` aliases for nalgebra's `Vector2<f32>`/`Vector3<f32>`,
  and a `stable_flight::nalgebra` re-export of the version the public types use.
- `euler_angles(&UnitQuaternion<f32>) -> (roll, pitch, yaw)`: the frame
  contract's Euler angles. Prefer it to nalgebra's `UnitQuaternion::euler_angles`,
  which maps a NaN attitude to a finite pitch of -π/2 and yaw 0, and costs more
  flash. NaN in gives NaN out; pitch stays finite at ±90°.
- `validate()` on every controller config and on `CopterStabilizerConfig`, returning the new
  `controllers::InvalidConfig` (`#[non_exhaustive]`: field, requirement, and the loop
  `section` when a composite reports it, e.g. `body_rate.max_integral`). `CopterStabilizer::new`
  and `CopterGuidance::new` still panic on an invalid config. `CopterStabilizer::new` now also
  rejects a negative or non-finite `full_scale_rates_radps` component, which silently zeroed
  every rate-mode moment ([#175](https://github.com/wboayue/stable-flight/issues/175)), and a
  non-positive or non-finite `thrust_curve_exponent`, under which zero throttle became full
  thrust.

- `Configurable` trait (`type Config`, `config()`, `set_config()`): read a component's config,
  or replace it between ticks with validation (an invalid config is rejected and the current
  one kept); runtime state is kept. Implemented by `CopterGuidance` and `CopterStabilizer`.
  `FlightLoop::guidance()`/`stabilizer()` read them and `set_guidance_config`/
  `set_stabilizer_config` retune a running loop
  ([#146](https://github.com/wboayue/stable-flight/issues/146)).

### Changed

- **Breaking:** `LateralController` is a position → velocity → acceleration cascade:
  `control(config, dt, LateralTarget, LateralEstimate)`, and `LateralControllerConfig` is
  `{ position_p, velocity: Gains, max_integral }` (integral on velocity error). The defaults
  (`position_p` 0.5, velocity P 1.26) match the previous position PD below the speed limit
  ([#109](https://github.com/wboayue/stable-flight/issues/109)).
- `CopterGuidance` waypoint flight honors `WaypointCommand::cruise_speed_mps` (desired speed
  cap; zero, negative, or NaN means none) and feeds `velocity_mps` forward ([#109](https://github.com/wboayue/stable-flight/issues/109)).
- **Breaking:** `CopterGuidance` waypoint flight holds the heading-hold target instead of
  north, tilting relative to the vehicle's heading; rate flight keeps the target on the
  vehicle's heading, so attitude or waypoint flight after it doesn't snap back to an old one;
  leaving waypoint flight clears the altitude and lateral loops ([#70](https://github.com/wboayue/stable-flight/issues/70)).
- `CopterStabilizer` clears its attitude and yaw loops during rate flight, so attitude
  flight after it starts from fresh state ([#70](https://github.com/wboayue/stable-flight/issues/70)).
- **Breaking:** `Command` is now a struct `{ arm, input }` (`#[non_exhaustive]`; build
  with `Command::new`, `armed`, or `disarmed`), and the old enum is `ControlInput`
  (`Attitude`/`Rate`/`Waypoint`). `arm` moved off `AttitudeCommand` and `RateCommand`, so
  it covers waypoint flight too. `Guidance::guide` takes `&ControlInput`. `WaypointFollower`
  emits armed commands. `ControlInput` implements `Default` (zero-throttle attitude), so
  `Command` and `testing::ConstantCommander` derive it (disarmed) ([#125](https://github.com/wboayue/stable-flight/issues/125)).
- **Breaking:** `FlightLoop` gates the motors on `Command::arm`: a disarmed tick skips
  guidance, the stabilizer, and the mixer, actuates `ActuatorCommand::default()`, and
  records a neutral setpoint and zero control. Each disarmed → armed edge re-primes guidance
  and the stabilizer ([#125](https://github.com/wboayue/stable-flight/issues/125)).
- **Breaking:** `Guidance::prime`/`Stabilizer::prime` now also reset runtime state and run on
  every arm, not just once. `CopterStabilizer::prime` clears its rate, attitude, and yaw
  controllers; `CopterGuidance::prime` also clears its altitude and lateral controllers
  and held tilt ([#125](https://github.com/wboayue/stable-flight/issues/125)).
- `AttitudeController` and `BodyRateController` no longer kick the derivative term on the
  first tick after construction (they differenced against a zero previous error); like
  `YawController`, the first tick is P + I only
  ([#68](https://github.com/wboayue/stable-flight/issues/68)).
- **Breaking:** `AttitudeSetpoint` carries `roll_rad`/`pitch_rad` in radians instead of
  `roll`/`pitch` as a fraction of the stabilizer's maximum tilt, so retuning
  `max_angles_rad` no longer rescales the lateral loop's gain. `AttitudeController::control`
  takes radians and saturates them to `max_angles_rad` (NaN → level)
  ([#72](https://github.com/wboayue/stable-flight/issues/72)).
- **Breaking:** `CopterGuidanceConfig::max_tilt` (fraction) is now `max_tilt_rad` (radians,
  in `(0, π/2)`; `DEFAULT_MAX_TILT` → `DEFAULT_MAX_TILT_RAD`, 27°). New
  `max_stick_angles_rad` (default 45° per axis, `DEFAULT_MAX_STICK_ANGLE_RAD`) scales
  attitude-mode sticks to radians; out-of-range sticks now saturate in guidance
  ([#72](https://github.com/wboayue/stable-flight/issues/72)).
- **Breaking:** `LateralController::control` returns horizontal acceleration (m/s²,
  unsaturated) instead of normalized tilt. `CopterGuidance` flies it as a tilt of
  `atan(|a| / g)` (was `a / g` × max tilt). Default lateral gains scaled by π/4
  (P 0.8 → 0.63, D 1.6 → 1.26) to keep the small-angle loop gain
  ([#71](https://github.com/wboayue/stable-flight/issues/71)).
- **Breaking:** `Command` and `Setpoint` are `#[non_exhaustive]`; `Guidance::guide` and
  `Stabilizer::stabilize` document the neutral fallback for a variant they can't fly
  ([#73](https://github.com/wboayue/stable-flight/issues/73)).
- **Breaking:** every PID controller's `control`, `CopterGuidance::guide`, and
  `CopterStabilizer::stabilize` panic unless `dt` is positive and finite
  ([#177](https://github.com/wboayue/stable-flight/issues/177)). `AltitudeController` and
  `LateralController` used to accept any `dt`; attitude, guidance, and the stabilizer accepted
  an infinite one.
- **Breaking:** the remaining public physical quantities carry unit suffixes
  ([#161](https://github.com/wboayue/stable-flight/issues/161)):
  - `VehicleState`: `roll`/`pitch`/`heading` → `roll_rad`/`pitch_rad`/`heading_rad`,
    `angular_velocity` → `angular_velocity_radps`.
  - `WaypointCommand`: `position_m`, `velocity_mps`, `altitude_m`, `cruise_speed_mps`,
    `roll_rad`.
  - `AttitudeSetpoint::heading` → `heading_rad`.
  - `CopterGuidanceConfig::navigation_period` → `navigation_period_s`
    (`DEFAULT_NAVIGATION_PERIOD` → `DEFAULT_NAVIGATION_PERIOD_S`);
    `AltitudeControllerConfig::max_rate` → `max_rate_mps`; `max_rate` → `max_rate_radps`
    on the attitude, lateral, and yaw configs; `AttitudeControllerConfig::max_angles` →
    `max_angles_rad`; `DEFAULT_ACCEPTANCE_RADIUS` → `DEFAULT_ACCEPTANCE_RADIUS_M`.
  - `testing::conformance::Scenario`: `body_rates_deg_s` → `body_rates_degps`,
    `position_neu_m`, `velocity_neu_mps`, `acceleration_neu_mps2`; mismatches report
    the new names (e.g. `roll_rad`, `angular_velocity_radps.z`).
  - `Vec2`/`Vec3` are plain `f32` aliases (no type parameter).

  Normalized fields (`AttitudeCommand`/`AttitudeSetpoint` roll and pitch, rate commands and
  setpoints, thrust, throttle) and `max_integral` keep their names.
- **Breaking:** `CopterGuidanceConfig` embeds the loop configs, matching
  `CopterStabilizerConfig` ([#116](https://github.com/wboayue/stable-flight/issues/116)):
  `altitude_gains` + `hover_thrust` → `altitude: AltitudeControllerConfig` and `lateral_gains` →
  `lateral: LateralControllerConfig`, so every loop setting (e.g. `max_integral`) is reachable.
  It is `Copy` and has `validate()`; `CopterGuidance::new` panics with the failing field
  (e.g. `altitude.hover_thrust`). Its defaults are unchanged.
- `AltitudeControllerConfig`, `LateralControllerConfig`, and `YawControllerConfig` default to
  the flight-checked QuadCopterX gains (altitude P 1.5/I 0.2/D 2.0, lateral P 0.8/D 1.6,
  heading P 2.5/D 0.06) that `CopterGuidanceConfig`/`CopterStabilizerConfig` used to hard-code
  over untuned controller defaults. The composites' defaults, and so the copter's flight, are
  unchanged.
- **Breaking:** `AltitudeController::control` takes an `AltitudeTarget` (altitude, climb rate)
  and an `AltitudeEstimate` (altitude, climb rate, attitude) after `config` and `dt`, in place
  of five positional arguments ([#179](https://github.com/wboayue/stable-flight/issues/179)).
  Behavior is unchanged.
- **Breaking:** the unused `AltitudeControllerConfig::max_rate_mps`
  ([#115](https://github.com/wboayue/stable-flight/issues/115)) and
  `LateralControllerConfig::max_rate_radps`
  ([#71](https://github.com/wboayue/stable-flight/issues/71)) are removed; neither limited
  anything.
- **Breaking:** the PID controllers (`BodyRateController`, `AttitudeController`,
  `YawController`, `AltitudeController`, `LateralController`) hold only state; their config is
  passed to every call: `control(&config, dt, …)`
  ([#145](https://github.com/wboayue/stable-flight/issues/145)). A config change applies on the
  next tick without resetting the integral. `new()` and `with_config()` are removed (build
  with `Default`), as are `BodyRateController`'s public `gains`/`yaw_gains`/`max_integral` and
  `YawController::config`. Invalid limits no longer panic at construction: check them with
  `validate()`; inside `control` a negative or NaN limit acts as 0 instead of panicking.
  `AltitudeControllerConfig` and `LateralControllerConfig` derive `Copy`.
- **Breaking:** `CopterStabilizerConfig` is plain `Copy` data: it holds controller configs, not
  controllers ([#144](https://github.com/wboayue/stable-flight/issues/144)).
  `rates_controller: BodyRateController` → `body_rate: BodyRateControllerConfig`,
  `attitude_controller: AttitudeController` → `attitude: AttitudeControllerConfig`, and
  `yaw_gains: Gains` → `yaw: YawControllerConfig`. Values hard-coded in
  `CopterStabilizer::new` are now settable: the heading loop's `max_integral` and
  `max_rate_radps`, and `full_scale_rates_radps` (body rates a full-scale `RateSetpoint` maps
  to). Defaults are unchanged. `BodyRateControllerConfig` and `YawControllerConfig` derive `Copy`.
- **Breaking:** `BodyRateController`'s `integral_term` and `last_error` are private (controller
  state, not configuration).
- **Breaking:** `VehicleState` translational fields are plain SI `f32` vectors with
  unit-suffixed names: `position` → `position_m`, `velocity` → `velocity_mps`,
  `acceleration` → `acceleration_mps2`. The `uom` dependency is removed
  ([#141](https://github.com/wboayue/stable-flight/issues/141)). Conformance mismatches
  report the new field names (e.g. `position_m.z`).
- **Breaking:** vector and quaternion types come from `nalgebra` 0.35 instead of
  `robomath` ([#87](https://github.com/wboayue/stable-flight/issues/87)).
  `VehicleState::attitude` is a `UnitQuaternion<f32>`; the other vector fields
  (`VehicleState`, `WaypointCommand`, `ControlOutput::moments`, controller
  configs) are `Vec2`/`Vec3`. Migrate:
  - `vec3(x, y, z)` / `Vec3 { x, y, z }` → `Vec3::new(x, y, z)`; zero vectors →
    `Vec3::zeros()`.
  - `a * b` of two vectors (element-wise in robomath) → `a.component_mul(&b)`;
    `a / b` → `a.component_div(&b)`.
  - Vector `clamp(min, max)` → `v.map(|c| c.clamp(min, max))`; `is_finite()` →
    `v.iter().all(|c| c.is_finite())`.
  - `Quaternion::from_euler(yaw, pitch, roll)` →
    `UnitQuaternion::from_euler_angles(roll, pitch, yaw)` (note the order).
  - `Quaternion { w, x, y, z }` / `Quaternion::new(w, x, y, z)` →
    `UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z))`, or
    `new_unchecked` to keep the components as given.
  - `q.roll()`/`q.pitch()`/`q.yaw()` → `stable_flight::euler_angles(&q)`.
  - `rotation_matrix_i_wrt_b()` → `to_rotation_matrix()` (the same matrix);
    `rotation_matrix_b_wrt_i()` → `to_rotation_matrix().transpose()`.
  - `robomath::to_radians` → `f32::to_radians`.
- **Breaking:** `Stabilizer` is the inner loop only: `stabilize` takes a
  `&Setpoint` instead of a `&Command`. `CopterStabilizer` no longer has the
  altitude and lateral controllers or the heading-hold target (now in
  `CopterGuidance`), and `CopterStabilizerConfig` drops `altitude_gains` and
  `lateral_gains` (now `CopterGuidanceConfig`). Wire `CopterGuidance` into
  `FlightLoop::new` between the vehicle and the stabilizer. Flight behaviour is
  unchanged: the guidance + stabilizer pair matches the old stabilizer bit for bit.
- **Breaking:** `CopterStabilizer`'s controller and `max_rates` fields are no
  longer public, and `CopterStabilizer::config` is gone (it was a stale copy).
- **Breaking:** the PID controllers and `Gains` moved from
  `stabilizers::controllers`/`stabilizers` to `controllers`, since guidance uses
  them too (`use stable_flight::controllers::{Gains, AttitudeController, …}`).
- Waypoint flight runs the lateral position loop at 50 Hz instead of every tick
  (250 Hz in the RealFlight examples), holding its roll/pitch in between; the
  altitude loop still runs every tick. Set `navigation_period: 0.0` for the old
  behaviour.
- The attitude-mode heading-hold target slews at 1 rad/s at full yaw stick, scaled by
  `dt`. It moved a fixed 0.0001 rad per tick, so its rate depended on the loop rate,
  and that rate (0.025 rad/s at 250 Hz) was tuned against RealFlight's
  double-radian heading bug ([#62](https://github.com/wboayue/stable-flight/issues/62)).
- **Breaking:** `BodyRateController` is a plain PID: the moment is
  `P·e + I·∫e + D·ė`, saturated to [-1, 1] per axis. It used to normalize by a
  `dt`-dependent "max moment" and scale by 1000, so uniform gain changes had no
  effect and the effective gain was ~370× typical. Gains are now in normalized
  moment per rad/s; defaults are P 0.15, D 0.003 (roll/pitch) and P 0.3, I 0.1
  (yaw), with `max_integral` 1.0. `BodyRateControllerConfig` drops
  `max_angular_rates` (it only fed the normalization) and gains `yaw_gains`,
  since yaw authority (differential prop drag) needs its own gains
  ([#62](https://github.com/wboayue/stable-flight/issues/62)).
- `CopterGuidanceConfig::default()` lateral gains are P 0.8, I 0, D 1.6 (ζ ≈ 0.8),
  flight-checked on RealFlight's QuadCopterX. The old P 1.5, D 0.075 (ζ ≈ 0.03)
  orbited the target in widening circles
  ([#108](https://github.com/wboayue/stable-flight/issues/108)).
- The lateral controller saturates the length of its roll/pitch request instead of
  each axis, so a far diagonal waypoint keeps its direction (per-axis clipping
  bent it toward 45°).
- `QuadCopterMixer` gives attitude priority over thrust: when a motor would pass
  full throttle, all four drop by the excess so the moments survive, instead of
  clipping that motor.
- **Breaking:** `CopterStabilizer` tracks attitude setpoints from
  `VehicleState::attitude` (the quaternion) only; `AttitudeSetpoint::reference` and
  `AttitudeReference` are removed. They existed only because RealFlight's Euler
  fields were wrong ([#63](https://github.com/wboayue/stable-flight/issues/63)).
- `CopterStabilizer`'s thrust curve saturates its input (NaN → 0), so it meets the
  input policy itself instead of relying on callers. No change for the built-in
  flight modes, which already saturated; only a custom `Guidance` sending
  out-of-range thrust sees it.
- `Guidance::prime` and `Stabilizer::prime` default to doing nothing.

### Removed

- **Breaking:** `BodyRateController::reset` and `YawController::reset`
  ([#177](https://github.com/wboayue/stable-flight/issues/177)). The controllers hold only
  state, so `*controller = Default::default()` does the same.

### Fixed

- The command's arm flag was ignored, so a disarmed vehicle still drove its motors ([#125](https://github.com/wboayue/stable-flight/issues/125)).
- `AltitudeController` feeds gravity forward and compensates tilt correctly:
  thrust is `hover_thrust · (g + a) / (g · cos tilt)`. It computed `(a − g) / cos`,
  so zero error meant zero thrust (the integral carried all of hover, making climbs
  slow) and a bank *lowered* thrust. The tilt boost stops at 60°, the integral only
  accumulates within 1 m of the target (so a long climb can't wind it up into an
  overshoot), and its clamp honours `max_integral` (it was hard-coded to ±100). Default altitude
  gains are P 1.5, I 0.2, D 2.0 (ζ ≈ 0.8; were P 3, I 0.3, D 0.15, ζ ≈ 0.04) and
  `AltitudeControllerConfig::max_integral` defaults to 5
  ([#96](https://github.com/wboayue/stable-flight/issues/96)).

- `QuadCopterMixer`'s yaw sign: a positive yaw moment now speeds up the CCW props
  (front right, rear left), turning the nose right as the frame contract requires.
  It turned the nose left; RealFlight's nose-left-positive yaw rate hid this until
  the frame fix ([#62](https://github.com/wboayue/stable-flight/issues/62)).

- Waypoint flight far from the target no longer panics the flight loop. The
  lateral controller's roll/pitch request is saturated to the normalized
  [-1, 1] range (it previously passed an unbounded `accel / g`, e.g. -4.6),
  its integral term is clamped to the configured `max_integral` (it was
  unbounded), and `AttitudeController::control` saturates out-of-range or NaN
  inputs instead of `assert!`ing. Far from a waypoint the vehicle now commands
  full tilt (`max_angles_rad`) toward it. A NaN target, position, or attitude
  estimate no longer sticks in the lateral or attitude integral.
  `LateralController::with_config` now rejects a negative or non-finite
  `max_integral` (it would otherwise panic inside the loop).
- Holding yaw in attitude mode no longer panics the flight loop after a few
  minutes. The heading-hold target grew without bound until the yaw
  controller's `[-2π, 2π]` `assert!` fired; it is now wrapped to `[-π, π)` each
  tick. `YawController::control` accepts any finite target (it already works on
  the wrapped error), and a non-finite target or estimate returns a zero rate
  without corrupting the controller state or kicking the derivative afterwards.
  A heading-hold target that is NaN (e.g. primed from a NaN estimate) re-latches
  from the next valid estimate instead of disabling heading hold.
- Out-of-range or non-finite commands and estimates no longer panic the flight
  loop (see architecture.md, "Input Policy"):
  - `QuadCopterMixer::mix` saturates thrust to [0, 1] and moments to [-1, 1] (NaN →
    0) instead of asserting; rate-mode throttle, which was never clamped, used to
    reach this assert.
  - Attitude-command throttle and yaw saturate instead of asserting: yaw in
    `CopterGuidance`, throttle in `CopterStabilizer`'s thrust curve, roll and pitch
    in the attitude controller.
  - `BodyRateController::control` returns zero moments on non-finite desired or
    estimated rates instead of panicking.
  - `AttitudeController::control` returns zero rates on a non-finite error.
  - `AltitudeController::control` returns hover thrust on a non-finite altitude or
    velocity error, and keeps it out of the integral. Before, a NaN estimate stuck
    there and cut the motors for good once the mixer saturated NaN thrust to 0.
  - `LateralController::control` returns level (no tilt) on a non-finite error; an
    infinite error used to command full tilt.
  - Rate-mode NaN rate commands mean "hold zero rate" (per-axis saturation);
    `clamp` let NaN through to zero every moment.
  - `AttitudeController::with_config` and `YawController::with_config` reject a
    negative or non-finite `max_integral`/`max_rate_radps`, and
    `BodyRateController::with_config` a negative or non-finite `max_integral`,
    which would otherwise make a per-tick `clamp` panic.

  Both controllers skip the derivative on the first finite tick after a
  non-finite one.

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight
