# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.3.0] - 2026-11-12

### Added

- `plane-autotune-xplane` example (`just fly-xplane-plane-autotune`): plane autotune of the
  Alia 250's cruise side from an attitude hold 6 km offshore (`flights/alia_plane_tune.csv`),
  then home and a landing on the spawn (#537).
- `plane-probe-xplane` holds the rudders open loop on each leg (`--yaw-hold`, default 0.15;
  4 s right, 8 s left, 4 s right) for the yaw damping and sideslip fit; `--no-pulses` and
  `--no-roll-holds` skip the pulses and the roll holds. The log's times are the recording's
  `wall_s` (#458).

### Changed

- **Breaking (behavior):** `airframes::alia250()`'s plane yaw rate feedforward is 0.999
  (`PSTAB_YAW_FF`, was 3.57), the sim Alia's from its yaw damping and rudder power fitted to
  X-Plane's rudder holds (#458). On `flights/alia_transition_lap.csv` it holds a third of the
  rudder in plane flight (mean 0.030 against 0.097) and flies the lap as before: peak sink
  3.31 m/s against 3.33, handoff dip 4.1 m against 4.0, roll RMS 1.72° against 1.71°.
- `plane-autotune-xplane` lets the Alia's roll design clip for 0.3 s, as its roll step tests
  may: holding its 15°/s roll rate limit takes more aileron than the design's 0.8 (#458).
- **Breaking (behavior):** `airframes::alia250()`'s plane side answers altitude errors on a
  3 s total energy time constant (`PGUID_TCONST`, was 5 s) and flies pitch rate I 1
  (`PSTAB_PIT_I`, was 0.5) (#547). A descent turning at the 20° bank cap sank 5.65 m/s past
  its 5 m/s limit on `flights/alia_descent_lap.csv`; it now peaks at 5.14–5.24 m/s over
  three runs. The handoff out of a forward transition sinks faster, 3.2 m/s against 2.1, and
  dips 4.2 m against 2.4 on `flights/alia_transition_lap.csv`.
- **Breaking (behavior):** `airframes::alia250()`'s plane roll and pitch rate gains and
  attitude time constants are the plane design's in X-Plane, the median of three runs: roll
  P 0.0138 I 2.56 FF 2.81, τ 0.707 s; pitch P 0.482 I 10 FF 0.859, τ 0.876 s (#537). Pitch
  rate steps reach 0.82 of the step, against 0.50 on the hand tune.

## [0.2.0] - 2026-10-30

First release.

### Added

X-Plane 12 simulator backend over [xplane-bridge](https://crates.io/crates/xplane-bridge)
0.1 (#351):

- `XPlaneVehicle`: `StateSource` + `Actuator` over UDP datarefs. `connect` subscribes the
  state, resubscribes until every dataref has a value (or `ConnectError::NotReady` names the
  missing), and takes the throttles (and, with `surfaces` or `neutral_wings`, the control
  surfaces).
- Each `XPlaneVehicle` tick polls without blocking and writes the mapped engines and surfaces
  once per frame. A silence or a lost subscription resubscribes, returning frames take control
  again, and dropping the vehicle hands control back to X-Plane.
- `vehicle_state`: an `XPlaneSample` in the core frame contract (quaternion and body rates as
  reported, OpenGL local frame to north/east, elevation above sea level as up, true airspeed).
- `LocalOrigin`: position relative to where the aircraft started (the bridge's `--origin`),
  continuous when X-Plane moves its local origin, re-anchored when the aircraft relocates.
- `ActuatorMapping`: motor slots to `ENGN_thro_use` elements, idle engines held at 0, servo
  slots to wing surfaces (`SurfaceLink`, `Surface`: degrees at each end of the servo's
  range, #392), and wings whose surfaces are held at 0°.
- `StateDatarefs`, `XPlaneSample`, `wait_ready`: the raw state subscriptions, for tools that
  read X-Plane without flying it.
- `capture`: raw samples to and from CSV, with a writer's own columns after them; the
  `capture-state-xplane` example records them read-only while X-Plane flies, for
  `check_frames --sim xplane`.
- `airframes::alia250()`: the Alia 250 as a quadplane (mixer, engine and surface mapping,
  gains): its hover from an engine sweep and a sim autotune, flown in X-Plane (#351), its
  pitch rate P from the X-Plane autotune (#540), and its angle I from step tests there
  (#576); its plane side and transitions tuned on `stable-flight-sim`'s `alia250()` (#392).
  Plane flight below 20 m falls back to a hover (`TRAN_FW_MIN_HGT`, #450).
- Examples flying the Alia 250: `quadplane-manual-xplane` (joystick; `--record`, `--param`),
  `quadplane-auto-xplane` (a waypoint script with plane legs,
  `flights/alia_transition_lap.csv`, #494), and `copter-companion-xplane` (the link in copter
  flight, QGC through the bridge).
- `XPlaneVehicle::state_is_stale`: no frame for 0.5 s, a lagging dataref, or X-Plane paused;
  `XPlaneVehicle::is_paused`.
- The `copter-autotune-xplane` example autotunes the Alia's hover on its lift rotors
  (`StagePlan::large_copter_search` from `flights/alia_hover.csv`, `--step-test`,
  `--stages none`), waiting while the state is stale (#540).
- `sweep::EngineSweep` and `engine-sweep-xplane`: a multirotor's yaw sign per diagonal,
  liftoff throttle, motor lag, and moment response, measured on the runway.

[unreleased]: https://github.com/wboayue/stable-flight/compare/stable-flight-sitl-xplane-v0.3.0...HEAD
[0.3.0]: https://github.com/wboayue/stable-flight/releases/tag/stable-flight-sitl-xplane-v0.3.0
[0.2.0]: https://github.com/wboayue/stable-flight/releases/tag/stable-flight-sitl-xplane-v0.2.0
