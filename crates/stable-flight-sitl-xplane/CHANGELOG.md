# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
  gains): its hover from an engine sweep and a sim autotune, flown in X-Plane (#351), and its
  pitch rate P from the X-Plane autotune (#540); its plane side and transitions tuned on
  `stable-flight-sim`'s `alia250()` (#392). Plane flight below 20 m falls back to a hover
  (`TRAN_FW_MIN_HGT`, #450).
- Examples flying the Alia 250: `quadplane-manual-xplane` (joystick; `--record`, `--param`),
  `quadplane-auto-xplane` (a waypoint script with plane legs,
  `flights/alia_transition_lap.csv`, #494), and `copter-companion-xplane` (the link in copter
  flight, QGC through the bridge).
- `XPlaneVehicle::state_is_stale`: no frame for 0.5 s, a lagging dataref, or X-Plane paused;
  `XPlaneVehicle::is_paused`.
  The `copter-autotune-xplane` example autotunes the Alia's hover on its lift rotors
  (`StagePlan::large_copter_search` from `flights/alia_hover.csv`, `--step-test`,
  `--stages none`), waiting while the state is stale (#540).
- `sweep::EngineSweep` and `engine-sweep-xplane`: a multirotor's yaw sign per diagonal,
  liftoff throttle, motor lag, and moment response, measured on the runway.
