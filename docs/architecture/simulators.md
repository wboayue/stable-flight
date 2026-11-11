## Simulator Backends

Each simulator gets its own `stable-flight-sitl-<simulator>` crate, so its dependencies
(protocol clients, FFI, system libraries) never reach the core, the harness, or another backend.

* **Dependency rule.** A backend depends on the core, its simulator client, and small shared
  utilities, and on `stable-flight-sitl` only as a dev-dependency (examples), so it works with
  any runner: the sitl runners, a HIL rig, or a third-party harness. CI fails if a backend lists
  `stable-flight-sitl` under `cargo tree -e normal`.
* **Frame contract.** Every `StateSource` reports `VehicleState` in one convention (the table in
  its rustdoc), whatever the simulator uses natively, so nothing downstream knows which
  simulator is attached. The body frame is the airframe's own: a tailsitter's is the airplane's
  (x along the nose, as ArduPilot does), so it hovers at pitch +90°. A simulator body frame that
  differs is a constant rotation applied at the `StateSource` boundary
  (`RealFlightConfiguration::body_rotation`, set per airframe); on hardware the same rotation is
  the IMU driver's or estimator's board orientation
  ([#241](https://github.com/wboayue/stable-flight/issues/241)). Controllers never see it.
* **Timing contract.** `FlightLoop` calls `state` then `actuate` once per tick, so a backend
  says which call talks to the simulator and how old its state is. The `Vehicle` rustdoc is the
  reference; each backend documents its own timing.
* **Conformance.** `stable_flight::testing::conformance` (core `testing` feature) lists physical
  scenarios and the `VehicleState` each must yield; `conformance_tests!` runs a backend's native
  fixtures through them. It lives in the core so backends and in-core estimators test against it
  without the harness.
* **Airframes.** Tuned gains, the mixer, channel mapping, and body rotation live in the
  backend's `airframes` module, next to the simulated model they were tuned on.
* **Per-backend mapping types.** Each backend has its own `ActuatorMapping` (and `Airframe`):
  RealFlight maps slots to twelve 0–1 channels, X-Plane maps motors to engines and servos to
  wing surfaces in degrees (`SurfaceLink`: each end of the servo's range its own deflection, so
  a surface's sign and uneven travel are data). A shared type waits until a third backend shows a common shape.

### X-Plane

X-Plane runs free in real time and streams datarefs over UDP at most once per rendered frame
(60–120 Hz), with no lockstep, so `XPlaneVehicle` runs under `RealTimeRunner`, holding each
state for the ticks between frames, rather than a lockstep runner. Its local frame has a
whole-degree origin that moves when the aircraft relocates or flies far from it;
`LocalOrigin` keeps position relative to where the aircraft started (the bridge's
`--origin`), absorbing a moved origin from the last position and velocity rather than by
geodesy, which would mix X-Plane's projection with a flat-earth one at the jump. Up is the
elevation above sea level, not `local_y`, which is height over the plane tangent at the local
origin and drifts from the ground with distance. Rejected: horizontal position from latitude and
longitude (`f32` on the wire: 0.4–0.8 m steps).

`actuate` writes once per received frame: X-Plane applies only the last write before each
frame, and between frames the state, so the commands, barely move (only `dt`-driven ramps and
integrators). A quadplane writes 21 datarefs a frame; writing every tick would send five times
that for nothing. Rejected: writing only changed values, which needs a periodic refresh to
recover a lost datagram, where a write per frame refreshes everything anyway.

### In-Repo Simulator

RealFlight and X-Plane can't run in `cargo test` or CI, so without another plant
the pipeline's closed loop is only checked by hand. `stable-flight-sim` is that plant: a rigid
body on a ground plane in deterministic wind, as a multirotor (motor lag, quadratic thrust,
linear drag), an airplane (stability-derivative aerodynamics, servo-driven surfaces, a
tractor prop), or a tailsitter (the airplane's aerodynamics with its motors along the wing and
the elevons in their wash), stepped in lockstep with `FlightLoop` (no wall clock, only seeded noise), so a
test flight repeats bit for bit and runs in milliseconds. It aims at plausible dynamics, not fidelity; RealFlight stays
the check for realism.

* **Why not JSBSim (or Gazebo).** An external engine brings a C++ build or a process to
  manage in CI, nondeterminism from its own scheduling, and no `no_std` path. A small Rust
  model is enough for what the tests assert (loops close, modes sequence, limits flag) and,
  being `no_std` with fixed-size math, can later drive a board in hardware-in-the-loop runs.
* **Geometry from the mixer.** Each motor's torque comes from its `TableMixer` row's factors
  (a factor of 1 at the model's moment arm), and each servo moves the aileron, elevator, and
  rudder axes its row names (an axis's deflection is the servos' least-squares fit to the
  factors, so elevons and separate surfaces read back alike). The sim and mixer can't
  disagree on motor order, spin, or surface slots, and there is no second table to keep in
  sync.
* **Four vehicles over shared parts.** `SimCopter`, `SimPlane`, `SimTailsitter`, and
  `SimQuadPlane` are separate types over crate-private components: lockstep timing, the
  actuator delay, wind, gyro noise, the rigid body, and ground contact live once; each vehicle
  adds only its loads. The airplane, tailsitter, and quadplane share one fixed-wing dynamics:
  the tailsitter adds its motor arm, wash, and resting on its tail; the quadplane adds lift
  rotors along body `−z`, laid out by the mixer's motor rows as on the copter, and takes its
  pusher from the mixer's one pusher row (`MixRow::pusher`), so the slot has one source. A
  public generic
  vehicle (`SimVehicle<P: Physics>`) was rejected: its trait would have to absorb what the
  vehicles differ in (how each rests on the ground, plane-only flow angles) and stay stable as a
  public API, for less duplication than the vehicles' thin builder glue. A generic VTOL
  (per-rotor position and thrust axis, tilt servos) was rejected for the quadplane too: fixed
  lift rotors need no axis per rotor, and taking geometry from the model instead of the mixer
  would let the two disagree. It waits for a tiltrotor.
* **Prop wash by momentum theory.** A tailsitter's elevons hold it in a hover only in the
  props' wash, so the surface terms see the actuator disc's far-wake pressure (thrust over
  disc area) on top of the free stream's; lift, drag, and damping see the free stream.
  Sinking tail first, the air meets the wash head on and the surfaces see the static jet
  less the sink, so authority fades with the sink, as the eXplora's did in RealFlight (#123).
  ArduPilot's SITL instead adds 20 m/s × throttle to the airspeed of every moment
  (`SITL::Plane::getTorque`), which ties authority to throttle rather than thrust and damps
  the body in a still hover.
* **Bounded at any angle of attack.** A tailsitter flies broadside to the air in a hover and
  backward when it sinks, so past the stall drag blends into a flat plate's (`2·sin²α`)
  with the lift, and static pitch stability acts on `sin α`. The attached-flow formulas,
  extrapolated, give a drag coefficient near 6 broadside and a pitch moment that flips sign
  across reversed flow.
* **Spec bands, not snapshots.** Step-response tests hold each loop to ArduPilot's and PX4's
  autotune targets (cited in `tests/step_response.rs`), not to whatever the sim measured, so
  a regression and a tuning gap both fail. Each airframe's roll and pitch gains come from
  running the core autotune on the sim.
* **Perturbation ranges, not a fitted plant.** Delay, gyro noise and vibration, a disturbance
  moment, and motor mismatch are parameters to sweep, so the autotune *method* is checked on
  many plausible plants and transfers to hardware. Fitting the sim to RealFlight's QuadCopterX
  (#365) was rejected: it would match one simulated plant, and its gains still wouldn't be a
  board's.
* **Not a backend crate.** It meets the frame and timing contracts and covers every
  conformance scenario like a backend, but it has no simulator client, so it isn't a
  `stable-flight-sitl-<simulator>` crate. Core can't take it as a dev-dependency (a cycle
  builds two copies of core), so the closed-loop tests live in its `tests/`.
