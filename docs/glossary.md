# Glossary

Control and airframe terms as this repo uses them, for readers who aren't controls specialists.
Code comments and docs link here instead of redefining a term.

**Airframes and flight**

- **Regime / flight mode:** a `FlightRegime` is what the commander asks for (copter or plane);
  a `FlightMode` is what guidance is flying, including the transitions between the two.
- **Copter / hover:** rotor-borne flight; the rotors carry the weight and tilt to move.
- **Plane / wing-borne:** forward flight where the wing carries the weight and control
  surfaces steer.
- **Tailsitter:** a VTOL that hovers nose-up on its propellers and pitches 90° over to fly as
  a plane (the eXplora). `View::tailsitter` flies its hover on the copter controllers in a
  rotated frame.
- **Quadplane / lift+cruise:** a VTOL with separate lift rotors for hover and a pusher
  propeller for cruise (Titan Cobra, Alia 250).
- **Transition:** the handover between regimes. Forward: copter to plane, gaining airspeed
  until the wing carries the weight. Back: plane to copter; a quadplane's back transition pitches
  the nose up to slow it, as the pusher stops and the wing carries less and less.
- **Quadchute / fallback:** a VTOL giving up plane flight for copter flight on its own: a
  forward transition that times out or rolls too far, or plane flight below its minimum
  height (`TRAN_FW_MIN_HGT`). Plane flight is then refused until copter flight is asked for,
  and a waypoint is flown as a hover where copter flight took over; sticks keep flying (PX4's
  name; `ModeRefusal` says why). A quadplane's lift rotors take all the lift at once, without
  the back transition's ramp.
- **Blend:** during a quadplane transition, the hover and wing-borne setpoints flown together,
  each weighted by a lift share (`Setpoint::Blend`).
- **FBWA (fly-by-wire A):** plane stick mode where the sticks command bank and pitch angles
  and the loops hold them; centered sticks fly level.
- **Acro:** stick mode where the sticks command rotation rates; centered sticks hold the
  current attitude.
- **Maneuver:** a scripted sequence of body-rate segments flown by `ManeuverRunner` (a roll,
  a loop, a doublet), then a recovery to level.
- **Path hold:** a maneuver segment's steering back onto its path (a roll's line, a loop's
  circle) with pitch and yaw rate on top of its own rate, harder the further off it
  (`maneuver::PathConfig`). Without it a roll sinks as its lift tips over.
- **Prop wash:** the propeller's slipstream; on a tailsitter it flows over the elevons, so
  their authority grows with thrust as well as airspeed, and damps the wing's turns where it
  covers the span, the only damping in a hover.
- **Offboard:** setpoints sent by a companion computer or ground station rather than sticks.
- **Failsafe:** what the vehicle flies on its own when the link goes quiet: a fallback
  commander takes over ([architecture](../architecture.md#link-failsafe)).

**Frames and signs**

- **NED / FRD:** the world frame is north-east-down, the body frame forward-right-down. Signs,
  units, and which quantities read positive up are in the `VehicleState` rustdoc
  (core `src/lib.rs`).
- **Roll, pitch, yaw / heading:** rotation about the body's forward, right, and down axes.
  Positive is right wing down, nose up, nose right. Heading is yaw measured clockwise from
  north.
- **Moment:** a torque about one body axis, the stabilizer's output alongside thrust.

**Control loops**

- **Setpoint:** the target a loop drives toward (an angle, a rate, a velocity).
- **Error:** setpoint minus measurement.
- **Cascade:** loops nested outer to inner, each one's output the next one's setpoint. For a
  copter: position → velocity → attitude angle → body rate → moment. Inner loops run faster
  and are tuned first.
- **PID:** a loop's output as the sum of three terms: **P** proportional to the error, **I**
  the accumulated error over time (removes steady offsets), **D** the error's rate of change
  (damps overshoot). Gains `kp`, `ki`, `kd` scale each term.
- **Feedforward:** output computed from the setpoint itself (e.g. the expected moment for a
  commanded rate) rather than from the error, so the loop doesn't wait to fall behind.
- **Notch filter:** a filter that cuts one narrow band of frequencies and passes the rest,
  for a vibration tone (a motor's) that a low-pass just below it would barely touch; here a
  biquad on the roll and pitch rates ahead of the gyro low-pass
  (`CopterStabilizerConfig::gyro_notch`).
- **Integral windup / anti-windup:** while the output is pinned at a limit, the I term keeps
  growing and overshoots once the limit releases. Anti-windup stops it accumulating (a
  `max_integral` clamp (on the copter rate loop, the most moment the I term holds, as
  ArduPilot's IMAX), holding it while saturated in the error's direction, or fading it
  toward zero while saturated, as the TECS pitch integral does).
- **Derivative kick:** a D-term spike when the setpoint jumps or a loop starts with no previous
  sample; the loops skip it on their first tick, and the copter rate loop avoids the setpoint
  jump's by taking D on the measurement.
- **D on measurement:** a PID whose D term damps the measured value's change instead of the
  error's: the same inside the loop, but a setpoint step moves the output through P and I only.
- **Trim:** the steady output that holds the vehicle in balance with no error (a plane's
  elevator at cruise, a hover's thrust). A trim term carries it so the I term doesn't have to.
- **Saturation:** a value pinned at its limit (tilt cap, thrust range, motor range). Reported
  per tick in `Saturation`, so outer loops and telemetry know the request wasn't met.
- **Headroom:** the margin left between a command and its limit, e.g. motor range kept free so
  differential thrust can still make moments at full collective.
- **Mixer:** maps thrust and moments to per-motor and per-servo commands for one airframe.
- **Tilt cap:** the most a copter may lean to chase a position, leaving thrust to hold altitude.
- **Slew / slew rate:** a limit on how fast a value may change per second, so a step request
  becomes a ramp.
- **Leash:** a limit on how far a moving target may run ahead of the vehicle.
- **Braking curve:** the speed toward a target that a set deceleration can still bring to zero
  on it, `√(2 · a · distance)` far out and proportional to distance close in (ArduPilot's
  square-root controller). Waypoint descents and lateral legs, shaped (`GUID_LAT_ACC`) or
  braked unshaped (`GUID_LAT_BRK`), slow along one, so braking starts in time.
- **Deadband:** a band around zero where small inputs are ignored, so noise doesn't cause
  constant small corrections.
- **Low-pass filter:** smooths a noisy signal by attenuating fast changes, at the cost of lag;
  its cutoff frequency divides what passes from what's cut, and more poles cut harder past it.
  The copter's rate loop flies its gyro rates through one (the gyro filter).
- **High-pass filter:** the signal less its low-pass: passes changes, drops what holds
  steady. The TECS high-passes the airspeed rate so a steady acceleration isn't a rate
  error.
- **Airspeed scaling:** a control surface's moment grows with dynamic pressure (airspeed
  squared), so plane rate gains are scaled by trim airspeed over current airspeed to keep the
  loop's response the same across speeds.
- **Angle of attack (α):** the angle between the wing and the air it meets: pitch less the
  flight path's climb angle in wings-level flight. Lift grows with it up to the stall.
- **Pitch stiffness / damping:** how a wing resists pitching. Stiffness is a nose-down moment
  growing with the angle of attack (the tail weathervaning into the airflow); damping a moment
  against the pitch rate. A held pitch rate needs the surface to beat both; a held roll rate
  only the damping.
- **Plane of symmetry:** the plane through the nose and the belly that splits the airframe into
  mirror halves. A wing's lift and drag come from the air in it; air along the span (sideslip)
  pushes only sideways.
- **Turn coordination:** adding yaw and pitch in a banked turn so the plane turns without
  sideslip (skidding sideways through the air).
- **Step response:** how a loop answers a sudden setpoint change: **rise time** (how fast),
  **overshoot** (how far past), **settling time** (until it stays within a band).
- **Doublet:** a rate step one way, then the same step the other way, then a pause: measures
  a step response and ends near the starting attitude (`Maneuver::doublet`).
- **Autotune:** flies step probes on each loop and picks gains from the measured response
  ([architecture](../architecture.md#autotune)).
- **Envelope (autotune):** the limits a stage must stay inside: tilt, rate, and sink for a
  copter; bank, pitch, rate, sink, and an airspeed band for a plane (the **forward-flight
  envelope**, `PlaneEnvelope`). Leaving it fails the stage and puts the last kept gains back.
- **Tune hold:** the steady attitude hold a plane autotune's probes ride on
  (`PlaneTuneHold`): wings level at cruise thrust, the pitch trimmed slowly for airspeed, so
  a step stands out against a hold that doesn't move.
- **System identification:** fitting a model of how the vehicle responds (here, a rate axis's
  body rate to its moment command, with a lag and a delay) from flight data recorded while a
  known signal shakes it: here a square wave added to the rate setpoint, as PX4 does. The
  autotune's design search designs gains from the model instead of flying a step per
  candidate gain ([architecture](../architecture.md#autotune)). On a noisy window the fit
  keeps only the part of the moment the known signal explains (a *projection*), since in
  closed loop the rest carries the controller's answer to gyro noise.
- **Phase margin:** how far a loop is from oscillating, as the phase short of −180° where its
  open-loop gain crosses 1 (the crossover). More margin: less overshoot, a slower loop.
  **Loop shaping** picks gains for a target margin from the model's frequency response.

**Navigation**

- **Waypoint / acceptance:** a waypoint counts as reached inside its acceptance radius
  (horizontal), or once passed along its leg, and inside its altitude band. On a plane leg,
  near or past it but outside the band, the follower waits on it for the altitude, loitering
  around it; a plane leg ending in a back transition takes its waypoint at any altitude.
- **Wind estimate:** wind inferred in rotor-borne flight from the air's drag on the vehicle
  (`estimation::wind`); hover guidance uses it to hold a heading that copes with the wind
  (eXplora: a wingtip into it).
