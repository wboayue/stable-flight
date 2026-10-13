# Flight Control Controllers

PID building blocks shared by guidance (altitude, lateral) and stabilizers (attitude, yaw, body rate, plane rate). Each controller runs one loop of a cascade, where outer loops provide setpoints for inner loops.

## Architecture Overview

```text
Copter (CopterGuidance → CopterStabilizer):

  waypoint ─┬─ Lateral ─▶ accel ─▶ tilt = atan(a/g) ─┐
            │                                         ├─▶ Attitude ─┐
            │                          heading hold ──┴─▶ Yaw ──────┴─▶ Body Rate ─▶ moments
            └─ Altitude ─▶ thrust

Plane (PlaneGuidance → PlaneStabilizer):

  attitude setpoint ─▶ error / time constant + turn feedforward ─▶ Plane Rate ─▶ moments
  rate setpoint ─────────────────────────────────────────────────▶ Plane Rate ─▶ moments
```

Guidance runs the position loops; the stabilizers run the attitude and rate loops. A rate
setpoint skips the attitude loop on either side.

## State and Config

Each controller holds only its state (integral and derivative history) and is built with
`Default`. Gains and limits live in its `*Config`, passed to every `control` call, so a config
change (e.g. live tuning) applies on the next tick without resetting the state. Configs are plain
`Copy` data with a `validate()` that checks them once at construction and on every change;
`control` assumes a valid config but never panics on a bad limit (a negative or NaN limit acts
as 0).

## Controllers

### Body Rate Controller (`body_rate.rs`)

**Purpose**: Innermost control loop that directly controls angular rates (roll, pitch, yaw rates).

**Input**: Desired angular rates (rad/s), estimated angular rates (rad/s)  
**Output**: Normalized moments [-1, 1] for motor mixing

**Key Features**:
- Plain PID per axis (`P·e + I·∫e + D·ė`, saturated to [-1, 1]), with anti-windup
- Gains per axis (`roll_gains`, `pitch_gains`, `yaw_gains`): frames differ in roll and pitch inertia, and yaw authority (differential prop drag) is weaker than either
- A non-finite rate on one axis zeroes that axis's moment only; the others keep their loop

**Usage**:
```rust
use stable_flight::controllers::{BodyRateController, BodyRateControllerConfig};
use stable_flight::Vec3;

let config = BodyRateControllerConfig::default();
let mut controller = BodyRateController::default();
let dt = 0.004; // seconds; 250 Hz control loop
let desired_rates = Vec3::new(0.1, 0.0, 0.05); // Roll, pitch, yaw rates
let estimated_rates = Vec3::zeros();

let moments = controller.control(&config, dt, desired_rates, estimated_rates);
```

### Attitude Controller (`attitude.rs`)

**Purpose**: Controls vehicle orientation (roll and pitch angles) by commanding angular rates.

**Input**: Desired angles (rad, saturated to `max_angles_rad`; NaN is level), estimated angles (rad)  
**Output**: Desired angular rates (rad/s)

**Key Features**:
- PID per axis, output clamped to `max_rate_radps`, with anti-windup
- `CopterStabilizer` feeds it the roll and pitch of a quaternion attitude error instead of Euler
  differences, so its rates are body rates at any attitude (a tailsitter in transition)

**Usage**:
```rust
use stable_flight::controllers::{AttitudeController, AttitudeControllerConfig};
use stable_flight::Vec2;

let config = AttitudeControllerConfig::default();
let mut controller = AttitudeController::default();
let dt = 0.004;
let desired_angles = Vec2::new(0.5, -0.2); // roll right, pitch down, in radians
let estimated_angles = Vec2::new(0.1, 0.0); // Current roll/pitch in radians

let angular_rates = controller.control(&config, dt, desired_angles, estimated_angles);
```

### Yaw Controller (`yaw.rs`)

**Purpose**: Controls vehicle heading by commanding yaw rate.

**Input**: Desired yaw angle (rad), estimated yaw angle (rad)  
**Output**: Desired yaw rate (rad/s)

**Key Features**:
- Wraps the error to [-π, π), so it turns the short way across ±π
- PID, output clamped to `max_rate_radps`
- `CopterStabilizer` feeds it the heading part of the quaternion attitude error

**Usage**:
```rust
use stable_flight::controllers::{YawController, YawControllerConfig};
use std::f32::consts::PI;

let config = YawControllerConfig::default();
let mut controller = YawController::default();
let dt = 0.004;
let desired_yaw = PI / 2.0; // 90 degrees
let estimated_yaw = 0.0;

let yaw_rate = controller.control(&config, dt, desired_yaw, estimated_yaw);
```

### Altitude Controller (`altitude.rs`)

**Purpose**: Controls vehicle altitude by commanding thrust.

**Input**: `AltitudeTarget` (altitude m, climb rate m/s) and `AltitudeEstimate` (altitude m, climb rate m/s, attitude)  
**Output**: Normalized thrust command [0, 1]

**Key Features**:
- PID on altitude error (D on climb-rate error) gives a vertical acceleration
- Gravity feedforward via `hover_thrust`; the integral only trims the residual, and only
  accumulates within 1 m of the target
- Tilt compensation (boost capped at 60°); output saturated to [0, 1]
- Non-finite inputs return hover thrust without touching the integral

**Usage**:
```rust
use stable_flight::controllers::{
    AltitudeController, AltitudeControllerConfig, AltitudeEstimate, AltitudeTarget,
};
use stable_flight::nalgebra::UnitQuaternion;

let config = AltitudeControllerConfig::default();
let mut controller = AltitudeController::default();
let dt = 0.004;
let target = AltitudeTarget { altitude_m: 10.0, velocity_mps: 0.0 }; // hover at 10 m
let estimate = AltitudeEstimate {
    altitude_m: 9.5,
    velocity_mps: -0.1, // slight descent
    attitude: UnitQuaternion::identity(),
};

let thrust = controller.control(&config, dt, target, estimate);
```

### Lateral Controller (`lateral.rs`)

**Purpose**: Controls horizontal position (X-Y) by commanding a horizontal acceleration.

**Input**: `LateralTarget` (position (m), feedforward velocity (m/s), speed limit (m/s)),
`LateralEstimate` (position (m), velocity (m/s)), and the acceleration the caller last capped
(m/s², zero when none)  
**Output**: Desired horizontal acceleration (m/s², unsaturated). `CopterGuidance` turns it into a
tilt of `atan(|a| / g)` toward it, relative to the heading, capped at its `max_tilt_rad`.

**Key Features**:
- Cascade: position P → desired velocity (plus feedforward, capped at the speed limit) →
  velocity PID → acceleration
- Airframe-independent: tilt conversion and limits belong to the guidance
- Velocity I trims a steady wind; the integral doesn't grow along the capped acceleration
  (ArduPilot `libraries/AC_PID/AC_PID_2D.cpp:127-146`) and is clamped to `max_integral`

**Usage**:
```rust
use stable_flight::controllers::{
    LateralController, LateralControllerConfig, LateralEstimate, LateralTarget,
};
use stable_flight::Vec2;

let config = LateralControllerConfig::default();
let mut controller = LateralController::default();
let dt = 0.004;
let target = LateralTarget {
    position_m: Vec2::new(5.0, 2.0), // 5m north, 2m east
    velocity_mps: Vec2::zeros(),     // stop there
    max_speed_mps: 7.0,
};
let estimate = LateralEstimate {
    position_m: Vec2::new(4.8, 1.9),
    velocity_mps: Vec2::new(0.1, 0.05),
};

// Horizontal acceleration request (north, east) in m/s²
let accel_mps2 = controller.control(&config, dt, target, estimate, Vec2::zeros());
```

### Plane Rate Controller (`plane_rate.rs`)

**Purpose**: The fixed-wing body-rate loop, under `PlaneStabilizer`.

**Input**: Desired and estimated body rates (rad/s), the airspeed scale `s` (trim ÷ airspeed), and whether to integrate  
**Output**: Normalized moments [-1, 1] for the control surfaces

**Key Features**:
- Per axis `s²·(P·e + I·∫e) + s·FF·setpoint`, clamped to [-1, 1]: a surface's moment grows with dynamic pressure, so feedback shrinks with `s²`, and the feedforward (the deflection that holds a rate against the wing's damping) with `s` (PX4 `FixedwingRateControl`)
- The integral holds below the wing's minimum airspeed, where the surfaces have little authority
- A non-finite rate on one axis zeroes that axis's moment only

## Configuration

Each controller has a `Config` struct; check it with `validate()` before use:

```rust
use stable_flight::controllers::*;
use stable_flight::{Vec2, Vec3};
use std::f32::consts::PI;

// Custom PID gains
let gains = Gains { p: 2.0, i: 0.1, d: 0.5 };

// Body rate controller config
let body_rate_config = BodyRateControllerConfig {
    roll_gains: gains,
    pitch_gains: gains,
    yaw_gains: Gains { p: 0.3, i: 0.1, d: 0.0 },
    max_integral: 1.0,
};

// Attitude controller config  
let attitude_config = AttitudeControllerConfig {
    roll_gains: gains,
    pitch_gains: gains,
    max_angles_rad: Vec2::new(PI/3.0, PI/3.0), // 60 degrees max
    max_integral: 10.0,
    max_rate_radps: Vec2::new(3.0, 3.0), // rad/s
};

assert!(body_rate_config.validate().is_ok());
assert!(attitude_config.validate().is_ok());

let mut body_rate = BodyRateController::default();
let mut attitude = AttitudeController::default();
let rates = attitude.control(&attitude_config, 0.004, Vec2::new(0.2, 0.0), Vec2::zeros());
let moments = body_rate.control(&body_rate_config, 0.004, rates.push(0.0), Vec3::zeros());
```

## Design Principles

- **No panics in the tick:** outputs saturate, and a non-finite input gives neutral output
  without touching integral or derivative state
  ([input policy](https://github.com/wboayue/stable-flight/blob/main/architecture.md#input-policy-no-panics-in-the-tick)).
- **Anti-windup:** the integral is clamped to `max_integral` every tick. On a retune, the
  composite's `set_config` calls each controller's `rescale_integral`, so a new `i` is bumpless.
- **No allocation:** fixed-size `nalgebra` math, so the loops run on firmware.
- **Validated config:** `validate()` rejects negative or non-finite limits; config fields
  document their units.
- **Tested:** unit tests cover normal, boundary, and non-finite-input cases, table-driven where
  the cases are systematic; the stabilizer and guidance tests run the cascade end to end.

## Testing

```bash
cargo test -p stable-flight controllers   # unit tests
cargo test -p stable-flight --doc         # the snippets in this file and the controllers' rustdoc
```

## Mathematical Background

### PID Control
The PID loops implement the discrete PID algorithm (the altitude loop takes D from the measured
climb rate; the plane rate loop scales by airspeed, above):

```text
u(k) = Kp×e(k) + Ki×∑e(k)×dt + Kd×(e(k)-e(k-1))/dt
```

Where:
- `u(k)` = control output at time k
- `e(k)` = error at time k  
- `Kp, Ki, Kd` = proportional, integral, derivative gains
- `dt` = time step

### Anti-Windup Protection
Integral terms are clamped to prevent windup:

```rust,ignore
integrated_error = clamp_symmetric(integrated_error + dt * error, config.max_integral);
```

### Angle Wrapping (Yaw Controller)
Yaw errors are normalized to [-π, π) for shortest-path control with the core's
[`stable_flight::wrap_angle`](https://docs.rs/stable-flight/latest/stable_flight/fn.wrap_angle.html),
a Euclidean remainder (so negative angles wrap correctly; `%` alone would not):

```rust,ignore
fn angle_diff(desired: f32, current: f32) -> f32 {
    wrap_angle(desired - current)
}
```

## Common Issues and Solutions

### Controller Oscillation
- **Cause**: Excessive proportional or derivative gain
- **Solution**: Reduce P and D gains, increase I gain slightly

### Steady-State Error  
- **Cause**: Insufficient integral gain
- **Solution**: Increase I gain, ensure max_integral is adequate

### Sluggish Response
- **Cause**: Insufficient proportional gain  
- **Solution**: Increase P gain, verify rate limits aren't constraining

### Integral Windup
- **Cause**: Inadequate integral limiting
- **Solution**: Reduce max_integral

## Integration with Flight Controller

The position controllers (altitude, lateral) run in `CopterGuidance`, which turns commands into setpoints; the attitude, yaw, and body-rate controllers run in `CopterStabilizer`, which tracks them:

```rust,ignore
// Guidance (src/guidance/copter.rs), waypoint mode; the lateral loop runs at its own slower
// rate, so it takes that loop's `step`, not the tick's `dt`
let accel = self.lateral.control(&self.config.lateral, step, lateral_target, lateral_estimate);
let tilt = tilt_for_acceleration(accel, heading);
let target = AltitudeTarget { altitude_m: waypoint.altitude_m, velocity_mps: 0.0 };
let estimate = AltitudeEstimate { altitude_m: z, velocity_mps: vz, attitude };
let thrust = self.altitude.control(&self.config.altitude, dt, target, estimate);

// Stabilizer (src/stabilizers/copter.rs): one quaternion attitude error feeds both angle loops
let error = attitude_error(attitude, roll_rad, pitch_rad, heading_rad);
let rates = self.attitude.control_error(&config.attitude, dt, error.xy());
let yaw_rate = self.yaw.control_error(&config.yaw, dt, error.z);
let moments = self.body_rate.control(&config.body_rate, dt, desired_rates, current_rates);
```

`PlaneStabilizer` runs `PlaneRateController` the same way under its own attitude loop
(`src/stabilizers/plane.rs`).