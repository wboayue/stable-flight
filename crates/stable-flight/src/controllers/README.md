# Flight Control Controllers

PID building blocks shared by guidance (altitude, lateral) and stabilizers (attitude, yaw, body rate). Each controller handles a specific aspect of vehicle stabilization and follows the cascaded control architecture commonly used in flight control systems.

## Architecture Overview

The controllers implement a cascaded control structure where outer loops provide setpoints for inner loops:

```text
Position Commands → Lateral Controller → Attitude Controller → Body Rate Controller → Motor Commands
                 ↘ Altitude Controller ↗             ↘ Yaw Controller ↗
```

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
- Separate yaw gains: yaw authority (differential prop drag) is weaker than roll/pitch
- Real-time performance optimized for high-frequency control loops
- Comprehensive input validation and error handling

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

**Input**: Desired angles (normalized [-1, 1]), estimated angles (rad)  
**Output**: Desired angular rates (rad/s)

**Key Features**:
- Converts normalized joystick inputs to physical angle setpoints
- PID control with derivative damping to prevent overshoot
- Configurable maximum angles and rates for safety
- Anti-windup protection for integral term

**Usage**:
```rust
use stable_flight::controllers::{AttitudeController, AttitudeControllerConfig};
use stable_flight::Vec2;

let config = AttitudeControllerConfig::default();
let mut controller = AttitudeController::default();
let dt = 0.004;
let desired_angles = Vec2::new(0.5, -0.2); // 50% roll right, 20% pitch down
let estimated_angles = Vec2::new(0.1, 0.0); // Current roll/pitch in radians

let angular_rates = controller.control(&config, dt, desired_angles, estimated_angles);
```

### Yaw Controller (`yaw.rs`)

**Purpose**: Controls vehicle heading by commanding yaw rate.

**Input**: Desired yaw angle (rad), estimated yaw angle (rad)  
**Output**: Desired yaw rate (rad/s)

**Key Features**:
- Handles angle wrapping for shortest-path navigation
- PID control with proper derivative filtering
- Configurable maximum yaw rates
- Robust handling of ±π boundary crossings

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

**Purpose**: Controls horizontal position (X-Y) by commanding roll and pitch angles.

**Input**: Desired position (m), estimated position (m), velocity (m/s)  
**Output**: Desired roll and pitch as normalized tilt [-1, 1] (the attitude controller scales them by its `max_angles_rad`)

**Key Features**:
- Position-to-angle conversion using acceleration commands
- PID control with velocity damping
- Gravity-based angle calculation for acceleration
- Anti-windup protection

**Usage**:
```rust
use stable_flight::controllers::{LateralController, LateralControllerConfig};
use stable_flight::Vec2;

let config = LateralControllerConfig::default();
let mut controller = LateralController::default();
let dt = 0.004;
let desired_position = Vec2::new(5.0, 2.0); // 5m north, 2m east
let estimated_position = Vec2::new(4.8, 1.9);
let estimated_velocity = Vec2::new(0.1, 0.05);

// Normalized (roll, pitch) tilt request
let tilt = controller.control(&config, dt, desired_position, estimated_position, estimated_velocity);
```

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
    gains,
    yaw_gains: Gains { p: 0.3, i: 0.1, d: 0.0 },
    max_integral: 1.0,
};

// Attitude controller config  
let attitude_config = AttitudeControllerConfig {
    gains,
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

### Safety-Critical Operation
- All controllers include input validation and bounds checking
- Integral windup protection prevents control instability
- Output limiting ensures commands stay within physical constraints
- Graceful handling of edge cases and error conditions

### Real-Time Performance  
- Optimized for 250Hz control loops (4ms cycle time)
- Minimal memory allocations in control paths
- Fixed-size `nalgebra` vector math (no allocation)
- Low-latency processing suitable for safety-critical applications

### Modularity and Testing
- Each controller is independently testable and configurable
- Unit tests cover normal, boundary, and non-finite-input cases, table-driven where the
  cases are systematic
- The cascade is exercised end to end by the stabilizer and guidance tests

### Tuning and Configuration
- Sensible default parameters for typical multirotor vehicles
- Config fields document their units and effects
- `validate()` rejects invalid settings (negative or non-finite limits)

## Testing

```bash
cargo test -p stable-flight controllers   # unit tests
cargo test -p stable-flight --doc         # the snippets in this file and the controllers' rustdoc
```

## Mathematical Background

### PID Control
All controllers implement the discrete PID algorithm:

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
integral_term = clamp_symmetric(integral_term + dt * error, config.max_integral);
```

### Angle Wrapping (Yaw Controller)
Yaw errors are normalized to [-π, π) for shortest-path control with the core's
crate-private [`math::wrap_angle`](https://github.com/wboayue/stable-flight/blob/main/crates/stable-flight/src/math.rs),
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
- **Solution**: Reduce max_integral, verify anti-windup implementation

## Integration with Flight Controller

The position controllers (altitude, lateral) run in `CopterGuidance`, which turns commands into setpoints; the attitude, yaw, and body-rate controllers run in `CopterStabilizer`, which tracks them:

```rust,ignore
// Guidance (src/guidance/copter.rs), waypoint mode
let tilt = self.lateral.control(&self.config.lateral, dt, waypoint, position, velocity);
let target = AltitudeTarget { altitude_m: waypoint.altitude_m, velocity_mps: 0.0 };
let estimate = AltitudeEstimate { altitude_m: z, velocity_mps: vz, attitude };
let thrust = self.altitude.control(&self.config.altitude, dt, target, estimate);

// Stabilizer (src/stabilizers/copter.rs)
let attitude_rates = self.attitude.control(&config.attitude, dt, setpoint_tilt, current_tilt);
let yaw_rate = self.yaw.control(&config.yaw, dt, setpoint.heading_rad, current_heading);
let body_moments = self.body_rate.control(&config.body_rate, dt, desired_rates, current_rates);
```

This ensures proper sequencing and data flow through the cascaded control architecture.