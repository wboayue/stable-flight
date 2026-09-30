# Flight Control Controllers

This module contains the core PID controllers for autonomous flight control in the Stable Flight SITL system. Each controller handles a specific aspect of vehicle stabilization and follows the cascaded control architecture commonly used in flight control systems.

## Architecture Overview

The controllers implement a cascaded control structure where outer loops provide setpoints for inner loops:

```
Position Commands → Lateral Controller → Attitude Controller → Body Rate Controller → Motor Commands
                 ↘ Altitude Controller ↗             ↘ Yaw Controller ↗
```

## Controllers

### Body Rate Controller (`body_rate.rs`)

**Purpose**: Innermost control loop that directly controls angular rates (roll, pitch, yaw rates).

**Input**: Desired angular rates (rad/s), estimated angular rates (rad/s)  
**Output**: Normalized moments [-1, 1] for motor mixing

**Key Features**:
- PID control with anti-windup protection
- Configurable maximum angular rates and integral limits
- Real-time performance optimized for high-frequency control loops
- Comprehensive input validation and error handling

**Usage**:
```rust
use stable_flight::stabilizers::controllers::{BodyRateController, BodyRateControllerConfig};
use std::time::Duration;
use robomath::vec3;

let mut controller = BodyRateController::new();
let dt = Duration::from_millis(4); // 250Hz control loop
let desired_rates = vec3(0.1, 0.0, 0.05); // Roll, pitch, yaw rates
let estimated_rates = vec3(0.0, 0.0, 0.0);

let moments = controller.control(dt, desired_rates, estimated_rates);
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
use stable_flight::stabilizers::controllers::{AttitudeController, AttitudeControllerConfig};
use std::time::Duration;
use robomath::vec2;

let mut controller = AttitudeController::new();
let dt = Duration::from_millis(4);
let desired_angles = vec2(0.5, -0.2); // 50% roll right, 20% pitch down
let estimated_angles = vec2(0.1, 0.0); // Current roll/pitch in radians

let angular_rates = controller.control(dt, desired_angles, estimated_angles);
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
use stable_flight::stabilizers::controllers::{YawController, YawControllerConfig};
use std::time::Duration;
use std::f32::consts::PI;

let mut controller = YawController::new();
let dt = Duration::from_millis(4);
let desired_yaw = PI / 2.0; // 90 degrees
let estimated_yaw = 0.0;

let yaw_rate = controller.control(dt, desired_yaw, estimated_yaw);
```

### Altitude Controller (`altitude.rs`)

**Purpose**: Controls vehicle altitude by commanding thrust.

**Input**: Desired altitude (m), estimated altitude (m), velocity, attitude  
**Output**: Normalized thrust command [0, 1]

**Key Features**:
- Thrust mapping accounting for vehicle attitude
- Gravity compensation and acceleration limiting
- PID control with velocity feedforward
- Safety limits on maximum acceleration

**Usage**:
```rust
use stable_flight::stabilizers::controllers::{AltitudeController, AltitudeControllerConfig};
use std::time::Duration;
use robomath::Quaternion;

let mut controller = AltitudeController::new();
let dt = Duration::from_millis(4);
let desired_altitude = 10.0; // 10 meters
let desired_velocity = 0.0;  // Hover
let estimated_altitude = 9.5;
let estimated_velocity = -0.1; // Slight descent
let estimated_attitude = Quaternion::default();

let thrust = controller.control(
    dt, 
    desired_altitude, 
    desired_velocity, 
    estimated_altitude, 
    estimated_velocity, 
    estimated_attitude
);
```

### Lateral Controller (`lateral.rs`)

**Purpose**: Controls horizontal position (X-Y) by commanding roll and pitch angles.

**Input**: Desired position (m), estimated position (m), velocity (m/s)  
**Output**: Desired roll and pitch angles (rad)

**Key Features**:
- Position-to-angle conversion using acceleration commands
- PID control with velocity damping
- Gravity-based angle calculation for acceleration
- Anti-windup protection

**Usage**:
```rust
use stable_flight::stabilizers::controllers::{LateralController, LateralControllerConfig};
use std::time::Duration;
use robomath::vec2;

let mut controller = LateralController::new();
let dt = Duration::from_millis(4);
let desired_position = vec2(5.0, 2.0); // 5m north, 2m east
let estimated_position = vec2(4.8, 1.9);
let estimated_velocity = vec2(0.1, 0.05);

let angles = controller.control(dt, desired_position, estimated_position, estimated_velocity);
```

## Configuration

All controllers support custom configuration through their respective `Config` structs:

```rust
use stable_flight::stabilizers::{Gains, controllers::*};

// Custom PID gains
let gains = Gains { p: 2.0, i: 0.1, d: 0.5 };

// Body rate controller config
let body_rate_config = BodyRateControllerConfig {
    gains,
    max_integral: 50.0,
    max_angular_rates: vec3(4.0, 4.0, 2.0), // rad/s
};

// Attitude controller config  
let attitude_config = AttitudeControllerConfig {
    gains,
    max_angles: vec2(PI/3.0, PI/3.0), // 60 degrees max
    max_integral: 10.0,
    max_rate: vec2(3.0, 3.0), // rad/s
};

let body_rate_controller = BodyRateController::with_config(&body_rate_config);
let attitude_controller = AttitudeController::with_config(&attitude_config);
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
- Efficient mathematical operations using `robomath` library
- Low-latency processing suitable for safety-critical applications

### Modularity and Testing
- Each controller is independently testable and configurable
- Comprehensive unit tests covering normal, boundary, and error cases
- Table-driven tests for systematic validation
- Property-based testing for mathematical invariants

### Tuning and Configuration
- Sensible default parameters for typical multirotor vehicles
- Extensive documentation of parameter effects
- Configuration validation to prevent invalid settings
- Support for online parameter adjustment during flight testing

## Testing

The controllers include extensive test suites covering:

- **Unit Tests**: Individual controller functionality
- **Integration Tests**: Controller interactions and cascaded operation  
- **Table-Driven Tests**: Systematic validation across input ranges
- **Property-Based Tests**: Mathematical invariants and edge cases
- **Performance Tests**: Real-time execution benchmarks

Run the tests with:
```bash
cargo test controllers
```

Generate test coverage:
```bash
cargo tarpaulin -o html --output-dir coverage
```

## Mathematical Background

### PID Control
All controllers implement the discrete PID algorithm:

```
u(k) = Kp×e(k) + Ki×∑e(k)×dt + Kd×(e(k)-e(k-1))/dt
```

Where:
- `u(k)` = control output at time k
- `e(k)` = error at time k  
- `Kp, Ki, Kd` = proportional, integral, derivative gains
- `dt` = time step

### Anti-Windup Protection
Integral terms are clamped to prevent windup:

```rust
integral_term = (integral_term + dt * error).clamp(-max_integral, max_integral);
```

### Angle Wrapping (Yaw Controller)
Yaw errors are normalized to [-π, π) for shortest-path control, using the shared
the core's crate-private `math::wrap_angle` (`rem_euclid`, so negative angles wrap correctly; `%` would not):

```rust
fn wrap_angle(angle: f32) -> f32 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

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
- **Solution**: Increase P gain, verify max_rate limits aren't constraining

### Integral Windup
- **Cause**: Inadequate integral limiting
- **Solution**: Reduce max_integral, verify anti-windup implementation

## Integration with Flight Controller

The controllers integrate into the main flight loop through the `CopterStabilizer`:

```rust
// In the main control loop (src/stabilizers/copter.rs)
let attitude_rates = self.attitude_controller.control(dt, command.attitude, state.attitude);
let body_moments = self.rate_controller.control(dt, attitude_rates, state.angular_rates);
let thrust = self.altitude_controller.control(dt, command.altitude, state.altitude, ...);
```

This ensures proper sequencing and data flow through the cascaded control architecture.