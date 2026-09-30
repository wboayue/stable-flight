# stable-flight-sitl-realflight

[RealFlight](https://www.realflight.com) simulator backend for the [Stable Flight](https://github.com/wboayue/stable-flight) flight controller. It provides `RealFlightVehicle`, a `Vehicle` (`StateEstimator` + `Actuator`) that talks to RealFlight over [RealFlight Link](https://crates.io/crates/realflight-bridge), plus tuned `airframes` for the models it flies. See the [project README](https://github.com/wboayue/stable-flight#readme) for simulator setup and [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md) for the backend contract.

The crate depends only on the [`stable-flight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight) core, so any runner can drive it. The runnable examples pair it with the [`stable-flight-sitl`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl) harness (real-time runner, commanders, telemetry).

## Setup

RealFlight runs on Windows; on macOS, run it in a Windows VM (e.g. Parallels). Enable **Settings → Physics → Quality → RealFlight Link Enabled**.

## Usage

```rust
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight_sitl_realflight::realflight_bridge::RealFlightRemoteBridge;
use stable_flight_sitl_realflight::{RealFlightConfiguration, RealFlightVehicle, airframes};

// Tuned gains and channel mapping for the QuadCopterX model.
let airframe = airframes::quadcopter_x();

let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
    bridge: RealFlightRemoteBridge::new("10.211.55.3:8080").expect("Failed to create bridge"),
    mapping: airframe.mapping,
});
let stabilizer = CopterStabilizer::new(airframe.stabilizer);
```

Use `RealFlightLocalBridge` when the controller runs on the same host as RealFlight, and `RealFlightRemoteBridge` otherwise. `realflight_bridge` is re-exported, so you don't need to depend on it yourself. `ActuatorMapping` maps logical motors and control surfaces to the model's RealFlight channels.

Each tick makes one RealFlight exchange, in `actuate`; `estimate_state` returns the cached result, so the stabilizer sees state one tick old. The `RealFlightVehicle` rustdoc covers timing and frame conversion.

## Airframes

| Function | RealFlight model |
|---|---|
| `airframes::quadcopter_x()` | ArduPilot SITL_Models [QuadCopterX](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX) |

## Examples

Run these from the repository root (`just fly-manual` / `just fly-auto` wrap them):

```bash
# Joystick (Logitech Extreme 3D); RealFlight host defaults to 10.211.55.3:8080
cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- --host 192.168.1.20:8080

# Scripted figure-eight (script path is the first argument)
cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight -- flights/figure_eight.csv
```

## Known issues

Some RealFlight conversions don't meet the `VehicleState` frame contract yet. Roll, pitch, heading, and body rates are converted to radians twice, so they come out about 57× too small. Vertical velocity and acceleration are also still down-positive. The conformance tests for those scenarios are `#[ignore]`d until a flight-checked fix lands.
