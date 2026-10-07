# stable-flight-sitl-realflight

[RealFlight](https://www.realflight.com) simulator backend for the [Stable Flight](https://github.com/wboayue/stable-flight) flight controller. It provides `RealFlightVehicle`, a `Vehicle` (`StateEstimator` + `Actuator`) that talks to RealFlight over [RealFlight Link](https://crates.io/crates/realflight-bridge), plus tuned `airframes` for the models it flies. See [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#simulator-backends) for the backend contracts.

The crate builds on the [`stable-flight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight) core and never on the sitl harness, so any runner can drive it. The runnable examples pair it with the [`stable-flight-sitl`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl) harness (real-time runner, commanders, telemetry).

## Setup

RealFlight runs on Windows; on macOS, run it in a Windows VM (e.g. Parallels). Enable **Settings → Physics → Quality → RealFlight Link Enabled**. Test models: QuadCopterX (multirotor) and eXplora VTOL (tailsitter); the ArduPilot [SITL_Models](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models) repo has more.

`RealFlightRemoteBridge` talks to the `realflight_bridge_proxy` running on the RealFlight host. Install it there with `cargo install realflight-bridge --features rt-tokio`. The proxy must be `realflight-bridge` 2.x to match this crate (the 2.0 wire format is incompatible with 1.x).

## Usage

```rust
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight_sitl_realflight::realflight_bridge::RealFlightRemoteBridge;
use stable_flight_sitl_realflight::{RealFlightConfiguration, RealFlightVehicle, airframes};

// Tuned gains, mixer, and channel mapping for the QuadCopterX model.
let airframe = airframes::quadcopter_x();

let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
    bridge: RealFlightRemoteBridge::new("10.211.55.3:8080").expect("Failed to create bridge"),
    mapping: airframe.mapping,
});
let guidance = CopterGuidance::new(airframe.guidance);
let stabilizer = CopterStabilizer::new(airframe.stabilizer);
let mixer = airframe.mixer;
```

Use `RealFlightLocalBridge` when the controller runs on the same host as RealFlight, and `RealFlightRemoteBridge` otherwise. `realflight_bridge` is re-exported, so you don't need to depend on it yourself. `ActuatorMapping` maps the mixer's motors and control surfaces to the model's RealFlight channels, so an airframe's mapping only fits its own mixer.

Each tick makes one RealFlight exchange, in `actuate`; `estimate_state` returns the cached result, so guidance and the stabilizer see state one tick old. The `RealFlightVehicle` rustdoc covers timing and frame conversion.

## Airframes

| Function | RealFlight model |
|---|---|
| `airframes::quadcopter_x()` | ArduPilot SITL_Models [QuadCopterX](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX) |

## Examples

Run these from the repository root (`just fly-manual` / `just fly-auto` / `just capture` wrap them):

```bash
# Joystick (Logitech Extreme 3D); RealFlight host defaults to 10.211.55.3:8080
cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- --host 192.168.1.20:8080

# Scripted figure-eight (script path is the first argument)
cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight -- flights/figure_eight.csv

# Record raw SimulatorState to CSV while flying your own transmitter (frame checks, #62)
cargo run -p stable-flight-sitl-realflight --example capture-state-realflight -- --output capture.csv
```

## Known issues

Some RealFlight conversions don't meet the `VehicleState` frame contract yet. Roll, pitch, heading, and body rates are converted to radians twice, so they come out about 57× too small. Vertical velocity and acceleration are also still down-positive. Position `x`/`y` are passed through as reported, though `realflight-bridge` 2.0 documents them as east/north, and the yaw-rate sign and quaternion axes are unverified. The conformance tests for those scenarios are `#[ignore]`d until a flight-checked fix lands. Run them with `cargo test -p stable-flight-sitl-realflight -- --ignored`.
