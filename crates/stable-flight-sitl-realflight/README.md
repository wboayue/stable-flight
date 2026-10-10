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

A failed exchange is logged at most once a second. A remote bridge is unusable after a connection error, so give the vehicle a way to build a new one with `with_reconnect`: it retries with backoff and switches RealFlight to Link control on the new bridge, keeping the last state meanwhile. The flight examples do; the `RealFlightVehicle` rustdoc (Connection loss) has the backoff and the details.

```rust
use std::time::Duration;
use stable_flight_sitl_realflight::realflight_bridge::{BridgeError, RealFlightRemoteBridge};

let host = String::from("10.211.55.3:8080");
let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
    bridge: RealFlightRemoteBridge::new(&host).expect("Failed to create bridge"),
    mapping: airframe.mapping,
})
// Each attempt blocks its tick, so keep the connect timeout short.
.with_reconnect(move || {
    RealFlightRemoteBridge::with_timeout(&host, Duration::from_millis(250)).map_err(BridgeError::from)
});
```

## Airframes

| Function | RealFlight model |
|---|---|
| `airframes::quadcopter_x()` | ArduPilot SITL_Models [QuadCopterX](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX) |

## Examples

Run these from the repository root (`just fly-manual` / `just fly-auto` / `just capture` / `just fly-companion` wrap them). Each serves params and telemetry over the native link (UDP; binds `127.0.0.1:14560`, peer `127.0.0.1:14561`, set with `--link-bind`/`--link-peer`) for [`stable-flight-bridge`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-bridge) (`just bridge`), which QGC connects to; start the bridge once and leave it running:

```bash
# Joystick (Logitech Extreme 3D); RealFlight host defaults to 10.211.55.3:8080
cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- --host 192.168.1.20:8080
cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- --link-peer 192.168.1.30:14561
# Spektrum transmitter through the WS3000T dongle instead
cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- --commander spektrum

# Scripted figure-eight (script path is the first argument): flies it once, then hovers; --end loop repeats it, --end land lands on the last waypoint and disarms
cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight -- flights/figure_eight.csv

# Flown from the native link, holding position when it goes quiet; link_pilot plays the companion
# (in place of the bridge). Stop the pilot mid-flight and the quad holds. Or: just fly-companion, just link-pilot
cargo run -p stable-flight-sitl-realflight --example copter-companion-realflight
cargo run --bin link_pilot

# Joystick (or --script) flight recording raw SimulatorState and channels to CSV (frame checks)
cargo run -p stable-flight-sitl-realflight --example capture-state-realflight -- --output capture.csv
cargo run -p stable-flight-sitl-realflight --example capture-state-realflight -- --script flights/hover.csv
```

Joystick flights start disarmed: the motors stay off until you arm (Extreme 3D: top trigger; base button disarms. Spektrum: gear switch). Scripted flights are armed from the first tick. The companion example arms only when the link says so (`Control::Arm`), never from its hold fallback.
