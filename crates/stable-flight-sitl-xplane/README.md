# stable-flight-sitl-xplane

[![Build](https://github.com/wboayue/stable-flight/actions/workflows/test.yaml/badge.svg?branch=main)](https://github.com/wboayue/stable-flight/actions/workflows/test.yaml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![crates.io](https://img.shields.io/crates/v/stable-flight-sitl-xplane.svg)](https://crates.io/crates/stable-flight-sitl-xplane)
[![Documentation](https://docs.rs/stable-flight-sitl-xplane/badge.svg)](https://docs.rs/stable-flight-sitl-xplane)
[![Coverage Status](https://coveralls.io/repos/github/wboayue/stable-flight/badge.svg?branch=main)](https://coveralls.io/github/wboayue/stable-flight?branch=main)

[X-Plane 12](https://www.x-plane.com) simulator backend for the [Stable Flight](https://github.com/wboayue/stable-flight) flight controller. It provides `XPlaneVehicle`, a `Vehicle` (`StateSource` + `Actuator`) that talks to X-Plane over its UDP dataref protocol through [xplane-bridge](https://crates.io/crates/xplane-bridge), with no plugin. See [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#x-plane) for why it looks the way it does.

The crate builds on the [`stable-flight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight) core and never on the sitl harness, so any runner can drive it; the examples pair it with [`stable-flight-sitl`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl).

Status: the backend reads and converts X-Plane's state and drives engines; no airframe is tuned yet. The Alia 250 in copter mode is planned ([#351](https://github.com/wboayue/stable-flight/issues/351)), its transitions next ([#392](https://github.com/wboayue/stable-flight/issues/392)).

## Setup

- X-Plane 12 on macOS, Windows, or Linux. Its UDP port is under **Settings → Network** (default `49000`); no data-output rows or network settings need changing, also for a controller on another machine on the LAN (replies go to the sender's address).
- X-Plane ships no multirotors. The stock Alia 250 (lift+cruise eVTOL) flies on its four lift rotors.

## Usage

```rust,no_run
use stable_flight_sitl_xplane::xplane_bridge::XPlaneUdpBridge;
use stable_flight_sitl_xplane::{ActuatorMapping, XPlaneConfiguration, XPlaneVehicle};

// The Alia 250's lift rotors in `TableMixer::quad_x` order (FL, FR, RL, RR) on engines 1, 0,
// 2, 3; its pusher (engine 4) held at 0; wing 2's surfaces held neutral.
let mapping = ActuatorMapping {
    motors: [Some(1), Some(0), Some(2), Some(3), None, None, None, None],
    idle_engines: &[4],
    neutral_wings: &[2],
};
let bridge = XPlaneUdpBridge::connect("127.0.0.1:49000").expect("Failed to open socket");
let vehicle = XPlaneVehicle::connect(XPlaneConfiguration::new(bridge, mapping))
    .expect("X-Plane not ready");
println!("bridge --origin {:?}", vehicle.anchor());
```

`XPlaneUdpBridge::connect` takes a hostname too (`oya.local:49000`). `xplane_bridge` is re-exported, so you don't need to depend on it yourself.

- **Connect** subscribes 25 datarefs and waits (10 s by default, `XPlaneConfiguration::ready_timeout`) until each has a value, resubscribing every 250 ms: X-Plane drops subscriptions that arrive in a burst. A timeout names the missing datarefs. It then sets `override_throttles`, zeroes every mapped and idle engine, and, with `neutral_wings`, sets `override_control_surfaces` and holds those wings' surfaces at 0°.
- **Each tick**, `state` polls without blocking and converts the newest sample; X-Plane sends one per rendered frame (60–120 Hz), so a 250 Hz loop sees each state for a few ticks. `actuate` writes each mapped engine's throttle. The `XPlaneVehicle` rustdoc covers timing.
- **Liveness:** after 0.5 s without a frame (X-Plane restarted, reloading, or gone), and while any dataref stops arriving, it resubscribes every 0.25 s; when frames return it takes control again. Pauses are logged.
- **Shutdown:** dropping the vehicle (Ctrl-C through `ShutdownSignal` in the examples) writes the overrides back to 0, so X-Plane flies the aircraft again.

### Frames and position

`vehicle_state` converts each `XPlaneSample` to the core frame contract: X-Plane's quaternion and body rates as reported (degrees to radians), its OpenGL local frame (x east, y up, z south) as north/east/up, elevation above mean sea level as up, true airspeed. Position is relative to where the aircraft was at connect: `XPlaneVehicle::anchor`, logged with the bridge's `--origin lat,lon,0`, so QGC's map matches X-Plane. `LocalOrigin` keeps it continuous when X-Plane moves its local origin and re-anchors (logged) when the aircraft relocates. The `vehicle_state` and `LocalOrigin` rustdoc have the details.

## Examples

```bash
# Raw state to CSV while you fly with X-Plane's own controls (no override); stop with Ctrl-C.
# Or: just capture-xplane --output capture.csv
cargo run -p stable-flight-sitl-xplane --example capture-state-xplane -- --output capture.csv

# Check it against the frame contract: body rates vs the quaternions, velocity vs position,
# gravity at rest.
cargo run --bin check_frames -- --sim xplane capture.csv
```

Run examples with `RUST_LOG=info` to see connect, anchor, and liveness events.
