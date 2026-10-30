# stable-flight-sitl-xplane

[![Build](https://github.com/wboayue/stable-flight/actions/workflows/test.yaml/badge.svg?branch=main)](https://github.com/wboayue/stable-flight/actions/workflows/test.yaml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![crates.io](https://img.shields.io/crates/v/stable-flight-sitl-xplane.svg)](https://crates.io/crates/stable-flight-sitl-xplane)
[![Documentation](https://docs.rs/stable-flight-sitl-xplane/badge.svg)](https://docs.rs/stable-flight-sitl-xplane)
[![Coverage Status](https://coveralls.io/repos/github/wboayue/stable-flight/badge.svg?branch=main)](https://coveralls.io/github/wboayue/stable-flight?branch=main)

[X-Plane 12](https://www.x-plane.com) simulator backend for the [Stable Flight](https://github.com/wboayue/stable-flight) flight controller. It provides `XPlaneVehicle`, a `Vehicle` (`StateSource` + `Actuator`) that talks to X-Plane over its UDP dataref protocol through [xplane-bridge](https://crates.io/crates/xplane-bridge), with no plugin. See [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#x-plane) for why it looks the way it does.

The crate builds on the [`stable-flight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight) core and never on the sitl harness, so any runner can drive it; the examples pair it with [`stable-flight-sitl`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl).

Status: flies the Alia 250 on its four lift rotors from a joystick and from the link (QGC through the bridge, or `link_pilot`): takeoff, hover, waypoint legs, the link-loss hold, and landing. From a waypoint script (`quadplane-auto-xplane`) it also transitions to plane flight and back: `flights/alia_transition_lap.csv` flies plane legs offshore from Oceano.

## Setup

- X-Plane 12 on macOS, Windows, or Linux. Its UDP port is under **Settings → Network** (default `49000`); no data-output rows or network settings need changing, also for a controller on another machine on the LAN (replies go to the sender's address).
- X-Plane ships no multirotors. The stock BETA Technologies Alia 250 (lift+cruise eVTOL) hovers on its four lift rotors and cruises on its wing and pusher: load it on a runway at X-Plane's default weight (6,299 lb in **Flight Configuration → Weight, Balance & Fuel**), which `airframes::alia250()` is tuned for, in calm weather.
- The examples take X-Plane's throttles and control surfaces, so a joystick X-Plane also reads moves nothing; buttons bound to X-Plane commands still act. Clear the Extreme 3D's hat in X-Plane's joystick settings: bound to view panning, a hat press also moves the view, and a Left to pan it back asks for copter (the D-pad picks the regime; [#446](https://github.com/wboayue/stable-flight/issues/446)).
- After X-Plane relocates the aircraft (a new airport, a reset elsewhere), the vehicle re-anchors position there and logs the new `--origin`: restart the bridge with it, or QGC's map is off by the move.

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
    ..Default::default()
};
let bridge = XPlaneUdpBridge::connect("127.0.0.1:49000").expect("Failed to open socket");
let vehicle = XPlaneVehicle::connect(XPlaneConfiguration::new(bridge, mapping))
    .expect("X-Plane not ready");
println!("bridge --origin {:?}", vehicle.anchor());
```

`XPlaneUdpBridge::connect` takes a hostname too (`oya.local:49000`). `xplane_bridge` is re-exported, so you don't need to depend on it yourself.

Surfaces are `SurfaceLink`s: a servo slot deflects one wing's aileron, elevator, or rudder (the `.acf`'s wing number), linearly to its own degrees at servo +1 and −1, so a surface's sign and its travel each way (ailerons that rise further than they drop) are data. `airframes::alia250()` lists the Alia's.

- **Connect** subscribes 25 datarefs and waits (10 s by default, `XPlaneConfiguration::ready_timeout`) until each has a value, resubscribing every 250 ms: X-Plane drops subscriptions that arrive in a burst. A timeout names the missing datarefs. It then sets `override_throttles`, zeroes every mapped and idle engine, and, with `surfaces` or `neutral_wings`, sets `override_control_surfaces` and holds the neutral wings' surfaces at 0°.
- **Each tick**, `state` polls without blocking and converts the newest sample; X-Plane sends one per rendered frame (60–120 Hz), so a 250 Hz loop sees each state for a few ticks. `actuate` writes each mapped engine's throttle and each linked surface's deflection, once per frame: X-Plane applies only the last write before a frame. The `XPlaneVehicle` rustdoc covers timing.
- **Liveness:** after 0.5 s without a frame (X-Plane restarted, reloading, or gone), and while any dataref stops arriving, it resubscribes every 0.25 s; when frames return it takes control again. Pauses are logged.
- **Shutdown:** dropping the vehicle (Ctrl-C through `ShutdownSignal` in the examples) writes the overrides back to 0, so X-Plane flies the aircraft again.

### Frames and position

`vehicle_state` converts each `XPlaneSample` to the core frame contract: X-Plane's quaternion and body rates as reported (degrees to radians), its OpenGL local frame (x east, y up, z south) as north/east/up, elevation above mean sea level as up, true airspeed. Position is relative to where the aircraft was at connect: `XPlaneVehicle::anchor`, logged with the bridge's `--origin lat,lon,0`, so QGC's map matches X-Plane. `LocalOrigin` keeps it continuous when X-Plane moves its local origin and re-anchors (logged) when the aircraft relocates. The `vehicle_state` and `LocalOrigin` rustdoc have the details.

## Examples

```bash
# Joystick (Logitech Extreme 3D; --commander spektrum) flying the Alia 250: hover, D-pad
# Right for the transition to plane, Left back to copter. Params and telemetry over the native
# link for `just bridge`; --record writes a CSV per tick, --param NAME=VALUE sets a param.
# Or: just fly-xplane --record flight.csv
cargo run --release -p stable-flight-sitl-xplane --example quadplane-manual-xplane

# The Alia through a waypoint script from Oceano County Airport (L52): hover climb, forward
# transition, plane legs offshore, back transition, hover home (flights/alia_transition_lap.csv;
# flights/alia_copter_leg_lap.csv flies a copter leg west first, turning the nose along it;
# flights/alia_steps.csv steps altitude ±30 m on its legs; flights/alia_climb_lap.csv climbs one
# leg steeper than the Alia can and loiters on its waypoint for the altitude). --param NAME=VALUE tunes a run,
# --record writes a CSV per tick. Reset X-Plane's flight before each run.
# Or: just fly-xplane-auto --end land --record lap.csv
cargo run --release -p stable-flight-sitl-xplane --example quadplane-auto-xplane -- --end land --record lap.csv

# The Alia's wing measured for the sim model (stable-flight-sim's alia250()): level legs at
# 48, 46, 44, and 40 m/s from Oceano with roll, pitch, and yaw pulses on each
# (flights/alia_plane_probe.csv). For the full-power climb, fly flights/alia_full_power.csv on
# quadplane-auto-xplane with --param PGUID_THR_MIN=1.
cargo run --release -p stable-flight-sitl-xplane --example plane-probe-xplane -- --record probe.csv

# Plot a recording: time series (altitude, airspeed, attitude vs setpoint, lift share, pusher,
# surfaces, mode) and the ground track.
cargo run --bin plot_flight -- --record flight.csv flight.png

# The Alia flown from the link in copter flight: QGC through the bridge, with the origin the
# example logs at connect (QGC's map then matches X-Plane), or `just link-pilot`. A quiet link
# holds position.
# Or: just fly-xplane-companion, then just bridge --origin <lat,lon,0>
cargo run --release -p stable-flight-sitl-xplane --example copter-companion-xplane

# Autotune of the Alia's hover on its lift rotors (the copter controllers): climbs to 20 m over
# the spawn (flights/alia_hover.csv), searches roll and pitch rate, then angle, on slow steps,
# prints the gains as an airframes.rs snippet, and lands. --step-test appends step tests;
# --stages none --step-test measures the committed gains. Reset X-Plane's flight before each run.
# Or: just fly-xplane-autotune --step-test
cargo run --release -p stable-flight-sitl-xplane --example copter-autotune-xplane -- --step-test

# A multirotor's lift engines on the runway, parking brake off: the yaw sign of each diagonal,
# the liftoff throttle, the motor lag, and an airborne roll, pitch, and yaw pulse.
# Or: just xplane-sweep --output sweep.csv
cargo run --release -p stable-flight-sitl-xplane --example engine-sweep-xplane -- --output sweep.csv

# Raw state to CSV while you fly with X-Plane's own controls (no override); stop with Ctrl-C.
# Or: just capture-xplane --output capture.csv
cargo run -p stable-flight-sitl-xplane --example capture-state-xplane -- --output capture.csv

# Check it against the frame contract: body rates vs the quaternions, velocity vs position,
# gravity at rest.
cargo run --bin check_frames -- --sim xplane capture.csv
```

Run examples with `RUST_LOG=info` to see connect, anchor, and liveness events.

## Airframes

`airframes::alia250()` is the Alia 250 as a quadplane: `TableMixer::quadplane` with the lift rotors' yaw reversed at thrust factor 0.8 and yaw ratio 1, its lift rotors on engines 1, 0, 2, 3, the pusher on 4, the ailerons, elevators, and rudders as X-Plane's yoke moves them. The hover values and the measurements behind them (an `engine-sweep-xplane` run, the `stable-flight-sim` autotune of a model sized from it, two X-Plane flights) and the plane and transition values (tuned on `stable-flight-sim`'s `alia250()`, a model fitted to X-Plane's, and pinned to it by a test) are in its rustdoc; how to tune one, in the [quadplane tuning guide](https://github.com/wboayue/stable-flight/blob/main/docs/tuning/quadplane.md). Its limits:

- **Thrust:** thrust-to-weight is about 1.34 at the default load: climbs are slow, and a heavier load leaves less.
- **Yaw:** at yaw ratio 1 (a higher ratio sank it, the mixer giving up collective for yaw), the tail turns it off heading at a few m/s sideways ([#440](https://github.com/wboayue/stable-flight/issues/440)).
- **Forward drift:** the front rotors are canted 4° forward, so hands-off in attitude mode it creeps forward; position hold takes it out.
- **Transitions:** forward waits on the lift rotors until 40 m/s, the speed the wing holds level at: about 27 s from a hover (the sim model, slower through 30–40 m/s, takes 42 s).
- **Scale:** it cruises at 45 m/s and turns at 20° of bank, about 850 m across: plane legs span kilometers.
