# stable-flight-sitl-realflight

[![Build](https://github.com/wboayue/stable-flight/actions/workflows/test.yaml/badge.svg?branch=main)](https://github.com/wboayue/stable-flight/actions/workflows/test.yaml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![crates.io](https://img.shields.io/crates/v/stable-flight-sitl-realflight.svg)](https://crates.io/crates/stable-flight-sitl-realflight)
[![Documentation](https://docs.rs/stable-flight-sitl-realflight/badge.svg)](https://docs.rs/stable-flight-sitl-realflight)
[![Coverage Status](https://coveralls.io/repos/github/wboayue/stable-flight/badge.svg?branch=main)](https://coveralls.io/github/wboayue/stable-flight?branch=main)

[RealFlight](https://www.realflight.com) simulator backend for the [Stable Flight](https://github.com/wboayue/stable-flight) flight controller. It provides `RealFlightVehicle`, a `Vehicle` (`StateSource` + `Actuator`) that talks to RealFlight over [RealFlight Link](https://crates.io/crates/realflight-bridge), plus tuned `airframes` for the models it flies. See [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#simulator-backends) for the backend contracts.

The crate builds on the [`stable-flight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight) core and never on the sitl harness, so any runner can drive it. The runnable examples pair it with the [`stable-flight-sitl`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl) harness (real-time runner, commanders).

## Setup

RealFlight runs on Windows; on macOS, run it in a Windows VM (e.g. Parallels). Enable **Settings → Physics → Quality → RealFlight Link Enabled**. Tuned model: QuadCopterX (multirotor); eXplora VTOL (tailsitter) hovers from a joystick (`tailsitter-manual-realflight`) or a waypoint script (`tailsitter-auto-realflight`), and transitions to plane flight and back, stabilized from the joystick or navigating waypoints from a script (`flights/explora_transition.csv`). Titan Cobra VTOL (quadplane) hovers, transitions to plane flight and back, and flies copter legs with forward thrust, from a waypoint script (`quadplane-auto-realflight`); import the [upstream RFX](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/QuadPlanes/Titan_Cobra_VTOL) unmodified, its channels already one per actuator (table below). The ArduPilot [SITL_Models](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models) repo has more.

`RealFlightRemoteBridge` talks to the `realflight_bridge_proxy` running on the RealFlight host. Install it there with `cargo install realflight-bridge --features rt-tokio` (2.2 or later, to match this crate's error handling). Its default request timeout (150 ms) covers RealFlight's slower requests, and [`REMOTE_IO_TIMEOUT`](https://docs.rs/stable-flight-sitl-realflight/latest/stable_flight_sitl_realflight/constant.REMOTE_IO_TIMEOUT.html), which the examples use, stays above it.

## Usage

```rust
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight_sitl_realflight::realflight_bridge::RealFlightRemoteBridge;
use stable_flight_sitl_realflight::{RealFlightConfiguration, RealFlightVehicle, airframes};

// Tuned gains, mixer, channel mapping, and body rotation for the QuadCopterX model.
let airframe = airframes::quadcopter_x();

let bridge = RealFlightRemoteBridge::new("10.211.55.3:8080").expect("Failed to create bridge");
let vehicle = RealFlightVehicle::new(RealFlightConfiguration::from_airframe(&airframe, bridge));
let guidance = CopterGuidance::new(airframe.guidance);
let stabilizer = CopterStabilizer::new(airframe.stabilizer);
let mixer = airframe.mixer;
```

Use `RealFlightLocalBridge` when the controller runs on the same host as RealFlight, and `RealFlightRemoteBridge` otherwise. `realflight_bridge` is re-exported, so you don't need to depend on it yourself. `ActuatorMapping` maps the mixer's motor and servo slots to the model's RealFlight channels (servos rescaled from -1–1 to RealFlight's 0–1, negated first for any slot in `reversed_servos`, and offset by its `servo_trims`), so an airframe's mapping only fits its own mixer (the `ActuatorMapping` rustdoc covers unmapped channels). `RealFlightConfiguration::from_airframe` takes the mapping and body rotation from an airframe. `RealFlightVehicle::new` panics on an invalid mapping (a channel out of range or used twice); `try_new` returns a `MappingError` instead.

Each tick makes one RealFlight exchange, in `actuate`; `state` returns the cached result, so guidance and the stabilizer see state one tick old. The `RealFlightVehicle` rustdoc covers timing and frame conversion.

`body_rotation` is a constant rotation from RealFlight's body frame to the vehicle's, like a board's mounting; identity unless RealFlight's frame for the model differs (a tailsitter). The `RealFlightVehicle` rustdoc (Body rotation) covers it.

A failed exchange is logged at most once a second. A remote bridge is unusable after a connection error, so give the vehicle a way to build a new one with `with_reconnect`: it retries with backoff and switches RealFlight to Link control on the new bridge, keeping the last state meanwhile. The flight examples do; the `RealFlightVehicle` rustdoc (Connection loss) has the backoff and the details.

```rust
use std::time::Duration;
use stable_flight_sitl_realflight::REMOTE_IO_TIMEOUT;
use stable_flight_sitl_realflight::realflight_bridge::{BridgeError, RealFlightRemoteBridge};

let host = String::from("10.211.55.3:8080");
// Each reconnect attempt blocks its tick, so keep the timeouts short.
let connect =
    move || RealFlightRemoteBridge::with_timeouts(&host, Duration::from_millis(250), REMOTE_IO_TIMEOUT);
let vehicle = RealFlightVehicle::new(RealFlightConfiguration::from_airframe(
    &airframe,
    connect().expect("Failed to create bridge"),
))
.with_reconnect(move || connect().map_err(BridgeError::from));
```

## Airframes

| Function | RealFlight model |
|---|---|
| `airframes::quadcopter_x()` | ArduPilot SITL_Models [QuadCopterX](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX) |
| `airframes::explora_vtol()` | [eXplora VTOL](https://github.com/robustini/eXploraVTOL/tree/main/simulation) tailsitter, for `TailsitterGuidance`/`TailsitterStabilizer` (hover gains in the tailsitter `View`'s axes, hand-tuned, with thrust floor, sink guard, and boost ceiling; plane and transition starting values) |
| `airframes::titan_cobra()` | ArduPilot SITL_Models [Titan Cobra VTOL](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/QuadPlanes/Titan_Cobra_VTOL) lift+cruise quadplane, for `QuadPlaneGuidance`/`QuadPlaneStabilizer` on `TableMixer::quadplane_vtail` (hover with measured trim and autotuned roll rate; surfaces checked in flight, tail trimmed +0.2; transitions at ArduPilot's 11 m/s; forward thrust 1.2) |

## Examples

Run these from the repository root (`just fly-manual` / `just fly-tailsitter` / `just fly-auto` / `just fly-tailsitter-auto` / `just fly-quadplane-auto` / `just capture` / `just fly-companion` / `just fly-tailsitter-companion` wrap them). Each serves params and telemetry over the native link (UDP; binds `127.0.0.1:14560`, peer `127.0.0.1:14561`, set with `--link-bind`/`--link-peer`) for [`stable-flight-bridge`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-bridge) (`just bridge`), which QGC connects to; start the bridge once and leave it running:

```bash
# Joystick (Logitech Extreme 3D); RealFlight host defaults to 10.211.55.3:8080
cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- --host 192.168.1.20:8080
cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- --link-peer 192.168.1.30:14561
# --record writes every tick to CSV (rates, attitude, moments, attitude setpoint, stick yaw).
cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- --record flight.csv
# Spektrum transmitter through the WS3000T dongle instead
cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- --commander spektrum

# eXplora VTOL tailsitter on TailsitterGuidance/TailsitterStabilizer: D-pad Right asks for plane
# (transition), Left for copter, Up rate sticks; --record writes every tick to CSV.
# Or: just fly-tailsitter
cargo run -p stable-flight-sitl-realflight --example tailsitter-manual-realflight -- --record hover.csv

# Scripts are meters north and east of where the vehicle starts and meters above it, so they fly
# the same route at any field; routes keep their compass headings, and obstacles differ.
# Scripted figure-eight (script path is the first argument): flies it once, then hovers; --end loop repeats it, --end land lands on the last waypoint and disarms
# (flights/figure_eight.csv starts and ends over the QuadCopterX spawn point)
cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight -- flights/figure_eight.csv

# The same for the eXplora tailsitter, in the copter regime: a small, slow figure-eight
# starting and ending over its spawn point by default (flights/explora_figure_eight.csv); flights/explora_hover.csv
# holds 5 m over it, flights/explora_descent.csv climbs to 60 m and descends to 20 m,
# flights/explora_heading_box.csv holds 20 s at each of 8 headings at 40 m and 60 m,
# flights/explora_wind_cal.csv and explora_wind_check.csv fly the wind estimate's calibration
# and check holds (docs/tuning/tailsitter.md); --record
# also logs RealFlight's own wind and the hover view's specific force beside guidance's
# wind estimate; --param
# sets a param before takeoff. Or: just fly-tailsitter-auto
cargo run -p stable-flight-sitl-realflight --example tailsitter-auto-realflight -- --end land

# A transition mission: hover climb, two plane laps of a rectangle (the CSV's regime column),
# back to hover and home; flights/explora_transition_lap.csv flies one lap, for transition checks
cargo run -p stable-flight-sitl-realflight --example tailsitter-auto-realflight -- flights/explora_transition.csv --end land

# Titan Cobra VTOL quadplane on QuadPlaneGuidance/QuadPlaneStabilizer: holds 5 m over its spawn
# point by default (flights/cobra_hover.csv); a regime column flies transitions; --record writes
# every tick (both sides, every lift rotor, the pusher, the surfaces). Or: just fly-quadplane-auto
cargo run -p stable-flight-sitl-realflight --example quadplane-auto-realflight -- --end land --record hover.csv
# One plane lap with transitions both ways; copter legs ahead and behind, --param sets a param
# before takeoff (here forward thrust off, to compare)
cargo run -p stable-flight-sitl-realflight --example quadplane-auto-realflight -- flights/cobra_transition_lap.csv --end land
cargo run -p stable-flight-sitl-realflight --example quadplane-auto-realflight -- flights/cobra_legs.csv --end land --param FWD_THR_GAIN=0

# Which way each surface pushes, in flight: a fast copter leg (pusher on) with a cruise-moment
# pulse per axis and sign; logs the lift rotors' push-back and a verdict per pulse, lands ahead
cargo run -p stable-flight-sitl-realflight --example surface-probe-realflight -- --record probe.csv

# Flown from the native link, holding position when it goes quiet; link_pilot plays the companion
# (in place of the bridge). Stop the pilot mid-flight and the quad holds. Or: just fly-companion, just link-pilot
cargo run -p stable-flight-sitl-realflight --example copter-companion-realflight
cargo run --bin link_pilot

# The eXplora flown from the native link, transitions included: with the bridge as a tailsitter,
# QGC shows its VTOL state and its transition button flies it to plane and back
cargo run -p stable-flight-sitl-realflight --example tailsitter-companion-realflight
cargo run -p stable-flight-bridge -- --vehicle-type tailsitter

# Autotune: climbs to the hover point, tunes roll and pitch rate, then roll and
# pitch angle, by design search (each axis fitted from a 3 s square wave, its gains designed
# from the fit; then each loop's P searched by steps from its design), logs
# each fit and judgment, prints the tuned gains as an airframes.rs snippet, lands, and disarms.
# --stages search runs the step-response search instead (per rate loop a relay sets D; P is
# searched, verified against the start, and kept only if no worse).
# --step-test appends step tests of roll/pitch rate and angle and logs a response summary per
# loop (--stages none --step-test: the hand tune). --stages rate runs the relay autotune
# (--stages angle|all adds relay angle and heading stages, whose gains oscillated on
# QuadCopterX); --rate-p-scale starts from bad rate gains; --record run.csv writes every tick
# (rates, angles, moments, setpoint) for diagnosis.
# RUST_LOG=info,copter_autotune_realflight=debug logs each stage's status every second and each step.
# Or: just fly-autotune
cargo run -p stable-flight-sitl-realflight --example copter-autotune-realflight -- --step-test
cargo run -p stable-flight-sitl-realflight --example copter-autotune-realflight -- --stages search
cargo run -p stable-flight-sitl-realflight --example copter-autotune-realflight -- --stages none --step-test
cargo run -p stable-flight-sitl-realflight --example copter-autotune-realflight -- --stages rate --rate-p-scale 10
# The Titan Cobra's lift rotors (--airframe titan-cobra; pusher stopped, surfaces centered)
cargo run -p stable-flight-sitl-realflight --example copter-autotune-realflight -- --airframe titan-cobra --step-test

# Joystick (or --script) flight recording raw SimulatorState and channels to CSV (frame checks)
cargo run -p stable-flight-sitl-realflight --example capture-state-realflight -- --output capture.csv
cargo run -p stable-flight-sitl-realflight --example capture-state-realflight -- --script flights/hover.csv
```

To map a new model's channels and check the frame contract on it, with no flight loop:
set the model in RealFlight so each actuator has its own channel (no model-side mixes or gyro),
put it on the ground, and sweep. The sweep holds every channel at neutral (motors 0, the rest
centered), rests 2 s, moves each servo full both ways, pulses each motor alone, then hops on
all motors with a servo kick so the model tumbles; `--script` runs your own `t_s,channel,value`
steps instead. `check_frames` then compares the capture with the frame contract: reported body
rates against the quaternions, gravity at rest, and RealFlight's Euler angles near ±90° pitch.
`--pitch-deg` converts with the model's body rotation (`--pitch-deg 90` for the eXplora, which
then reads pitch +90° at rest on its tail).

On the ground a sweep moves no surface (no airflow); `surface-probe-realflight` (above) checks
their directions in flight.

Titan Cobra VTOL, swept with `--motors 4,5,6,7,2 --servos 0,1,3` and hops (channels 0-based;
`check_frames` passes with no body rotation;
[#433](https://github.com/wboayue/stable-flight/issues/433)):

| Channel | Actuator | Slot in `airframes::titan_cobra()` |
|---|---|---|
| 0 | ailerons | servo 0 |
| 1 | left A-tail panel | servo 2 |
| 2 | pusher | motor 4 |
| 3 | right A-tail panel | servo 1 |
| 4 | front-right lift rotor, counterclockwise from above | motor 1 |
| 5 | rear-left, counterclockwise | motor 2 |
| 6 | front-left, clockwise | motor 0 |
| 7 | rear-right, clockwise | motor 3 |

```bash
cargo run -p stable-flight-sitl-realflight --example channel-sweep-realflight -- --motors 0,1 --servos 2,3 --output sweep.csv
cargo run --bin check_frames -- sweep.csv
```

To time the RealFlight round trip on its own (no flight loop, no link), with the aircraft on the ground: the probe logs each exchange over `--threshold-ms` (default 20) and prints latency percentiles on Ctrl-C, or on the first connection error (such as an exchange over the I/O timeout). `--local` talks to RealFlight directly, on the RealFlight machine, leaving out the proxy and the network. The flight examples also log exchanges over 20 ms at `debug` (`RUST_LOG=info,stable_flight_sitl_realflight=debug`).

```bash
cargo run -p stable-flight-sitl-realflight --example exchange-latency-realflight -- --host 192.168.1.20:8080
cargo run -p stable-flight-sitl-realflight --example exchange-latency-realflight -- --local 127.0.0.1:18083
```

Joystick flights start disarmed: the motors stay off until you arm (Extreme 3D: top trigger; base button disarms. Spektrum: gear switch). Scripted flights (including the autotune) are armed from the first tick. The companion example arms only when the link says so (`Control::Arm`), never from its hold fallback.

## Timing

Each tick waits on one RealFlight round trip ([`RealFlightVehicle`](https://docs.rs/stable-flight-sitl-realflight/latest/stable_flight_sitl_realflight/struct.RealFlightVehicle.html) rustdoc, "Timing"). Measured through the proxy to RealFlight in a Parallels VM ([#255](https://github.com/wboayue/stable-flight/issues/255)):

- **Usual round trip:** about 2 ms (median 2.2 ms, p99 17 ms), out of a 4 ms tick at 250 Hz. A few each second take 20–30 ms.
- **Dips:** for stretches of seconds to a minute, RealFlight answers once per refresh of the Mac's display: every round trip takes 8.3 ms on a 120 Hz (ProMotion) display, 16.7 ms at 60 Hz, and the loop runs at 120 or 60 Hz ([#438](https://github.com/wboayue/stable-flight/issues/438)). The pacing is in the VM's display path, not the flight loop, the model, or the VM's CPU:
  - On the ground with the motors off (`exchange-latency-realflight`), the window visible: fast, a few slow exchanges a second. Covered by another window: locked for as long as it stays covered, 102–119 refresh-paced exchanges a second.
  - In flight with the window visible, the lock comes and goes on its own: on Titan Cobra transition laps, 44–50 of 165 s locked, the rest at 220–250 Hz; QuadCopterX autotunes ran at 165–205 Hz on average, 149–610 s of each slow.
  - In flight with the window minimized, RealFlight has nothing to render and keeps up: QuadCopterX autotunes at 240.6 and 250.0 Hz on average, 33.5 s and 0 s slow, completing in 6.8 and 5.2 min ([#518](https://github.com/wboayue/stable-flight/issues/518)). (A ground test with the motors off read minimized as locked, #447; flights since haven't.)
  - Neither RealFlight's VSync (off in every run) nor a lower resolution helps; at 800×600 a lap never reached 245 Hz, where 1272×768 locked about a third of one.
  - **Tuned gains at 120 Hz:** rate gains tuned at 250 Hz can ring with the moments in [saturation](https://github.com/wboayue/stable-flight/blob/main/docs/glossary.md): a 120 Hz tick is twice as long, so the loop's delay doubles, and every tick still passes the nominal `dt`. On QuadCopterX after its autotune, the moments saturated on 1–2% of ticks at 250 Hz and on 7–61% at 120 Hz across 6 searched loops (0% on one kept at P 1.17, [#525](https://github.com/wboayue/stable-flight/issues/525)). The gains are right for the 250 Hz loop (AGENTS.md **Tick rate** drops the slow stretches when judging a flight).

  Minimize the RealFlight window during scripted runs (restore it only to watch, at the lock's cost), and screen each run's tick rate (`grep "host slow"` on its `RUST_LOG=info` log, AGENTS.md **Tick rate**); `copter-autotune-realflight` waits out the dips on its own (its fits run through them).
- **How the loop absorbs them:** it falls behind until it is 100 ms late, then the runner resyncs. That's the `stalled, resyncing (~25 ticks skipped)` debug line: lag built up over many slow ticks, not one 100 ms block.
- **Stalls:** in the examples, each call to the proxy times out after 250 ms (`REMOTE_IO_TIMEOUT`), and the proxy's request to RealFlight after 150 ms. A request RealFlight doesn't answer in time comes back as `BridgeError::Simulator` and is retried on the same connection next tick (an aircraft reset at startup is retried up to three times). A call that times out on the client side fails like a lost connection: the vehicle closes the connection (the proxy serves one client at a time) and the flight examples reconnect, so a stalled proxy blocks one tick per attempt, not the loop.
- **Build mode:** for timing-sensitive runs, use `cargo run --release` (the `just` recipes build debug). Debug builds add enough per tick to resync about three times as often.

`exchange-latency-realflight` and the `debug` slow-exchange log (see Examples) measure this.
