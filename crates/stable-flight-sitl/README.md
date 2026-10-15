# stable-flight-sitl

[![Build](https://github.com/wboayue/stable-flight/actions/workflows/test.yaml/badge.svg?branch=main)](https://github.com/wboayue/stable-flight/actions/workflows/test.yaml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![crates.io](https://img.shields.io/crates/v/stable-flight-sitl.svg)](https://crates.io/crates/stable-flight-sitl)
[![Documentation](https://docs.rs/stable-flight-sitl/badge.svg)](https://docs.rs/stable-flight-sitl)
[![Coverage Status](https://coveralls.io/repos/github/wboayue/stable-flight/badge.svg?branch=main)](https://coveralls.io/github/wboayue/stable-flight?branch=main)

Simulator-independent software-in-the-loop harness for the Stable Flight flight controller: commanders, the real-time runner, and per-tick CSV recording. Simulator backends are separate crates, starting with [`stable-flight-sitl-realflight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl-realflight). Part of the [Stable Flight](https://github.com/wboayue/stable-flight) workspace; see the [project README](https://github.com/wboayue/stable-flight#readme) for simulator setup and the runnable examples, and [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md) for design rationale.

## Components

The RealFlight examples (in the backend crate) wire the same six-component pipeline and are copy-paste starting points for your own setup. Let's walk through the components a `FlightLoop` needs: a **Commander**, **Vehicle**, **Guidance**, **Stabilizer**, **Mixer**, and **TelemetryPublisher**. The traits and data types come from the [`stable-flight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight) core crate; this crate provides the commanders and the real-time runner; telemetry goes over the link.

### Commander

Commanders produce each tick's `Command`: arm, the requested flight regime (copter or plane), and a `ControlInput` (attitude or rate sticks, a waypoint, a land, or offboard absolute attitudes and rates) for guidance. Commands may come from input devices, scripts, or external systems. Currently, commanders are available to support scripting, Spektrum transmitters (through the WS3000T dongle), and Extreme3D joysticks. You can support new input devices by implementing the Commander trait.

The following example demonstrates creating a commander for the Extreme3D joystick:

```rust
// Default: Attitude mode
let commander = Extreme3DCommander::new()
    .expect("Failed to initialize Extreme3D commander");

// Or specify mode explicitly for Rate mode
let commander = Extreme3DCommander::with_mode(Mode::Rate)
    .expect("Failed to initialize Extreme3D commander");
```

Use `new()` for Attitude mode (default) or `with_mode()` to start in Rate mode. Mode can also be switched at runtime via D-pad buttons.

`SpektrumCommander::new()` likewise returns a `Result`; it doesn't expose configuration options.

### Vehicle

Vehicles report state and actuate motors and control surfaces. Each simulator backend is its own crate that builds on the core and never depends on this crate (except for its examples); RealFlight's is [`stable-flight-sitl-realflight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl-realflight), whose README shows how to configure it. To support another simulator, create a `stable-flight-sitl-<simulator>` crate that implements `StateSource` (read state) and `Actuator` (send commands) for a type that talks to it; `Vehicle` is implemented automatically for any type with both. It has to meet the frame and timing contracts ([architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#simulator-backends)) and cover the core's conformance scenarios (`stable_flight::conformance_tests!`, `testing` feature).

### Guidance, Stabilizer, and Mixer

Guidance (commands → attitude/rate setpoints, including waypoint navigation), the stabilizer (cascaded PID control), and the mixer (thrust/moments → motor and surface commands) live in the [`stable-flight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight) core crate so the same code can run on hardware. See its README for configuration; the RealFlight examples take all three from an airframe: `CopterGuidance`, `CopterStabilizer`, and `TableMixer::quad_x()` for QuadCopterX; `TailsitterGuidance`, `TailsitterStabilizer`, and `TableMixer::tailsitter_2p()` for the eXplora VTOL.

### TelemetryPublisher

TelemetryPublishers publish the system state to external systems. A SITL run sends it over the native link, as a board does, and [`stable-flight-bridge`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-bridge) speaks MAVLink to the ground station. [`stable-flight-link`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-link)'s `FcLink` wires the publisher with the param server in one call:

```rust
let (link_rx, link_tx) = UdpLink::bind(DEFAULT_FC_ADDR, DEFAULT_BRIDGE_ADDR)?.into_halves();
let mut fc_link = FcLink::<32, 16>::new();
// `telemetry` is the flight loop's publisher; `poller` serves params between ticks (`run_with`).
let (mut poller, telemetry) = fc_link.wire(link_rx, link_tx);
```

To read a flight afterward, fan the link out with a `CsvRecorder` (`recorder` module). It writes one row per tick, `wall_s` then a `Columns` set: `CopterColumns` (body rates, attitude, altitude, horizontal position and ground velocity, moments, thrust, the attitude setpoint, and the stick yaw) or `TailsitterColumns` (adds the attitude in the tailsitter view, flight mode, actuators, airspeed, and guidance's setpoint). Without a path it records nothing, and a failed write never stops the flight:

```rust
// `--record <csv>`; `None` records nothing.
let recorder = CsvRecorder::create(args.record.as_deref(), CopterColumns)?;
let telemetry = (telemetry, recorder);
```

### Running the Loop

With all components configured, assemble a core `FlightLoop` and drive it with this crate's `RealTimeRunner`, stopping on a `ShutdownSignal`:

```rust
let mut flight_loop = FlightLoop::new(commander, vehicle, guidance, stabilizer, mixer, telemetry);

// Ctrl-C (SIGINT) or SIGTERM requests a graceful stop between ticks.
let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");

// Prime the loop, then tick at 250 Hz until shutdown.
let stats = RealTimeRunner::new(250.0)
    .expect("valid loop frequency")
    .run(&mut flight_loop, &shutdown);
```

`run` returns `RunStats` (ticks, elapsed, overruns, and the ticks skipped by resyncing after a stall). `run_with` takes a closure called with `&mut FlightLoop` and the `RunStats` so far before each tick, for changes that must land between ticks (param writes from a link, config swaps) and for reacting to stalls (`skipped_ticks` growing). `ShutdownSignal::new()` gives an unconnected flag you can `trigger()` yourself, e.g. in tests. Pacing and shutdown behaviour are described in [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#real-time-loop-management).
