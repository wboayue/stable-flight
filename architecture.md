## Architecture

This document expands upon the high-level introduction provided in the [project README](README.md).

## Crate Boundaries

The repository is a Cargo workspace. Dependencies point one way:

```
stable-flight-tools ──► stable-flight-sitl ─────────────► stable-flight ◄── stable-flight-sitl-realflight
                          ▲       │                         ▲     ▲                 ┊
                          ┊       ▼                         │     │                 ┊
                          ┊  stable-flight-mavlink ─────────┘  stable-flight-link   ┊
                          ┊       ▲                                ▲                ┊
                          ┊       └─────── stable-flight-bridge ───┘                ┊
                          └┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄ dev ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┘
```

* **`stable-flight`** (`crates/stable-flight`): the flight-control core — pipeline traits, the data passed between them (`Command`/`ControlInput`, `Setpoint`, `VehicleState`, `ControlOutput`, `ActuatorCommand`, `TelemetryRecord`), copter guidance, the cascaded-PID stabilizers and their shared PID `controllers`, the mixers, and `FlightLoop`, which sequences one tick. No simulator, I/O, clock, or thread dependencies, and `no_std`, so it can run on hardware. Published.
* **`stable-flight-link`** (`crates/stable-flight-link`): the wire protocol between the flight controller and a companion computer — `Uplink`/`Downlink` messages, the frame codec, and the byte-level `Link` trait (see [Link](#link)). `no_std`, depends only on the core (with its `serde` feature). Published.
* **`stable-flight-mavlink`** (`crates/stable-flight-mavlink`): MAVLink message builders, pure functions from core values to `mavlink` payloads, plus the telemetry-only estimates they need (home, landed state, the geodetic projection). `std`; depends only on the core and `mavlink`, so the bridge's encoding is testable without I/O and reusable by an onboard MAVLink option ([#156](https://github.com/wboayue/stable-flight/issues/156)). Published.
* **`stable-flight-bridge`** (`crates/stable-flight-bridge`): the companion-computer binary that translates the native link to MAVLink for ground stations (see [Link](#link)). `std`; depends on the link crate, `stable-flight-mavlink`, and `mavlink`, never on the sitl harness, so it runs the same against SITL and a board. Published.
* **`stable-flight-sitl`** (`crates/stable-flight-sitl`): the simulator-independent software-in-the-loop harness — joystick and scripted commanders, `RealTimeRunner` (wall-clock pacing of a `FlightLoop`), and `ShutdownSignal` (SIGINT/SIGTERM/Ctrl-C flag). Published.
* **`stable-flight-sitl-realflight`** (`crates/stable-flight-sitl-realflight`): the RealFlight backend — `RealFlightVehicle`, `ActuatorMapping`, tuned `airframes`, and the RealFlight examples. Published.
* **`stable-flight-tools`** (`crates/stable-flight-tools`): developer binaries (`plot_flight`, `generate_figure_eight`). Unpublished, so library consumers never compile `plotters`.
* **`firmware/`**: excluded from the workspace; each board is its own nested workspace with its own target, profiles, and lockfile (e.g. `firmware/ark-fpv`).

## Simulator Backends

Each simulator gets its own `stable-flight-sitl-<simulator>` crate, so its dependencies (protocol clients, FFI, system libraries) never reach the core, the harness, or another backend.

* **Dependency rule.** A backend depends on the core, its simulator client, and small shared utilities (`log`; math types come through the core's `nalgebra` re-export), and never on `stable-flight-sitl` except as a dev-dependency (examples). So the backend works with any runner: the sitl runners, a HIL rig, or a third-party harness. CI fails if a backend lists `stable-flight-sitl` under `cargo tree -e normal`.
* **Frame contract.** Every `StateEstimator` reports `VehicleState` in one convention, whatever the simulator uses natively, so nothing downstream knows which simulator is attached. Each backend converts at its boundary. The table in the `VehicleState` rustdoc is the reference.
* **Timing contract.** `FlightLoop` calls `estimate_state` then `actuate` once per tick, so a backend must say which call talks to the simulator and how old its state is. The `Vehicle` rustdoc is the reference; each backend documents its own timing (e.g. `RealFlightVehicle`).
* **Conformance.** `stable_flight::testing::conformance` (core `testing` feature) lists physical scenarios and the `VehicleState` each must yield, and `conformance_tests!` runs a backend's native fixtures through them. It lives in the core so backends, and in-core estimator adapters, test against it without pulling in the harness.
* **Airframes.** Tuned gains, the mixer, and channel mappings live in the backend's `airframes` module, next to the simulated model they were tuned on. Examples still inline their pipeline wiring.

## Link

`stable-flight-link` is the native protocol between the flight controller and a companion computer ([#150](https://github.com/wboayue/stable-flight/issues/150)). The wire format is the core types themselves (`Command`, `VehicleState`, an owned `TelemetryRecord`) in postcard, so the flight controller carries no MAVLink dialect; the bridge on the companion translates to MAVLink in `std` code.

* **Frame.** `[version u8][postcard payload][crc16 LE]`, COBS-encoded, `0x00`-terminated. COBS gives the length, so there is no length field. Postcard is not self-describing (a reordered field decodes as garbage, not an error), so every frame carries `PROTOCOL_VERSION` and the receiver rejects a mismatch; any message shape change bumps it. The CRC (CRC-16/IBM-3740, bitwise to save flash) is checked before the version, so a corrupted version byte reads as corruption, not a version mismatch.
* **Sealed messages.** Only `Uplink` and `Downlink` implement `Message`, so nothing else travels under the protocol version.
* **Fixed size.** Every message fits `MAX_FRAME_LEN` (256 bytes; a test checks the worst case), so both ends use stack buffers and nothing allocates.
* **One frame per decode.** `decode` rejects bytes after the delimiter, so a transport that hands over two frames at once fails loudly instead of losing the second.
* **Transport-agnostic.** `Link` moves encoded frames without blocking; the transport is picked where the pipeline is wired. Like `Vehicle`, it is two traits blanket-joined (`LinkRx` + `LinkTx`), so the router that receives and the publisher that sends can each own one half of the same transport. Every transport carries the same frames, so a host run exercises the real wire format:

| Setup | `Link` |
|---|---|
| Tests | `loopback::Loopback` (`testing` feature) |
| Flight controller (SITL) and bridge on one host, no USB | `udp::UdpLink` (`std` feature), UDP on localhost |
| Real board | USB-CDC, later a TELEM UART ([#155](https://github.com/wboayue/stable-flight/issues/155)) |

On the flight controller, reading the link is separate from commanding: `UplinkRouter` drains `LinkRx` between ticks (or in a receive task) and routes each uplink to a bounded queue, params to the `ParamServer` and `Control` to a link-fed commander ([#153](https://github.com/wboayue/stable-flight/issues/153)), so params work whichever commander flies. `LinkPublisher` is the `TelemetryPublisher` that owns `LinkTx`.

On the companion, `stable-flight-bridge` holds the other end of the link and speaks MAVLink for the flight controller (system 1, component 1), so MAVLink never runs on the board. It is a pure state machine (messages in, messages out, clock injected) behind a thin I/O loop. Its state is only what translation needs: the flight controller's last heartbeat (to send `HEARTBEAT` only while it is alive) and a param cache (to answer `PARAM_REQUEST_READ` without a round trip). Listings and writes go to the flight controller, whose echo is forwarded unchanged, so the flight controller stays the only judge of a param value. What it translates today: the [bridge README](crates/stable-flight-bridge/README.md).

Staleness, failsafe, and arm decisions stay on the flight controller ([#152](https://github.com/wboayue/stable-flight/issues/152)); the bridge translates and never decides.

## Flight Loop and Runners

The heart of the system is `FlightLoop` (core). It owns the six components and runs one tick per `step(dt)`, with no notion of time. A runner decides when ticks happen: `RealTimeRunner` (sitl) paces them on the wall clock until a `ShutdownSignal` is set; a lockstep simulator runner or a hardware timer interrupt can drive the same `FlightLoop` later and reuse the same `ShutdownSignal`. Shutdown, pacing, and tick sequencing are three separate pieces so each runner only supplies pacing.

## Core Components

* **Commander**: Generates Rate, Attitude, or Waypoint commands for guidance. Sources include joysticks, pre-recorded scripts, and autonomous navigation systems.
* **Vehicle**: Represents a physical or simulated vehicle. It is the composition `StateEstimator + Actuator` (blanket-implemented), so sensing and actuation can be separate components on hardware (IMU/estimator, ESC output) while a simulator bridge implements both on one type. `ComposedVehicle<E, A>` (core) joins a separate estimator and actuator into a `Vehicle`. This abstraction allows for testing with simulators before deploying to real hardware.
* **Guidance**: The outer loop. Turns a `Command`'s `ControlInput` into a `Setpoint` (attitude or rate) for the stabilizer: position and altitude loops for waypoints, and stick interpretation that needs memory, such as the heading-hold target. Split from the stabilizer so the two can run at different rates (guidance is conventionally ~50 Hz, the inner loop ~250 Hz). `FlightLoop` calls both every tick, and a guidance paces its own slow loops (e.g. `CopterGuidance`'s lateral loop; its rustdoc has the rate). Pacing inside guidance rather than in the loop keeps pilot sticks read every tick; a loop-level divider would add up to a guidance period of stick latency.
* **Stabilizer**: The inner loop — typically cascaded PID loops — calculating `ControlOutput` based on the difference between the setpoint and actual state (setpoint units: `Setpoint` rustdoc).
* **Mixer**: Translates `ControlOutput` into actuator commands, mapping abstract thrust/moment values to specific motor outputs or control surface positions.
* **TelemetryPublisher**: Publishes the current state of the vehicle for monitoring, debugging, and external systems. A tuple `(A, B)` of publishers is itself a publisher that fans each tick out to both, and `()` is the no-op sink (see `stable_flight::composition`).

## Flight Stabilization Loop

Each `FlightLoop::step` (called by the runner at its configured frequency) does the following. This section is the reference for tick ordering; other docs link here.

* The commander sees the **previous** tick's state; guidance and the stabilizer see this tick's **fresh** estimate.
* `FlightLoop::prime` runs once before the first tick: `estimate_state(Duration::ZERO)`, then `Guidance::prime` and `Stabilizer::prime` with that state.
* **`Command::arm` gates the motors.** A disarmed tick still runs the commander, the estimator, `actuate`, and telemetry, but skips guidance, the stabilizer, and the mixer: it actuates `ActuatorCommand::default()` (motors off) and records a neutral setpoint at the current heading with zero control, so nothing winds up on the ground. On each disarmed → armed edge (including the first armed tick) the loop re-primes guidance, then the stabilizer, from the fresh estimate, so integrals start from zero and the heading hold latches where the vehicle points. `arm` is a level the commander returns every tick; arming checks (throttle low, estimator healthy, link present) are commander-side policy ([#136](https://github.com/wboayue/stable-flight/issues/136), [#152](https://github.com/wboayue/stable-flight/issues/152)).
* Telemetry is published after `actuate`, with the whole tick in one `TelemetryRecord`, including which limits it hit: guidance, the stabilizer, and the mixer each report a `Saturation` for their last call (only they know what they clipped), and the loop merges them (none while disarmed).

```mermaid
sequenceDiagram
    participant FlightLoop
    participant Commander
    participant Vehicle
    participant Guidance
    participant Stabilizer
    participant Mixer
    participant TelemetryPublisher

    FlightLoop->>Commander: next_command(dt, state)
    Commander-->>FlightLoop: Command
    FlightLoop->>Vehicle: estimate_state(dt)
    Vehicle-->>FlightLoop: VehicleState
    Note over FlightLoop: disarmed: skip to actuate(default)
    FlightLoop->>Guidance: guide(dt, ControlInput, VehicleState)
    Guidance-->>FlightLoop: Setpoint
    FlightLoop->>Stabilizer: stabilize(dt, Setpoint, VehicleState)
    Stabilizer-->>FlightLoop: ControlOutput
    FlightLoop->>Mixer: mix(ControlOutput)
    Mixer-->>FlightLoop: ActuatorCommand
    FlightLoop->>Vehicle: actuate(ActuatorCommand)
    FlightLoop->>TelemetryPublisher: publish(TelemetryRecord)
```

## Input Policy (no panics in the tick)

Nothing that runs every tick panics on its inputs. Guidance, stabilizers, controllers, and mixers:

* **Saturate** out-of-range normalized commands (roll/pitch/yaw and moments to [-1, 1], throttle and thrust to [0, 1]) and physical targets to their configured limit (attitude setpoint angles to the stabilizer's `max_angles_rad`). A controller that produces a normalized command saturates its output; one that consumes it saturates its input too.
* Give an **unsupported `ControlInput` or `Setpoint` variant** (both are `#[non_exhaustive]`) neutral output rather than panicking: a guidance returns a level setpoint at the current heading with zero thrust, a stabilizer zero thrust and moments.
* Treat a **NaN command as neutral** (level, hold zero rate, no thrust).
* Return **neutral output on a non-finite error** (NaN/∞ setpoint or estimate), without letting it into integral or derivative state; the first finite tick afterwards has no derivative kick, and neither does the first tick after construction (altitude takes D from measured velocity, so it never kicks). Neutral is zero tilt/rate/moment, except altitude, whose neutral is **hover thrust** (0.5), not zero.
* **Wrap angles** to [-π, π) instead of range-checking them.

Only construction-time config validation and a positive, finite `dt` (guaranteed by the runners; one shared assert in every controller and composite) are asserted. The helpers live in the core's crate-private `math` module.

These are last-resort behaviors that keep the loop alive, not a failsafe. A NaN thrust reaching the mixer becomes 0, which cuts the motors. Holding the last thrust could fly away instead, and the stateless mixer can't choose better. Real failsafes (estimator health checks, then a land or disarm commander) belong upstream ([#66](https://github.com/wboayue/stable-flight/issues/66)).

## Config Ownership

Controllers hold only state (integral and derivative history); their gains and limits are a plain `Copy` config passed to every `control` call. Each composite (`CopterStabilizer`, `CopterGuidance`) owns exactly one config value (`CopterStabilizerConfig`, `CopterGuidanceConfig`, which embed the per-loop configs) and passes sub-references down each tick; nothing below it holds a copy, so the period of the guidance's lateral loop is also read from it every tick.

```text
FlightLoop ──set_guidance_config()───▶ CopterGuidance { config } ───&config.altitude──▶ AltitudeController
                                                                  ──&config.lateral───▶ LateralController
           ──set_stabilizer_config()─▶ CopterStabilizer { config } ─&config.body_rate─▶ BodyRateController (…)
```

The one write point is `Configurable::set_config` (see its rustdoc), which validates before committing; `FlightLoop::set_guidance_config`/`set_stabilizer_config` forward to it between ticks. Config access is a separate, opt-in trait rather than part of `Guidance`/`Stabilizer`, so implementors without config aren't burdened, and `FlightLoop` hands out no `&mut` to its components, so nothing can run them out of tick order. The contract:

* **Valid on entry.** Each config has `validate()` (composites tag errors with the loop, e.g. `body_rate.max_integral`); `new` and `set_config` enforce it. Controllers don't re-check every tick.
* **Bad config still never panics the tick.** Limits go through `math::clamp_symmetric`, under which a negative or NaN limit acts as 0 (neutral output) instead of panicking like `f32::clamp`.
* **Tolerate change between ticks.** State must stay coherent when the config changes: the integral is re-clamped to the current `max_integral` every tick, so shrinking it self-heals. Future filter state (e.g. a D-term low-pass cutoff) needs the same treatment.
* **Retuning `i` is bumpless.** Controllers integrate the error, not the I term, so a new integral gain would rescale `i · ∫e` and jump the output. `set_config` rescales each integral by `old_i / new_i` so the term carries over. Across a zero gain it resets the integral instead: an integral accumulated under `i = 0` was never applied, and a zeroed term has nothing to carry. Rescaling rather than resetting keeps a hover trim through a retune. The per-tick `max_integral` clamp still applies, so a large cut in `i` can still bump the output.

### Params

`stable_flight::params` is the naming layer for live tuning (e.g. MAVLink `PARAM_SET`): a flat name → `f32` view over the same configs, not a second store. Each composite config lists its fields in a `ParamTable` (`STAB_RATE_P` → `body_rate.gains.p`). Any `Configurable` with a table gets `Params`, whose `set_param` writes a copy of the config and commits it through `set_config`. Validation and the integrator policy above therefore apply to a param write as to any other config change, and an invalid value leaves the config unchanged. `FlightLoop` combines its guidance's and stabilizer's params.

* **Names** fit MAVLink's 16-byte `param_id`: uppercase, component prefix (`STAB_`, `GUID_`), then loop (`RATE_`, `ATT_`, `YAW_`, `ALT_`, `LAT_`). Vector components end in `_X`/`_Y`/`_Z`, which can't be confused with a `_P` gain. The full list with units is in the `params` rustdoc.
* **Units:** angles and angular rates are degrees and deg/s, as PX4 and ArduPilot expose them. They convert to the config's radians in the table row, so control code stays SI. Gains and integral limits keep the config's units.
* **Addressing:** params are indexed `0..param_count()` in stable table order, and that is the trait's primitive. Name lookup, read, write, and visit are default methods over it, so a combined view (`FlightLoop`) splits only the index. Indices let the flight controller send a few params per tick from a cursor, and `Downlink::Param` carries them, so the bridge passes them through as MAVLink's `param_index`.
* **Stores or rejects:** a successful write stores the value as given, up to deg ↔ rad rounding. Out-of-range values are rejected, never clamped, so a link can confirm a write by echoing what it sent.
* **Over the link:** the commander that decodes a request and the publisher that sends replies both run inside the tick, with no `&mut` to guidance or the stabilizer. So two bounded queues carry requests to `stable_flight_link::params::ParamServer`, polled between ticks (sitl `RealTimeRunner::run_with`, or the firmware loop task), and its replies to the publisher. Pacing, and the echo that lets a ground station confirm a write, are in its rustdoc.
* Out of scope: persistence and non-`f32` params. The commander/publisher wiring and MAVLink translation are [#153](https://github.com/wboayue/stable-flight/issues/153) and [#154](https://github.com/wboayue/stable-flight/issues/154).

## Design Patterns

### Dependency Injection

The system uses dependency injection to allow flexible configuration of components. This enables easy swapping of:

* Different command sources (joystick vs script)
* Vehicle implementations (simulation vs real hardware)
* Control algorithms
* Output mixing strategies for different vehicle configurations
* Telemetry systems

### Trait-Based Polymorphism

Each major component is defined by a trait interface, allowing multiple implementations while keeping a consistent API:

* Multiple Commander implementations (e.g., the core `WaypointFollower`, which follows any waypoint list, looping or holding at the end; sitl's ScriptCommander, which loads CSV waypoint scripts from `flights/` into a `WaypointFollower`; Extreme3DCommander; SpektrumCommander)
* Different Vehicle implementations (e.g., RealFlightVehicle in `stable-flight-sitl-realflight`)
* Guidance and stabilizer variants for different airframes (e.g., CopterGuidance and CopterStabilizer). A tailsitter stabilizer for the [eXplora VTOL](https://github.com/robustini/eXploraVTOL/tree/main/simulation) model is planned but not started.
* Mixer implementations for various aircraft configurations (e.g., QuadCopterMixer). `TailSitterMixer` is a placeholder: it returns zeroed actuator commands until the tailsitter stabilizer exists.
* Telemetry publishers (e.g., the link crate's `LinkPublisher`, which SITL and boards share; MAVLink is the bridge's job)

## Real-Time Loop Management

`RealTimeRunner` (sitl) implements the timing around `FlightLoop::step`:

* Operates at a fixed frequency (e.g., 250 Hz) validated at construction (`RealTimeRunner::new` rejects non-positive or non-finite rates and ones whose period isn't a representable non-zero `Duration`)
* Primes the loop, then steps it with the nominal period and sleeps to the next accumulated deadline
* Checks the `ShutdownSignal` before each tick; SIGINT or SIGTERM (Unix) / Ctrl-C or Ctrl-Break (Windows) set it once installed, and a second stop signal terminates the process
* Catches up after a slow tick: deadlines accumulate, so missed ticks run back-to-back (resync-on-stall is a planned follow-up)
* Returns `RunStats` (ticks, elapsed, average rate)

## Configuration Example

Wiring a quadcopter in RealFlight means choosing one implementation per trait (e.g. `Extreme3DCommander` or `ScriptCommander`, `RealFlightVehicle`, `CopterGuidance` and `CopterStabilizer` with gains from `airframes::quadcopter_x()`, `QuadCopterMixer`, and the link crate's `LinkPublisher` via `FcLink`), assembling them into a `FlightLoop`, and handing it to a `RealTimeRunner` with a `ShutdownSignal` (`run_with`, so the link's poller serves params between ticks). The runnable code lives in the [`stable-flight-sitl` README](crates/stable-flight-sitl/README.md#running-the-loop) and `crates/stable-flight-sitl-realflight/examples/copter-*-realflight.rs`.

This architecture's modularity allows for easy testing with different components while maintaining the same core stabilization loop, making it ideal for iterative flight control system development.
