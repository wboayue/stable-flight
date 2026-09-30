# stable-flight

Flight-control core for [Stable Flight](https://github.com/wboayue/stable-flight): the pipeline traits, the data passed between them, cascaded-PID stabilizers, mixers, and `FlightLoop`, which sequences one control tick. It has no simulator, I/O, or timing dependencies, so the same code runs in software-in-the-loop simulation ([`stable-flight-sitl`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl)) and, later, on flight hardware. See [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md) for design rationale.

## Pipeline

`FlightLoop::step(dt)` runs one tick through the pipeline traits (`Commander`, `StateEstimator` + `Actuator` = `Vehicle`, `Stabilizer`, `Mixer`, `TelemetryPublisher`); the tick order and what each stage sees are specified in [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#flight-stabilization-loop). `FlightLoop` has no clock: a runner decides when to call `step` (`RealTimeRunner` in `stable-flight-sitl` today).

```rust
let mut flight_loop = FlightLoop::new(commander, vehicle, stabilizer, mixer, telemetry);
flight_loop.prime(); // initial estimate (dt = 0) + stabilizer priming
loop {
    flight_loop.step(Duration::from_millis(4)); // 250 Hz
    // ...wait for the next period
}
```

### Commanders

`commanders::WaypointFollower` flies and loops any waypoint list without allocating — an array or `&'static` slice on firmware, a `Vec` on a host:

```rust
let mut commander = WaypointFollower::new(PATH)?.with_acceptance_radius(2.0);
```

Input-device and file-based commanders (joysticks, CSV scripts) live in `stable-flight-sitl`.

### Composition

- `ComposedVehicle::new(estimator, actuator)` makes a `Vehicle` from a separate `StateEstimator` and `Actuator` — the usual shape on hardware (IMU/estimator + ESC driver).
- A tuple of telemetry publishers is a publisher: `(mavlink, (file_log, console))` sends every tick to all three; `()` publishes nothing.
- `FlightLoop::into_parts()` hands the components back after a run.

### Testing

Enable the `testing` feature in `[dev-dependencies]` for allocation-free doubles in `stable_flight::testing`: `NullVehicle` (fixed state, counts calls), `ConstantCommander`, and `RecordingTelemetry<N>` (keeps the last `N` ticks).

### Stabilizer

Stabilizers calculate the difference between the vehicle's current state and the desired state, generating `ControlOutput`s that the mixer then translates into actuator commands. This component contains the core flight stabilization logic. `CopterStabilizer` targets multirotors. This is where you would start if you want to implement custom flight control logic.

The simplest setup uses default gains:

```rust
let stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
```

For tuned gains, build the inner PID controllers explicitly (see [`copter-manual-realflight.rs`](https://github.com/wboayue/stable-flight/blob/main/crates/stable-flight-sitl/examples/copter-manual-realflight.rs) for a full example):

```rust
let rates_controller = BodyRateController::with_config(&BodyRateControllerConfig {
    gains: Gains { p: 7.0, i: 0.0, d: 0.01 },
    max_integral: 100.0,
    max_angular_rates: vec3(3.0, 3.0, 3.0),
});

let attitude_controller = AttitudeController::with_config(&AttitudeControllerConfig {
    gains: Gains { p: 3.0, i: 0.0, d: 0.1 },
    max_integral: 100.0,
    max_angles: vec2(PI / 4.0, PI / 4.0),
    max_rate: vec2(3.0, 3.0),
});

let stabilizer = CopterStabilizer::new(CopterStabilizerConfig {
    rates_controller,
    attitude_controller,
    ..Default::default()
});
```

### Mixer

Mixers convert control outputs into motor commands and control surface movements:

```rust
let mixer = QuadCopterMixer::default();
```
