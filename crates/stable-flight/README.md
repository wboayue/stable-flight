# stable-flight

Flight-control core for [Stable Flight](https://github.com/wboayue/stable-flight): the pipeline traits, the data passed between them, copter guidance, cascaded-PID stabilizers, mixers, and `FlightLoop`, which sequences one control tick. It is `no_std` and has no simulator, I/O, or timing dependencies, so the same code runs in software-in-the-loop simulation ([`stable-flight-sitl`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl)) and, later, on flight hardware. See [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md) for design rationale.

## Pipeline

`FlightLoop::step(dt)` runs one tick through the pipeline traits (`Commander`, `StateEstimator` + `Actuator` = `Vehicle`, `Guidance`, `Stabilizer`, `Mixer`, `TelemetryPublisher`); the tick order and what each stage sees are specified in [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#flight-stabilization-loop). `FlightLoop` has no clock: a runner decides when to call `step` (`RealTimeRunner` in `stable-flight-sitl` today).

```rust
let mut flight_loop = FlightLoop::new(commander, vehicle, guidance, stabilizer, mixer, telemetry);
flight_loop.prime(); // initial estimate (dt = 0) + guidance and stabilizer priming
loop {
    flight_loop.step(Duration::from_millis(4)); // 250 Hz
    // ...wait for the next period
}
```

### Commanders

`commanders::WaypointFollower` flies any waypoint list without allocating — an array or `&'static` slice on firmware, a `Vec` on a host — looping it or holding the last waypoint (`RouteEnd`):

```rust
let mut commander = WaypointFollower::new(PATH)?
    .with_acceptance_radius(2.0)
    .with_route_end(RouteEnd::Hold);
```

`commanders::FailsafeCommander` flies a link-fed primary (a `LinkedCommander`, which reports how long ago it last heard the link) and switches to a `Fallback` when the link goes stale, such as `HoldCommander`, which holds the position where it took over. It starts in the fallback and returns only after the link has been back for a while; a disarm is always honored, and the fallback never arms. Rules: [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#link-failsafe).

```rust
let mut commander = FailsafeCommander::new(link_commander, HoldCommander::default(), FailsafeConfig::default())?;
```

Input-device and file-based commanders (joysticks, CSV scripts) live in `stable-flight-sitl`.

### Composition

- `ComposedVehicle::new(estimator, actuator)` makes a `Vehicle` from a separate `StateEstimator` and `Actuator` — the usual shape on hardware (IMU/estimator + ESC driver).
- A tuple of telemetry publishers is a publisher: `(mavlink, (file_log, console))` sends every tick to all three; `()` publishes nothing.
- `FlightLoop::into_parts()` hands the components back after a run.

### Testing

Enable the `testing` feature in `[dev-dependencies]` for allocation-free doubles in `stable_flight::testing`: `NullVehicle` (fixed state, counts calls), `ConstantCommander`, and `RecordingTelemetry<N>` (keeps the last `N` ticks). `settled_yaw_rate` checks a body-rate config's yaw loop against a constant torque, such as an airframe's measured hover yaw imbalance.

`testing::conformance` checks a `StateEstimator` against the `VehicleState` frame contract. It is a table of physical scenarios (level, banked, nose up, heading east, climbing, moving northeast, yaw and combined rates) with the state each must produce. A simulator backend or estimator adapter builds its native input for each scenario, and `conformance_tests!` generates one test per scenario, plus one that fails if a scenario is left uncovered:

```rust
fn state_for(scenario: &Scenario) -> VehicleState {
    // Build the simulator's native report for `scenario` and convert it.
}

stable_flight::conformance_tests! {
    state_for;
    conforms_level: LEVEL;
    conforms_climbing: CLIMBING, ignore = "vertical sign not converted yet";
    // ... one row per scenario
}
```

### Guidance

Guidance is the outer loop: it turns the commander's `ControlInput` into a `Setpoint` (an attitude or rate target) for the stabilizer. `CopterGuidance` flies waypoints with altitude and lateral position loops (the lateral loop at a configurable, slower rate, capped at each waypoint's cruise speed), turns attitude-mode yaw stick into a heading-hold target that waypoint flight also holds, and passes rate commands through.

```rust
let guidance = CopterGuidance::new(CopterGuidanceConfig::default());
```

### Stabilizer

Stabilizers track the guidance `Setpoint`: they calculate the difference between the vehicle's current state and the desired state, generating `ControlOutput`s that the mixer then translates into actuator commands. This component contains the core flight stabilization logic. `CopterStabilizer` targets multirotors. This is where you would start if you want to implement custom flight control logic.

The simplest setup uses default gains:

```rust
let stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
```

For tuned gains, set the inner loops' configs (simulator backends ship tuned sets, e.g. [`airframes::quadcopter_x`](https://github.com/wboayue/stable-flight/blob/main/crates/stable-flight-sitl-realflight/src/airframes.rs)):

```rust
let stabilizer = CopterStabilizer::new(CopterStabilizerConfig {
    body_rate: BodyRateControllerConfig {
        gains: Gains { p: 0.5, i: 0.2, d: 0.004 },
        yaw_gains: Gains { p: 0.6, i: 0.2, d: 0.0 },
        max_integral: 1.0,
    },
    attitude: AttitudeControllerConfig {
        gains: Gains { p: 3.0, i: 0.0, d: 0.1 },
        max_integral: 100.0,
        max_angles_rad: Vec2::new(PI / 4.0, PI / 4.0),
        max_rate_radps: Vec2::new(3.0, 3.0),
    },
    ..Default::default()
});
```

### Params

Guidance and stabilizer gains and limits can be retuned between ticks by name: a flat name → `f32` view over the configs, sized for MAVLink's 16-byte `param_id`, with angles in degrees. A write is validated like any config change, and an invalid value leaves the config unchanged. The full name list is in the [`params`](https://docs.rs/stable-flight/latest/stable_flight/params/) rustdoc.

```rust
use stable_flight::params::Params;

flight_loop.set_param("STAB_RATE_P", 0.2)?;
flight_loop.visit_params(&mut |name, value| println!("{name} = {value}"));
```

### Param metadata

The `param-metadata` feature adds `params::metadata`. For each param it gives a one-line description, units, a category (the component) and group (the loop), and the valid range, for ground stations: [`stable-flight-bridge`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-bridge) serves it to QGC. A config describes its params by implementing `ParamMetadata`: a `METADATA` table next to its `ParamTable`, with the same names in the same order. Ranges restate what `validate` accepts, and tests check each one against it. The feature is host-side data only; firmware leaves it off, so it costs no flash.

```rust
use stable_flight::params::metadata::ParamMetadata;
use stable_flight::guidance::copter::CopterGuidanceConfig;

for meta in CopterGuidanceConfig::METADATA {
    println!("{} ({}): {}", meta.name(), meta.units(), meta.description());
}
```

### Serialization

The `serde` feature derives `Serialize`/`Deserialize` on the pipeline data (`Command`, `Setpoint`, `VehicleState`, `ControlOutput`, `ActuatorCommand`, `Saturation`, and their parts) without `std`, so the same types cross a wire. [`stable-flight-link`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-link) carries them between the flight controller and a companion. `TelemetryRecord` borrows its parts, so the link sends an owned copy.

### Mixer

Mixers convert control outputs into motor commands and control surface movements:

```rust
let mixer = QuadCopterMixer::default();
```
