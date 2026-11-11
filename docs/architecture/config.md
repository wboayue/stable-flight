Part of the [architecture overview](../../architecture.md).

## Config Ownership

Controllers hold only state (integral and derivative history); their gains and limits are a
plain `Copy` config passed to every `control` call. Each composite (`CopterStabilizer`,
`CopterGuidance`) owns exactly one config value, which embeds the per-loop configs, and passes
sub-references down each tick; nothing below it holds a copy.

```text
FlightLoop ──set_guidance_config()───▶ CopterGuidance { config } ───&config.altitude──▶ AltitudeController
                                                                  ──&config.lateral───▶ LateralController
           ──set_stabilizer_config()─▶ CopterStabilizer { config } ─&config.body_rate─▶ BodyRateController (…)
```

The one write point is `Configurable::set_config`, which validates before committing;
`FlightLoop::set_guidance_config`/`set_stabilizer_config` forward to it between ticks. Config
access is a separate, opt-in trait rather than part of `Guidance`/`Stabilizer`, so implementors
without config aren't burdened, and `FlightLoop` hands out no `&mut` to its components, so
nothing can run them out of tick order: what callers need between ticks it forwards
(`set_*_config`, and `start_probe`/`probe`/`stop_probe` on a `Tunable` stabilizer). The
contract:

* **Valid on entry.** Each config has `validate()` (composites tag errors with the loop, e.g.
  `body_rate.max_integral`); `new` and `set_config` enforce it.
* **Bad config still never panics the tick.** Limits go through `math::clamp_symmetric`, under
  which a negative or NaN limit acts as 0 instead of panicking like `f32::clamp`.
* **Tolerate change between ticks.** The integral is re-clamped to the current `max_integral`
  every tick, so shrinking it self-heals. Filter state needs the same treatment.
* **The lateral integral doesn't wind against the tilt cap.** `CopterGuidance` passes the
  lateral loop the acceleration it last cut to `max_tilt_rad`, and the integral doesn't grow
  along it (ArduPilot's `AC_PID_2D::update_i`). Clamping to `max_integral`
  alone would let a long leg at the tilt limit wind it full and overshoot the waypoint.
* **A leg's drag is fed forward, not integrated.** Under the tilt cap, the lateral integral
  would learn the drag of a leg's speed as a disturbance and, still holding it when the
  vehicle stops, carry it past the waypoint: 1–2.5 m on the sim tailsitters (#609).
  `CopterGuidance` adds the drag its wind config's model puts on the lateral loop's commanded
  velocity, past a 0.3 m/s hold band and through the estimated wind, so the integral trims
  only the wind and gusts. The hold's own corrections stay under the band: fed forward, they
  stiffen the hold against the angle loop, as #588 found for a slow angle loop. Rejected:
  freezing or gating the integral in transit (it then can't trim a crosswind, and the leg
  never settles); capping the error it integrates; braking earlier; a shaped position target
  as ArduPilot's `AC_PosControl` and PX4's trajectory use (it removes the target's lag, not the
  drag); the drag at the measured velocity (it feeds the vehicle's own speed back).
* **Retuning `i` is bumpless.** The copter rate loop integrates the I term itself, `∫i·e`, in
  moment, and limits it in moment, as ArduPilot's `AC_PID::update_i` and PX4's
  `RateControl::updateIntegral` do: a new `i` changes how fast the term grows, not what it
  holds, and `i = 0` drops it. The other controllers integrate the error, so a new integral
  gain would rescale `i · ∫e` and jump the output; `set_config` rescales each such integral by
  `old_i / new_i` so the term carries over (a hover trim survives a retune). Across a zero gain
  it resets the integral: one accumulated under `i = 0` was never applied. The per-tick
  `max_integral` clamp still applies, so a large cut in `i` can still bump their output.
  The rate loop's limit in moment is what lets a small `i` hold a trim: limited to `i` × 1 rad of
  error, core's default rate gains held 0.03 of moment, and the sim autotune couldn't start
  under a 1 cm CG offset (#374). The attitude, heading, altitude, lateral, and plane rate
  integrals stay in error units, so their I term's ceiling still scales with `i`; no row has
  needed otherwise.
* **Tuning probes are runtime state, not config.** An autotune probe (`Tunable::start_probe`)
  replaces one loop's setpoint inside the tick; it lives next to the controllers'
  state, never in `CopterStabilizerConfig` or its params. Handing the loop back clears its
  derivative history, so the setpoint's jump back doesn't kick it, and keeps its integral; a
  following `set_config` rescales it like any retune. While a step flies a rate loop, the
  angle or heading loop over it holds its integral: the step tilts the vehicle away from that
  loop's setpoint, an error it isn't flying. Clearing the integral instead would drop the trim
  it holds.

### Params

`stable_flight::params` is the naming layer for live tuning (e.g. MAVLink `PARAM_SET`): a flat
name → `f32` view over the same configs, not a second store. Each composite config lists its
fields in a `ParamTable` (`STAB_RATE_RLL_P` → `body_rate.roll_gains.p`). Any `Configurable` with a
table gets `Params`, whose `set_param` writes a copy of the config and commits it through
`set_config`, so validation and the integrator policy above apply to a param write too, and an
invalid value leaves the config unchanged. `FlightLoop` combines its guidance's and
stabilizer's params.

* **Names** fit MAVLink's 16-byte `param_id`: uppercase, component prefix (`STAB_`, `GUID_`),
  then loop (`RATE_`, `ATT_`, `YAW_`, `ALT_`, `LAT_`). Vector components end in `_X`/`_Y`/`_Z`,
  which can't be confused with a `_P` gain. The full list with units is in the `params`
  rustdoc.
* **Units:** angles and angular rates are degrees and deg/s, as PX4 and ArduPilot expose them,
  converted to the config's radians in the table row, so control code stays SI.
* **Addressing:** params are indexed `0..param_count()` in stable table order, the trait's
  primitive; name lookup, read, write, and visit are default methods over it, so a combined view
  (`FlightLoop`) splits only the index. Indices let the flight controller send a few params per
  tick from a cursor, and the bridge passes them through as MAVLink's `param_index`.
* **Stores or rejects:** a successful write stores the value as given, up to deg ↔ rad rounding.
  Out-of-range values are rejected, never clamped, so a link can confirm a write by echoing it.
* **Over the link:** the commander that decodes a request and the publisher that sends replies
  both run inside the tick, with no `&mut` to guidance or the stabilizer. So two bounded queues
  carry requests to `stable_flight_link::params::ParamServer`, polled between ticks (sitl
  `RealTimeRunner::run_with`, or the firmware loop task), and its replies to the publisher.
* Out of scope: persistence and non-`f32` params.
