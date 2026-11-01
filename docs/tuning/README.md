# Tuning guides

How to tune an airframe on Stable Flight. Each guide covers:

- the order to tune in;
- each knob and its param;
- the symptoms that point to it;
- what to read in a recording;
- a worked example on a reference airframe.

| Guide | Airframe type | Reference airframe |
|---|---|---|
| [Tailsitter](tailsitter.md) | Two-motor, two-elevon tailsitter (`TailsitterGuidance` + `TailsitterStabilizer`) | RealFlight eXplora VTOL (`airframes::explora_vtol()`) |
| [Quadplane](quadplane.md) | Lift+cruise VTOL: four lift rotors, a pusher, and plane surfaces (`QuadPlaneGuidance` + `QuadPlaneStabilizer`) | Simulator quadplane (`stable_flight_sim::airframes::quadplane()`) |

Each guide quotes the reference airframe's values, but the airframe code is the authority: its
comments hold the value in use and the flight (`#n`) that set it. A tuned value lands in the
airframe with that flight cited.

The copter rate and angle loops can also be tuned automatically, by a step search from a
loop-shaping design:
`just fly-autotune` ([architecture.md](../../architecture.md#autotune)). On the plane side,
core designs the roll and pitch loops from a fit flown in a tune hold, then checks them on
attitude steps (`PlaneAutotune` with `StagePlan::plane_design`, `PlaneTuneHold`); it meets
the plane spec on the sim, and the simulator runs are planned
([#535](https://github.com/wboayue/stable-flight/issues/535)–#537).
