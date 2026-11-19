# Autotune a copter from QGC

The flight controller can tune a copter's rate and angle loops in flight: the copter design
search (`StagePlan::copter_design_search`) fits each axis, designs its gains, then checks them
on steps ([autotune design](../architecture/autotune.md#autotune)). This guide starts that run
from QGroundControl, follows it, and keeps or drops the result. The same run without a ground
station is `just fly-autotune`.

It covers the RealFlight QuadCopterX, flown from the link by `copter-companion-realflight`.

## What you need

- RealFlight with the QuadCopterX loaded and RealFlight Link on
  ([setup](../../crates/stable-flight-sitl-realflight/README.md#setup)).
- QGroundControl on the same machine as the bridge, or reachable over UDP
  ([bridge options](../../crates/stable-flight-bridge/README.md#options)).
- Room to hover: the angle steps tilt the copter 5° at a time and it drifts about a meter
  while they run. The run fails a stage past 35° of tilt, 4 rad/s, or 3 m of sink
  (`Envelope::default`). Calm wind.

## Set up QGC once

1. Copy
   [`crates/stable-flight-bridge/qgc/AutotuneActions.json`](../../crates/stable-flight-bridge/qgc/AutotuneActions.json)
   into QGC's `MavlinkActions` folder. On a desktop that is
   `Documents/QGroundControl/MavlinkActions`; QGC shows the folder it reads under Application
   Settings → Fly View → MAVLink Actions.
2. In that same box, set **Fly View Actions** to `AutotuneActions.json`.
3. Under Application Settings → Fly View → Virtual Joystick, turn on **Virtual Joystick**.

The Fly view now has an **Actions** button (the tool strip's hamburger) holding **Autotune
start** and **Autotune stop / revert**. Each sends `MAV_CMD_DO_AUTOTUNE_ENABLE` once: the file sends
param1 1 to start, 0 to stop. QGC's own autotune page is PX4's only, which is why the actions file is needed.

## Fly

Start the flight controller and the bridge, each in its own terminal, then open QGC:

```bash
just fly-companion   # the QuadCopterX in RealFlight, flown from the link
just bridge          # link ⇄ MAVLink for QGC on UDP 14550
```

1. **Arm and take off** on the virtual joystick. A centered throttle asks for thrust 0.5,
   which the QuadCopterX can't hover on (it needs about 0.82), so released the copter sinks
   ([#441](https://github.com/wboayue/stable-flight/issues/441)). Until that's fixed, turn
   off **Auto-Center Throttle** under Virtual Joystick, so the throttle stays where you leave
   it, and set it about 80% up, then adjust until the copter holds its height.
2. **Hover** a few meters up, sticks still.
3. **Start:** Actions → **Autotune start**. The flight controller holds the copter where it is
   for the whole run, whatever the throttle stick says, and QGC shows **AutoTune: started, 9
   stages**.
4. **Follow it** in QGC's messages (the bell, top right). QGC speaks the `NOTICE` lines.
5. **Let it finish:** 3 to 5 minutes on the in-repo sim (202–280 s for the design search,
   [#296](https://github.com/wboayue/stable-flight/issues/296)). **AutoTune: complete: keep,
   or stop to revert** ends it; the tuned gains fly from then on, and the hold hands the copter
   back to the sticks.
6. **Land** on the throttle stick and disarm.

## Reading the messages

Each line starts with `AutoTune:`, as ArduPilot's do. A run reads like this:

| Message | Means | Severity |
|---|---|---|
| `started, 9 stages` | The run started where the copter is | NOTICE |
| `1/9 roll rate` | A stage started: its loop, and which stage of how many | INFO |
| `roll rate fitted P0.620 I0.310 D0.0100` | A stage committed its loop's gains: `fitted` designed from the fit, `tuned` found by the step search, `tested` measured only, `kept` no better gains found | NOTICE |
| `pitch rate P0.620 I0.310 D0.0100 pass` | A search flew a pair of steps on these gains, and they met (`pass`) or missed (`fail`) its target | INFO |
| `yaw rate lowered P… I… D…` | A search's start gains wouldn't settle, so it lowered them | INFO |
| `pitch rate unsettled, fitting again` | A design's gains didn't hold the copter still; it fits again | INFO |
| `complete: keep, or stop to revert` | Every stage committed; the tuned gains fly | NOTICE |
| `roll failed: tilt` | A stage failed and the run ended (below) | ERROR, a pop-up |
| `reverted to the start gains` | The gains from before the run are back | NOTICE |
| `nothing to stop` | A stop found no run to undo | INFO |

The stages run roll rate, pitch rate, roll, pitch (designed), then yaw rate, roll rate, pitch
rate, roll, pitch (searched). The flight controller's terminal logs more: each probe, each
stage's step metrics, and at completion the tuned gains as an airframe snippet.

## Keep or drop the result

The tuned gains fly once the run completes, and stay until the flight controller restarts.
Nothing is saved on the flight controller.

- **Keep them:** copy the snippet the flight controller printed at completion into the
  airframe (`airframes::quadcopter_x()` in `stable-flight-sitl-realflight`), citing the
  flight, as the [tuning guides](README.md) say. QGC's Parameters page shows them meanwhile,
  and can save them to a file.
- **Drop them:** Actions → **Autotune stop / revert**, in the air or after landing. QGC shows
  **AutoTune: reverted to the start gains**.

## Stopping early

- **Autotune stop / revert** mid-run puts back the gains from before the run and hands the
  copter back to the sticks.
- **Moving the roll or pitch stick** past 5% of full deflection does the same: the pilot takes over,
  as in PX4. Mind the throttle stick: the copter flies on it again at once.
- **Disarming** before completion reverts the run too.
- **Losing the link** doesn't stop the run: the hold keeps the copter where it was, and the
  flight controller's failsafe keeps holding if the link stays quiet.

## Troubleshooting

| QGC shows | Why | Do |
|---|---|---|
| `AutoTune: not started: disarmed` | The start came before the copter armed | Arm and take off first |
| `AutoTune: already running` | A run is in progress | Wait for it, or stop it |
| A stage failed: `tilt`, `rate`, or `sank` | The copter left the envelope: the gains under test flew too hard, or wind or a bump moved it | Check the wind, hover higher, start again; the stages already kept stay |
| A stage failed: `not settled` | The copter wouldn't hold still long enough between probes | Calmer air; if the copter drifts or rocks in a plain hover, fix that first |
| A stage failed: `host stalled` or `host fault` | The simulator froze or lost the vehicle for too long | Check RealFlight's frame rate and the link |
| "command failed" from QGC | No report answered within 1 s: the flight controller isn't running `copter-companion-realflight`, or the link is down | Check the flight controller's terminal |
| `Autotune start` unsupported | The bridge runs as a VTOL (`--vehicle-type`) | Only a copter autotunes from QGC for now |
