# stable-flight-bridge

Companion-computer bridge for a [Stable Flight](https://github.com/wboayue/stable-flight) flight controller. It speaks the flight controller's native link ([`stable-flight-link`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-link)) on one side and MAVLink v2 on the other, presenting the flight controller to a ground station such as QGroundControl as MAVLink system 1, component 1. It translates and never decides: arming, failsafe, and param validation stay on the flight controller. Design: [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#link).

## Status

Params-first ([#154](https://github.com/wboayue/stable-flight/issues/154)): QGC connects, lists params, and edits them. Attitude, position, commands, and arming come next.

| MAVLink | Bridge |
|---|---|
| `HEARTBEAT` | Sent at 1 Hz from the flight controller's heartbeat (armed state, mode as `custom_mode`), only while the flight controller has been heard from within 3 s |
| `PARAM_REQUEST_LIST` | Forwarded; the flight controller streams every param |
| `PARAM_REQUEST_READ` | Answered from a cache of the params the flight controller has sent (by index or name) |
| `PARAM_SET` | Forwarded; the flight controller's echo (the value sent if accepted, the current value if rejected) goes out as `PARAM_VALUE`. Every param is `REAL32`; a set of another type is rejected with the current value |
| `MISSION_REQUEST_LIST` | `MISSION_COUNT` 0 of the requested type (mission, fence, rally) |
| `COMMAND_LONG` | `REQUEST_MESSAGE(AUTOPILOT_VERSION)`: acked, then `AUTOPILOT_VERSION` (MAVLink 2, C-cast params). Anything else: `UNSUPPORTED` |

Only messages addressed to system 1 / component 1 (or broadcast) are handled.

## Running against SITL

On one machine, flight controller and bridge over UDP on localhost (the defaults on both sides):

```bash
just fly-tune      # flight controller in RealFlight: link on 127.0.0.1:14560
just bridge        # link on 127.0.0.1:14561, MAVLink to QGC on 127.0.0.1:14550
```

Open QGroundControl; it listens on UDP 14550 and connects on the first heartbeat. Params appear on the vehicle setup Parameters page.

## Options

```text
--link-bind <ADDR>   local link address           [default: 127.0.0.1:14561]
--link-peer <ADDR>   flight controller's address  [default: 127.0.0.1:14560]
--gcs <ADDR>         ground station, as a mavlink::connect address
                     (udpout:host:port, udpin:host:port, tcpout:host:port, ...)
                                                  [default: udpout:127.0.0.1:14550]
```

Logging goes through `env_logger` at `info` by default (`RUST_LOG=debug` shows dropped frames).
