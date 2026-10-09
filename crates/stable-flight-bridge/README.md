# stable-flight-bridge

Companion-computer bridge for a [Stable Flight](https://github.com/wboayue/stable-flight) flight controller. It speaks the flight controller's native link ([`stable-flight-link`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-link)) on one side and MAVLink v2 on the other, presenting the flight controller to a ground station such as QGroundControl as MAVLink system 1, component 1. It translates and never decides: arming, failsafe, and param validation stay on the flight controller. Design: [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#link).

## Status

[#154](https://github.com/wboayue/stable-flight/issues/154) params and state slices: QGC connects, lists and edits params, and shows attitude, position, home, and the landed state. Commands and arming come next.

| MAVLink | Bridge |
|---|---|
| `HEARTBEAT` | Sent at 1 Hz from the flight controller's heartbeat (armed state; the FC's `ControlMode` as standard `base_mode` flags, so QGC names it, e.g. "Manual,Stabilize"), only while the flight controller has been heard from within 3 s |
| `PARAM_REQUEST_LIST` | Forwarded; the flight controller streams every param |
| `PARAM_REQUEST_READ` | Answered from a cache of the params the flight controller has sent (by index or name) |
| `PARAM_SET` | Forwarded; the flight controller's echo (the value sent if accepted, the current value if rejected) goes out as `PARAM_VALUE`. Every param is `REAL32`; a set of another type is rejected with the current value |
| `ATTITUDE`, `LOCAL_POSITION_NED`, `VFR_HUD` | 10 Hz from the flight controller's latest state (it streams 50 Hz). `VFR_HUD` throttle is the commanded thrust from the latest telemetry record, 0 if the flight controller sends none |
| `GLOBAL_POSITION_INT` | 10 Hz, projected around `--origin`, once there is a home; relative altitude from home |
| `HOME_POSITION` | 1 Hz and when home moves: the position at the latest arming, or the first one before any arm |
| `EXTENDED_SYS_STATE` | 1 Hz and when it changes: landed state from the arm state and height above home |
| `MISSION_REQUEST_LIST` | `MISSION_COUNT` 0 of the requested type (mission, fence, rally) |
| `COMMAND_LONG` | `REQUEST_MESSAGE(AUTOPILOT_VERSION)`: acked, then `AUTOPILOT_VERSION` (MAVLink 2, C-cast params). Anything else: `UNSUPPORTED` |

Only messages addressed to system 1 / component 1 (or broadcast) are handled. Telemetry goes out only while the flight controller is alive, and starts over (fresh home, no stale state) when it comes back. The payloads, home, and landed-state rules are [`stable-flight-mavlink`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-mavlink)'s.

## Running against SITL

On one machine, flight controller and bridge over UDP on localhost (the defaults on both sides):

```bash
just fly-manual    # flight controller in RealFlight: link on 127.0.0.1:14560 (fly-auto, capture too)
just bridge        # link on 127.0.0.1:14561, MAVLink to QGC on 127.0.0.1:14550
```

Open QGroundControl; it listens on UDP 14550 and connects on the first heartbeat. Params appear on the vehicle setup Parameters page; the vehicle shows on the map at `--origin`.

## Options

```text
--link-bind <ADDR>   local link address           [default: 127.0.0.1:14561]
--link-peer <ADDR>   flight controller's address  [default: 127.0.0.1:14560]
--gcs <ADDR>         ground station, as a mavlink::connect address
                     (udpout:host:port, udpin:host:port, tcpout:host:port, ...)
                                                  [default: udpout:127.0.0.1:14550]
--origin <LAT,LON,ALT>
                     where the flight controller's local origin is on
                     the earth (degrees, degrees, meters above sea level)
                                                  [default: 34.884389,-120.4035,0]
--no-origin          send no global position or home
```

Logging goes through `env_logger` at `info` by default (`RUST_LOG=debug` shows dropped frames).
