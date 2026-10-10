# stable-flight-bridge

Companion-computer bridge for a [Stable Flight](https://github.com/wboayue/stable-flight) flight controller. It speaks the flight controller's native link ([`stable-flight-link`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-link)) on one side and MAVLink v2 on the other, presenting the flight controller to a ground station such as QGroundControl as MAVLink system 1, component 1. It translates and never decides: arming, failsafe, and param validation stay on the flight controller. Design: [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#link).

## Status

[#154](https://github.com/wboayue/stable-flight/issues/154) params, state, and commands slices: QGC connects, lists and edits params, shows attitude, position, home, and the landed state, arms and disarms, and flies a link-fed flight controller with its joystick; offboard apps send position targets and land. QGC has no guided actions (takeoff, go-to, land, RTL) for a generic autopilot: its base firmware plugin reports no guided-mode capability, which hides those buttons (QGC `FirmwarePlugin.h:128`, `GuidedActionsController.qml:129-131`, at `a76f229`).

| MAVLink | Bridge |
|---|---|
| `HEARTBEAT` | Sent at 1 Hz from the flight controller's heartbeat (armed state; the FC's `ControlMode` as standard `base_mode` flags, so QGC names it, e.g. "Manual,Stabilize"), only while the flight controller has been heard from within 3 s |
| `PARAM_REQUEST_LIST` | Forwarded; the flight controller streams every param |
| `PARAM_REQUEST_READ` | Answered from a cache of the params the flight controller has sent (by index or name) |
| `PARAM_SET` | Forwarded; the flight controller's echo (the value sent if accepted, the current value if rejected) goes out as `PARAM_VALUE`. Every param is `REAL32`; a set of another type is rejected with the current value |
| `ATTITUDE`, `LOCAL_POSITION_NED`, `VFR_HUD` | 10 Hz from the flight controller's latest state (it streams 50 Hz). `VFR_HUD` throttle is the commanded thrust from the latest telemetry record, 0 if the flight controller sends none |
| `GLOBAL_POSITION_INT` | 10 Hz, projected around `--origin`, once there is a home; relative altitude from home |
| `HOME_POSITION` | 1 Hz and when home moves: the position at the latest arming, or the first one before any arm |
| `EXTENDED_SYS_STATE` | 1 Hz and when it changes: landed state from the arm state and height above home, `LANDING` while in the air in the flight controller's land mode (QGC shows it only as still flying) |
| `MANUAL_CONTROL` | QGC's joystick or virtual joystick: forwarded as an attitude command (roll = y, pitch = −x, yaw = r, throttle = z, ÷ 1000). Stops going fresh on the flight controller when the stream stops |
| `SET_POSITION_TARGET_LOCAL_NED` | From an offboard app (QGC sends none for a generic vehicle): in `LOCAL_NED` with the position included, forwarded as a waypoint (altitude = −z; velocity as fly-through when included; at most `--offboard-speed`). Other frames and masks are dropped |
| `SET_ATTITUDE_TARGET` | From an offboard app (e.g. MAVSDK; QGC never sends it): forwarded as an absolute attitude (roll, pitch, heading from `q`; `thrust` as collective thrust, not tilt-compensated). Body rates are ignored. Dropped when the mask ignores the attitude or throttle or sets `THRUST_BODY_SET`, or `q` is zero or non-finite. A stream like `MANUAL_CONTROL`: stops going fresh when it stops |
| `STATUSTEXT` | The flight controller's status text (e.g. "Link back: move the sticks to take control"), at warning severity, cut to 50 bytes |
| `MISSION_REQUEST_LIST` | `MISSION_COUNT` 0 of the requested type (mission, fence, rally) |
| `COMMAND_LONG` | `REQUEST_MESSAGE(AUTOPILOT_VERSION)`: acked, then `AUTOPILOT_VERSION` (MAVLink 2, C-cast params). `REQUEST_MESSAGE(COMPONENT_METADATA)`: acked, then `COMPONENT_METADATA` pointing to the param metadata (below). `COMPONENT_ARM_DISARM` (QGC's Arm/Disarm; force arm and emergency stop alike): forwarded as `Control::Arm`, acked `ACCEPTED` once the flight controller's heartbeat shows the new state, `FAILED` if not within 1 s, `TEMPORARILY_REJECTED` with no flight controller or another request pending. `NAV_LAND` (offboard apps): forwarded as a land at the latest position (param5/6 ignored), acked `ACCEPTED` once the heartbeat shows the land mode, `FAILED` if not within 1 s; `DENIED` while disarmed, `ACCEPTED` at once while already landing, `TEMPORARILY_REJECTED` before any position or as above; the flight controller disarms on touchdown. One request is pending at a time, so after an arm, wait for its ack before landing. Anything else: `UNSUPPORTED` |

Only messages addressed to system 1 / component 1 (or broadcast) are handled. Fly from one source at a time: a joystick and an offboard app sending at once both go through, and the last message wins.

While the ground station is alive (any message within 3 s; QGC also heartbeats at 1 Hz) and the flight controller is too, the bridge sends `Control::Heartbeat` to the flight controller at 5 Hz. That keeps a link-fed flight controller's commander fresh, and losing QGC, or the bridge itself, lets its failsafe hold position ([`copter-companion-realflight`](https://github.com/wboayue/stable-flight/blob/main/crates/stable-flight-sitl-realflight/examples/copter-companion-realflight.rs)). Telemetry goes out only while the flight controller is alive, and starts over (fresh home, no stale state) when it comes back. The payloads, home, and landed-state rules are [`stable-flight-mavlink`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-mavlink)'s.

## Param metadata

QGC shows each param with a description, units, and its group, and refuses an out-of-range edit before sending it. The bridge builds the metadata at startup from the core's param tables ([`stable-flight`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight)'s `param-metadata` feature) and serves it over HTTP: `component_general.json` and `parameters.json`, plain JSON, at paths versioned by their CRC, because QGC caches downloads by URL and CRC. QGC asks for it while connecting (`COMPONENT_METADATA`), with no setup on its side. No defaults yet ([#229](https://github.com/wboayue/stable-flight/issues/229)). An exclusive bound (e.g. `GUID_TILT_MAX` below 90°) is sent as its value, which the flight controller still rejects.

The server listens on `127.0.0.1:14580` by default, which only reaches a QGC on the same machine. For QGC on another machine, bind an address it can reach and advertise the URL it should use; the bridge refuses to start with an all-interfaces bind (`0.0.0.0`, `::`) and no `--http-url`, since that address isn't one QGC can fetch from:

```bash
just bridge --gcs udpout:192.168.1.20:14550 --http-bind 0.0.0.0:14580 --http-url http://192.168.1.30:14580
```

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
--http-bind <ADDR>   param metadata HTTP server   [default: 127.0.0.1:14580]
--http-url <URL>     base URL QGC fetches the metadata from
                                                  [default: http://HTTP_BIND]
--no-metadata        serve no param metadata
--offboard-speed <MPS>
                     speed cap for position targets  [default: 5]
```

Logging goes through `env_logger` at `info` by default (`RUST_LOG=debug` shows dropped frames).
