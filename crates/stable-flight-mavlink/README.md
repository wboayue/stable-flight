# stable-flight-mavlink

MAVLink message builders for a [Stable Flight](https://github.com/wboayue/stable-flight) flight controller: pure functions from core values to MAVLink payloads (the [`mavlink`](https://crates.io/crates/mavlink) crate, `common` dialect). No connection, clock, or scheduling. When and where to send is the caller's job.

The in-process SITL publisher ([`stable-flight-sitl`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl)'s `MavlinkPublisher`) and the companion bridge ([`stable-flight-bridge`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-bridge)) both build their messages here, so a vehicle looks the same to a ground station either way.

## Builders

Each builder returns a payload (`*_DATA`) for the caller to wrap; vehicle identity is a parameter.

- `heartbeat(vehicle_type, armed, mode)`: `HEARTBEAT` for a generic autopilot of the given `MavType`. The arm state, and the core `ControlMode` as standard `base_mode` flags that ground stations name (QGC: "Manual,Stabilize" for attitude). The mapping table is in the rustdoc.

- `attitude`, `local_position_ned`: `ATTITUDE` (Euler angles, yaw wrapped to [-π, π]) and `LOCAL_POSITION_NED` (the core's north/east/up as NED).
- `global_position_int`, `home_position`: `GLOBAL_POSITION_INT` (relative altitude from home; `None` for a non-finite position) and `HOME_POSITION`, projected through a `FlatEarth` around a `GeodeticOrigin` (`DEFAULT_ORIGIN`, or your own; `FlatEarth::new` rejects an out-of-range one with `InvalidOrigin`).
- `vfr_hud(state, thrust)`: `VFR_HUD`, throttle from the normalized commanded thrust.
- `extended_sys_state(landed_state)`: `EXTENDED_SYS_STATE`, no VTOL state.

```rust
use stable_flight::ControlMode;
use stable_flight_mavlink::heartbeat;
use stable_flight_mavlink::mavlink::common::MavType;

let data = heartbeat(MavType::MAV_TYPE_QUADROTOR, true, ControlMode::Attitude);
```

## Home and landed state

Telemetry-only estimates, stateful but with no clock or I/O, so every caller derives them the same way:

- `HomeTracker`: home is the position at the latest arming, or before the first arm, the first finite one.
- `LandDetector`: the landed state from the arm state and height above home (in the air above 0.5 m, landed again below 0.2 m, on the ground while disarmed).
- `GroundTracker`: both, feeding the detector the height above the tracked home, and reporting when either changes so `HOME_POSITION` and `EXTENDED_SYS_STATE` can go out at once.

`mavlink` is re-exported (`stable_flight_mavlink::mavlink`) so callers name the same types the builders return.

## Features

- `testing`: `testing::MockConnection`, an in-memory `MavConnection` with scripted receives (then `WouldBlock`) and recorded sends, optionally failing. For `[dev-dependencies]` only.
