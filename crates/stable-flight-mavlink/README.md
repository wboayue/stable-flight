# stable-flight-mavlink

MAVLink message builders for a [Stable Flight](https://github.com/wboayue/stable-flight) flight controller: pure functions from core values to MAVLink payloads (the [`mavlink`](https://crates.io/crates/mavlink) crate, `common` dialect). No connection, clock, or scheduling. When and where to send is the caller's job.

The in-process SITL publisher ([`stable-flight-sitl`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-sitl)'s `MavlinkPublisher`) and the companion bridge ([`stable-flight-bridge`](https://github.com/wboayue/stable-flight/tree/main/crates/stable-flight-bridge)) both build their messages here, so a vehicle looks the same to a ground station either way.

## Builders

- `heartbeat(armed, mode)`: `HEARTBEAT` for a quadrotor with a generic autopilot. The arm state, and the core `ControlMode` as standard `base_mode` flags that ground stations name (QGC: "Manual,Stabilize" for attitude). The mapping table is in the rustdoc.

```rust
use stable_flight::ControlMode;
use stable_flight_mavlink::heartbeat;

let data = heartbeat(true, ControlMode::Attitude);
```

`mavlink` is re-exported (`stable_flight_mavlink::mavlink`) so callers name the same types the builders return.
