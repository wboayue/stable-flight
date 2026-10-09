# stable-flight-link

Wire protocol between a [Stable Flight](https://github.com/wboayue/stable-flight) flight controller and a companion computer (or a laptop running the bridge). It provides the message types, the frame codec, and a byte-level `Link` trait that transports implement. It is `no_std` and allocation-free, so the flight controller and the host share one implementation. See [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#link) for design rationale.

## Messages

- `Uplink` (companion → FC): `Heartbeat`, `Command` (a core `Command`), `Arm`, `SetParam`, `RequestParams`.
- `Downlink` (FC → companion): `Heartbeat`, `State` (a core `VehicleState`), `Record` (an owned copy of a tick's `TelemetryRecord`), `Param`, `Status`.

Parameter names are the core's [`params`](https://docs.rs/stable-flight/latest/stable_flight/params/) names.

## Frames

A frame is `[version u8][postcard payload][crc16 LE]`, COBS-encoded and terminated by `0x00`. The CRC is CRC-16/IBM-3740 (CCITT-FALSE) over version + payload. Postcard is not self-describing, so a receiver rejects any frame whose version isn't `PROTOCOL_VERSION`. Every message fits `MAX_FRAME_LEN` (256 bytes).

```rust
use stable_flight_link::{Link, MAX_FRAME_LEN, Message, Uplink};

let mut frame = [0u8; MAX_FRAME_LEN];
let len = Uplink::Arm(true).encode(&mut frame)?;
link.send(&frame[..len])?;

if let Some(len) = link.try_recv(&mut frame)? {
    let msg = Uplink::decode(&mut frame[..len])?; // CRC, then version, then payload
}
```

## Transports

`Link` moves encoded frames without blocking; transports live outside this crate, so the same frames cross every one:

| Setup | `Link` |
|---|---|
| Tests | `loopback::Loopback` (`testing` feature) |
| Flight controller and bridge on one host (no USB) | UDP on localhost ([#153](https://github.com/wboayue/stable-flight/issues/153)) |
| Real board | USB-CDC ([#155](https://github.com/wboayue/stable-flight/issues/155)) |

## Features

- `testing`: `loopback::Loopback`, two connected in-memory `Link` ends for tests. For `[dev-dependencies]` only.
