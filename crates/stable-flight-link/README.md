# stable-flight-link

Wire protocol between a [Stable Flight](https://github.com/wboayue/stable-flight) flight controller and a companion computer (or a laptop running the bridge). It provides the message types, the frame codec, and the byte-level `Link` trait (`LinkRx` + `LinkTx`) that transports implement. It is `no_std` and allocation-free, so the flight controller and the host share one implementation. See [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#link) for design rationale.

## Messages

- `Uplink` (companion → FC): `Heartbeat`, `Command` (a core `Command`), `Arm`, `Param` (a `ParamRequest`: set one, or list all).
- `Downlink` (FC → companion): `Heartbeat`, `State` (a core `VehicleState`), `Record` (an owned copy of a tick's `TelemetryRecord`), `Param` (a `ParamValue`: name, value, index, count), `Status`.

Parameter names are the core's [`params`](https://docs.rs/stable-flight/latest/stable_flight/params/) names.

## Params

`params::ParamServer` serves the flight controller's core `Params` over the link: the commander queues `ParamRequest`s inside the tick, and `poll(&mut flight_loop)` between ticks applies them and queues `ParamValue` replies for the publisher. Pacing and reply rules: its [rustdoc](https://docs.rs/stable-flight-link/latest/stable_flight_link/params/struct.ParamServer.html).

## Frames

A frame is `[version u8][postcard payload][crc16 LE]`, COBS-encoded and terminated by `0x00`; every message fits `MAX_FRAME_LEN` (256 bytes). `decode` rejects bad COBS, bytes after the delimiter, a CRC mismatch, another `PROTOCOL_VERSION`, or an invalid payload (`DecodeError`). Why it looks this way: [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#link).

`SendMessage::send_message` and `RecvMessage::recv_message` (blanket-implemented over the link halves) do the encode-and-send and receive-and-decode in one call, with a caller-supplied buffer; a `LinkError::Decode` loses only that frame. Underneath:

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

A transport implements `LinkRx` (non-blocking `try_recv`) and `LinkTx` (non-blocking `send`, may drop); `Link` is blanket-implemented over both, and the split lets a commander and a telemetry publisher each own one half. `udp::UdpLink` (`std` feature) connects one fixed peer over UDP, one frame per datagram, for SITL and the bridge on a host; board transports (USB-CDC) live with the firmware. Which one runs where is in [architecture.md](https://github.com/wboayue/stable-flight/blob/main/architecture.md#link).

## Features

- `std`: `udp::UdpLink`, a non-blocking UDP transport (splittable into rx/tx halves). An absent peer drops sends and receives nothing, without error.
- `testing`: `loopback::Loopback`, two connected in-memory `Link` ends (each splittable into rx/tx halves) for tests. For `[dev-dependencies]` only.
