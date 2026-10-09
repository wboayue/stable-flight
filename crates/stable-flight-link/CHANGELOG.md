# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Not yet released. Until the first release this section summarizes what the crate provides,
not each change; per-change entries start after it.

### Added

- **Messages:** `Uplink` (heartbeat, core `Command`, arm, set/request params) and `Downlink`
  (heartbeat, `VehicleState`, `Record` telemetry, params, status text). `Record` is an owned
  `TelemetryRecord`, convertible both ways.
- **Frames:** `[version][postcard][crc16]`, COBS-encoded and `0x00`-terminated, through the
  sealed `Message` trait (`encode`/`decode`, with `EncodeError`/`DecodeError`). Rejects bad
  COBS, bytes after the delimiter, short frames, CRC mismatches, other `PROTOCOL_VERSION`s,
  and invalid payloads. Every message fits `MAX_FRAME_LEN`.
- **`Link`:** non-blocking, byte-level transport, split into `LinkRx` + `LinkTx`
  (blanket-implemented) so a receiver and a sender can each own one half.
- **Message I/O:** `SendMessage::send_message` / `RecvMessage::recv_message`, blanket over
  `LinkTx` / `LinkRx`: encode-and-send and receive-and-decode with a caller-supplied buffer,
  failing with `LinkError` (transport, encode, or decode; a bad frame loses only itself).
- **`std` feature:** `udp::UdpLink`, a non-blocking UDP transport to one fixed peer, one frame
  per datagram, with rx/tx halves; tolerates an absent peer.
- **`testing` feature:** `loopback::Loopback`, an in-memory `Link` pair with rx/tx halves.
- **`no_std`** (without `std`), allocation-free; builds for `thumbv7em-none-eabihf`.

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight-link
