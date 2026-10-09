# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Not yet released. Until the first release this section summarizes what the crate provides,
not each change; per-change entries start after it.

### Added

- **Messages:** `Uplink` (`Control`: heartbeat, core `Command`, arm; `ParamRequest`) and `Downlink`
  (heartbeat with arm state and core `ControlMode`, `VehicleState`, `Record` telemetry, `ParamValue`, status text). `Record` is an owned
  `TelemetryRecord`, convertible both ways. The heartbeat's `link_ok` is `false` while a
  failsafe flies its fallback.
- **Frames:** `[version][postcard][crc16]`, COBS-encoded and `0x00`-terminated, through the
  sealed `Message` trait (`encode`/`decode`, with `EncodeError`/`DecodeError`). Rejects bad
  COBS, bytes after the delimiter, short frames, CRC mismatches, other `PROTOCOL_VERSION`s,
  and invalid payloads. Every message fits `MAX_FRAME_LEN`.
- **`Link`:** non-blocking, byte-level transport, split into `LinkRx` + `LinkTx`
  (blanket-implemented) so a receiver and a sender can each own one half.
- **Message I/O:** `SendMessage::send_message` / `RecvMessage::recv_message`, blanket over
  `LinkTx` / `LinkRx`: encode-and-send and receive-and-decode with a caller-supplied buffer,
  failing with `LinkError` (transport, encode, or decode; a bad frame loses only itself).
- **Params:** `params::ParamServer` serves a core `Params` between ticks over bounded
  `heapless::spsc` queues, paced and lossless.
- **Router:** `router::UplinkRouter` decodes uplinks and routes params and `Control` messages
  to optional queues, independent of the active commander.
- **Publisher:** `publisher::LinkPublisher`, a core `TelemetryPublisher`: heartbeat, state, and
  optional records at configured rates, and paced param replies.
- **`fc::FcLink`:** the flight controller's side (param queues, router, param server, publisher)
  wired in one call (`wire`, or `wire_with` and `FcLinkConfig` for publisher rates and per-poll
  limits); its `FcPoller` runs between ticks. `wire_linked` also routes control messages to a
  `LinkCommander`.
- **`commander::LinkCommander`:** flies from the link's `Control` messages (latest command;
  arming only from `Arm`), and implements core `LinkedCommander` for a `FailsafeCommander`.
- **`std` feature:** `udp::UdpLink`, a non-blocking UDP transport to one fixed peer, one frame
  per datagram, with rx/tx halves; `bind`s or wraps a connected socket; tolerates an absent peer.
  `DEFAULT_FC_ADDR` / `DEFAULT_BRIDGE_ADDR` are the localhost ports both ends default to.
- **`testing` feature:** `loopback::Loopback`, an in-memory `Link` pair with rx/tx halves, and
  `loopback::FailingLink` for error paths.
- **`no_std`** (without `std`), allocation-free; builds for `thumbv7em-none-eabihf`.

[unreleased]: https://github.com/wboayue/stable-flight/commits/main/crates/stable-flight-link
