# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- **Breaking:** `PROTOCOL_VERSION` 3 (#124): core `ControlInput::AbsoluteRate`,
  `ControlMode::AbsoluteRate`, `Setpoint::BodyRate`/`Plane`/`PlaneRate`, and
  `ModeRefusal::RollError` change the message shapes.
- **Breaking:** `PROTOCOL_VERSION` 2 (#181): `Downlink::Heartbeat` carries the flown
  `flight_mode` (a core `FlightModeReport`), `Record` gains `flight_mode`, and the core message
  types changed shape (`Command::regime`, `VehicleState::airspeed_mps`; `AttitudeCommand::mode`,
  `RateCommand::mode`, and `WaypointCommand::roll_rad` removed). Flight controller and companion
  must both update.

## [0.1.0] - 2026-10-10

First release.

### Added

- **Messages:** `Uplink` (`Control`: heartbeat, core `Command`, arm; `ParamRequest`) and `Downlink`
  (heartbeat with arm state and core `ControlMode`, `VehicleState`, `Output`, `Record` telemetry, `ParamValue`, status text). `Output` is the control
  output streamed with state: commanded thrust and the `Saturation` flags. `Record` is an owned
  `TelemetryRecord`, convertible both ways, for opt-in debugging. The heartbeat's `link_ok` is `false` while a
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
- **Publisher:** `publisher::LinkPublisher`, a core `TelemetryPublisher`: heartbeat, state (each
  followed by an `Output` whose saturation flags are ORed over every tick since the last), and
  optional records at configured rates, and paced param replies; each failsafe change is
  announced once as status text.
- **`fc::FcLink`:** the flight controller's side (param queues, router, param server, publisher)
  wired in one call (`wire`, or `wire_with` and `FcLinkConfig` for publisher rates and per-poll
  limits); its `FcPoller` runs between ticks. `wire_linked` also routes control messages to a
  `LinkCommander`.
- **`commander::LinkCommander`:** flies from the link's `Control` messages (latest command;
  arming only from `Arm`), and implements core `LinkedCommander` for a `FailsafeCommander`:
  any message keeps a waypoint fresh, only commands keep a stream (sticks, absolute attitude)
  fresh; `sync_arm` drops a stale
  latched arm at a failsafe handover.
- **`std` feature:** `udp::UdpLink`, a non-blocking UDP transport to one fixed peer, one frame
  per datagram, with rx/tx halves; `bind`s or wraps a connected socket; tolerates an absent peer.
  `DEFAULT_FC_ADDR` / `DEFAULT_BRIDGE_ADDR` are the localhost ports both ends default to.
- **`testing` feature:** `loopback::Loopback`, an in-memory `Link` pair with rx/tx halves, and
  `loopback::FailingLink` for error paths.
- **`no_std`** (without `std`), allocation-free; builds for `thumbv7em-none-eabihf`.

[unreleased]: https://github.com/wboayue/stable-flight/compare/stable-flight-link-v0.1.0...HEAD
[0.1.0]: https://github.com/wboayue/stable-flight/releases/tag/stable-flight-link-v0.1.0
