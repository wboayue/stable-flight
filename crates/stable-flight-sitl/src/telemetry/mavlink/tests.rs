use std::sync::{Arc, Mutex};

use assert2::check;
use mavlink::error::{MessageReadError, MessageWriteError};
use mavlink::{MAVLinkMessageRaw, Message};
use stable_flight::{ActuatorCommand, Command, ControlOutput, Setpoint, VehicleState};

use super::*;

/// Records every sent message; fails every send when `fail` is set.
#[derive(Clone, Default)]
struct MockConnection {
    sent: Arc<Mutex<Vec<(MavHeader, MavMessage)>>>,
    fail: bool,
}

impl MockConnection {
    fn sent(&self) -> Vec<(MavHeader, MavMessage)> {
        self.sent.lock().expect("mock lock").clone()
    }
}

fn unsupported() -> MessageReadError {
    MessageReadError::Io(io::Error::from(io::ErrorKind::Unsupported))
}

impl MavConnection<MavMessage> for MockConnection {
    fn recv(&self) -> Result<(MavHeader, MavMessage), MessageReadError> {
        Err(unsupported())
    }
    fn recv_raw(&self) -> Result<MAVLinkMessageRaw, MessageReadError> {
        Err(unsupported())
    }
    fn try_recv(&self) -> Result<(MavHeader, MavMessage), MessageReadError> {
        Err(unsupported())
    }
    fn send(&self, header: &MavHeader, data: &MavMessage) -> Result<usize, MessageWriteError> {
        self.sent
            .lock()
            .expect("mock lock")
            .push((*header, data.clone()));
        if self.fail {
            Err(MessageWriteError::Io(io::Error::from(
                io::ErrorKind::ConnectionRefused,
            )))
        } else {
            Ok(0)
        }
    }
    fn set_protocol_version(&mut self, _version: MavlinkVersion) {}
    fn protocol_version(&self) -> MavlinkVersion {
        MavlinkVersion::V2
    }
    fn set_allow_recv_any_version(&mut self, _allow: bool) {}
    fn allow_recv_any_version(&self) -> bool {
        false
    }
}

/// Publishes `ticks` default records `dt` apart on `connection`.
fn run(connection: &MockConnection, ids: (u8, u8), dt: Duration, ticks: usize) {
    let mut publisher =
        MavlinkPublisher::with_connection(Box::new(connection.clone()), ids.0, ids.1);
    let state = VehicleState::default();
    for _ in 0..ticks {
        publisher.publish(&TelemetryRecord {
            dt,
            command: &Command::Attitude(Default::default()),
            setpoint: &Setpoint::Attitude(Default::default()),
            state: &state,
            control: &ControlOutput::default(),
            actuators: &ActuatorCommand::default(),
        });
    }
}

fn count(sent: &[(MavHeader, MavMessage)], name: &str) -> usize {
    sent.iter()
        .filter(|(_, m)| m.message_name() == name)
        .count()
}

#[test]
fn sends_each_stream_at_its_rate() {
    let connection = MockConnection::default();
    // Two seconds of a 60 Hz loop.
    run(&connection, (1, 1), Duration::from_nanos(16_666_667), 120);
    let sent = connection.sent();

    // (message, expected sends in 2 s: the first tick plus one per period)
    let cases = [
        ("HEARTBEAT", 3),
        ("VFR_HUD", 21),
        ("LOCAL_POSITION_NED", 21),
        ("ATTITUDE", 21),
    ];
    for (name, expected) in cases {
        check!(count(&sent, name) == expected, "{name}");
    }
    check!(sent.len() == 66);
}

#[test]
fn headers_carry_configured_ids_and_a_wrapping_sequence() {
    let connection = MockConnection::default();
    // dt equals the stream interval: 3 streams every tick, plus a heartbeat every 10th.
    run(&connection, (42, 7), STREAM_INTERVAL, 90);
    let sent = connection.sent();

    check!(sent.len() > 256, "enough messages to wrap the sequence");
    for (i, (header, _)) in sent.iter().enumerate() {
        check!(header.system_id == 42);
        check!(header.component_id == 7);
        check!(header.sequence == i as u8, "message {i}");
    }
}

#[test]
fn failed_sends_are_dropped_and_publishing_continues() {
    let connection = MockConnection {
        fail: true,
        ..Default::default()
    };
    run(&connection, (1, 1), STREAM_INTERVAL, 20);

    // Every tick still attempted its sends.
    check!(count(&connection.sent(), "ATTITUDE") == 20);
}

#[test]
fn connect_reports_a_malformed_address() {
    let config = MavlinkConfiguration {
        address: "not-a-mavlink-address".to_owned(),
        ..Default::default()
    };

    let result = MavlinkPublisher::connect(config);

    assert2::assert!(let Err(MavlinkError::Connect { address, .. }) = result);
    check!(address == "not-a-mavlink-address");
}

#[test]
fn connect_opens_udp_out() {
    // udpout binds an ephemeral local port and needs no listener.
    let config = MavlinkConfiguration {
        address: "udpout:127.0.0.1:14550".to_owned(),
        ..Default::default()
    };

    check!(MavlinkPublisher::connect(config).is_ok());
}

#[test]
fn every_stream_sends_on_the_first_publish() {
    let connection = MockConnection::default();
    run(&connection, (1, 1), Duration::from_millis(4), 1);
    let sent = connection.sent();

    for name in ["HEARTBEAT", "VFR_HUD", "LOCAL_POSITION_NED", "ATTITUDE"] {
        check!(count(&sent, name) == 1, "{name}");
    }
}
