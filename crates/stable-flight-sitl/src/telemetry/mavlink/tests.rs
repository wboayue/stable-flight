use std::sync::{Arc, Mutex};

use assert2::check;
use mavlink::common::{GLOBAL_POSITION_INT_DATA, HOME_POSITION_DATA, MavLandedState};
use mavlink::error::{MessageReadError, MessageWriteError};
use mavlink::{MAVLinkMessageRaw, Message};
use stable_flight::{ActuatorCommand, Command, ControlOutput, Saturation, Setpoint, VehicleState};

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
    let publisher = MavlinkPublisher::with_connection(Box::new(connection.clone()), ids.0, ids.1);
    publish_states(
        publisher,
        dt,
        &vec![(false, VehicleState::default()); ticks],
    );
}

/// Publishes one record per `(armed, state)`, `dt` apart.
fn publish_states(mut publisher: MavlinkPublisher, dt: Duration, records: &[(bool, VehicleState)]) {
    for (armed, state) in records {
        publisher.publish(&TelemetryRecord {
            dt,
            command: &Command::new(*armed, Default::default()),
            setpoint: &Setpoint::Attitude(Default::default()),
            state,
            control: &ControlOutput::default(),
            actuators: &ActuatorCommand::default(),
            saturation: Saturation::default(),
        });
    }
}

fn global_positions(sent: &[(MavHeader, MavMessage)]) -> Vec<GLOBAL_POSITION_INT_DATA> {
    sent.iter()
        .filter_map(|(_, m)| match m {
            MavMessage::GLOBAL_POSITION_INT(data) => Some(data.clone()),
            _ => None,
        })
        .collect()
}

fn home_positions(sent: &[(MavHeader, MavMessage)]) -> Vec<HOME_POSITION_DATA> {
    sent.iter()
        .filter_map(|(_, m)| match m {
            MavMessage::HOME_POSITION(data) => Some(data.clone()),
            _ => None,
        })
        .collect()
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
        ("EXTENDED_SYS_STATE", 3),
        ("VFR_HUD", 21),
        ("LOCAL_POSITION_NED", 21),
        ("ATTITUDE", 21),
    ];
    for (name, expected) in cases {
        check!(count(&sent, name) == expected, "{name}");
    }
    check!(sent.len() == 69);
    for name in ["GLOBAL_POSITION_INT", "HOME_POSITION"] {
        check!(count(&sent, name) == 0, "no origin, no {name}");
    }
}

#[test]
fn sends_global_position_at_10_hz_with_an_origin() {
    let connection = MockConnection::default();
    // Two seconds of a 60 Hz loop.
    publish_states(
        with_origin(&connection),
        Duration::from_nanos(16_666_667),
        &vec![(false, VehicleState::default()); 120],
    );

    let positions = global_positions(&connection.sent());
    check!(positions.len() == 21);
    check!(positions[0].lat == 348_843_890);
    check!(positions[0].lon == -1_204_035_000);
}

fn with_origin(connection: &MockConnection) -> MavlinkPublisher {
    MavlinkPublisher::with_connection(Box::new(connection.clone()), 1, 1)
        .with_origin(DEFAULT_ORIGIN)
        .expect("valid origin")
}

fn at_altitude(altitude_m: f32) -> VehicleState {
    VehicleState {
        position_m: [0.0, 0.0, altitude_m].into(),
        ..Default::default()
    }
}

#[test]
fn relative_altitude_is_measured_from_home_latched_at_each_arming() {
    // (armed, altitude_m, expected relative_alt in mm; None: position not finite, skipped)
    let ticks = [
        (false, f32::NAN, None),      // no finite altitude yet: no home, nothing sent
        (false, 30.0, Some(0)),       // first finite altitude becomes home
        (false, 35.0, Some(5_000)),   // moved while disarmed: home stays
        (true, 50.0, Some(0)),        // arming re-latches home
        (true, 60.0, Some(10_000)),   // climbing
        (false, 20.0, Some(-30_000)), // disarmed below home
        (true, f32::NAN, None),       // arming on a NaN altitude: no home yet
        (true, 55.0, Some(0)),        // the next finite altitude becomes home
        (true, 60.0, Some(5_000)),
    ];
    let records: Vec<_> = ticks
        .iter()
        .map(|&(armed, z, _)| (armed, at_altitude(z)))
        .collect();
    let connection = MockConnection::default();
    publish_states(with_origin(&connection), STREAM_INTERVAL, &records);

    let positions = global_positions(&connection.sent());
    let expected: Vec<_> = ticks.iter().filter_map(|&(_, _, alt)| alt).collect();
    check!(positions.iter().map(|p| p.relative_alt).collect::<Vec<_>>() == expected);
    check!(positions[1].alt == 35_000, "alt stays above sea level");
}

#[test]
fn rejects_an_out_of_range_origin() {
    let origin = |latitude_deg, longitude_deg, altitude_m| GeodeticOrigin {
        latitude_deg,
        longitude_deg,
        altitude_m,
    };
    // (origin, valid)
    let cases = [
        (DEFAULT_ORIGIN, true),
        (origin(89.9, -180.0, -400.0), true),
        (origin(-89.9, 180.0, 8_000.0), true),
        (origin(90.0, 0.0, 0.0), false), // pole: longitude degenerates
        (origin(-90.0, 0.0, 0.0), false),
        (origin(f64::NAN, 0.0, 0.0), false),
        (origin(0.0, 180.5, 0.0), false),
        (origin(0.0, f64::NAN, 0.0), false),
        (origin(0.0, 0.0, f64::INFINITY), false),
    ];

    for (origin, valid) in cases {
        let result = MavlinkPublisher::with_connection(Box::new(MockConnection::default()), 1, 1)
            .with_origin(origin);
        if valid {
            check!(result.is_ok(), "{origin:?}");
        } else {
            assert2::assert!(let Err(MavlinkError::InvalidOrigin(rejected)) = result);
            // Compared as text: NaN never equals itself.
            check!(format!("{rejected:?}") == format!("{origin:?}"));
        }
    }
}

#[test]
fn connect_reports_an_invalid_origin() {
    let config = MavlinkConfiguration {
        origin: Some(GeodeticOrigin {
            latitude_deg: 91.0,
            ..DEFAULT_ORIGIN
        }),
        ..Default::default()
    };

    assert2::assert!(let Err(MavlinkError::InvalidOrigin(_)) = MavlinkPublisher::connect(config));
}

#[test]
fn headers_carry_configured_ids_and_a_wrapping_sequence() {
    let connection = MockConnection::default();
    // dt equals the stream interval: 3 streams every tick, plus 2 slow streams every 10th.
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

    for name in [
        "HEARTBEAT",
        "EXTENDED_SYS_STATE",
        "VFR_HUD",
        "LOCAL_POSITION_NED",
        "ATTITUDE",
    ] {
        check!(count(&sent, name) == 1, "{name}");
    }
}

#[test]
fn sends_home_position_at_1_hz_with_an_origin() {
    let connection = MockConnection::default();
    // Two seconds of a 60 Hz loop.
    publish_states(
        with_origin(&connection),
        Duration::from_nanos(16_666_667),
        &vec![(false, VehicleState::default()); 120],
    );

    let homes = home_positions(&connection.sent());
    check!(homes.len() == 3);
    check!(homes[0].latitude == 348_843_890);
    check!(homes[0].longitude == -1_204_035_000);
}

#[test]
fn home_position_is_sent_as_soon_as_home_is_latched() {
    let at = |north_m: f32, up_m: f32| VehicleState {
        position_m: [north_m, 0.0, up_m].into(),
        ..Default::default()
    };
    // (armed, state, expected HOME_POSITION (x, z) sent this tick); dt is 100 ms, so the
    // 1 Hz timer alone would fire only on the first tick.
    let ticks = [
        (false, at(f32::NAN, 0.0), None), // no finite position: no home, nothing sent
        (false, at(1.0, 2.0), Some((1.0, -2.0))), // first finite position becomes home
        (false, at(5.0, 2.0), None),      // moved while disarmed: home stays
        (true, at(5.0, 3.0), Some((5.0, -3.0))), // arming re-latches home
        (true, at(9.0, 9.0), None),       // flying
        (false, at(9.0, 0.0), None),
        (true, at(f32::NAN, 0.0), None), // arming on a non-finite position: no home yet
        (true, at(7.0, 1.0), Some((7.0, -1.0))), // the next finite position becomes home
        (true, at(8.0, 4.0), None),
    ];
    let records: Vec<_> = ticks
        .iter()
        .map(|(armed, s, _)| (*armed, s.clone()))
        .collect();
    let connection = MockConnection::default();
    publish_states(with_origin(&connection), STREAM_INTERVAL, &records);

    let homes = home_positions(&connection.sent());
    let expected: Vec<_> = ticks.iter().filter_map(|&(_, _, home)| home).collect();
    check!(homes.iter().map(|h| (h.x, h.z)).collect::<Vec<_>>() == expected);
}

#[test]
fn landed_state_is_sent_as_soon_as_it_changes() {
    use MavLandedState::{
        MAV_LANDED_STATE_IN_AIR as AIR, MAV_LANDED_STATE_ON_GROUND as GROUND,
        MAV_LANDED_STATE_UNDEFINED as UNDEFINED,
    };
    // (armed, altitude_m, expected EXTENDED_SYS_STATE sent this tick); dt is 100 ms, so the
    // 1 Hz timer alone would fire only on the first tick.
    let ticks = [
        (true, f32::NAN, Some(UNDEFINED)), // the first publish; no home yet
        (true, 10.0, Some(GROUND)),        // home latched here: height 0
        (true, 10.4, None),                // below the takeoff height
        (true, 12.0, Some(AIR)),           // took off
        (true, 11.0, None),
        (true, 10.1, Some(GROUND)), // landed
        (false, 10.1, None),
    ];
    let records: Vec<_> = ticks
        .iter()
        .map(|&(armed, z, _)| (armed, at_altitude(z)))
        .collect();
    let connection = MockConnection::default();
    let publisher = MavlinkPublisher::with_connection(Box::new(connection.clone()), 1, 1);
    publish_states(publisher, STREAM_INTERVAL, &records);

    let sent: Vec<_> = connection
        .sent()
        .into_iter()
        .filter_map(|(_, m)| match m {
            MavMessage::EXTENDED_SYS_STATE(data) => Some(data.landed_state),
            _ => None,
        })
        .collect();
    let expected: Vec<_> = ticks.iter().filter_map(|&(_, _, state)| state).collect();
    check!(sent == expected);
}
