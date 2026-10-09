use std::io;

use assert2::check;
use mavlink::Message;
use mavlink::error::ParserError;
use stable_flight_link::loopback::{FailingLink, Loopback, LoopbackLink, LoopbackRx, LoopbackTx};
use stable_flight_link::{ParamRequest, Uplink};
use stable_flight_mavlink::testing::MockConnection;

use super::*;
use crate::bridge::BridgeConfig;
use crate::test_support::{GCS, heartbeat, param, request_list};

const DEPTH: usize = 128;

type LoopbackPump<'a> = Pump<LoopbackRx<'a, DEPTH>, LoopbackTx<'a, DEPTH>>;

/// A pump on one end of `loopback`, with the flight controller's end and the ground station
/// returned for the test to drive.
fn pump(
    loopback: &Loopback<DEPTH>,
    gcs: MockConnection,
) -> (LoopbackPump<'_>, LoopbackLink<'_, DEPTH>) {
    let (bridge_end, fc) = loopback.split();
    let (rx, tx) = bridge_end.into_halves();
    let pump = Pump::new(rx, tx, Box::new(gcs), Bridge::new(BridgeConfig::default()));
    (pump, fc)
}

fn send(fc: &mut LoopbackLink<'_, DEPTH>, msg: &Downlink) {
    fc.send_message(msg, &mut [0u8; MAX_FRAME_LEN])
        .expect("loopback has room");
}

fn uplinks(fc: &mut LoopbackLink<'_, DEPTH>) -> Vec<Uplink> {
    let mut buf = [0u8; MAX_FRAME_LEN];
    std::iter::from_fn(|| fc.recv_message::<Uplink>(&mut buf).expect("valid uplink")).collect()
}

#[test]
fn moves_messages_both_ways_as_the_flight_controller() {
    let loopback = Loopback::new();
    let gcs = MockConnection::default();
    let (mut pump, mut fc) = pump(&loopback, gcs.clone());

    send(&mut fc, &heartbeat(false));
    send(&mut fc, &Downlink::Param(param("A", 1.0, 0, 1)));
    let activity = pump.step(Instant::now());

    check!(activity.downlinks == 2);
    let sent = gcs.sent();
    let config = BridgeConfig::default();
    check!(
        sent.iter()
            .all(|(h, _)| (h.system_id, h.component_id) == (config.system_id, config.component_id)),
        "sent as the flight controller"
    );
    let names: Vec<_> = sent.iter().map(|(_, msg)| msg.message_name()).collect();
    check!(names == ["PARAM_VALUE", "HEARTBEAT"]);
    check!(
        uplinks(&mut fc) == [Uplink::Param(ParamRequest::List)],
        "listing on appearing"
    );

    gcs.push_received(GCS, request_list());
    let activity = pump.step(Instant::now());

    check!(activity.mavlink == 1);
    check!(uplinks(&mut fc) == [Uplink::Param(ParamRequest::List)]);
}

#[test]
fn forwards_flight_controller_state_as_mavlink() {
    let loopback = Loopback::new();
    let gcs = MockConnection::default();
    let (mut pump, mut fc) = pump(&loopback, gcs.clone());

    send(&mut fc, &heartbeat(false));
    send(
        &mut fc,
        &Downlink::State(stable_flight_link::stable_flight::VehicleState {
            roll_rad: 10_f32.to_radians(),
            ..Default::default()
        }),
    );
    pump.step(Instant::now());

    let attitude = gcs.sent().into_iter().find_map(|(_, msg)| match msg {
        MavMessage::ATTITUDE(data) => Some(data),
        _ => None,
    });
    check!(attitude.map(|a| a.roll) == Some(10_f32.to_radians()));
}

/// How a row sets up the pass.
enum Setup {
    /// Raw bytes on the link, then a heartbeat.
    GarbageFrameThenHeartbeat,
    /// The ground station's `try_recv` fails with this, then a param listing request arrives.
    GcsErrorThenRequest(fn() -> MessageReadError),
    /// The flight controller appears, so the pump sends a heartbeat; every send fails.
    FailingSends,
}

#[test]
fn applies_the_error_policy_per_side() {
    // (case, setup, expected activity, ground-station results left unread)
    let cases = [
        (
            "link frame that doesn't decode is skipped; the next is read",
            Setup::GarbageFrameThenHeartbeat,
            Activity {
                downlinks: 1,
                dropped: 1,
                ..Activity::default()
            },
            0,
        ),
        (
            "MAVLink that doesn't parse is skipped; the next is read",
            Setup::GcsErrorThenRequest(|| {
                MessageReadError::Parse(ParserError::UnknownMessage { id: 9999 })
            }),
            Activity {
                mavlink: 1,
                dropped: 1,
                ..Activity::default()
            },
            0,
        ),
        (
            "ground-station receive error ends its reads for the pass",
            Setup::GcsErrorThenRequest(|| {
                MessageReadError::Io(io::Error::from(io::ErrorKind::ConnectionReset))
            }),
            Activity {
                failures: 1,
                ..Activity::default()
            },
            1,
        ),
        (
            "failed sends are dropped; the pass continues",
            Setup::FailingSends,
            Activity {
                downlinks: 1,
                failures: 1,
                ..Activity::default()
            },
            0,
        ),
    ];
    for (case, setup, expected, unread) in cases {
        let loopback = Loopback::new();
        let gcs = match setup {
            Setup::FailingSends => MockConnection::failing(),
            _ => MockConnection::default(),
        };
        let (mut pump, mut fc) = pump(&loopback, gcs.clone());
        match setup {
            Setup::GarbageFrameThenHeartbeat => {
                fc.send(&[1, 2, 3, 0]).expect("loopback has room");
                send(&mut fc, &heartbeat(false));
            }
            Setup::GcsErrorThenRequest(error) => {
                gcs.push_error(error());
                gcs.push_received(GCS, request_list());
            }
            Setup::FailingSends => send(&mut fc, &heartbeat(false)),
        }

        check!(pump.step(Instant::now()) == expected, "{case}");
        check!(gcs.pending() == unread, "{case}");
    }
}

#[test]
fn link_failure_still_serves_the_ground_station() {
    let gcs = MockConnection::default();
    let mut pump = Pump::new(
        FailingLink,
        FailingLink,
        Box::new(gcs.clone()),
        Bridge::new(BridgeConfig::default()),
    );
    gcs.push_received(GCS, request_list());

    let activity = pump.step(Instant::now());

    // The receive fails, the request is read, and forwarding the listing request fails.
    check!(
        activity
            == Activity {
                mavlink: 1,
                failures: 2,
                ..Activity::default()
            }
    );
}

#[test]
fn reads_at_most_a_bounded_batch_per_side_per_pass() {
    let loopback = Loopback::new();
    let gcs = MockConnection::default();
    let (mut pump, mut fc) = pump(&loopback, gcs.clone());
    let queued = MAX_READS_PER_PASS + 6;
    for _ in 0..queued {
        send(&mut fc, &heartbeat(false));
        gcs.push_received(GCS, request_list());
    }

    let first = pump.step(Instant::now());
    let second = pump.step(Instant::now());

    check!((first.downlinks, first.mavlink) == (MAX_READS_PER_PASS, MAX_READS_PER_PASS));
    check!((second.downlinks, second.mavlink) == (6, 6));
}
