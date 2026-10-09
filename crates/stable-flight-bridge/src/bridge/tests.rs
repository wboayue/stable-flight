use std::time::{Duration, Instant};

use assert2::{assert, check};
use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use stable_flight::mixers::QuadCopterMixer;
use stable_flight::params::Params;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight::testing::{ConstantCommander, NullVehicle};
use stable_flight::{Command, FlightLoop};
use stable_flight_link::heapless::spsc::Queue;
use stable_flight_link::loopback::{Loopback, LoopbackLink};
use stable_flight_link::params::ParamServer;
use stable_flight_link::publisher::{LinkPublisher, LinkPublisherConfig};
use stable_flight_link::router::UplinkRouter;
use stable_flight_link::{MAX_FRAME_LEN, ParamValue, RecvMessage, SendMessage};

use super::*;

/// A ground station's header.
const GCS: MavHeader = MavHeader {
    system_id: 255,
    component_id: 190,
    sequence: 0,
};

fn param(name: &str, value: f32, index: u16, count: u16) -> Downlink {
    Downlink::Param(ParamValue {
        name: name.try_into().unwrap(),
        value,
        index,
        count,
    })
}

fn heartbeat(armed: bool) -> Downlink {
    Downlink::Heartbeat {
        armed,
        mode: 0,
        link_ok: true,
    }
}

fn set(name: &str, value: f32, param_type: MavParamType) -> MavMessage {
    MavMessage::PARAM_SET(PARAM_SET_DATA {
        param_value: value,
        target_system: 1,
        target_component: 1,
        param_id: name.into(),
        param_type,
    })
}

fn read_index(index: i16) -> MavMessage {
    MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
        param_index: index,
        target_system: 1,
        target_component: 1,
        param_id: "".into(),
    })
}

fn read_name(name: &str) -> MavMessage {
    MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
        param_index: -1,
        target_system: 1,
        target_component: 1,
        param_id: name.into(),
    })
}

fn request_message(id: u32) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
        param1: id as f32,
        target_system: 1,
        target_component: 1,
        ..Default::default()
    })
}

/// A bridge whose flight controller is up, with params `A` (index 0) = 1.0 and `B` = 2.0 cached.
fn bridge_with_params(now: Instant) -> Bridge {
    let mut bridge = Bridge::new(BridgeConfig::default());
    let mut out = Outbox::default();
    bridge.on_downlink(&heartbeat(false), now, &mut out);
    bridge.on_downlink(&param("A", 1.0, 0, 2), now, &mut out);
    bridge.on_downlink(&param("B", 2.0, 1, 2), now, &mut out);
    bridge
}

/// `(name, value, index, count)` of a `PARAM_VALUE`.
fn param_value_of(msg: &MavMessage) -> Option<(String, f32, u16, u16)> {
    let MavMessage::PARAM_VALUE(data) = msg else {
        return None;
    };
    check!(data.param_type == MavParamType::MAV_PARAM_TYPE_REAL32);
    Some((
        data.param_id.to_str().unwrap().to_owned(),
        data.param_value,
        data.param_index,
        data.param_count,
    ))
}

fn uplink_set(name: &str, value: f32) -> Uplink {
    Uplink::Param(ParamRequest::Set {
        name: name.try_into().unwrap(),
        value,
    })
}

#[test]
fn params_translate_between_mavlink_and_the_link() {
    let list = Uplink::Param(ParamRequest::List);
    let a = Some(("A".to_owned(), 1.0, 0, 2));
    // (case, from the ground station, expected uplinks, expected PARAM_VALUE)
    let cases = [
        (
            "list goes to the flight controller",
            MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA {
                target_system: 1,
                target_component: 0,
            }),
            vec![list.clone()],
            None,
        ),
        (
            "read by index from the cache",
            read_index(0),
            vec![],
            a.clone(),
        ),
        (
            "read by name from the cache",
            read_name("A"),
            vec![],
            a.clone(),
        ),
        (
            "read of an uncached index: no reply",
            read_index(5),
            vec![],
            None,
        ),
        (
            "read of an uncached name: no reply",
            read_name("C"),
            vec![],
            None,
        ),
        (
            "REAL32 set goes to the flight controller",
            set("A", 30.0, MavParamType::MAV_PARAM_TYPE_REAL32),
            vec![uplink_set("A", 30.0)],
            None,
        ),
        (
            "set of an uncached name still goes; the flight controller decides",
            set("C", 3.0, MavParamType::MAV_PARAM_TYPE_REAL32),
            vec![uplink_set("C", 3.0)],
            None,
        ),
        (
            "non-REAL32 set is rejected with the current value",
            set("A", 30.0, MavParamType::MAV_PARAM_TYPE_INT32),
            vec![],
            a.clone(),
        ),
        (
            "set without a name is ignored",
            set("", 30.0, MavParamType::MAV_PARAM_TYPE_REAL32),
            vec![],
            None,
        ),
    ];
    for (case, msg, to_fc, to_gcs) in cases {
        let now = Instant::now();
        let mut bridge = bridge_with_params(now);
        let mut out = Outbox::default();
        bridge.on_mavlink(&GCS, &msg, &mut out);
        check!(out.to_fc == to_fc, "{case}");
        check!(
            out.to_gcs.iter().map(param_value_of).collect::<Vec<_>>()
                == Vec::from_iter(to_gcs.map(Some)),
            "{case}"
        );
    }
}

#[test]
fn every_flight_controller_param_goes_out_as_param_value() {
    // (case, downlink)
    let cases = [
        ("listing entry", param("A", 1.0, 0, 2)),
        (
            "accepted write's echo: the value sent",
            param("STAB_ATT_AMAX_X", 30.0, 4, 20),
        ),
        (
            "rejected write's echo: the current value",
            param("STAB_RATE_IMAX", 0.3, 7, 20),
        ),
    ];
    for (case, downlink) in cases {
        let Downlink::Param(sent) = &downlink else {
            unreachable!()
        };
        let mut bridge = Bridge::new(BridgeConfig::default());
        let mut out = Outbox::default();
        bridge.on_downlink(&downlink, Instant::now(), &mut out);
        let expected = (sent.name.to_string(), sent.value, sent.index, sent.count);
        check!(
            out.to_gcs.iter().map(param_value_of).collect::<Vec<_>>() == [Some(expected)],
            "{case}"
        );
    }
}

#[test]
fn echo_updates_the_cache() {
    let now = Instant::now();
    let mut bridge = bridge_with_params(now);
    let mut out = Outbox::default();
    bridge.on_downlink(&param("A", 5.0, 0, 2), now, &mut out);
    out.to_gcs.clear();

    bridge.on_mavlink(&GCS, &read_name("A"), &mut out);

    check!(
        out.to_gcs.iter().map(param_value_of).collect::<Vec<_>>()
            == [Some(("A".to_owned(), 5.0, 0, 2))]
    );
}

#[test]
fn ignores_mavlink_addressed_elsewhere() {
    // (case, target system, target component, handled)
    let cases = [
        ("the flight controller", 1, 1, true),
        ("broadcast component", 1, 0, true),
        ("broadcast system", 0, 1, true),
        ("another system", 2, 1, false),
        ("another component", 1, 191, false),
    ];
    for (case, target_system, target_component, handled) in cases {
        let mut bridge = Bridge::new(BridgeConfig::default());
        let mut out = Outbox::default();
        let msg = MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA {
            target_system,
            target_component,
        });
        bridge.on_mavlink(&GCS, &msg, &mut out);
        check!(!out.to_fc.is_empty() == handled, "{case}");
    }
}

#[test]
fn answers_every_mission_type_with_an_empty_list() {
    let types = [
        MavMissionType::MAV_MISSION_TYPE_MISSION,
        MavMissionType::MAV_MISSION_TYPE_FENCE,
        MavMissionType::MAV_MISSION_TYPE_RALLY,
    ];
    for mission_type in types {
        let mut bridge = Bridge::new(BridgeConfig::default());
        let mut out = Outbox::default();
        let msg = MavMessage::MISSION_REQUEST_LIST(MISSION_REQUEST_LIST_DATA {
            target_system: 1,
            target_component: 1,
            mission_type,
        });
        bridge.on_mavlink(&GCS, &msg, &mut out);

        let [MavMessage::MISSION_COUNT(count)] = out.to_gcs.as_slice() else {
            panic!(
                "{mission_type:?}: expected one MISSION_COUNT, got {:?}",
                out.to_gcs
            );
        };
        check!(count.count == 0, "{mission_type:?}");
        check!(count.mission_type == mission_type);
        check!((count.target_system, count.target_component) == (GCS.system_id, GCS.component_id));
        check!(out.to_fc.is_empty());
    }
}

#[test]
fn acks_commands_and_serves_autopilot_version() {
    let arm = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        param1: 1.0,
        target_system: 1,
        target_component: 1,
        ..Default::default()
    });
    // (case, command, expected result, AUTOPILOT_VERSION follows)
    let cases = [
        (
            "AUTOPILOT_VERSION request",
            request_message(AUTOPILOT_VERSION_DATA::ID),
            MavResult::MAV_RESULT_ACCEPTED,
            true,
        ),
        (
            "COMPONENT_METADATA request (#206)",
            request_message(397),
            MavResult::MAV_RESULT_UNSUPPORTED,
            false,
        ),
        (
            "arm (later slice)",
            arm,
            MavResult::MAV_RESULT_UNSUPPORTED,
            false,
        ),
    ];
    for (case, msg, result, version) in cases {
        let mut bridge = Bridge::new(BridgeConfig::default());
        let mut out = Outbox::default();
        bridge.on_mavlink(&GCS, &msg, &mut out);

        let Some(MavMessage::COMMAND_ACK(ack)) = out.to_gcs.first() else {
            panic!("{case}: expected an ack first, got {:?}", out.to_gcs);
        };
        check!(ack.result == result, "{case}");
        check!((ack.target_system, ack.target_component) == (GCS.system_id, GCS.component_id));
        match out.to_gcs.get(1) {
            Some(MavMessage::AUTOPILOT_VERSION(data)) => {
                check!(version, "{case}");
                check!(data.capabilities.contains(
                    MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
                        | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST
                ));
            }
            other => {
                check!(!version && other.is_none(), "{case}: {other:?}");
            }
        }
        check!(out.to_fc.is_empty(), "{case}");
    }
}

#[test]
fn heartbeats_only_while_the_flight_controller_is_alive() {
    let config = BridgeConfig::default();
    let t0 = Instant::now();
    let mut bridge = Bridge::new(config);
    let mut out = Outbox::default();
    let heartbeats = |out: &mut Outbox| {
        let n = out
            .to_gcs
            .iter()
            .filter(|msg| matches!(msg, MavMessage::HEARTBEAT(_)))
            .count();
        out.to_gcs.clear();
        n
    };

    bridge.poll(t0, &mut out);
    check!(heartbeats(&mut out) == 0, "no flight controller yet");

    bridge.on_downlink(&heartbeat(true), t0, &mut out);
    bridge.poll(t0, &mut out);
    let sent = out.to_gcs.clone();
    check!(heartbeats(&mut out) == 1, "first heartbeat right away");
    let [MavMessage::HEARTBEAT(data)] = sent.as_slice() else {
        unreachable!()
    };
    check!(
        data.base_mode
            .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED)
    );

    bridge.poll(t0 + config.heartbeat_interval / 2, &mut out);
    check!(heartbeats(&mut out) == 0, "not due yet");

    bridge.poll(t0 + config.heartbeat_interval, &mut out);
    check!(heartbeats(&mut out) == 1, "due");

    bridge.poll(t0 + config.fc_timeout, &mut out);
    check!(heartbeats(&mut out) == 0, "flight controller timed out");
}

#[test]
fn refills_the_cache_whenever_the_flight_controller_appears() {
    let config = BridgeConfig::default();
    let t0 = Instant::now();
    let mut bridge = bridge_with_params(t0);
    let list = [Uplink::Param(ParamRequest::List)];
    let mut out = Outbox::default();

    bridge.on_downlink(&heartbeat(false), t0 + Duration::from_millis(200), &mut out);
    check!(out.to_fc.is_empty(), "still alive: no new listing");

    let back = t0 + Duration::from_millis(200) + config.fc_timeout;
    bridge.on_downlink(&heartbeat(false), back, &mut out);
    check!(out.to_fc == list, "back after a timeout: list again");

    bridge.on_mavlink(&GCS, &read_index(0), &mut out);
    check!(out.to_gcs.is_empty(), "cache cleared");
}

#[test]
fn first_heartbeat_lists_params() {
    let mut bridge = Bridge::new(BridgeConfig::default());
    let mut out = Outbox::default();
    bridge.on_downlink(&heartbeat(false), Instant::now(), &mut out);
    check!(out.to_fc == [Uplink::Param(ParamRequest::List)]);
}

type FcLoop<'a> = FlightLoop<
    ConstantCommander,
    NullVehicle,
    CopterGuidance,
    CopterStabilizer,
    QuadCopterMixer,
    LinkPublisher<'a, stable_flight_link::loopback::LoopbackTx<'a, 64>>,
>;

/// Runs the bridge against a real flight-controller link stack (router, param server, link
/// publisher, flight loop) over a loopback, until `done` holds for the ground-station messages
/// collected so far.
fn exchange(
    bridge: &mut Bridge,
    companion: &mut LoopbackLink<'_, 64>,
    fc: &mut impl FnMut(),
    gcs: &mut Vec<MavMessage>,
    done: &dyn Fn(&[MavMessage]) -> bool,
) {
    let mut buf = [0u8; MAX_FRAME_LEN];
    let mut out = Outbox::default();
    for _ in 0..10_000 {
        let now = Instant::now();
        fc();
        while let Some(msg) = companion.recv_message::<Downlink>(&mut buf).expect("valid") {
            bridge.on_downlink(&msg, now, &mut out);
        }
        bridge.poll(now, &mut out);
        for msg in out.to_fc.drain(..) {
            companion.send_message(&msg, &mut buf).expect("sends");
        }
        gcs.append(&mut out.to_gcs);
        if done(gcs) {
            return;
        }
    }
    panic!("no answer after 10k ticks");
}

#[test]
fn ground_station_lists_and_sets_flight_controller_params() {
    const DT: Duration = Duration::from_millis(4);
    let loopback = Loopback::<64>::new();
    let (mut companion, fc_end) = loopback.split();
    let (rx, tx) = fc_end.into_halves();
    let mut requests = Queue::<ParamRequest, 16>::new();
    let mut replies = Queue::<ParamValue, 8>::new();
    let (to_server, from_router) = requests.split();
    let (to_publisher, from_server) = replies.split();
    let mut router = UplinkRouter::new(rx, 8).with_params(to_server);
    let mut server = ParamServer::new(from_router, to_publisher, 4);
    let telemetry =
        LinkPublisher::new(tx, LinkPublisherConfig::default()).with_param_replies(from_server);
    let mut flight_loop: FcLoop = FlightLoop::new(
        ConstantCommander::new(Command::default()),
        NullVehicle::new(),
        CopterGuidance::new(CopterGuidanceConfig::default()),
        CopterStabilizer::new(CopterStabilizerConfig::default()),
        QuadCopterMixer::default(),
        telemetry,
    );
    let count = flight_loop.param_count();
    let default_imax = flight_loop.param("STAB_RATE_IMAX").expect("exists");
    flight_loop.prime();
    let mut fc = || {
        // One tick, as `RealTimeRunner::run_with` runs it.
        router.poll().expect("link up");
        server.poll(&mut flight_loop);
        flight_loop.step(DT);
    };
    let mut bridge = Bridge::new(BridgeConfig::default());
    let mut gcs = Vec::new();
    let param_values =
        |gcs: &[MavMessage]| gcs.iter().filter_map(param_value_of).collect::<Vec<_>>();

    // The first heartbeat lists params into the cache (and out to the ground station).
    exchange(&mut bridge, &mut companion, &mut fc, &mut gcs, &|gcs| {
        param_values(gcs).len() >= count
    });
    let listing = param_values(&gcs);
    check!(listing.len() == count);
    for (i, (_, _, index, total)) in listing.iter().enumerate() {
        check!(usize::from(*index) == i);
        check!(usize::from(*total) == count);
    }
    check!(
        gcs.iter()
            .any(|msg| matches!(msg, MavMessage::HEARTBEAT(_)))
    );

    // QGC re-reads an index it missed: served from the cache.
    gcs.clear();
    let mut out = Outbox::default();
    bridge.on_mavlink(&GCS, &read_index(3), &mut out);
    check!(param_values(&out.to_gcs) == [listing[3].clone()]);

    // (case, set, echoed value)
    let cases = [
        (
            "accepted degree param echoes the value sent",
            "STAB_ATT_AMAX_X",
            30.0,
            30.0,
        ),
        (
            "rejected value echoes the current value",
            "STAB_RATE_IMAX",
            -1.0,
            default_imax,
        ),
    ];
    for (case, name, value, echoed) in cases {
        gcs.clear();
        let mut out = Outbox::default();
        bridge.on_mavlink(
            &GCS,
            &set(name, value, MavParamType::MAV_PARAM_TYPE_REAL32),
            &mut out,
        );
        for msg in out.to_fc.drain(..) {
            companion
                .send_message(&msg, &mut [0u8; MAX_FRAME_LEN])
                .expect("sends");
        }
        exchange(&mut bridge, &mut companion, &mut fc, &mut gcs, &|gcs| {
            !param_values(gcs).is_empty()
        });
        let (echo_name, echo_value, index, _) = param_values(&gcs).remove(0);
        check!(echo_name == name, "{case}");
        check!(echo_value == echoed, "{case}");
        let listed = listing.iter().find(|(n, ..)| n == name).expect("listed");
        check!(index == listed.2, "{case}: the echo keeps the listed index");
    }

    drop(fc);
    let stored = flight_loop.param("STAB_ATT_AMAX_X").expect("exists");
    assert!((stored - 30.0).abs() < 1e-4);
}
