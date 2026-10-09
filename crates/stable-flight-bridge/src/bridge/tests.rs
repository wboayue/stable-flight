use std::time::{Duration, Instant};

use assert2::{assert, check};
use mavlink::Message;
use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use stable_flight::mixers::QuadCopterMixer;
use stable_flight::params::Params;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight::testing::{ConstantCommander, NullVehicle};
use stable_flight::{Command, FlightLoop};
use stable_flight_link::loopback::{Loopback, LoopbackLink};
use stable_flight_link::publisher::LinkPublisher;
use stable_flight_link::testing::FcLink;
use stable_flight_link::{MAX_FRAME_LEN, ParamRequest, RecvMessage, SendMessage, Uplink};

use super::*;
use crate::test_support::{
    GCS, at_altitude, earth, heartbeat, home_zs, param, param_value_of, read_index, request_list,
    set,
};

/// A bridge whose flight controller is up, with params `A` (index 0) = 1.0 and `B` = 2.0 cached.
fn bridge_with_params(now: Instant) -> Bridge {
    let mut bridge = Bridge::new(BridgeConfig::default());
    let mut out = Outbox::default();
    bridge.on_downlink(&heartbeat(false), now, &mut out);
    bridge.on_downlink(&Downlink::Param(param("A", 1.0, 0, 2)), now, &mut out);
    bridge.on_downlink(&Downlink::Param(param("B", 2.0, 1, 2)), now, &mut out);
    bridge
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
fn heartbeats_only_while_the_flight_controller_is_alive() {
    let config = BridgeConfig::default();
    let t0 = Instant::now();
    let mut bridge = Bridge::new(config);
    let mut out = Outbox::default();
    let mut heartbeats_at = |bridge: &mut Bridge, at: Instant| {
        bridge.poll(at, &mut out);
        let n = out.to_gcs.len();
        out.to_gcs.clear();
        n
    };

    check!(
        heartbeats_at(&mut bridge, t0) == 0,
        "no flight controller yet"
    );
    bridge.on_downlink(&heartbeat(true), t0, &mut Outbox::default());
    check!(heartbeats_at(&mut bridge, t0) == 1, "right away");
    check!(
        heartbeats_at(&mut bridge, t0 + config.fc_timeout) == 0,
        "timed out"
    );
}

#[test]
fn routes_each_message_to_its_service() {
    let mission_list = MavMessage::MISSION_REQUEST_LIST(MISSION_REQUEST_LIST_DATA {
        target_system: 1,
        target_component: 1,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
    });
    let command = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        target_system: 1,
        target_component: 1,
        ..Default::default()
    });
    // (case, message, uplinks sent, MAVLink message names sent)
    let cases = [
        ("list: params", request_list(), 1, vec![]),
        (
            "read: params, from the cache",
            MavMessage::PARAM_REQUEST_READ(read_index(0)),
            0,
            vec!["PARAM_VALUE"],
        ),
        (
            "set: params",
            MavMessage::PARAM_SET(set("A", 3.0, MavParamType::MAV_PARAM_TYPE_REAL32)),
            1,
            vec![],
        ),
        (
            "mission list: connect",
            mission_list,
            0,
            vec!["MISSION_COUNT"],
        ),
        ("command: connect", command, 0, vec!["COMMAND_ACK"]),
        (
            "unrouted message: ignored",
            MavMessage::HEARTBEAT(HEARTBEAT_DATA::default()),
            0,
            vec![],
        ),
    ];
    for (case, msg, uplinks, sent) in cases {
        let mut bridge = bridge_with_params(Instant::now());
        let mut out = Outbox::default();
        bridge.on_mavlink(&GCS, &msg, &mut out);
        check!(out.to_fc.len() == uplinks, "{case}");
        check!(
            out.to_gcs
                .iter()
                .map(|m| m.message_name())
                .collect::<Vec<_>>()
                == sent,
            "{case}"
        );
    }
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

    bridge.on_mavlink(
        &GCS,
        &MavMessage::PARAM_REQUEST_READ(read_index(0)),
        &mut out,
    );
    check!(out.to_gcs.is_empty(), "cache cleared");
}

#[test]
fn takes_state_only_while_the_flight_controller_is_alive_with_its_arm_state() {
    let config = BridgeConfig {
        earth: Some(earth()),
        ..BridgeConfig::default()
    };
    let at = |altitude_m| Downlink::State(at_altitude(altitude_m));
    let homes = |out: &Outbox| home_zs(&out.to_gcs);
    let t0 = Instant::now();
    let ms = Duration::from_millis;
    let mut bridge = Bridge::new(config);
    let mut out = Outbox::default();

    bridge.on_downlink(&at(5.0), t0, &mut out);
    bridge.poll(t0, &mut out);
    check!(
        out.to_gcs.is_empty(),
        "no flight controller yet: state ignored"
    );

    bridge.on_downlink(&heartbeat(false), t0, &mut out);
    bridge.on_downlink(&at(10.0), t0, &mut out);
    out = Outbox::default();
    bridge.poll(t0, &mut out);
    check!(homes(&out) == [-10.0], "first state is home");

    // The arm state reaches the state service from the heartbeat: arming re-latches home.
    bridge.on_downlink(&heartbeat(true), t0 + ms(10), &mut out);
    bridge.on_downlink(&at(12.0), t0 + ms(10), &mut out);
    out = Outbox::default();
    bridge.poll(t0 + ms(10), &mut out);
    check!(homes(&out) == [-12.0], "armed: home re-latched");

    let gone = t0 + ms(10) + config.fc_timeout;
    out = Outbox::default();
    bridge.on_downlink(&at(20.0), gone, &mut out);
    bridge.poll(gone, &mut out);
    check!(out.to_gcs.is_empty(), "timed out: nothing sent");
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
    let mut fc_link = FcLink::<16, 8>::new();
    let (mut poller, telemetry) = fc_link.wire(rx, tx);
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
        poller.poll(&mut flight_loop).expect("link up");
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
    bridge.on_mavlink(
        &GCS,
        &MavMessage::PARAM_REQUEST_READ(read_index(3)),
        &mut out,
    );
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
            &MavMessage::PARAM_SET(set(name, value, MavParamType::MAV_PARAM_TYPE_REAL32)),
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
