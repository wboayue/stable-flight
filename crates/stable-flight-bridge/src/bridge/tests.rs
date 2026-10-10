use std::time::{Duration, Instant};

use assert2::{assert, check};
use mavlink::Message;
use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use stable_flight::mixers::QuadCopterMixer;
use stable_flight::params::Params;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight::testing::{ConstantCommander, NullVehicle};
use stable_flight::{
    Command, ControlInput, FlightLoop, LandCommand, Saturation, Vec2, VehicleState,
};
use stable_flight_link::fc::FcLink;
use stable_flight_link::loopback::{Loopback, LoopbackLink};
use stable_flight_link::publisher::LinkPublisher;
use stable_flight_link::{MAX_FRAME_LEN, Output, ParamRequest, RecvMessage, SendMessage, Uplink};

use super::*;
use crate::test_support::{
    GCS, at_altitude, earth, heartbeat, home_zs, landed_states, param, param_value_of, read_index,
    request_list, set,
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
        bridge.on_mavlink(&GCS, &msg, Instant::now(), &mut out);
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

/// A level `SET_ATTITUDE_TARGET` for `target_system`.
fn attitude_target(target_system: u8) -> MavMessage {
    MavMessage::SET_ATTITUDE_TARGET(SET_ATTITUDE_TARGET_DATA {
        target_system,
        target_component: 1,
        q: [1.0, 0.0, 0.0, 0.0],
        thrust: 0.5,
        ..Default::default()
    })
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
        ("attitude target: setpoints", attitude_target(1), 1, vec![]),
        (
            "attitude target for another system: ignored",
            attitude_target(2),
            0,
            vec![],
        ),
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
        bridge.on_mavlink(&GCS, &msg, Instant::now(), &mut out);
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
        Instant::now(),
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
fn takes_control_output_only_while_the_flight_controller_is_alive() {
    let output = Downlink::Output(Output {
        thrust: 0.4,
        saturation: Saturation::default(),
    });
    let throttle = |out: &Outbox| {
        out.to_gcs.iter().find_map(|m| match m {
            MavMessage::VFR_HUD(d) => Some(d.throttle),
            _ => None,
        })
    };
    let t0 = Instant::now();
    let mut bridge = Bridge::new(BridgeConfig::default());
    let mut out = Outbox::default();

    bridge.on_downlink(&output, t0, &mut out);
    bridge.on_downlink(&heartbeat(true), t0, &mut out);
    bridge.on_downlink(&Downlink::State(at_altitude(10.0)), t0, &mut out);
    bridge.poll(t0, &mut out);
    check!(
        throttle(&out) == Some(0),
        "no flight controller yet: output ignored"
    );

    bridge.on_downlink(&output, t0, &mut out);
    out = Outbox::default();
    bridge.poll(t0 + Duration::from_secs(1), &mut out);
    check!(throttle(&out) == Some(40));
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
        Instant::now(),
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
            Instant::now(),
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

/// The control messages in `out` to the flight controller.
fn controls(out: &Outbox) -> Vec<Control> {
    out.to_fc
        .iter()
        .filter_map(|u| match u {
            Uplink::Control(c) => Some(c.clone()),
            _ => None,
        })
        .collect()
}

#[test]
fn heartbeats_the_link_only_while_both_ends_are_alive() {
    let config = BridgeConfig::default();
    let t0 = Instant::now();
    let ms = Duration::from_millis;
    let gcs_heartbeat = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
    // (case, at ms, FC heartbeat, GCS message, expected link heartbeats from a poll)
    let ticks = [
        ("nobody yet", 0, false, false, 0),
        ("flight controller only", 10, true, false, 0),
        ("ground station appears: at once", 20, false, true, 1),
        ("within the interval", 100, true, false, 0),
        ("due", 220, true, false, 1),
        ("ground station silent, still alive", 2_900, true, false, 1),
        ("ground station gone", 3_100, true, false, 0),
        ("back: at once", 3_200, true, true, 1),
        ("flight controller gone", 6_300, false, true, 0),
    ];
    let mut bridge = Bridge::new(config);
    for (case, at_ms, fc, gcs, expected) in ticks {
        let now = t0 + ms(at_ms);
        let mut out = Outbox::default();
        if fc {
            bridge.on_downlink(&heartbeat(false), now, &mut out);
        }
        if gcs {
            bridge.on_mavlink(&GCS, &gcs_heartbeat, now, &mut out);
        }
        let mut polled = Outbox::default();
        bridge.poll(now, &mut polled);
        let sent = controls(&polled)
            .into_iter()
            .filter(|c| *c == Control::Heartbeat)
            .count();
        check!(sent == expected, "{case}");
    }
    check!(config.gcs_timeout == Duration::from_secs(3));
}

#[test]
fn arms_through_the_flight_controller_and_acks_unhandled_commands_unsupported() {
    let t0 = Instant::now();
    let ms = Duration::from_millis;
    let mut bridge = Bridge::new(BridgeConfig::default());
    bridge.on_downlink(&heartbeat(false), t0, &mut Outbox::default());
    let command = |command, param1| {
        MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            command,
            param1,
            target_system: 1,
            target_component: 1,
            ..Default::default()
        })
    };
    let ack_results = |out: &Outbox| -> Vec<(MavCmd, MavResult)> {
        out.to_gcs
            .iter()
            .filter_map(|m| match m {
                MavMessage::COMMAND_ACK(a) => Some((a.command, a.result)),
                _ => None,
            })
            .collect()
    };

    let mut out = Outbox::default();
    bridge.on_mavlink(
        &GCS,
        &command(MavCmd::MAV_CMD_NAV_TAKEOFF, 0.0),
        t0,
        &mut out,
    );
    check!(
        ack_results(&out)
            == [(
                MavCmd::MAV_CMD_NAV_TAKEOFF,
                MavResult::MAV_RESULT_UNSUPPORTED
            )],
        "no service handles it"
    );

    let mut out = Outbox::default();
    bridge.on_mavlink(
        &GCS,
        &command(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0),
        t0,
        &mut out,
    );
    check!(controls(&out) == [Control::Arm(true)]);
    check!(
        ack_results(&out).is_empty(),
        "waits for the flight controller"
    );

    let mut out = Outbox::default();
    bridge.on_downlink(&heartbeat(true), t0 + ms(200), &mut out);
    check!(
        ack_results(&out)
            == [(
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                MavResult::MAV_RESULT_ACCEPTED
            )],
        "confirmed by its heartbeat"
    );
}

#[test]
fn lands_at_the_latest_position_and_reports_landing_while_the_heartbeat_shows_it() {
    let t0 = Instant::now();
    let ms = Duration::from_millis;
    let mut bridge = Bridge::new(BridgeConfig::default());
    let land_heartbeat = Downlink::Heartbeat {
        armed: true,
        mode: ControlMode::Land,
        link_ok: true,
    };
    let nav_land = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        command: MavCmd::MAV_CMD_NAV_LAND,
        target_system: 1,
        target_component: 1,
        ..Default::default()
    });
    let landed = |out: &Outbox| landed_states(&out.to_gcs);
    let mut out = Outbox::default();
    bridge.on_downlink(&heartbeat(true), t0, &mut out);
    bridge.on_downlink(
        &Downlink::State(VehicleState {
            position_m: [3.0, 4.0, 10.0].into(),
            ..Default::default()
        }),
        t0,
        &mut out,
    );
    bridge.on_downlink(
        &Downlink::State(VehicleState {
            position_m: [3.0, 4.0, 12.0].into(),
            ..Default::default()
        }),
        t0,
        &mut out,
    );
    out = Outbox::default();
    bridge.poll(t0, &mut out);
    check!(landed(&out) == [MavLandedState::MAV_LANDED_STATE_IN_AIR]);

    out = Outbox::default();
    bridge.on_mavlink(&GCS, &nav_land, t0, &mut out);
    check!(
        controls(&out)
            == [Control::Command(Command::new(
                false,
                ControlInput::Land(LandCommand {
                    position_m: Vec2::new(3.0, 4.0),
                }),
            ))],
        "a land at the latest position"
    );

    out = Outbox::default();
    bridge.on_downlink(&land_heartbeat, t0 + ms(200), &mut out);
    bridge.on_downlink(&Downlink::State(at_altitude(11.0)), t0 + ms(200), &mut out);
    bridge.poll(t0 + ms(200), &mut out);
    check!(out.to_gcs.iter().any(|m| matches!(
        m,
        MavMessage::COMMAND_ACK(a)
            if a.command == MavCmd::MAV_CMD_NAV_LAND && a.result == MavResult::MAV_RESULT_ACCEPTED
    )));
    check!(landed(&out) == [MavLandedState::MAV_LANDED_STATE_LANDING]);
}

#[test]
fn forwards_the_flight_controllers_status_text_to_the_ground_station() {
    let mut bridge = Bridge::new(BridgeConfig::default());
    let mut out = Outbox::default();
    let text = "Link back: move the sticks to take control";
    bridge.on_downlink(
        &Downlink::Status(text.try_into().expect("short")),
        Instant::now(),
        &mut out,
    );
    let [MavMessage::STATUSTEXT(data)] = out.to_gcs.as_slice() else {
        panic!("expected one STATUSTEXT, got {:?}", out.to_gcs);
    };
    check!(data.text.to_str().ok() == Some(text));
}
