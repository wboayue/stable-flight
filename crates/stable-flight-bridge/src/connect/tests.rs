use std::time::{Duration, Instant};

use assert2::check;

use super::*;
use crate::test_support::GCS;

const INTERVAL: Duration = Duration::from_secs(1);

fn service() -> ConnectService {
    ConnectService::new(MavType::MAV_TYPE_QUADROTOR, INTERVAL)
}

fn request_message(id: u32) -> COMMAND_LONG_DATA {
    COMMAND_LONG_DATA {
        command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
        param1: id as f32,
        target_system: 1,
        target_component: 1,
        ..Default::default()
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
        let mut out = Outbox::default();
        let data = MISSION_REQUEST_LIST_DATA {
            target_system: 1,
            target_component: 1,
            mission_type,
        };
        service().on_mission_request_list(&GCS, &data, &mut out);

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
    let arm = COMMAND_LONG_DATA {
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        param1: 1.0,
        target_system: 1,
        target_component: 1,
        ..Default::default()
    };
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
    for (case, data, result, version) in cases {
        let mut out = Outbox::default();
        service().on_command_long(&GCS, &data, &mut out);

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
fn heartbeat_reports_the_arm_state_mode_and_vehicle_type() {
    let tailsitter = MavType::MAV_TYPE_VTOL_TAILSITTER_QUADROTOR;
    // (arm state, mode): the mapping itself is `stable_flight_mavlink::heartbeat`'s; the service
    // passes the values and its vehicle type through.
    let cases = [
        (false, ControlMode::Rate),
        (true, ControlMode::Attitude),
        (true, ControlMode::Waypoint),
    ];
    for (armed, mode) in cases {
        let now = Instant::now();
        let mut connect = ConnectService::new(tailsitter, INTERVAL);
        let mut out = Outbox::default();
        connect.fc_appeared(now, &mut Outbox::default());
        connect.poll(now, armed, mode, &mut out);

        let expected = stable_flight_mavlink::heartbeat(tailsitter, armed, mode);
        check!(
            out.to_gcs == [MavMessage::HEARTBEAT(expected)],
            "{mode:?} armed={armed}"
        );
    }
}

#[test]
fn heartbeat_goes_out_when_the_flight_controller_appears_then_on_the_interval() {
    let t0 = Instant::now();
    let mut connect = service();
    let mut out = Outbox::default();
    let mut heartbeats_at = |connect: &mut ConnectService, at: Instant| {
        connect.poll(at, false, ControlMode::Attitude, &mut out);
        let n = out.to_gcs.len();
        out.to_gcs.clear();
        n
    };

    check!(heartbeats_at(&mut connect, t0) == 0, "before it appears");
    connect.fc_appeared(t0, &mut Outbox::default());
    check!(heartbeats_at(&mut connect, t0) == 1, "right away");
    check!(
        heartbeats_at(&mut connect, t0 + INTERVAL / 2) == 0,
        "not due yet"
    );
    check!(heartbeats_at(&mut connect, t0 + INTERVAL) == 1, "due");
}
