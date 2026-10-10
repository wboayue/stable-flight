use assert2::check;

use super::*;
use stable_flight_link::stable_flight::Command;

use crate::test_support::GCS;

const TIMEOUT: Duration = Duration::from_secs(1);

fn arm_disarm(param1: f32, param2: f32) -> COMMAND_LONG_DATA {
    COMMAND_LONG_DATA {
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        param1,
        param2,
        target_system: 1,
        target_component: 1,
        ..Default::default()
    }
}

fn nav_land() -> COMMAND_LONG_DATA {
    COMMAND_LONG_DATA {
        command: MavCmd::MAV_CMD_NAV_LAND,
        target_system: 1,
        target_component: 1,
        ..Default::default()
    }
}

/// A live flight controller, `armed` in attitude mode at (3, 4).
fn fc(armed: bool) -> Option<FcView> {
    Some(FcView {
        armed,
        mode: ControlMode::Attitude,
        position_m: Some(Vec2::new(3.0, 4.0)),
    })
}

/// The results of the arm/disarm acks in `out`, in order.
fn acks(out: &Outbox) -> Vec<MavResult> {
    acks_of(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, out)
}

/// The results of the acks of `command` in `out`, in order.
fn acks_of(command: MavCmd, out: &Outbox) -> Vec<MavResult> {
    out.to_gcs
        .iter()
        .map(|m| match m {
            MavMessage::COMMAND_ACK(ack) => {
                check!(ack.command == command);
                check!(
                    (ack.target_system, ack.target_component) == (GCS.system_id, GCS.component_id)
                );
                ack.result
            }
            other => panic!("expected only acks, got {other:?}"),
        })
        .collect()
}

#[test]
fn answers_each_request_by_the_flight_controllers_state() {
    use MavResult::*;
    // (case, param1, param2, FC (None: not alive), expected uplink, immediate ack)
    let cases = [
        ("arm", 1.0, 0.0, fc(false), Some(true), None),
        ("disarm", 0.0, 0.0, fc(true), Some(false), None),
        ("force arm", 1.0, 2989.0, fc(false), Some(true), None),
        ("emergency stop", 0.0, 21196.0, fc(true), Some(false), None),
        (
            "already armed",
            1.0,
            0.0,
            fc(true),
            None,
            Some(MAV_RESULT_ACCEPTED),
        ),
        (
            "already disarmed",
            0.0,
            0.0,
            fc(false),
            None,
            Some(MAV_RESULT_ACCEPTED),
        ),
        (
            "no flight controller",
            1.0,
            0.0,
            None,
            None,
            Some(MAV_RESULT_TEMPORARILY_REJECTED),
        ),
    ];
    for (case, param1, param2, fc, uplink, ack) in cases {
        let mut service = CommandService::new(TIMEOUT);
        let mut out = Outbox::default();
        let handled = service.on_command_long(
            &GCS,
            &arm_disarm(param1, param2),
            fc,
            Instant::now(),
            &mut out,
        );

        check!(handled, "{case}");
        let expected_uplinks: Vec<_> = uplink
            .map(|arm| Uplink::Control(Control::Arm(arm)))
            .into_iter()
            .collect();
        check!(out.to_fc == expected_uplinks, "{case}");
        check!(acks(&out) == Vec::from_iter(ack), "{case}");
    }
}

/// (case, heartbeats (offset ms, armed), then poll at ms, expected acks in order)
type Case = (
    &'static str,
    &'static [(u64, bool)],
    u64,
    &'static [MavResult],
);

#[test]
fn acks_once_the_heartbeat_confirms_or_fails_at_the_timeout() {
    use MavResult::*;
    let cases: [Case; 4] = [
        ("confirmed", &[(200, true)], 300, &[MAV_RESULT_ACCEPTED]),
        (
            "confirmed after a stale heartbeat",
            &[(100, false), (400, true)],
            500,
            &[MAV_RESULT_ACCEPTED],
        ),
        ("never confirmed", &[(200, false), (800, false)], 999, &[]),
        (
            "times out",
            &[(200, false), (800, false)],
            1000,
            &[MAV_RESULT_FAILED],
        ),
    ];
    for (case, heartbeats, poll_ms, expected) in cases {
        let t0 = Instant::now();
        let ms = Duration::from_millis;
        let mut service = CommandService::new(TIMEOUT);
        service.on_command_long(
            &GCS,
            &arm_disarm(1.0, 0.0),
            fc(false),
            t0,
            &mut Outbox::default(),
        );

        let mut out = Outbox::default();
        for &(at_ms, armed) in heartbeats {
            service.poll(t0 + ms(at_ms), &mut out);
            service.on_fc_heartbeat(armed, ControlMode::Attitude, &mut out);
        }
        service.poll(t0 + ms(poll_ms), &mut out);
        check!(acks(&out) == expected, "{case}");

        // Settled or not, a later poll and heartbeat ack nothing more.
        let mut later = Outbox::default();
        service.poll(t0 + ms(5000), &mut later);
        service.on_fc_heartbeat(true, ControlMode::Attitude, &mut later);
        let settled = !expected.is_empty();
        check!(later.to_gcs.is_empty() == settled, "{case}: acked once");
    }
}

#[test]
fn rejects_a_second_request_while_one_is_pending() {
    let t0 = Instant::now();
    let mut service = CommandService::new(TIMEOUT);
    let mut out = Outbox::default();
    service.on_command_long(&GCS, &arm_disarm(1.0, 0.0), fc(false), t0, &mut out);
    service.on_command_long(&GCS, &arm_disarm(0.0, 0.0), fc(false), t0, &mut out);

    check!(
        out.to_fc == [Uplink::Control(Control::Arm(true))],
        "only the first goes out"
    );
    check!(acks(&out) == [MavResult::MAV_RESULT_TEMPORARILY_REJECTED]);
}

#[test]
fn leaves_other_commands_to_the_caller() {
    let mut service = CommandService::new(TIMEOUT);
    let mut out = Outbox::default();
    let data = COMMAND_LONG_DATA {
        command: MavCmd::MAV_CMD_NAV_TAKEOFF,
        ..Default::default()
    };
    check!(!service.on_command_long(&GCS, &data, fc(false), Instant::now(), &mut out));
    check!(out.to_gcs.is_empty() && out.to_fc.is_empty());
}

#[test]
fn lands_at_the_latest_position_only_while_armed_in_the_air() {
    use MavResult::*;
    let land_at = |x: f32, y: f32| {
        Uplink::Control(Control::Command(Command::new(
            false,
            ControlInput::Land(LandCommand {
                position_m: Vec2::new(x, y),
            }),
        )))
    };
    let view = |armed: bool, mode: ControlMode, position_m: Option<Vec2>| {
        Some(FcView {
            armed,
            mode,
            position_m,
        })
    };
    let here = Some(Vec2::new(3.0, 4.0));
    // (case, FC (None: not alive), expected uplink, immediate ack)
    let cases = [
        (
            "armed",
            view(true, ControlMode::Waypoint, here),
            Some(land_at(3.0, 4.0)),
            None,
        ),
        (
            "disarmed",
            view(false, ControlMode::Attitude, here),
            None,
            Some(MAV_RESULT_DENIED),
        ),
        (
            "already landing",
            view(true, ControlMode::Land, here),
            None,
            Some(MAV_RESULT_ACCEPTED),
        ),
        (
            "no position yet",
            view(true, ControlMode::Attitude, None),
            None,
            Some(MAV_RESULT_TEMPORARILY_REJECTED),
        ),
        (
            "no flight controller",
            None,
            None,
            Some(MAV_RESULT_TEMPORARILY_REJECTED),
        ),
    ];
    for (case, fc, uplink, ack) in cases {
        let mut service = CommandService::new(TIMEOUT);
        let mut out = Outbox::default();
        check!(
            service.on_command_long(&GCS, &nav_land(), fc, Instant::now(), &mut out),
            "{case}"
        );
        check!(out.to_fc == Vec::from_iter(uplink), "{case}");
        check!(
            acks_of(MavCmd::MAV_CMD_NAV_LAND, &out) == Vec::from_iter(ack),
            "{case}"
        );
    }
}

#[test]
fn acks_a_land_once_the_heartbeat_shows_the_land_mode() {
    let t0 = Instant::now();
    let ms = Duration::from_millis;
    let mut service = CommandService::new(TIMEOUT);
    let mut out = Outbox::default();
    service.on_command_long(&GCS, &nav_land(), fc(true), t0, &mut out);
    check!(out.to_gcs.is_empty(), "pending");

    service.on_fc_heartbeat(true, ControlMode::Attitude, &mut out);
    service.poll(t0 + ms(200), &mut out);
    check!(out.to_gcs.is_empty(), "not landing yet");

    // An arm while the land is pending waits its turn.
    service.on_command_long(&GCS, &arm_disarm(0.0, 0.0), fc(true), t0, &mut out);
    check!(acks(&out) == [MavResult::MAV_RESULT_TEMPORARILY_REJECTED]);

    let mut out = Outbox::default();
    service.on_fc_heartbeat(true, ControlMode::Land, &mut out);
    check!(acks_of(MavCmd::MAV_CMD_NAV_LAND, &out) == [MavResult::MAV_RESULT_ACCEPTED]);

    // A land that never shows fails at the timeout.
    let mut service = CommandService::new(TIMEOUT);
    let mut out = Outbox::default();
    service.on_command_long(&GCS, &nav_land(), fc(true), t0, &mut out);
    service.on_fc_heartbeat(true, ControlMode::Attitude, &mut out);
    service.poll(t0 + TIMEOUT, &mut out);
    check!(acks_of(MavCmd::MAV_CMD_NAV_LAND, &out) == [MavResult::MAV_RESULT_FAILED]);
}
