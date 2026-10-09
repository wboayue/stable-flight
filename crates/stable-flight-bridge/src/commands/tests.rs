use assert2::check;

use super::*;
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

/// The results of the arm/disarm acks in `out`, in order.
fn acks(out: &Outbox) -> Vec<MavResult> {
    out.to_gcs
        .iter()
        .map(|m| match m {
            MavMessage::COMMAND_ACK(ack) => {
                check!(ack.command == MavCmd::MAV_CMD_COMPONENT_ARM_DISARM);
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
    // (case, param1, param2, FC armed (None: not alive), expected uplink, immediate ack)
    let cases = [
        ("arm", 1.0, 0.0, Some(false), Some(true), None),
        ("disarm", 0.0, 0.0, Some(true), Some(false), None),
        ("force arm", 1.0, 2989.0, Some(false), Some(true), None),
        (
            "emergency stop",
            0.0,
            21196.0,
            Some(true),
            Some(false),
            None,
        ),
        (
            "already armed",
            1.0,
            0.0,
            Some(true),
            None,
            Some(MAV_RESULT_ACCEPTED),
        ),
        (
            "already disarmed",
            0.0,
            0.0,
            Some(false),
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
    for (case, param1, param2, fc_armed, uplink, ack) in cases {
        let mut service = CommandService::new(TIMEOUT);
        let mut out = Outbox::default();
        let handled = service.on_command_long(
            &GCS,
            &arm_disarm(param1, param2),
            fc_armed,
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
            Some(false),
            t0,
            &mut Outbox::default(),
        );

        let mut out = Outbox::default();
        for &(at_ms, armed) in heartbeats {
            service.poll(t0 + ms(at_ms), &mut out);
            service.on_fc_heartbeat(armed, &mut out);
        }
        service.poll(t0 + ms(poll_ms), &mut out);
        check!(acks(&out) == expected, "{case}");

        // Settled or not, a later poll and heartbeat ack nothing more.
        let mut later = Outbox::default();
        service.poll(t0 + ms(5000), &mut later);
        service.on_fc_heartbeat(true, &mut later);
        let settled = !expected.is_empty();
        check!(later.to_gcs.is_empty() == settled, "{case}: acked once");
    }
}

#[test]
fn rejects_a_second_request_while_one_is_pending() {
    let t0 = Instant::now();
    let mut service = CommandService::new(TIMEOUT);
    let mut out = Outbox::default();
    service.on_command_long(&GCS, &arm_disarm(1.0, 0.0), Some(false), t0, &mut out);
    service.on_command_long(&GCS, &arm_disarm(0.0, 0.0), Some(false), t0, &mut out);

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
    check!(!service.on_command_long(&GCS, &data, Some(false), Instant::now(), &mut out));
    check!(out.to_gcs.is_empty() && out.to_fc.is_empty());
}
