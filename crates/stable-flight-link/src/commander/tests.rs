use assert2::check;
use heapless::spsc::Queue;
use stable_flight::{AttitudeCommand, RateCommand, WaypointCommand};

use super::*;

const DT: f32 = 0.1;

fn rate(roll: f32) -> Control {
    Control::Command(Command::new(
        true, // ignored: arming is `Control::Arm`'s
        ControlInput::Rate(RateCommand {
            roll_rate: roll,
            ..Default::default()
        }),
    ))
}

fn waypoint(altitude_m: f32) -> Control {
    Control::Command(Command::new(
        false, // ignored
        ControlInput::Waypoint(WaypointCommand {
            altitude_m,
            ..Default::default()
        }),
    ))
}

#[test]
fn flies_the_latest_command_armed_only_by_arm_messages() {
    let neutral = ControlInput::Attitude(AttitudeCommand::default());
    let rate_input = |roll: f32| {
        ControlInput::Rate(RateCommand {
            roll_rate: roll,
            ..Default::default()
        })
    };
    let waypoint_input = |altitude_m: f32| {
        ControlInput::Waypoint(WaypointCommand {
            altitude_m,
            ..Default::default()
        })
    };
    // (case, messages before the tick, expected arm, input, since_update_s)
    let ticks = [
        ("never heard", vec![], false, neutral, None),
        (
            "heartbeat: fresh, nothing to fly yet",
            vec![Control::Heartbeat],
            false,
            neutral,
            Some(0.0),
        ),
        ("silence grows by dt", vec![], false, neutral, Some(DT)),
        (
            "a command's arm is ignored",
            vec![rate(0.2)],
            false,
            rate_input(0.2),
            Some(0.0),
        ),
        (
            "arm, then the latest command wins",
            vec![Control::Arm(true), rate(0.3), waypoint(5.0)],
            true,
            waypoint_input(5.0),
            Some(0.0),
        ),
        (
            "silence keeps the command and arm",
            vec![],
            true,
            waypoint_input(5.0),
            Some(DT),
        ),
        (
            "and keeps growing",
            vec![],
            true,
            waypoint_input(5.0),
            Some(2.0 * DT),
        ),
        (
            "a heartbeat refreshes, nothing else",
            vec![Control::Heartbeat],
            true,
            waypoint_input(5.0),
            Some(0.0),
        ),
        (
            "arm messages apply in order",
            vec![Control::Arm(false), Control::Arm(true), Control::Arm(false)],
            false,
            waypoint_input(5.0),
            Some(0.0),
        ),
    ];

    let mut queue = Queue::<Control, 8>::new();
    let (mut router, consumer) = queue.split();
    let mut commander = LinkCommander::new(consumer);
    for (i, (case, messages, arm, input, since)) in ticks.into_iter().enumerate() {
        for message in messages {
            router.enqueue(message).expect("room");
        }
        let command = commander.next_command(DT, &VehicleState::default());

        check!(command.arm == arm, "tick {i}: {case}");
        check!(command.input == input, "tick {i}: {case}");
        match (commander.since_update_s(), since) {
            (Some(got), Some(want)) => {
                check!((got - want).abs() < 1e-6, "tick {i}: {case}: {got}");
            }
            (got, want) => {
                check!(got == want, "tick {i}: {case}");
            }
        }
    }
}

#[test]
fn sticks_stay_fresh_only_on_commands_waypoints_on_any_message() {
    // (case, messages before the tick, expected since_update_s)
    let ticks = [
        (
            "heartbeat, nothing flown yet: any message counts",
            vec![Control::Heartbeat],
            Some(0.0),
        ),
        ("sticks", vec![rate(0.1)], Some(0.0)),
        (
            "heartbeats don't refresh sticks",
            vec![Control::Heartbeat],
            Some(DT),
        ),
        ("still stale", vec![Control::Heartbeat], Some(2.0 * DT)),
        (
            "an arm doesn't either",
            vec![Control::Arm(true)],
            Some(3.0 * DT),
        ),
        ("a stick command does", vec![rate(0.2)], Some(0.0)),
        ("a waypoint", vec![waypoint(5.0)], Some(0.0)),
        (
            "heartbeats refresh a waypoint",
            vec![Control::Heartbeat],
            Some(0.0),
        ),
        ("silence", vec![], Some(DT)),
        (
            "back to sticks: counts from that command",
            vec![rate(0.0)],
            Some(0.0),
        ),
        (
            "and stale again on heartbeats",
            vec![Control::Heartbeat],
            Some(DT),
        ),
    ];

    let mut queue = Queue::<Control, 8>::new();
    let (mut router, consumer) = queue.split();
    let mut commander = LinkCommander::new(consumer);
    for (i, (case, messages, since)) in ticks.into_iter().enumerate() {
        for message in messages {
            router.enqueue(message).expect("room");
        }
        commander.next_command(DT, &VehicleState::default());
        let (got, want) = (commander.since_update_s(), since);
        match (got, want) {
            (Some(got), Some(want)) => {
                check!((got - want).abs() < 1e-6, "tick {i}: {case}: {got}");
            }
            _ => {
                check!(got == want, "tick {i}: {case}");
            }
        }
    }
}
