use assert2::check;
use heapless::spsc::Queue;
use stable_flight::{
    ActuatorCommand, AttitudeSetpoint, Command, ControlInput, ControlMode, ControlOutput,
    Saturation, Setpoint, VehicleState,
};

use super::*;
use crate::loopback::{FailingLink, Loopback, LoopbackLink};
use crate::{ParamName, RecvMessage};

const MS: fn(u64) -> Duration = Duration::from_millis;

/// One tick's telemetry, owned, with `dt` and `arm` set.
fn tick(dt: Duration, arm: bool) -> Record {
    Record {
        dt,
        command: Command {
            arm,
            ..Default::default()
        },
        setpoint: Setpoint::Attitude(AttitudeSetpoint::default()),
        state: VehicleState::default(),
        control: ControlOutput::default(),
        actuators: ActuatorCommand::default(),
        saturation: Saturation::default(),
        failsafe: None,
    }
}

fn received(companion: &mut LoopbackLink<'_, 64>) -> Vec<Downlink> {
    let mut buf = [0u8; MAX_FRAME_LEN];
    core::iter::from_fn(|| companion.recv_message::<Downlink>(&mut buf).unwrap()).collect()
}

#[derive(Debug, Default, PartialEq)]
struct Counts {
    heartbeats: usize,
    states: usize,
    outputs: usize,
    records: usize,
}

fn count(msgs: &[Downlink]) -> Counts {
    let mut counts = Counts::default();
    for msg in msgs {
        match msg {
            Downlink::Heartbeat { .. } => counts.heartbeats += 1,
            Downlink::State(_) => counts.states += 1,
            Downlink::Output(_) => counts.outputs += 1,
            Downlink::Record(_) => counts.records += 1,
            _ => {}
        }
    }
    counts
}

#[test]
fn streams_go_out_at_their_configured_rates() {
    let config = |record_interval| LinkPublisherConfig {
        heartbeat_interval: MS(200),
        state_interval: MS(20),
        record_interval,
        max_params_per_tick: 2,
    };
    // (case, config, ticks of 4 ms, expected counts: first tick fires every stream)
    let cases = [
        (
            "one second at 250 Hz, no records",
            config(None),
            250,
            Counts {
                heartbeats: 6,
                states: 51,
                outputs: 51,
                records: 0,
            },
        ),
        (
            "one second with 2 Hz records",
            config(Some(MS(500))),
            250,
            Counts {
                heartbeats: 6,
                states: 51,
                outputs: 51,
                records: 3,
            },
        ),
    ];
    for (case, config, ticks, expected) in cases {
        let loopback = Loopback::<64>::new();
        let (fc, mut companion) = loopback.split();
        let mut publisher = LinkPublisher::new(fc, config);

        let mut total = Counts::default();
        for _ in 0..ticks {
            publisher.publish(&tick(MS(4), false).telemetry());
            let counts = count(&received(&mut companion));
            total.heartbeats += counts.heartbeats;
            total.states += counts.states;
            total.outputs += counts.outputs;
            total.records += counts.records;
        }
        check!(total == expected, "{case}");
        check!(publisher.send_errors() == 0, "{case}");
    }
}

#[test]
fn heartbeat_reports_the_commanded_arm_state_and_mode() {
    // (arm, input, expected mode)
    let cases = [
        (
            false,
            ControlInput::Attitude(Default::default()),
            ControlMode::Attitude,
        ),
        (
            true,
            ControlInput::Rate(Default::default()),
            ControlMode::Rate,
        ),
        (
            true,
            ControlInput::Waypoint(Default::default()),
            ControlMode::Waypoint,
        ),
        (
            true,
            ControlInput::AbsoluteAttitude(Default::default()),
            ControlMode::AbsoluteAttitude,
        ),
    ];
    for (arm, input, mode) in cases {
        let loopback = Loopback::<64>::new();
        let (fc, mut companion) = loopback.split();
        let mut publisher = LinkPublisher::new(fc, LinkPublisherConfig::default());
        let mut record = tick(MS(4), arm);
        record.command.input = input;

        publisher.publish(&record.telemetry());

        let heartbeat = received(&mut companion)
            .into_iter()
            .find(|msg| matches!(msg, Downlink::Heartbeat { .. }));
        check!(
            heartbeat
                == Some(Downlink::Heartbeat {
                    armed: arm,
                    mode,
                    link_ok: true,
                }),
            "{input:?}"
        );
    }
}

#[test]
fn heartbeat_link_ok_is_false_only_while_a_failsafe_flies_its_fallback() {
    // (failsafe, expected link_ok)
    let cases = [
        (None, true),
        (Some(FailsafeState::Primary), true),
        (Some(FailsafeState::Fallback), false),
    ];
    for (failsafe, link_ok) in cases {
        let loopback = Loopback::<64>::new();
        let (fc, mut companion) = loopback.split();
        let mut publisher = LinkPublisher::new(fc, LinkPublisherConfig::default());
        let mut record = tick(MS(4), false);
        record.failsafe = failsafe;

        publisher.publish(&record.telemetry());

        let heartbeat = received(&mut companion)
            .into_iter()
            .find_map(|msg| match msg {
                Downlink::Heartbeat { link_ok, .. } => Some(link_ok),
                _ => None,
            });
        check!(heartbeat == Some(link_ok), "{failsafe:?}");
    }
}

#[test]
fn sends_at_most_the_param_limit_per_tick_in_order() {
    let loopback = Loopback::<64>::new();
    let (fc, mut companion) = loopback.split();
    let mut replies = Queue::<ParamValue, 8>::new();
    let (mut server, replies) = replies.split();
    let config = LinkPublisherConfig {
        max_params_per_tick: 2,
        ..Default::default()
    };
    let mut publisher = LinkPublisher::new(fc, config).with_param_replies(replies);

    for index in 0..5 {
        server
            .enqueue(ParamValue {
                name: ParamName::try_from("P").unwrap(),
                value: f32::from(index),
                index,
                count: 5,
            })
            .unwrap();
    }

    let mut per_tick = Vec::new();
    let mut indices = Vec::new();
    for _ in 0..4 {
        publisher.publish(&tick(MS(4), false).telemetry());
        let params: Vec<_> = received(&mut companion)
            .into_iter()
            .filter_map(|msg| match msg {
                Downlink::Param(value) => Some(value.index),
                _ => None,
            })
            .collect();
        per_tick.push(params.len());
        indices.extend(params);
    }
    check!(per_tick == [2, 2, 1, 0]);
    check!(indices == [0, 1, 2, 3, 4]);
}

#[test]
fn failed_sends_are_counted_not_returned() {
    let mut publisher = LinkPublisher::new(FailingLink, LinkPublisherConfig::default());

    publisher.publish(&tick(MS(4), false).telemetry()); // heartbeat + state + output
    publisher.publish(&tick(MS(4), false).telemetry()); // nothing due

    check!(publisher.send_errors() == 3);
}

#[test]
fn announces_each_failsafe_change_once() {
    use FailsafeState::{AwaitingSticks, Fallback, Primary};
    let (sticks, land) = (
        ControlInput::default(),
        ControlInput::Land(Default::default()),
    );
    // (failsafe this tick, input flown, expected status text sent)
    let ticks = [
        (None, sticks, None),
        (Some(Fallback), sticks, Some("Link quiet: holding position")),
        (Some(Fallback), sticks, None), // unchanged: once
        (
            Some(AwaitingSticks),
            sticks,
            Some("Link back: move the sticks to take control"),
        ),
        (Some(Primary), sticks, Some("Flying from the link")),
        (Some(Primary), land, None),
        (Some(Fallback), land, Some("Link quiet: landing")), // the fallback finishes it
        (Some(Primary), land, Some("Flying from the link")),
        (None, sticks, None), // no failsafe: nothing to announce
    ];
    let loopback = Loopback::<64>::new();
    let (fc, mut companion) = loopback.split();
    let mut publisher = LinkPublisher::new(fc, LinkPublisherConfig::default());
    for (i, (failsafe, input, expected)) in ticks.into_iter().enumerate() {
        let mut record = tick(MS(4), false);
        record.failsafe = failsafe;
        record.command.input = input;
        publisher.publish(&record.telemetry());

        let status: Vec<_> = received(&mut companion)
            .into_iter()
            .filter_map(|msg| match msg {
                Downlink::Status(text) => Some(text),
                _ => None,
            })
            .collect();
        check!(
            status.iter().map(|t| t.as_str()).collect::<Vec<_>>() == Vec::from_iter(expected),
            "tick {i}"
        );
    }
}

#[test]
fn output_carries_the_latest_thrust_and_every_saturation_since_the_last_output() {
    let tilt = {
        let mut saturation = Saturation::default();
        saturation.tilt = true;
        saturation
    };
    let yaw = {
        let mut saturation = Saturation::default();
        saturation.moments[2] = true;
        saturation
    };
    let none = Saturation::default();
    // (saturation this tick, thrust this tick, expected output). State every 20 ms with 4 ms
    // ticks: the first tick sends and carries 4 ms over, so ticks 0, 4, and 9 send.
    let ticks = [
        (none, 0.1, Some((0.1, none))),
        (tilt, 0.2, None), // one tick, between sends
        (none, 0.3, None),
        (yaw, 0.4, None),
        (none, 0.5, Some((0.5, tilt.merge(yaw)))),
        (none, 0.6, None),
        (none, 0.6, None),
        (none, 0.6, None),
        (none, 0.6, None),
        (none, 0.7, Some((0.7, none))), // cleared by the previous send
    ];
    let loopback = Loopback::<64>::new();
    let (fc, mut companion) = loopback.split();
    let mut publisher = LinkPublisher::new(fc, LinkPublisherConfig::default());
    for (i, (saturation, thrust, expected)) in ticks.into_iter().enumerate() {
        let mut record = tick(MS(4), true);
        record.saturation = saturation;
        record.control.thrust = thrust;
        publisher.publish(&record.telemetry());

        let output = received(&mut companion)
            .into_iter()
            .find_map(|msg| match msg {
                Downlink::Output(output) => Some((output.thrust, output.saturation)),
                _ => None,
            });
        check!(output == expected, "tick {i}");
    }
}
