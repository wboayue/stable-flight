use assert2::check;
use heapless::spsc::Queue;
use stable_flight::{
    ActuatorCommand, AttitudeSetpoint, Command, ControlOutput, Saturation, Setpoint, VehicleState,
};

use super::*;
use crate::loopback::{FrameTooLarge, Loopback, LoopbackLink};
use crate::{ParamName, RecvMessage};

const MS: fn(u64) -> Duration = Duration::from_millis;

/// One tick's telemetry, owned, with `dt` and `arm` set.
fn tick(dt: Duration, arm: bool) -> Record {
    let mut command = Command::default();
    command.arm = arm;
    Record {
        dt,
        command,
        setpoint: Setpoint::Attitude(AttitudeSetpoint::default()),
        state: VehicleState::default(),
        control: ControlOutput::default(),
        actuators: ActuatorCommand::default(),
        saturation: Saturation::default(),
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
    records: usize,
}

fn count(msgs: &[Downlink]) -> Counts {
    let mut counts = Counts::default();
    for msg in msgs {
        match msg {
            Downlink::Heartbeat { .. } => counts.heartbeats += 1,
            Downlink::State(_) => counts.states += 1,
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
            total.records += counts.records;
        }
        check!(total == expected, "{case}");
        check!(publisher.send_errors() == 0, "{case}");
    }
}

#[test]
fn heartbeat_reports_the_commanded_arm_state() {
    for arm in [false, true] {
        let loopback = Loopback::<64>::new();
        let (fc, mut companion) = loopback.split();
        let mut publisher = LinkPublisher::new(fc, LinkPublisherConfig::default());

        publisher.publish(&tick(MS(4), arm).telemetry());

        let heartbeat = received(&mut companion)
            .into_iter()
            .find(|msg| matches!(msg, Downlink::Heartbeat { .. }));
        check!(
            heartbeat
                == Some(Downlink::Heartbeat {
                    armed: arm,
                    mode: 0,
                    link_ok: true,
                })
        );
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
    struct Broken;
    impl LinkTx for Broken {
        type Error = FrameTooLarge;
        fn send(&mut self, _frame: &[u8]) -> Result<(), Self::Error> {
            Err(FrameTooLarge)
        }
    }
    let mut publisher = LinkPublisher::new(Broken, LinkPublisherConfig::default());

    publisher.publish(&tick(MS(4), false).telemetry()); // heartbeat + state
    publisher.publish(&tick(MS(4), false).telemetry()); // nothing due

    check!(publisher.send_errors() == 2);
}
