//! Failsafe handover: a companion flies the vehicle over the link (`FcLink::wire_linked`), goes
//! silent, and comes back. `FailsafeCommander` hands over to `HoldCommander`, which holds the
//! vehicle where it was, armed, and back to the link once it's heard again.

mod common;

use std::time::Duration;

use assert2::check;
use stable_flight::commanders::{FailsafeCommander, FailsafeConfig, HoldCommander};
use stable_flight::{Command, ControlInput, Vec3, WaypointCommand};
use stable_flight_link::fc::{FcLink, FcLinkConfig};
use stable_flight_link::loopback::{Loopback, LoopbackLink};
use stable_flight_link::publisher::LinkPublisherConfig;
use stable_flight_link::{Control, Downlink, MAX_FRAME_LEN, RecvMessage, SendMessage, Uplink};
use stable_flight_sim::airframes;

use common::{Plant, TICK};

/// Loopback queue depth, in frames.
const DEPTH: usize = 64;

/// The companion's heartbeat period while it talks, and the vehicle's back.
const HEARTBEAT: Duration = Duration::from_millis(100);

/// Where the link flies the vehicle (over the origin), in m.
const ALTITUDE_M: f32 = 5.0;

/// What the companion does during a phase.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Companion {
    Talks,
    Silent,
}

/// What the vehicle reported over one phase.
#[derive(Debug, Default)]
struct Phase {
    /// The last heartbeat's `link_ok`: the link, not the fallback, flies.
    link_flies: bool,
    /// Seconds into the phase of the first heartbeat whose `link_ok` differs from the phase's
    /// first one.
    switched_s: Option<f32>,
    /// Armed on every heartbeat.
    armed: bool,
    /// Largest distance from the position at the phase's start, in m.
    moved_m: f32,
}

/// Each heartbeat the vehicle has sent since the last call: `(armed, link_ok)`.
fn heartbeats(
    companion: &mut LoopbackLink<'_, DEPTH>,
    buf: &mut [u8; MAX_FRAME_LEN],
) -> Vec<(bool, bool)> {
    std::iter::from_fn(|| {
        companion
            .recv_message::<Downlink>(buf)
            .expect("valid frame")
    })
    .filter_map(|message| match message {
        Downlink::Heartbeat { armed, link_ok, .. } => Some((armed, link_ok)),
        _ => None,
    })
    .collect()
}

#[test]
fn holds_position_through_a_link_outage_and_hands_back() {
    let loopback = Loopback::<DEPTH>::new();
    let (mut companion, fc_end) = loopback.split();
    let (rx, tx) = fc_end.into_halves();
    let mut fc_link = FcLink::<16, 8>::new();
    let config = FcLinkConfig {
        publisher: LinkPublisherConfig {
            heartbeat_interval: HEARTBEAT,
            ..LinkPublisherConfig::default()
        },
        ..FcLinkConfig::default()
    };
    let (mut poller, telemetry, link_commander) = fc_link.wire_linked(rx, tx, config);
    let failsafe = FailsafeConfig::default();
    let commander = FailsafeCommander::new(link_commander, HoldCommander::default(), failsafe)
        .expect("valid config");
    let mut flight_loop =
        common::pipeline(&Plant::clean(airframes::quad_x()), commander, telemetry);

    let mut buf = [0u8; MAX_FRAME_LEN];
    let mut recv = [0u8; MAX_FRAME_LEN];
    let hover = Command::new(
        true,
        ControlInput::Waypoint(WaypointCommand {
            altitude_m: ALTITUDE_M,
            ..Default::default()
        }),
    );
    let mut send = |companion: &mut LoopbackLink<'_, DEPTH>, control: Control| {
        companion
            .send_message(&Uplink::Control(control), &mut buf)
            .expect("loopback has room");
    };

    // Connect (the failsafe trusts the link after its recovery time; an arm before that is
    // dropped), climb and settle on the link, lose it, then get it back.
    let phases = [
        (Companion::Talks, 2.0),
        (Companion::Talks, 15.0),
        (Companion::Silent, 5.0),
        (Companion::Talks, 5.0),
    ];
    let heartbeat_ticks = common::ticks(HEARTBEAT.as_secs_f32());
    let mut tick = 0;
    let mut reports = Vec::new();
    for (index, (talking, seconds)) in phases.into_iter().enumerate() {
        if index == 1 {
            send(&mut companion, Control::Arm(true));
            send(&mut companion, Control::Command(hover));
        }
        let start = flight_loop.state().position_m;
        let mut report = Phase {
            armed: true,
            ..Phase::default()
        };
        let mut first_link_ok = None;
        for phase_tick in 1..=common::ticks(seconds) {
            if talking == Companion::Talks && tick % heartbeat_ticks == 0 {
                send(&mut companion, Control::Heartbeat);
            }
            poller.poll(&mut flight_loop).expect("loopback receives");
            flight_loop.step(TICK);
            tick += 1;

            let moved: Vec3 = flight_loop.state().position_m - start;
            report.moved_m = report.moved_m.max(moved.norm());
            for (armed, link_ok) in heartbeats(&mut companion, &mut recv) {
                report.armed &= armed;
                report.link_flies = link_ok;
                let first = *first_link_ok.get_or_insert(link_ok);
                if link_ok != first && report.switched_s.is_none() {
                    report.switched_s = Some(common::seconds(phase_tick));
                }
            }
        }
        reports.push(report);
    }

    let [connect, climb, outage, back] = &reports[..] else {
        unreachable!("four phases");
    };
    check!(connect.link_flies, "{connect:?}");
    // The link flies the climb and holds there.
    check!(climb.link_flies, "{climb:?}");
    check!(climb.armed, "{climb:?}");
    // Silent: the hold takes over within the timeout plus a heartbeat, armed, in place.
    check!(!outage.link_flies, "{outage:?}");
    let handover_s = failsafe.timeout_s + HEARTBEAT.as_secs_f32();
    check!(
        outage.switched_s.is_some_and(|s| s <= handover_s),
        "{outage:?}"
    );
    check!(outage.armed, "{outage:?}");
    check!(outage.moved_m < 0.1, "{outage:?}");
    // Heard again: the link flies after the recovery time, still in place.
    check!(back.link_flies, "{back:?}");
    let recovered_s = failsafe.recover_after_s + HEARTBEAT.as_secs_f32();
    check!(
        back.switched_s.is_some_and(|s| s <= recovered_s),
        "{back:?}"
    );
    check!(back.armed, "{back:?}");
    check!(back.moved_m < 0.1, "{back:?}");
    let altitude_m = flight_loop.state().position_m.z;
    check!(
        (altitude_m - ALTITUDE_M).abs() < 0.1,
        "altitude {altitude_m}"
    );
}
