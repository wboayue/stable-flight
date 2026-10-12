//! Flying from the link end to end: a companion on a loopback sends control messages, and the
//! flight controller's stack (`FcLink::wire_linked`) feeds a `FailsafeCommander<LinkCommander,
//! HoldCommander>` in a `FlightLoop`, polled between ticks as `RealTimeRunner::run_with` does.
//! The companion reads the outcome from the heartbeats the flight controller sends back.

use std::time::Duration;

use assert2::check;
use stable_flight::commanders::{FailsafeCommander, FailsafeConfig, HoldCommander};
use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use stable_flight::mixers::TableMixer;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight::testing::NullVehicle;
use stable_flight::{Command, ControlInput, ControlMode, FlightLoop, WaypointCommand};
use stable_flight_link::fc::{FcLink, FcLinkConfig};
use stable_flight_link::loopback::Loopback;
use stable_flight_link::publisher::LinkPublisherConfig;
use stable_flight_link::{Control, Downlink, MAX_FRAME_LEN, RecvMessage, SendMessage, Uplink};

const DT: Duration = Duration::from_millis(100);

/// What the companion sends before a tick.
#[derive(Clone, Copy)]
enum Sends {
    Nothing,
    Heartbeat,
    ArmAndHover,
    Disarm,
}

#[test]
fn falls_back_on_silence_and_flies_the_link_once_it_is_back() {
    let hover = Control::Command(Command::new(
        false,
        ControlInput::Waypoint(WaypointCommand {
            altitude_m: 5.0,
            ..Default::default()
        }),
    ));
    // (send, expected heartbeat (armed, mode, link_ok)); a 0.35 s timeout and 0.25 s
    // recovery at 0.1 s ticks, clear of rounding at the boundaries.
    use ControlMode::Waypoint;
    use Sends::*;
    let ticks = [
        (Nothing, (false, Waypoint, false)), // never heard: the hold flies, disarmed
        (ArmAndHover, (false, Waypoint, false)), // fresh 0.1 s: the hold can't arm
        (Heartbeat, (false, Waypoint, false)), // 0.2 s
        (Heartbeat, (false, Waypoint, true)), // 0.3 s: the link flies; its arm, sent during
        // the hold, is dropped at the handover (#235)
        (ArmAndHover, (true, Waypoint, true)), // a new arm arms, hovering
        (Nothing, (true, Waypoint, true)),     // silent 0.1 s
        (Nothing, (true, Waypoint, true)),     // 0.2 s
        (Nothing, (true, Waypoint, true)),     // 0.3 s
        (Nothing, (true, Waypoint, false)),    // 0.4 s: stale, holds, still armed
        (Heartbeat, (true, Waypoint, false)),  // back, recovering
        (Disarm, (false, Waypoint, false)),    // a fresh disarm, honored during recovery
        (Heartbeat, (false, Waypoint, true)),  // recovered, disarmed
    ];

    let loopback = Loopback::<64>::new();
    let (mut companion, fc_end) = loopback.split();
    let (rx, tx) = fc_end.into_halves();
    let mut fc_link = FcLink::<16, 8>::new();
    let config = FcLinkConfig {
        publisher: LinkPublisherConfig {
            heartbeat_interval: DT, // one per tick
            ..LinkPublisherConfig::default()
        },
        ..FcLinkConfig::default()
    };
    let (mut poller, telemetry, link_commander) = fc_link.wire_linked(rx, tx, config);
    let commander = FailsafeCommander::new(
        link_commander,
        HoldCommander::default(),
        FailsafeConfig {
            timeout_s: 0.35,
            recover_after_s: 0.25,
            ..FailsafeConfig::default()
        },
    )
    .expect("valid config");
    let mut flight_loop = FlightLoop::new(
        commander,
        NullVehicle::new(),
        CopterGuidance::new(CopterGuidanceConfig::default()),
        CopterStabilizer::new(CopterStabilizerConfig::default()),
        TableMixer::quad_x(),
        telemetry,
    );
    flight_loop.prime();

    let mut buf = [0u8; MAX_FRAME_LEN];
    for (i, (send, (armed, mode, link_ok))) in ticks.into_iter().enumerate() {
        let controls: &[Control] = match send {
            Nothing => &[],
            Heartbeat => &[Control::Heartbeat],
            ArmAndHover => &[Control::Arm(true), hover.clone()],
            Disarm => &[Control::Arm(false)],
        };
        for control in controls {
            companion
                .send_message(&Uplink::Control(control.clone()), &mut buf)
                .expect("loopback has room");
        }
        poller.poll(&mut flight_loop).expect("loopback receives");
        flight_loop.step(DT);

        let heartbeats: Vec<_> =
            std::iter::from_fn(|| companion.recv_message::<Downlink>(&mut buf).expect("valid"))
                .filter_map(|msg| match msg {
                    Downlink::Heartbeat {
                        armed,
                        mode,
                        link_ok,
                    } => Some((armed, mode, link_ok)),
                    _ => None,
                })
                .collect();
        check!(
            heartbeats.last() == Some(&(armed, mode, link_ok)),
            "tick {i}"
        );
    }
}
