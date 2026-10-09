use assert2::{assert, check};
use heapless::spsc::Queue;
use stable_flight::{AttitudeCommand, Command, ControlInput};

use super::*;
use crate::loopback::{FailingLink, LinkDown, Loopback, LoopbackLink};
use crate::{LinkTx, SendMessage, encode_vec};

fn hover() -> Command {
    Command::armed(ControlInput::Attitude(AttitudeCommand {
        throttle: 0.5,
        ..Default::default()
    }))
}

fn send_all(companion: &mut LoopbackLink<'_, 16>, msgs: &[Uplink]) {
    let mut buf = [0u8; MAX_FRAME_LEN];
    for msg in msgs {
        companion.send_message(msg, &mut buf).unwrap();
    }
}

fn list() -> Uplink {
    Uplink::Param(ParamRequest::List)
}

#[test]
fn routes_params_and_control_to_their_queues() {
    let loopback = Loopback::<16>::new();
    let (mut companion, fc) = loopback.split();
    let mut params = Queue::<ParamRequest, 8>::new();
    let mut control = Queue::<Control, 8>::new();
    let (to_server, mut server) = params.split();
    let (to_commander, mut commander) = control.split();
    let mut router = UplinkRouter::new(fc, 16)
        .with_params(to_server)
        .with_control(to_commander);

    let controls = [
        Control::Heartbeat,
        Control::Command(hover()),
        Control::Arm(true),
    ];
    let mut msgs: Vec<_> = controls.iter().cloned().map(Uplink::Control).collect();
    msgs.insert(1, list());
    send_all(&mut companion, &msgs);

    check!(
        router.poll()
            == Ok(RouteStats {
                frames: 4,
                ..Default::default()
            })
    );
    check!(server.dequeue() == Some(ParamRequest::List));
    check!(server.dequeue() == None);
    let routed: Vec<_> = core::iter::from_fn(|| commander.dequeue()).collect();
    check!(routed == controls);
}

#[test]
fn drops_and_counts_what_has_no_room_or_no_consumer() {
    let control = Uplink::Control;
    // (case, messages, expected dropped); params get a queue with room for one, control none
    let cases = [
        (
            "control without a control queue",
            vec![control(Control::Heartbeat), control(Control::Arm(true))],
            2,
        ),
        ("params beyond a full queue", vec![list(); 3], 2),
    ];
    for (case, msgs, dropped) in &cases {
        let loopback = Loopback::<16>::new();
        let (mut companion, fc) = loopback.split();
        let mut params = Queue::<ParamRequest, 2>::new();
        let (to_server, _server) = params.split();
        let mut router = UplinkRouter::new(fc, 16).with_params(to_server);

        send_all(&mut companion, msgs);
        let stats = router.poll().unwrap();
        check!(stats.frames == msgs.len(), "{case}");
        check!(stats.dropped == *dropped, "{case}");
    }
}

#[test]
fn router_without_queues_drops_everything() {
    let loopback = Loopback::<16>::new();
    let (mut companion, fc) = loopback.split();
    let mut router = UplinkRouter::new(fc, 16);

    send_all(
        &mut companion,
        &[list(), Uplink::Control(Control::Heartbeat)],
    );

    check!(
        router.poll()
            == Ok(RouteStats {
                frames: 2,
                decode_errors: 0,
                dropped: 2
            })
    );
}

#[test]
fn counts_corrupt_frames_and_keeps_routing() {
    let loopback = Loopback::<16>::new();
    let (mut companion, fc) = loopback.split();
    let mut params = Queue::<ParamRequest, 4>::new();
    let (to_server, mut server) = params.split();
    let mut router = UplinkRouter::new(fc, 16).with_params(to_server);

    let mut corrupt = encode_vec(&Uplink::Control(Control::Heartbeat));
    corrupt[1] ^= 0x55;
    companion.send(&corrupt).unwrap();
    send_all(&mut companion, &[list()]);

    let stats = router.poll().unwrap();
    check!(
        stats
            == RouteStats {
                frames: 2,
                decode_errors: 1,
                dropped: 0
            }
    );
    check!(server.dequeue() == Some(ParamRequest::List));
}

#[test]
fn reads_at_most_the_frame_limit_per_poll() {
    let loopback = Loopback::<16>::new();
    let (mut companion, fc) = loopback.split();
    let mut params = Queue::<ParamRequest, 8>::new();
    let (to_server, _server) = params.split();
    let mut router = UplinkRouter::new(fc, 2).with_params(to_server);

    send_all(&mut companion, &vec![list(); 5]);

    let frames: Vec<_> = (0..4).map(|_| router.poll().unwrap().frames).collect();
    check!(frames == [2, 2, 1, 0]);
}

#[test]
fn transport_failure_is_returned() {
    let mut router = UplinkRouter::new(FailingLink, 4);
    assert!(router.poll() == Err(LinkDown));
}

#[test]
#[should_panic(expected = "max_frames_per_poll must be > 0")]
fn zero_frame_limit_is_rejected_at_construction() {
    UplinkRouter::new(FailingLink, 0);
}
