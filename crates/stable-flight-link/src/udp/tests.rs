use std::time::{Duration, Instant};

use assert2::{assert, check};

use super::*;
use crate::{Downlink, MAX_FRAME_LEN, Message, SendMessage, Uplink};

const WAIT: Duration = Duration::from_secs(2);

/// A blocking plain socket on an ephemeral localhost port, standing in for the peer.
fn bind_peer() -> UdpSocket {
    let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
    socket.set_read_timeout(Some(WAIT)).unwrap();
    socket
}

/// Polls `rx` until a frame arrives or `WAIT` passes.
fn recv_within(rx: &mut impl LinkRx<Error = io::Error>, buf: &mut [u8]) -> Option<usize> {
    let deadline = Instant::now() + WAIT;
    while Instant::now() < deadline {
        if let Some(len) = rx.try_recv(buf).unwrap() {
            return Some(len);
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    None
}

#[test]
fn frames_cross_in_both_directions() {
    let peer = bind_peer();
    let mut link = UdpLink::bind("127.0.0.1:0", peer.local_addr().unwrap()).unwrap();
    peer.connect(link.local_addr().unwrap()).unwrap();
    let mut buf = [0u8; MAX_FRAME_LEN];

    link.send_message(&Downlink::Status("up".try_into().unwrap()), &mut buf)
        .unwrap();
    let len = peer.recv(&mut buf).unwrap();
    check!(Downlink::decode(&mut buf[..len]) == Ok(Downlink::Status("up".try_into().unwrap())));

    let len = Uplink::Arm(true).encode(&mut buf).unwrap();
    peer.send(&buf[..len]).unwrap();
    assert!(let Some(len) = recv_within(&mut link, &mut buf));
    check!(Uplink::decode(&mut buf[..len]) == Ok(Uplink::Arm(true)));
}

#[test]
fn nothing_waiting_receives_none() {
    let peer = bind_peer();
    let mut link = UdpLink::bind("127.0.0.1:0", peer.local_addr().unwrap()).unwrap();
    let mut buf = [0u8; MAX_FRAME_LEN];

    check!(link.try_recv(&mut buf).unwrap() == None);
}

#[test]
fn absent_peer_drops_sends_without_error() {
    // Socket dropped: nothing listens there. A parallel test could rebind the port in between,
    // which only skips the refused path; the assertions still hold.
    let closed = bind_peer().local_addr().unwrap();
    let mut link = UdpLink::bind("127.0.0.1:0", closed).unwrap();
    let mut buf = [0u8; MAX_FRAME_LEN];

    for _ in 0..3 {
        check!(link.send_message(&Uplink::Heartbeat, &mut buf).is_ok());
        std::thread::sleep(Duration::from_millis(10)); // let a port-unreachable land
        check!(link.try_recv(&mut buf).unwrap() == None);
    }
}

#[test]
fn peer_coming_up_gets_the_next_frame() {
    let addr = bind_peer().local_addr().unwrap(); // dropped: refuses until rebound
    let mut link = UdpLink::bind("127.0.0.1:0", addr).unwrap();
    let mut buf = [0u8; MAX_FRAME_LEN];

    link.send(&[1, 0]).unwrap();
    std::thread::sleep(Duration::from_millis(10)); // let the port-unreachable land
    let peer = UdpSocket::bind(addr).expect("port still free");
    peer.set_read_timeout(Some(WAIT)).unwrap();

    link.send(&[2, 0]).unwrap(); // consumes the stale error, then sends
    check!(peer.recv(&mut buf).unwrap() == 2);
    check!(buf[..2] == [2, 0]);
}

#[test]
fn ignores_datagrams_from_other_senders() {
    let peer = bind_peer();
    let stranger = bind_peer();
    let mut link = UdpLink::bind("127.0.0.1:0", peer.local_addr().unwrap()).unwrap();
    let addr = link.local_addr().unwrap();
    let mut buf = [0u8; MAX_FRAME_LEN];

    stranger.send_to(&[9, 0], addr).unwrap();
    peer.send_to(&[1, 0], addr).unwrap();
    assert!(let Some(2) = recv_within(&mut link, &mut buf));
    check!(buf[..2] == [1, 0]);
}

#[test]
fn halves_share_one_socket() {
    let peer = bind_peer();
    let link = UdpLink::bind("127.0.0.1:0", peer.local_addr().unwrap()).unwrap();
    peer.connect(link.local_addr().unwrap()).unwrap();
    let (mut rx, mut tx) = link.into_halves();
    let mut buf = [0u8; MAX_FRAME_LEN];

    tx.send(&[1, 0]).unwrap();
    check!(peer.recv(&mut buf).unwrap() == 2);
    peer.send(&[2, 0]).unwrap();
    assert!(let Some(2) = recv_within(&mut rx, &mut buf));
    check!(buf[..2] == [2, 0]);
}

#[test]
fn unresolvable_address_fails_to_bind() {
    check!(UdpLink::bind("127.0.0.1:0", "not an address").is_err());
}
