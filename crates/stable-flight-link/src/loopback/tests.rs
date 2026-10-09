use assert2::{assert, check};

use super::*;
use crate::{Downlink, Message, Uplink, encode_vec};

#[test]
fn messages_cross_in_both_directions() {
    let loopback = Loopback::<2>::new();
    let (mut companion, mut fc) = loopback.split();
    let mut buf = [0u8; MAX_FRAME_LEN];

    companion.send(&encode_vec(&Uplink::Arm(true))).unwrap();
    assert!(let Ok(Some(len)) = fc.try_recv(&mut buf));
    check!(Uplink::decode(&mut buf[..len]) == Ok(Uplink::Arm(true)));

    let status = Downlink::Status("ok".try_into().unwrap());
    fc.send(&encode_vec(&status)).unwrap();
    assert!(let Ok(Some(len)) = companion.try_recv(&mut buf));
    check!(Downlink::decode(&mut buf[..len]) == Ok(status));
}

#[test]
fn halves_share_one_end() {
    let loopback = Loopback::<2>::new();
    let (companion, mut fc) = loopback.split();
    let (mut rx, mut tx) = companion.into_halves();
    let mut buf = [0u8; MAX_FRAME_LEN];

    tx.send(&[1, 0]).unwrap();
    check!(fc.try_recv(&mut buf) == Ok(Some(2)));
    fc.send(&[2, 0]).unwrap();
    check!(rx.try_recv(&mut buf) == Ok(Some(2)));
    check!(buf[..2] == [2, 0]);
}

#[test]
fn empty_queue_receives_nothing() {
    let loopback = Loopback::<2>::new();
    let (mut a, mut b) = loopback.split();
    let mut buf = [0u8; MAX_FRAME_LEN];
    check!(a.try_recv(&mut buf) == Ok(None));
    check!(b.try_recv(&mut buf) == Ok(None));
}

#[test]
fn full_queue_drops_newest_without_blocking() {
    let loopback = Loopback::<2>::new();
    let (mut a, mut b) = loopback.split();
    for first in [1u8, 2, 3] {
        check!(a.send(&[first, 0]) == Ok(()));
    }

    let mut buf = [0u8; MAX_FRAME_LEN];
    let mut received = Vec::new();
    while let Ok(Some(len)) = b.try_recv(&mut buf) {
        received.push(buf[..len].to_vec());
    }
    check!(received == [vec![1, 0], vec![2, 0]]);
}

#[test]
fn rejects_oversized_frame_and_short_buffer() {
    let loopback = Loopback::<2>::new();
    let (mut a, mut b) = loopback.split();
    check!(a.send(&[1; MAX_FRAME_LEN + 1]) == Err(FrameTooLarge));

    a.send(&[1, 2, 3, 0]).unwrap();
    let mut short = [0u8; 2];
    check!(b.try_recv(&mut short) == Err(BufferTooSmall));
}
