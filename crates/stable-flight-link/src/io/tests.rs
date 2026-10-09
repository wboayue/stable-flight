use assert2::{assert, check};

use super::*;
use crate::loopback::{BufferTooSmall, Loopback};
use crate::{Downlink, MAX_FRAME_LEN, ParamRequest, Uplink, encode_vec};

#[test]
fn messages_round_trip_in_both_directions() {
    let loopback = Loopback::<2>::new();
    let (mut companion, mut fc) = loopback.split();
    let mut buf = [0u8; MAX_FRAME_LEN];

    companion
        .send_message(&Uplink::Param(ParamRequest::List), &mut buf)
        .unwrap();
    check!(fc.recv_message::<Uplink>(&mut buf) == Ok(Some(Uplink::Param(ParamRequest::List))));

    let status = Downlink::Status("ok".try_into().unwrap());
    fc.send_message(&status, &mut buf).unwrap();
    check!(companion.recv_message::<Downlink>(&mut buf) == Ok(Some(status)));
}

#[test]
fn nothing_waiting_receives_none() {
    let loopback = Loopback::<2>::new();
    let (_companion, mut fc) = loopback.split();
    let mut buf = [0u8; MAX_FRAME_LEN];

    check!(fc.recv_message::<Uplink>(&mut buf) == Ok(None));
}

#[test]
fn corrupted_frame_is_skipped_and_the_next_decodes() {
    let loopback = Loopback::<2>::new();
    let (mut companion, mut fc) = loopback.split();
    let mut buf = [0u8; MAX_FRAME_LEN];

    let mut corrupted = encode_vec(&Uplink::Arm(true));
    corrupted[1] ^= 0x55;
    companion.send(&corrupted).unwrap();
    companion
        .send_message(&Uplink::Arm(false), &mut buf)
        .unwrap();

    assert!(let Err(LinkError::Decode(_)) = fc.recv_message::<Uplink>(&mut buf));
    check!(fc.recv_message::<Uplink>(&mut buf) == Ok(Some(Uplink::Arm(false))));
}

#[test]
fn small_send_buffer_is_an_encode_error() {
    let loopback = Loopback::<2>::new();
    let (mut companion, mut fc) = loopback.split();
    let mut small = [0u8; 2];
    let mut buf = [0u8; MAX_FRAME_LEN];

    check!(
        companion.send_message(&Uplink::Arm(true), &mut small)
            == Err(LinkError::Encode(EncodeError::BufferTooSmall))
    );
    check!(fc.recv_message::<Uplink>(&mut buf) == Ok(None));
}

#[test]
fn transport_errors_pass_through() {
    let loopback = Loopback::<2>::new();
    let (mut companion, mut fc) = loopback.split();
    let mut buf = [0u8; MAX_FRAME_LEN];
    let mut small = [0u8; 2];

    companion
        .send_message(&Uplink::Arm(true), &mut buf)
        .unwrap();
    check!(fc.recv_message::<Uplink>(&mut small) == Err(LinkError::Transport(BufferTooSmall)));
}

#[test]
fn errors_name_their_cause() {
    let error: LinkError<BufferTooSmall> = LinkError::Decode(DecodeError::Crc);
    check!(error.to_string() == "decode failed: frame CRC mismatch");
    let source = core::error::Error::source(&error).map(ToString::to_string);
    check!(source.as_deref() == Some("frame CRC mismatch"));
}
