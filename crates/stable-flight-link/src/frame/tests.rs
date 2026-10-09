use assert2::check;

use super::*;
use crate::{ParamRequest, Uplink, encode_vec};

/// COBS-encodes `raw` (version + payload + CRC) and terminates it.
fn frame_from_raw(raw: &[u8]) -> Vec<u8> {
    let mut frame = vec![0u8; cobs::max_encoding_length(raw.len()) + DELIMITER_LEN];
    let len = cobs::encode(raw, &mut frame);
    frame.truncate(len + DELIMITER_LEN);
    frame[len] = 0;
    frame
}

/// Version + payload + a valid CRC over both.
fn raw_with_crc(version: u8, payload: &[u8]) -> Vec<u8> {
    let mut raw = vec![version];
    raw.extend_from_slice(payload);
    let crc = CRC.checksum(&raw);
    raw.extend_from_slice(&crc.to_le_bytes());
    raw
}

fn payload(msg: &Uplink) -> Vec<u8> {
    let mut buf = [0u8; MAX_FRAME_LEN];
    postcard::to_slice(msg, &mut buf).expect("fits").to_vec()
}

fn arm() -> Uplink {
    Uplink::Arm(true)
}

#[test]
fn encoded_frame_has_only_the_trailing_zero() {
    // A payload full of zeros: COBS must remove every one.
    let frame = encode_vec(&Uplink::Param(ParamRequest::Set {
        name: Default::default(),
        value: 0.0,
    }));
    let (last, body) = frame.split_last().expect("non-empty");
    check!(*last == 0);
    check!(!body.contains(&0));
}

#[test]
fn decodes_with_or_without_delimiter() {
    let mut with = encode_vec(&arm());
    let mut without = with[..with.len() - 1].to_vec();
    check!(Uplink::decode(&mut with) == Ok(arm()));
    check!(Uplink::decode(&mut without) == Ok(arm()));
}

#[test]
fn frame_layout_is_version_payload_crc() {
    let mut frame = encode_vec(&arm());
    let expected = frame_from_raw(&raw_with_crc(PROTOCOL_VERSION, &payload(&arm())));
    check!(frame == expected);
    check!(Uplink::decode(&mut frame) == Ok(arm()));
}

#[test]
fn rejects_bad_frames() {
    let good_raw = raw_with_crc(PROTOCOL_VERSION, &payload(&arm()));
    let mut flipped = good_raw.clone();
    flipped[1] ^= 0x01;
    let mut trailing = payload(&arm());
    trailing.push(0x7f);

    let two_frames = [encode_vec(&arm()), encode_vec(&arm())].concat();

    let cases: [(&str, Vec<u8>, DecodeError); 9] = [
        ("empty", vec![], DecodeError::TooShort),
        ("delimiter only", vec![0], DecodeError::TooShort),
        (
            "version and CRC only",
            frame_from_raw(&[PROTOCOL_VERSION, 0x00]),
            DecodeError::TooShort,
        ),
        (
            "flipped payload bit",
            frame_from_raw(&flipped),
            DecodeError::Crc,
        ),
        (
            "wrong version, valid CRC",
            frame_from_raw(&raw_with_crc(PROTOCOL_VERSION + 1, &payload(&arm()))),
            DecodeError::Version {
                got: PROTOCOL_VERSION + 1,
            },
        ),
        (
            "unknown variant tag",
            frame_from_raw(&raw_with_crc(PROTOCOL_VERSION, &[0x7f])),
            DecodeError::Deserialize,
        ),
        (
            "trailing payload bytes",
            frame_from_raw(&raw_with_crc(PROTOCOL_VERSION, &trailing)),
            DecodeError::Deserialize,
        ),
        ("invalid COBS", vec![0x05, 0x01, 0x00], DecodeError::Cobs),
        (
            "two frames in one buffer",
            two_frames,
            DecodeError::TrailingBytes,
        ),
    ];
    for (scenario, mut frame, expected) in cases {
        check!(Uplink::decode(&mut frame) == Err(expected), "{scenario}");
    }
}

#[test]
fn rejects_truncated_frames() {
    let frame = encode_vec(&arm());
    // Cutting the encoded frame breaks the COBS structure or the CRC, or leaves too few bytes.
    for cut in 1..frame.len() - 1 {
        let mut truncated = frame[..frame.len() - 1 - cut].to_vec();
        let result = Uplink::decode(&mut truncated);
        check!(
            matches!(
                result,
                Err(DecodeError::Cobs | DecodeError::Crc | DecodeError::TooShort)
            ),
            "cut {cut}: {result:?}"
        );
    }
}

#[test]
fn rejects_output_buffer_too_small() {
    let len = encode_vec(&arm()).len();
    for short in [0, 1, len - 1] {
        let mut out = vec![0u8; short];
        check!(
            arm().encode(&mut out) == Err(EncodeError::BufferTooSmall),
            "{short} bytes"
        );
    }
}
