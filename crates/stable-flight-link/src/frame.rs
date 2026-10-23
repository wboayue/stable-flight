//! Frame codec: `[version][postcard payload][crc16 LE]`, COBS-encoded, `0x00`-terminated.

use core::fmt;

use crc::{CRC_16_IBM_3740, Crc, NoTable};
use serde::Serialize;
use serde::de::DeserializeOwned;

#[cfg(test)]
mod tests;

/// Wire protocol version, the first byte of every frame. The receiver rejects any other value.
///
/// Postcard is not self-describing, so any change to a message's shape (a new variant, a
/// reordered or retyped field) must bump this.
pub const PROTOCOL_VERSION: u8 = 10;

/// Largest encoded frame in bytes, trailing `0x00` delimiter included. Every [`Message`] fits.
pub const MAX_FRAME_LEN: usize = 384;

const VERSION_LEN: usize = 1;
const CRC_LEN: usize = 2;
const DELIMITER_LEN: usize = 1;

/// Largest frame before COBS encoding (version + payload + CRC) that still fits
/// [`MAX_FRAME_LEN`] once encoded and terminated.
const MAX_RAW_LEN: usize =
    MAX_FRAME_LEN - DELIMITER_LEN - cobs::max_encoding_overhead(MAX_FRAME_LEN);
const _: () = assert!(cobs::max_encoding_length(MAX_RAW_LEN) + DELIMITER_LEN <= MAX_FRAME_LEN);

/// CRC-16/IBM-3740 (CCITT-FALSE) over version + payload. Bitwise (no table) to save flash; it
/// runs at link rate, not loop rate.
const CRC: Crc<u16, NoTable> = Crc::<u16, NoTable>::new(&CRC_16_IBM_3740);

/// Why a message couldn't be encoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// The output buffer can't hold the encoded frame; [`MAX_FRAME_LEN`] always suffices.
    BufferTooSmall,
    /// The message failed to serialize or exceeds [`MAX_FRAME_LEN`]. Not expected for the
    /// protocol's own messages.
    Serialize,
}

impl fmt::Display for EncodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BufferTooSmall => f.write_str("output buffer too small for frame"),
            Self::Serialize => f.write_str("message failed to serialize"),
        }
    }
}

impl core::error::Error for EncodeError {}

/// Why a frame couldn't be decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// The frame isn't valid COBS.
    Cobs,
    /// Bytes follow the `0x00` delimiter: the buffer holds more than one frame.
    TrailingBytes,
    /// The frame is too short to hold a version byte and a CRC.
    TooShort,
    /// The CRC doesn't match: the frame was corrupted.
    Crc,
    /// The frame was written by another [`PROTOCOL_VERSION`].
    Version {
        /// Version byte the frame carried.
        got: u8,
    },
    /// The payload isn't a valid message of the expected type.
    Deserialize,
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Cobs => f.write_str("invalid COBS encoding"),
            Self::TrailingBytes => f.write_str("bytes after the frame delimiter"),
            Self::TooShort => f.write_str("frame too short"),
            Self::Crc => f.write_str("frame CRC mismatch"),
            Self::Version { got } => {
                write!(f, "protocol version {got}, expected {PROTOCOL_VERSION}")
            }
            Self::Deserialize => f.write_str("invalid message payload"),
        }
    }
}

impl core::error::Error for DecodeError {}

pub(crate) mod sealed {
    pub trait Sealed {}
}

/// A message of this protocol: [`Uplink`](crate::Uplink) or [`Downlink`](crate::Downlink).
/// Sealed, so only the protocol's own messages travel under [`PROTOCOL_VERSION`].
///
/// # Examples
///
/// ```
/// use stable_flight_link::{Control, MAX_FRAME_LEN, Message, Uplink};
///
/// let mut frame = [0u8; MAX_FRAME_LEN];
/// let len = Uplink::Control(Control::Arm(true)).encode(&mut frame).unwrap();
/// assert_eq!(frame[len - 1], 0x00); // delimiter
///
/// assert_eq!(Uplink::decode(&mut frame[..len]), Ok(Uplink::Control(Control::Arm(true))));
/// ```
pub trait Message: Serialize + DeserializeOwned + sealed::Sealed {
    /// Encodes `self` as one frame into `out`, trailing `0x00` included, and returns its
    /// length. An `out` of [`MAX_FRAME_LEN`] bytes always suffices.
    fn encode(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        encode(self, out)
    }

    /// Decodes one frame, with or without its trailing `0x00`, in place: `frame` is
    /// overwritten. Checks the CRC, then the version, then the payload. Bytes after the
    /// delimiter are rejected, not ignored.
    fn decode(frame: &mut [u8]) -> Result<Self, DecodeError> {
        decode(frame)
    }
}

fn encode<T: Serialize>(msg: &T, out: &mut [u8]) -> Result<usize, EncodeError> {
    let mut raw = [0u8; MAX_RAW_LEN];
    raw[0] = PROTOCOL_VERSION;
    let payload_len = postcard::to_slice(msg, &mut raw[VERSION_LEN..MAX_RAW_LEN - CRC_LEN])
        .map_err(|_| EncodeError::Serialize)?
        .len();

    let body_len = VERSION_LEN + payload_len;
    let crc = CRC.checksum(&raw[..body_len]);
    raw[body_len..body_len + CRC_LEN].copy_from_slice(&crc.to_le_bytes());

    let len = cobs::try_encode(&raw[..body_len + CRC_LEN], out)
        .map_err(|_| EncodeError::BufferTooSmall)?;
    *out.get_mut(len).ok_or(EncodeError::BufferTooSmall)? = 0;
    Ok(len + DELIMITER_LEN)
}

fn decode<T: DeserializeOwned>(frame: &mut [u8]) -> Result<T, DecodeError> {
    let report = cobs::decode_in_place_report(frame).map_err(|e| match e {
        cobs::DecodeError::EmptyFrame => DecodeError::TooShort,
        _ => DecodeError::Cobs,
    })?;
    // `parsed_size` stops at the delimiter; anything past it is another frame.
    if frame.len() > report.parsed_size() + DELIMITER_LEN {
        return Err(DecodeError::TrailingBytes);
    }
    let len = report.frame_size();

    let Some((body, crc)) = frame[..len].split_last_chunk::<CRC_LEN>() else {
        return Err(DecodeError::TooShort);
    };
    let [version, payload @ ..] = body else {
        return Err(DecodeError::TooShort);
    };
    if CRC.checksum(body) != u16::from_le_bytes(*crc) {
        return Err(DecodeError::Crc);
    }
    if *version != PROTOCOL_VERSION {
        return Err(DecodeError::Version { got: *version });
    }

    match postcard::take_from_bytes(payload) {
        Ok((msg, [])) => Ok(msg),
        _ => Err(DecodeError::Deserialize),
    }
}
