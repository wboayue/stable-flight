//! The byte-level transport contract.

/// Moves encoded frames between the flight controller and a companion: USB-CDC, UART, UDP, or
/// an in-memory loopback in tests (`testing` feature). A `Link` only moves bytes; encoding and
/// decoding are [`Message`](crate::Message)'s job, so every transport carries the same frames.
///
/// Both methods are non-blocking: the flight loop calls them every tick.
///
/// # Examples
///
/// ```
/// use stable_flight_link::{Link, MAX_FRAME_LEN, Message, Uplink};
///
/// fn send_heartbeat<L: Link>(link: &mut L) -> Result<(), L::Error> {
///     let mut frame = [0u8; MAX_FRAME_LEN];
///     let len = Uplink::Heartbeat
///         .encode(&mut frame)
///         .unwrap_or_else(|_| unreachable!("MAX_FRAME_LEN fits every message"));
///     link.send(&frame[..len])
/// }
/// ```
pub trait Link {
    /// Transport failure.
    type Error;

    /// Copies one received frame, with or without its trailing `0x00`, into `buf` and returns
    /// its length, or `None` when no complete frame is waiting. Never blocks. A `buf` of
    /// [`MAX_FRAME_LEN`](crate::MAX_FRAME_LEN) bytes holds any valid frame.
    fn try_recv(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Self::Error>;

    /// Queues one encoded frame, trailing `0x00` included. Never blocks: a transport that can't
    /// take the frame now may drop it and return `Ok`.
    fn send(&mut self, frame: &[u8]) -> Result<(), Self::Error>;
}
