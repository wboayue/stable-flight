//! The byte-level transport contract.

/// Receiving half of a transport. See [`Link`].
pub trait LinkRx {
    /// Transport failure.
    type Error;

    /// Copies one received frame, with or without its trailing `0x00`, into `buf` and returns
    /// its length, or `None` when no complete frame is waiting. Never blocks. A `buf` of
    /// [`MAX_FRAME_LEN`](crate::MAX_FRAME_LEN) bytes holds any valid frame. A frame that
    /// doesn't fit `buf` is lost; whether that surfaces as an error or as a truncated frame
    /// that fails decode is up to the transport. Either way, keep receiving.
    fn try_recv(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Self::Error>;
}

/// Sending half of a transport. See [`Link`].
pub trait LinkTx {
    /// Transport failure.
    type Error;

    /// Queues one encoded frame, trailing `0x00` included. Never blocks: a transport that can't
    /// take the frame now may drop it and return `Ok`.
    fn send(&mut self, frame: &[u8]) -> Result<(), Self::Error>;
}

/// Moves encoded frames between the flight controller and a companion: USB-CDC, UART, UDP, or
/// an in-memory loopback in tests (`testing` feature). A link only moves bytes; encoding and
/// decoding are [`Message`](crate::Message)'s job, so every transport carries the same frames.
///
/// `Link` is [`LinkRx`] + [`LinkTx`], blanket-implemented: implement those two, never `Link`
/// directly. The split lets a receiver (a commander) and a sender (a telemetry publisher) each
/// own one half of the same transport. Both halves are non-blocking: the flight loop calls
/// them every tick.
///
/// # Examples
///
/// ```
/// use stable_flight_link::{Link, LinkRx, LinkTx};
///
/// /// Drops everything it's given and never receives.
/// struct Sink;
///
/// impl LinkRx for Sink {
///     type Error = core::convert::Infallible;
///     fn try_recv(&mut self, _buf: &mut [u8]) -> Result<Option<usize>, Self::Error> {
///         Ok(None)
///     }
/// }
///
/// impl LinkTx for Sink {
///     type Error = core::convert::Infallible;
///     fn send(&mut self, _frame: &[u8]) -> Result<(), Self::Error> {
///         Ok(())
///     }
/// }
///
/// fn is_link(_: &impl Link) {}
/// is_link(&Sink);
/// ```
pub trait Link: LinkRx + LinkTx {}

impl<T: LinkRx + LinkTx> Link for T {}
