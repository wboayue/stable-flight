//! In-memory [`Link`] pair for tests (`testing` feature).
//!
//! ```
//! use stable_flight_link::loopback::Loopback;
//! use stable_flight_link::{Link, MAX_FRAME_LEN, Message, Uplink};
//!
//! let loopback = Loopback::<4>::new();
//! let (mut companion, mut fc) = loopback.split();
//!
//! let mut frame = [0u8; MAX_FRAME_LEN];
//! let len = Uplink::Arm(true).encode(&mut frame).unwrap();
//! companion.send(&frame[..len]).unwrap();
//!
//! let len = fc.try_recv(&mut frame).unwrap().unwrap();
//! assert_eq!(Uplink::decode(&mut frame[..len]), Ok(Uplink::Arm(true)));
//! ```

use core::cell::RefCell;
use core::fmt;

use heapless::Deque;

use crate::{Link, MAX_FRAME_LEN};

#[cfg(test)]
mod tests;

type Frame = heapless::Vec<u8, MAX_FRAME_LEN>;
type Queue<const DEPTH: usize> = RefCell<Deque<Frame, DEPTH>>;

/// Storage for two connected [`LoopbackLink`] ends, each direction queuing up to `DEPTH`
/// frames. Single-threaded: both ends borrow it.
pub struct Loopback<const DEPTH: usize> {
    a_to_b: Queue<DEPTH>,
    b_to_a: Queue<DEPTH>,
}

impl<const DEPTH: usize> Loopback<DEPTH> {
    /// Empty queues.
    pub const fn new() -> Self {
        Self {
            a_to_b: RefCell::new(Deque::new()),
            b_to_a: RefCell::new(Deque::new()),
        }
    }

    /// The two ends: what one sends, the other receives.
    pub fn split(&self) -> (LoopbackLink<'_, DEPTH>, LoopbackLink<'_, DEPTH>) {
        (
            LoopbackLink {
                tx: &self.a_to_b,
                rx: &self.b_to_a,
            },
            LoopbackLink {
                tx: &self.b_to_a,
                rx: &self.a_to_b,
            },
        )
    }
}

impl<const DEPTH: usize> Default for Loopback<DEPTH> {
    fn default() -> Self {
        Self::new()
    }
}

/// One end of a [`Loopback`]. When the peer's queue is full, [`Link::send`] drops the frame.
pub struct LoopbackLink<'a, const DEPTH: usize> {
    tx: &'a Queue<DEPTH>,
    rx: &'a Queue<DEPTH>,
}

/// [`LoopbackLink`] failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoopbackError {
    /// The sent frame exceeds [`MAX_FRAME_LEN`].
    FrameTooLarge,
    /// The receive buffer can't hold the next frame; the frame was dropped.
    BufferTooSmall,
}

impl fmt::Display for LoopbackError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::FrameTooLarge => f.write_str("frame exceeds MAX_FRAME_LEN"),
            Self::BufferTooSmall => f.write_str("receive buffer too small for frame"),
        }
    }
}

impl core::error::Error for LoopbackError {}

impl<const DEPTH: usize> Link for LoopbackLink<'_, DEPTH> {
    type Error = LoopbackError;

    fn try_recv(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Self::Error> {
        let Some(frame) = self.rx.borrow_mut().pop_front() else {
            return Ok(None);
        };
        let dest = buf
            .get_mut(..frame.len())
            .ok_or(LoopbackError::BufferTooSmall)?;
        dest.copy_from_slice(&frame);
        Ok(Some(frame.len()))
    }

    fn send(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        let frame = Frame::from_slice(frame).map_err(|_| LoopbackError::FrameTooLarge)?;
        // Full queue: drop, per the `Link` contract.
        let _ = self.tx.borrow_mut().push_back(frame);
        Ok(())
    }
}
