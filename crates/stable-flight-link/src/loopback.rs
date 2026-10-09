//! In-memory [`Link`] pair for tests (`testing` feature).
//!
//! ```
//! use stable_flight_link::loopback::Loopback;
//! use stable_flight_link::{Control, LinkRx, LinkTx, MAX_FRAME_LEN, Message, Uplink};
//!
//! let loopback = Loopback::<4>::new();
//! let (mut companion, mut fc) = loopback.split();
//!
//! let mut frame = [0u8; MAX_FRAME_LEN];
//! let len = Uplink::Control(Control::Arm(true)).encode(&mut frame).unwrap();
//! companion.send(&frame[..len]).unwrap();
//!
//! let len = fc.try_recv(&mut frame).unwrap().unwrap();
//! assert_eq!(Uplink::decode(&mut frame[..len]), Ok(Uplink::Control(Control::Arm(true))));
//! ```

use core::cell::RefCell;
use core::fmt;

use heapless::Deque;

#[cfg(doc)]
use crate::Link;
use crate::{LinkRx, LinkTx, MAX_FRAME_LEN};

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
                rx: LoopbackRx {
                    queue: &self.b_to_a,
                },
                tx: LoopbackTx {
                    queue: &self.a_to_b,
                },
            },
            LoopbackLink {
                rx: LoopbackRx {
                    queue: &self.a_to_b,
                },
                tx: LoopbackTx {
                    queue: &self.b_to_a,
                },
            },
        )
    }
}

impl<const DEPTH: usize> Default for Loopback<DEPTH> {
    fn default() -> Self {
        Self::new()
    }
}

/// One end of a [`Loopback`]: a [`Link`]. [`into_halves`](Self::into_halves) splits it so a
/// receiver and a sender can each own one half.
pub struct LoopbackLink<'a, const DEPTH: usize> {
    rx: LoopbackRx<'a, DEPTH>,
    tx: LoopbackTx<'a, DEPTH>,
}

impl<'a, const DEPTH: usize> LoopbackLink<'a, DEPTH> {
    /// The receiving and sending halves of this end.
    pub fn into_halves(self) -> (LoopbackRx<'a, DEPTH>, LoopbackTx<'a, DEPTH>) {
        (self.rx, self.tx)
    }
}

/// Receiving half of a [`LoopbackLink`].
pub struct LoopbackRx<'a, const DEPTH: usize> {
    queue: &'a Queue<DEPTH>,
}

/// Sending half of a [`LoopbackLink`]. When the peer's queue is full, [`LinkTx::send`] drops
/// the frame.
pub struct LoopbackTx<'a, const DEPTH: usize> {
    queue: &'a Queue<DEPTH>,
}

/// [`LoopbackRx`] failure: the receive buffer can't hold the next frame, which was dropped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BufferTooSmall;

impl fmt::Display for BufferTooSmall {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("receive buffer too small for frame")
    }
}

impl core::error::Error for BufferTooSmall {}

/// [`LoopbackTx`] failure: the sent frame exceeds [`MAX_FRAME_LEN`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameTooLarge;

impl fmt::Display for FrameTooLarge {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("frame exceeds MAX_FRAME_LEN")
    }
}

impl core::error::Error for FrameTooLarge {}

impl<const DEPTH: usize> LinkRx for LoopbackRx<'_, DEPTH> {
    type Error = BufferTooSmall;

    fn try_recv(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Self::Error> {
        let Some(frame) = self.queue.borrow_mut().pop_front() else {
            return Ok(None);
        };
        let dest = buf.get_mut(..frame.len()).ok_or(BufferTooSmall)?;
        dest.copy_from_slice(&frame);
        Ok(Some(frame.len()))
    }
}

impl<const DEPTH: usize> LinkTx for LoopbackTx<'_, DEPTH> {
    type Error = FrameTooLarge;

    fn send(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        let frame = Frame::from_slice(frame).map_err(|_| FrameTooLarge)?;
        // Full queue: drop, per the `LinkTx` contract.
        let _ = self.queue.borrow_mut().push_back(frame);
        Ok(())
    }
}

impl<const DEPTH: usize> LinkRx for LoopbackLink<'_, DEPTH> {
    type Error = BufferTooSmall;

    fn try_recv(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Self::Error> {
        self.rx.try_recv(buf)
    }
}

impl<const DEPTH: usize> LinkTx for LoopbackLink<'_, DEPTH> {
    type Error = FrameTooLarge;

    fn send(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        self.tx.send(frame)
    }
}

/// A [`Link`] whose every receive and send fails with [`LinkDown`], for testing error paths.
#[derive(Debug, Default, Clone, Copy)]
pub struct FailingLink;

/// [`FailingLink`]'s error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LinkDown;

impl fmt::Display for LinkDown {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("link down")
    }
}

impl core::error::Error for LinkDown {}

impl LinkRx for FailingLink {
    type Error = LinkDown;

    fn try_recv(&mut self, _buf: &mut [u8]) -> Result<Option<usize>, Self::Error> {
        Err(LinkDown)
    }
}

impl LinkTx for FailingLink {
    type Error = LinkDown;

    fn send(&mut self, _frame: &[u8]) -> Result<(), Self::Error> {
        Err(LinkDown)
    }
}
