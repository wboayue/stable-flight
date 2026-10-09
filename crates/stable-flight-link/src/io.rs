//! Message-level send and receive over a [`Link`](crate::Link) half: encode-then-send and
//! receive-then-decode in one call.

use core::fmt;

use crate::{DecodeError, EncodeError, LinkRx, LinkTx, Message};

#[cfg(test)]
mod tests;

/// Why [`SendMessage::send_message`] or [`RecvMessage::recv_message`] failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkError<E> {
    /// The transport failed.
    Transport(E),
    /// The message couldn't be encoded into the buffer.
    Encode(EncodeError),
    /// A received frame didn't decode. Only that frame is lost: keep receiving.
    Decode(DecodeError),
}

impl<E: fmt::Display> fmt::Display for LinkError<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Transport(e) => write!(f, "link transport failed: {e}"),
            Self::Encode(e) => write!(f, "encode failed: {e}"),
            Self::Decode(e) => write!(f, "decode failed: {e}"),
        }
    }
}

impl<E: core::error::Error + 'static> core::error::Error for LinkError<E> {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Transport(e) => Some(e),
            Self::Encode(e) => Some(e),
            Self::Decode(e) => Some(e),
        }
    }
}

/// Encodes and sends whole messages. Blanket-implemented for every [`LinkTx`].
///
/// # Examples
///
/// ```
/// use stable_flight_link::{
///     Control, LinkRx, LinkTx, MAX_FRAME_LEN, RecvMessage, SendMessage, Uplink,
/// };
///
/// /// Hands back the last frame sent.
/// #[derive(Default)]
/// struct Echo(Option<Vec<u8>>);
///
/// impl LinkTx for Echo {
///     type Error = core::convert::Infallible;
///     fn send(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
///         self.0 = Some(frame.to_vec());
///         Ok(())
///     }
/// }
///
/// impl LinkRx for Echo {
///     type Error = core::convert::Infallible;
///     fn try_recv(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Self::Error> {
///         Ok(self.0.take().map(|frame| {
///             buf[..frame.len()].copy_from_slice(&frame);
///             frame.len()
///         }))
///     }
/// }
///
/// let mut link = Echo::default();
/// let mut buf = [0u8; MAX_FRAME_LEN];
///
/// link.send_message(&Uplink::Control(Control::Arm(true)), &mut buf).unwrap();
/// assert_eq!(link.recv_message::<Uplink>(&mut buf), Ok(Some(Uplink::Control(Control::Arm(true)))));
/// assert_eq!(link.recv_message::<Uplink>(&mut buf), Ok(None));
/// ```
pub trait SendMessage: LinkTx {
    /// Encodes `msg` into `buf` and sends the frame. `buf` is scratch space supplied by the
    /// caller so a sender can reuse one buffer; [`MAX_FRAME_LEN`](crate::MAX_FRAME_LEN) bytes
    /// hold any message. Never blocks; the transport may drop the frame.
    fn send_message<M: Message>(
        &mut self,
        msg: &M,
        buf: &mut [u8],
    ) -> Result<(), LinkError<Self::Error>> {
        let len = msg.encode(buf).map_err(LinkError::Encode)?;
        let frame = buf
            .get(..len)
            .ok_or(LinkError::Encode(EncodeError::BufferTooSmall))?;
        self.send(frame).map_err(LinkError::Transport)
    }
}

impl<T: LinkTx + ?Sized> SendMessage for T {}

/// Receives and decodes whole messages. Blanket-implemented for every [`LinkRx`]. See
/// [`SendMessage`] for an example.
pub trait RecvMessage: LinkRx {
    /// Receives one frame into `buf` and decodes it, or returns `None` when no frame is
    /// waiting. Never blocks. A [`LinkError::Decode`] consumes the bad frame; call again for
    /// the next one. `buf` must hold [`MAX_FRAME_LEN`](crate::MAX_FRAME_LEN) bytes to fit any
    /// frame.
    fn recv_message<M: Message>(
        &mut self,
        buf: &mut [u8],
    ) -> Result<Option<M>, LinkError<Self::Error>> {
        let Some(len) = self.try_recv(buf).map_err(LinkError::Transport)? else {
            return Ok(None);
        };
        // A transport reporting more than it copied breaks the `LinkRx` contract; reject the
        // frame instead of panicking.
        let frame = buf
            .get_mut(..len)
            .ok_or(LinkError::Decode(DecodeError::TooShort))?;
        M::decode(frame).map(Some).map_err(LinkError::Decode)
    }
}

impl<T: LinkRx + ?Sized> RecvMessage for T {}
