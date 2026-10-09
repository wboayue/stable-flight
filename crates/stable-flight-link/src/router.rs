//! Reading the link: decode [`Uplink`]s and hand each to the component that consumes it.

use heapless::spsc::Producer;

use crate::{Control, LinkError, LinkRx, MAX_FRAME_LEN, ParamRequest, RecvMessage, Uplink};

#[cfg(test)]
mod tests;

/// What one [`UplinkRouter::poll`] did.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RouteStats {
    /// Frames received, decoded or not.
    pub frames: usize,
    /// Frames that failed to decode (corrupt, or another protocol version).
    pub decode_errors: usize,
    /// Decoded messages dropped: their queue was full, or nothing consumes them.
    pub dropped: usize,
}

/// Drains a [`LinkRx`] and routes each [`Uplink`] to its consumer's queue: params to the
/// [`ParamServer`](crate::params::ParamServer), [`Control`] to a link-fed commander.
///
/// Reading the link is separate from any commander so params work whichever commander flies.
/// Call [`poll`](Self::poll) between ticks, or from a receive task: the queues decouple it
/// from the loop. A full queue drops the message (a ground station retries a param request;
/// a commander only needs the latest command) rather than stalling the link.
///
/// # Examples
///
/// ```
/// use stable_flight_link::heapless::spsc::Queue;
/// use stable_flight_link::loopback::Loopback;
/// use stable_flight_link::router::UplinkRouter;
/// use stable_flight_link::{MAX_FRAME_LEN, ParamRequest, SendMessage, Uplink};
///
/// let loopback = Loopback::<4>::new();
/// let (mut companion, fc) = loopback.split();
/// let mut requests = Queue::<ParamRequest, 8>::new();
/// let (to_server, mut from_router) = requests.split();
/// let mut router = UplinkRouter::new(fc, 8).with_params(to_server);
///
/// let mut buf = [0u8; MAX_FRAME_LEN];
/// companion.send_message(&Uplink::Param(ParamRequest::List), &mut buf).unwrap();
///
/// let stats = router.poll().unwrap();
/// assert_eq!(stats.frames, 1);
/// assert_eq!(from_router.dequeue(), Some(ParamRequest::List));
/// ```
pub struct UplinkRouter<'a, R> {
    rx: R,
    buf: [u8; MAX_FRAME_LEN],
    params: Option<Producer<'a, ParamRequest>>,
    control: Option<Producer<'a, Control>>,
    max_frames_per_poll: usize,
}

impl<'a, R: LinkRx> UplinkRouter<'a, R> {
    /// A router reading `rx`, decoding at most `max_frames_per_poll` frames per
    /// [`poll`](Self::poll). Each kind of message is dropped (and counted) until a builder
    /// gives it a queue: [`with_params`](Self::with_params),
    /// [`with_control`](Self::with_control).
    ///
    /// # Panics
    ///
    /// If `max_frames_per_poll` is zero: nothing would be read.
    pub fn new(rx: R, max_frames_per_poll: usize) -> Self {
        assert!(max_frames_per_poll > 0, "max_frames_per_poll must be > 0");
        Self {
            rx,
            buf: [0; MAX_FRAME_LEN],
            params: None,
            control: None,
            max_frames_per_poll,
        }
    }

    /// Routes [`Uplink::Param`] requests to `params`, for a
    /// [`ParamServer`](crate::params::ParamServer).
    pub fn with_params(mut self, params: Producer<'a, ParamRequest>) -> Self {
        self.params = Some(params);
        self
    }

    /// Routes [`Uplink::Control`] messages to `control`, for a link-fed commander.
    pub fn with_control(mut self, control: Producer<'a, Control>) -> Self {
        self.control = Some(control);
        self
    }

    /// Decodes and routes waiting frames, at most `max_frames_per_poll`.
    ///
    /// # Errors
    ///
    /// The transport failed; frames before the failure were routed.
    pub fn poll(&mut self) -> Result<RouteStats, R::Error> {
        let mut stats = RouteStats::default();
        while stats.frames < self.max_frames_per_poll {
            let msg = match self.rx.recv_message::<Uplink>(&mut self.buf) {
                Ok(Some(msg)) => msg,
                Ok(None) => break,
                Err(LinkError::Transport(e)) => return Err(e),
                // `Encode` can't happen on receive; count it with decode failures.
                Err(LinkError::Decode(_) | LinkError::Encode(_)) => {
                    stats.frames += 1;
                    stats.decode_errors += 1;
                    continue;
                }
            };
            stats.frames += 1;
            if !self.route(msg) {
                stats.dropped += 1;
            }
        }
        Ok(stats)
    }

    /// Queues `msg` for its consumer; `false` if it was dropped.
    fn route(&mut self, msg: Uplink) -> bool {
        match msg {
            Uplink::Param(request) => offer(&mut self.params, request),
            Uplink::Control(control) => offer(&mut self.control, control),
        }
    }
}

/// Enqueues `item` if there's a queue with room; `false` if it was dropped.
fn offer<T>(queue: &mut Option<Producer<'_, T>>, item: T) -> bool {
    queue
        .as_mut()
        .is_some_and(|queue| queue.enqueue(item).is_ok())
}
