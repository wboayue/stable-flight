//! The flight controller's side of the link, wired in one call: the param queues,
//! [`UplinkRouter`], [`ParamServer`], and [`LinkPublisher`]. A flight controller (a SITL example,
//! a board) or a test gets the whole stack from [`FcLink::wire`]: the publisher is the flight
//! loop's telemetry, and the [`FcPoller`] runs between ticks.
//!
//! ```
//! use stable_flight_link::loopback::Loopback;
//! use stable_flight_link::stable_flight::params::Params;
//! use stable_flight_link::stable_flight::stabilizers::copter::{
//!     CopterStabilizer, CopterStabilizerConfig,
//! };
//! use stable_flight_link::fc::FcLink;
//! use stable_flight_link::{MAX_FRAME_LEN, ParamRequest, SendMessage, Uplink};
//!
//! let loopback = Loopback::<16>::new();
//! let (mut companion, fc_end) = loopback.split();
//! let (rx, tx) = fc_end.into_halves();
//! let mut link = FcLink::<16, 8>::new();
//! // `_publisher` is the flight loop's telemetry, which sends the echo.
//! let (mut poller, _publisher) = link.wire(rx, tx);
//! let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
//!
//! let set = ParamRequest::Set { name: "STAB_ATT_AMAX_X".try_into().unwrap(), value: 30.0 };
//! companion.send_message(&Uplink::Param(set), &mut [0u8; MAX_FRAME_LEN]).unwrap();
//! poller.poll(&mut stabilizer).unwrap(); // between ticks
//!
//! let stored = stabilizer.param("STAB_ATT_AMAX_X").unwrap();
//! assert!((stored - 30.0).abs() < 1e-4);
//! ```

use heapless::spsc::{Producer, Queue};
use stable_flight::params::Params;

use crate::commander::LinkCommander;
use crate::params::ParamServer;
use crate::publisher::{LinkPublisher, LinkPublisherConfig};
use crate::router::{RouteStats, UplinkRouter};
use crate::{Control, LinkRx, LinkTx, ParamRequest, ParamValue};

/// How an [`FcLink`] is wired: the publisher's rates and the per-poll limits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FcLinkConfig {
    /// The publisher's rates.
    pub publisher: LinkPublisherConfig,
    /// Uplink frames the router decodes per poll; more wait for the next. Must be > 0.
    pub max_frames_per_poll: usize,
    /// Param writes the server applies per poll; a ground station's "load from file" bursts
    /// hundreds, so this bounds the time a burst adds to one gap between ticks. Must be > 0.
    pub max_writes_per_poll: usize,
}

impl Default for FcLinkConfig {
    /// The default publisher rates, 8 frames and 4 param writes per poll.
    fn default() -> Self {
        Self {
            publisher: LinkPublisherConfig::default(),
            max_frames_per_poll: 8,
            max_writes_per_poll: 4,
        }
    }
}

/// Storage for the flight controller's link queues, sized `REQUESTS` (param requests), `REPLIES`
/// (param replies), and `CONTROLS` (control messages for a [`LinkCommander`],
/// [`wire_linked`](Self::wire_linked) only). Each is a `heapless` SPSC queue, which holds one
/// fewer than its size: an `FcLink<32, 16>` holds 31 requests, 15 replies, and 7 control
/// messages. [`wire`](Self::wire) borrows it
/// to build the stack.
pub struct FcLink<const REQUESTS: usize, const REPLIES: usize, const CONTROLS: usize = 8> {
    requests: Queue<ParamRequest, REQUESTS>,
    replies: Queue<ParamValue, REPLIES>,
    controls: Queue<Control, CONTROLS>,
}

impl<const REQUESTS: usize, const REPLIES: usize, const CONTROLS: usize>
    FcLink<REQUESTS, REPLIES, CONTROLS>
{
    /// Empty queues.
    pub const fn new() -> Self {
        Self {
            requests: Queue::new(),
            replies: Queue::new(),
            controls: Queue::new(),
        }
    }

    /// The stack over the flight controller's link halves: an [`FcPoller`] to call between
    /// ticks, and the [`LinkPublisher`] (param replies attached) to use as the flight loop's
    /// telemetry, with [`FcLinkConfig::default`].
    pub fn wire<R: LinkRx, T: LinkTx>(
        &mut self,
        rx: R,
        tx: T,
    ) -> (FcPoller<'_, R>, LinkPublisher<'_, T>) {
        self.wire_with(rx, tx, FcLinkConfig::default())
    }

    /// As [`wire`](Self::wire), with the publisher's rates and per-poll limits from `config`.
    ///
    /// # Panics
    ///
    /// If either per-poll limit is zero.
    ///
    /// # Examples
    ///
    /// ```
    /// use core::time::Duration;
    ///
    /// use stable_flight_link::fc::{FcLink, FcLinkConfig};
    /// use stable_flight_link::loopback::Loopback;
    /// use stable_flight_link::publisher::LinkPublisherConfig;
    ///
    /// let loopback = Loopback::<16>::new();
    /// let (_companion, fc_end) = loopback.split();
    /// let (rx, tx) = fc_end.into_halves();
    /// let mut link = FcLink::<32, 16>::new();
    /// let config = FcLinkConfig {
    ///     publisher: LinkPublisherConfig {
    ///         record_interval: Some(Duration::from_millis(200)),
    ///         ..LinkPublisherConfig::default()
    ///     },
    ///     max_writes_per_poll: 2,
    ///     ..FcLinkConfig::default()
    /// };
    /// let (_poller, _telemetry) = link.wire_with(rx, tx, config);
    /// ```
    pub fn wire_with<R: LinkRx, T: LinkTx>(
        &mut self,
        rx: R,
        tx: T,
        config: FcLinkConfig,
    ) -> (FcPoller<'_, R>, LinkPublisher<'_, T>) {
        stack(&mut self.requests, &mut self.replies, None, rx, tx, config)
    }

    /// As [`wire_with`](Self::wire_with), plus a [`LinkCommander`] fed by the link's
    /// [`Control`] messages, for a vehicle flown from the companion; wrap it in a
    /// [`FailsafeCommander`](stable_flight::commanders::FailsafeCommander).
    ///
    /// # Panics
    ///
    /// If either per-poll limit is zero.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight_link::fc::{FcLink, FcLinkConfig};
    /// use stable_flight_link::loopback::Loopback;
    /// use stable_flight_link::stable_flight::commanders::{
    ///     FailsafeCommander, FailsafeConfig, HoldCommander,
    /// };
    ///
    /// let loopback = Loopback::<16>::new();
    /// let (_companion, fc_end) = loopback.split();
    /// let (rx, tx) = fc_end.into_halves();
    /// let mut link = FcLink::<32, 16>::new();
    /// let (_poller, _telemetry, link_commander) =
    ///     link.wire_linked(rx, tx, FcLinkConfig::default());
    /// let hold = HoldCommander::default();
    /// let commander = FailsafeCommander::new(link_commander, hold, FailsafeConfig::default())?;
    /// # Ok::<(), stable_flight_link::stable_flight::controllers::InvalidConfig>(())
    /// ```
    pub fn wire_linked<R: LinkRx, T: LinkTx>(
        &mut self,
        rx: R,
        tx: T,
        config: FcLinkConfig,
    ) -> (FcPoller<'_, R>, LinkPublisher<'_, T>, LinkCommander<'_>) {
        let (to_commander, from_router) = self.controls.split();
        let (poller, publisher) = stack(
            &mut self.requests,
            &mut self.replies,
            Some(to_commander),
            rx,
            tx,
            config,
        );
        (poller, publisher, LinkCommander::new(from_router))
    }
}

/// The poller and publisher over `requests` and `replies`, routing control messages to
/// `control` when given.
fn stack<'a, R: LinkRx, T: LinkTx, const REQUESTS: usize, const REPLIES: usize>(
    requests: &'a mut Queue<ParamRequest, REQUESTS>,
    replies: &'a mut Queue<ParamValue, REPLIES>,
    control: Option<Producer<'a, Control>>,
    rx: R,
    tx: T,
    config: FcLinkConfig,
) -> (FcPoller<'a, R>, LinkPublisher<'a, T>) {
    let (to_server, from_router) = requests.split();
    let (to_publisher, from_server) = replies.split();
    let mut router = UplinkRouter::new(rx, config.max_frames_per_poll).with_params(to_server);
    if let Some(control) = control {
        router = router.with_control(control);
    }
    let poller = FcPoller {
        router,
        server: ParamServer::new(from_router, to_publisher, config.max_writes_per_poll),
    };
    let publisher = LinkPublisher::new(tx, config.publisher).with_param_replies(from_server);
    (poller, publisher)
}

impl<const REQUESTS: usize, const REPLIES: usize, const CONTROLS: usize> Default
    for FcLink<REQUESTS, REPLIES, CONTROLS>
{
    fn default() -> Self {
        Self::new()
    }
}

/// The between-ticks half of an [`FcLink`]: reads the link (queuing control messages for the
/// [`LinkCommander`], if wired), then serves params.
pub struct FcPoller<'a, R> {
    router: UplinkRouter<'a, R>,
    server: ParamServer<'a>,
}

impl<R: LinkRx> FcPoller<'_, R> {
    /// Routes waiting uplinks, then applies param requests to `params`: the hook
    /// `RealTimeRunner::run_with` runs between ticks.
    ///
    /// # Errors
    ///
    /// The link failed to receive; param requests already queued are still served.
    pub fn poll(&mut self, params: &mut impl Params) -> Result<RouteStats, R::Error> {
        let routed = self.router.poll();
        self.server.poll(params);
        routed
    }
}
