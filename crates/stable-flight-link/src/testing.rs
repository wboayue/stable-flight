//! The flight controller's side of the link, wired for tests (`testing` feature): the param
//! queues, [`UplinkRouter`], [`ParamServer`], and [`LinkPublisher`], as
//! `copter-tune-realflight` wires them, so a test drives a real flight-controller stack from the
//! companion's end.
//!
//! ```
//! use stable_flight_link::loopback::Loopback;
//! use stable_flight_link::stable_flight::params::Params;
//! use stable_flight_link::stable_flight::stabilizers::copter::{
//!     CopterStabilizer, CopterStabilizerConfig,
//! };
//! use stable_flight_link::testing::FcLink;
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

use heapless::spsc::Queue;
use stable_flight::params::Params;

use crate::params::ParamServer;
use crate::publisher::{LinkPublisher, LinkPublisherConfig};
use crate::router::{RouteStats, UplinkRouter};
use crate::{LinkRx, LinkTx, ParamRequest, ParamValue};

/// Uplink frames the router decodes per poll.
const MAX_FRAMES_PER_POLL: usize = 8;
/// Param writes the server applies per poll.
const MAX_WRITES_PER_POLL: usize = 4;

/// Storage for the flight controller's param queues: up to `REQUESTS` pending requests and
/// `REPLIES` pending replies. [`wire`](Self::wire) borrows it to build the stack.
pub struct FcLink<const REQUESTS: usize, const REPLIES: usize> {
    requests: Queue<ParamRequest, REQUESTS>,
    replies: Queue<ParamValue, REPLIES>,
}

impl<const REQUESTS: usize, const REPLIES: usize> FcLink<REQUESTS, REPLIES> {
    /// Empty queues.
    pub const fn new() -> Self {
        Self {
            requests: Queue::new(),
            replies: Queue::new(),
        }
    }

    /// The stack over the flight controller's link halves: an [`FcPoller`] to call between
    /// ticks, and the [`LinkPublisher`] (default config, param replies attached) to use as the
    /// flight loop's telemetry.
    pub fn wire<R: LinkRx, T: LinkTx>(
        &mut self,
        rx: R,
        tx: T,
    ) -> (FcPoller<'_, R>, LinkPublisher<'_, T>) {
        let (to_server, from_router) = self.requests.split();
        let (to_publisher, from_server) = self.replies.split();
        let poller = FcPoller {
            router: UplinkRouter::new(rx, MAX_FRAMES_PER_POLL).with_params(to_server),
            server: ParamServer::new(from_router, to_publisher, MAX_WRITES_PER_POLL),
        };
        let publisher =
            LinkPublisher::new(tx, LinkPublisherConfig::default()).with_param_replies(from_server);
        (poller, publisher)
    }
}

impl<const REQUESTS: usize, const REPLIES: usize> Default for FcLink<REQUESTS, REPLIES> {
    fn default() -> Self {
        Self::new()
    }
}

/// The between-ticks half of an [`FcLink`]: reads the link, then serves params.
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
