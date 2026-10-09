//! Writing the link: telemetry and param replies, at configured rates.

use core::time::Duration;

use heapless::spsc::Consumer;
use stable_flight::timing::Every;
use stable_flight::{TelemetryPublisher, TelemetryRecord};

use crate::{Downlink, LinkTx, MAX_FRAME_LEN, ParamValue, Record, SendMessage};

#[cfg(test)]
mod tests;

/// How often [`LinkPublisher`] sends each stream.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkPublisherConfig {
    /// [`Downlink::Heartbeat`] period: how quickly the companion notices a lost flight
    /// controller.
    pub heartbeat_interval: Duration,
    /// [`Downlink::State`] period.
    pub state_interval: Duration,
    /// [`Downlink::Record`] period, or `None` for no records. A record is the largest message
    /// (about 225 bytes); keep it slow.
    pub record_interval: Option<Duration>,
    /// Most [`Downlink::Param`] replies sent per tick, so a listing doesn't crowd out
    /// telemetry.
    pub max_params_per_tick: usize,
}

impl Default for LinkPublisherConfig {
    /// Heartbeat 5 Hz, state 50 Hz, no records, 2 params per tick.
    fn default() -> Self {
        Self {
            heartbeat_interval: Duration::from_millis(200),
            state_interval: Duration::from_millis(20),
            record_interval: None,
            max_params_per_tick: 2,
        }
    }
}

/// A [`TelemetryPublisher`] that sends [`Downlink`]s over a [`LinkTx`]: heartbeat, state, and
/// optional records at their [`LinkPublisherConfig`] rates, plus the
/// [`ParamServer`](crate::params::ParamServer)'s replies.
///
/// A stream is checked for being due before it's encoded, so a tick with nothing due costs
/// almost nothing. One frame buffer, kept in the struct, serves every send. Send failures are
/// counted ([`send_errors`](Self::send_errors)), never returned: telemetry is fire-and-forget.
///
/// [`Downlink::Heartbeat`] reports the command's arm state and [`mode`](stable_flight::ControlInput::mode);
/// until link staleness is tracked, `link_ok` is always `true`.
///
/// # Examples
///
/// ```
/// use stable_flight_link::loopback::Loopback;
/// use stable_flight_link::publisher::{LinkPublisher, LinkPublisherConfig};
///
/// let loopback = Loopback::<16>::new();
/// let (fc, _companion) = loopback.split();
/// let (_, tx) = fc.into_halves();
/// let telemetry = LinkPublisher::new(tx, LinkPublisherConfig::default());
/// // Hand `telemetry` to a `FlightLoop`; add `.with_param_replies(consumer)` to serve params.
/// ```
pub struct LinkPublisher<'a, T> {
    tx: T,
    buf: [u8; MAX_FRAME_LEN],
    heartbeat: Every,
    state: Every,
    record: Option<Every>,
    replies: Option<Consumer<'a, ParamValue>>,
    max_params_per_tick: usize,
    send_errors: u32,
}

impl<'a, T: LinkTx> LinkPublisher<'a, T> {
    /// A publisher sending on `tx` at `config`'s rates. Param replies need
    /// [`with_param_replies`](Self::with_param_replies).
    pub fn new(tx: T, config: LinkPublisherConfig) -> Self {
        Self {
            tx,
            buf: [0; MAX_FRAME_LEN],
            heartbeat: Every::new(config.heartbeat_interval),
            state: Every::new(config.state_interval),
            record: config.record_interval.map(Every::new),
            replies: None,
            max_params_per_tick: config.max_params_per_tick,
            send_errors: 0,
        }
    }

    /// Sends the [`ParamServer`](crate::params::ParamServer)'s replies from `replies`.
    pub fn with_param_replies(mut self, replies: Consumer<'a, ParamValue>) -> Self {
        self.replies = Some(replies);
        self
    }

    /// Sends that failed (transport or encode) since construction, saturating.
    pub fn send_errors(&self) -> u32 {
        self.send_errors
    }

    fn send(&mut self, msg: &Downlink) {
        if self.tx.send_message(msg, &mut self.buf).is_err() {
            self.send_errors = self.send_errors.saturating_add(1);
        }
    }
}

impl<T: LinkTx> TelemetryPublisher for LinkPublisher<'_, T> {
    fn publish(&mut self, record: &TelemetryRecord) {
        if self.heartbeat.ready(record.dt) {
            self.send(&Downlink::Heartbeat {
                armed: record.command.arm,
                mode: record.command.input.mode(),
                link_ok: true,
            });
        }
        if self.state.ready(record.dt) {
            self.send(&Downlink::State(record.state.clone()));
        }
        if self
            .record
            .as_mut()
            .is_some_and(|every| every.ready(record.dt))
        {
            self.send(&Downlink::Record(Record::from(record)));
        }
        for _ in 0..self.max_params_per_tick {
            let Some(reply) = self.replies.as_mut().and_then(Consumer::dequeue) else {
                break;
            };
            self.send(&Downlink::Param(reply));
        }
    }
}
