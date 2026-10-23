//! Writing the link: telemetry and param replies, at configured rates.

use core::time::Duration;

use heapless::spsc::Consumer;
use stable_flight::timing::Every;
use stable_flight::{
    ControlMode, FailsafeState, ModeRefusal, Saturation, TelemetryPublisher, TelemetryRecord,
};

use crate::{Downlink, LinkTx, MAX_FRAME_LEN, Output, ParamValue, Record, SendMessage, StatusText};

#[cfg(test)]
mod tests;

/// How often [`LinkPublisher`] sends each stream.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkPublisherConfig {
    /// [`Downlink::Heartbeat`] period: how quickly the companion notices a lost flight
    /// controller.
    pub heartbeat_interval: Duration,
    /// [`Downlink::State`] period, each followed by a [`Downlink::Output`].
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

/// A [`TelemetryPublisher`] that sends [`Downlink`]s over a [`LinkTx`]: heartbeat, state (each
/// followed by an output), and optional records at their [`LinkPublisherConfig`] rates, plus the
/// [`ParamServer`](crate::params::ParamServer)'s replies.
///
/// [`Downlink::Output`] carries the latest tick's thrust and the union of every tick's
/// [`Saturation`] since the previous output, so a one-tick saturation between sends isn't lost.
///
/// A stream is checked for being due before it's encoded, so a tick with nothing due costs
/// almost nothing. One frame buffer, kept in the struct, serves every send. Send failures are
/// counted ([`send_errors`](Self::send_errors)), never returned: telemetry is fire-and-forget.
///
/// [`Downlink::Heartbeat`] reports the command's arm state and [`mode`](stable_flight::ControlInput::mode);
/// `link_ok` is `false` while a failsafe flies its fallback (the record's
/// [`failsafe`](stable_flight::TelemetryRecord::failsafe)), and `true` otherwise. Each change
/// of failsafe state is announced once as [`Downlink::Status`] text for the pilot ("Link quiet:
/// holding position", "Link back: move the sticks to take control", "Flying from the link"),
/// and each new [`ModeRefusal`] in the record's [`flight_mode`](TelemetryRecord::flight_mode)
/// the same way ("Transition refused: too low"), except `Unsupported`, the airframe's own.
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
    /// The failsafe state last announced, to send [`Downlink::Status`] when it changes.
    failsafe: Option<FailsafeState>,
    /// The mode refusal last seen, to send [`Downlink::Status`] when a new one appears.
    refusal: Option<ModeRefusal>,
    /// Limits hit since the last [`Downlink::Output`].
    saturation: Saturation,
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
            failsafe: None,
            refusal: None,
            saturation: Saturation::default(),
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
        self.saturation = self.saturation.merge(record.saturation);
        if record.failsafe != self.failsafe {
            self.failsafe = record.failsafe;
            if let Some(text) = record
                .failsafe
                .and_then(|state| failsafe_status(state, record.command.input.mode()))
            {
                self.send(&Downlink::Status(text));
            }
        }
        let refusal = record.flight_mode.refusal;
        if refusal != self.refusal {
            self.refusal = refusal;
            if let Some(text) = refusal.and_then(refusal_status) {
                self.send(&Downlink::Status(text));
            }
        }
        if self.heartbeat.ready(record.dt) {
            self.send(&Downlink::Heartbeat {
                armed: record.command.arm,
                mode: record.command.input.mode(),
                flight_mode: record.flight_mode,
                link_ok: record.failsafe != Some(FailsafeState::Fallback),
            });
        }
        if self.state.ready(record.dt) {
            self.send(&Downlink::State(record.state.clone()));
            let saturation = core::mem::take(&mut self.saturation);
            self.send(&Downlink::Output(Output {
                thrust: record.control.thrust,
                saturation,
            }));
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

/// What a ground station should tell the pilot when the failsafe enters `state`, flying `mode`.
fn failsafe_status(state: FailsafeState, mode: ControlMode) -> Option<StatusText> {
    let text = match state {
        FailsafeState::Fallback if mode == ControlMode::Land => "Link quiet: landing",
        FailsafeState::Fallback => "Link quiet: holding position",
        FailsafeState::AwaitingSticks => "Link back: move the sticks to take control",
        FailsafeState::Primary => "Flying from the link",
        // `FailsafeState` is non-exhaustive: a new state announces nothing until mapped here.
        _ => return None,
    };
    // Each text is well under `MAX_STATUS_LEN`.
    StatusText::try_from(text).ok()
}

/// What a ground station should tell the pilot when guidance starts refusing a regime for
/// `refusal`. [`ModeRefusal::Unsupported`] announces nothing: it is the airframe's, not the
/// pilot's (a plane's guidance refuses the default `Copter` request from its first tick).
fn refusal_status(refusal: ModeRefusal) -> Option<StatusText> {
    let text = match refusal {
        ModeRefusal::OnGround => "Transition refused: on the ground",
        ModeRefusal::BelowMinAltitude => "Transition refused: too low",
        ModeRefusal::TransitionTimeout => "Transition timed out",
        ModeRefusal::RollError => "Transition aborted: roll error",
        ModeRefusal::BelowPlaneMinAltitude => "Quadchute: below plane minimum height",
        ModeRefusal::Unsupported => return None,
        // `ModeRefusal` is non-exhaustive: a new refusal announces nothing until mapped here.
        _ => return None,
    };
    // Each text is well under `MAX_STATUS_LEN`.
    StatusText::try_from(text).ok()
}
