//! One pass of the bridge's I/O: see [`Pump`].

use std::fmt::{self, Display};
use std::io::ErrorKind;
use std::time::{Duration, Instant};

use log::{debug, warn};
use mavlink::common::MavMessage;
use mavlink::error::MessageReadError;
use mavlink::{MavConnection, MavHeader};
use stable_flight_link::{
    Downlink, LinkError, LinkRx, LinkTx, MAX_FRAME_LEN, RecvMessage, SendMessage,
};

use crate::bridge::Bridge;
use crate::outbox::Outbox;

#[cfg(test)]
mod tests;

/// Messages read from each side per pass, so neither side starves the other.
const MAX_READS_PER_PASS: usize = 64;
/// Shortest interval between reports of a repeating failure.
const ERROR_REPORT_INTERVAL: Duration = Duration::from_secs(1);

/// The ground station connection, as `mavlink::connect` returns it. `Send + Sync` though the
/// pump is single-threaded: an event-driven wait (#216) reads it from its own thread.
pub(crate) type GcsConnection = Box<dyn MavConnection<MavMessage> + Send + Sync>;

/// What one [`Pump::step`] did.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Activity {
    /// Link messages received and handled.
    pub(crate) downlinks: usize,
    /// MAVLink messages received and handled.
    pub(crate) mavlink: usize,
    /// Link frames that didn't decode, and MAVLink messages that didn't parse: skipped.
    pub(crate) dropped: usize,
    /// Receive or send failures on either side.
    pub(crate) failures: usize,
}

impl Activity {
    /// Something arrived from either side.
    pub(crate) fn received(&self) -> bool {
        self.downlinks + self.mavlink > 0
    }
}

/// Moves messages between the flight controller's link and the ground station through a
/// [`Bridge`]. Each [`step`](Self::step):
///
/// 1. reads up to [`MAX_READS_PER_PASS`] link frames: a frame that doesn't decode is skipped;
///    a transport error ends the link's reads for this pass;
/// 2. reads up to as many MAVLink messages: `WouldBlock` ends them, a message that doesn't
///    parse (another dialect, a bad CRC) is skipped, another error ends them;
/// 3. polls the bridge for what is due;
/// 4. sends everything queued, the MAVLink as the flight controller ([`Bridge::header`]); a
///    failed send is dropped.
///
/// Failures are logged at most once a second per side, counting the ones skipped.
pub(crate) struct Pump<R, T> {
    link_rx: R,
    link_tx: T,
    gcs: GcsConnection,
    header: MavHeader,
    bridge: Bridge,
    out: Outbox,
    buf: [u8; MAX_FRAME_LEN],
    link_errors: ErrorReport,
    gcs_errors: ErrorReport,
}

impl<R, T> Pump<R, T>
where
    R: LinkRx<Error: Display>,
    T: LinkTx<Error: Display>,
{
    /// A pump between the link halves and `gcs`, through `bridge`.
    pub(crate) fn new(link_rx: R, link_tx: T, gcs: GcsConnection, bridge: Bridge) -> Self {
        Self {
            link_rx,
            link_tx,
            gcs,
            header: bridge.header(),
            bridge,
            out: Outbox::default(),
            buf: [0; MAX_FRAME_LEN],
            link_errors: ErrorReport::default(),
            gcs_errors: ErrorReport::default(),
        }
    }

    /// One pass at `now`; see [`Pump`].
    pub(crate) fn step(&mut self, now: Instant) -> Activity {
        let mut activity = Activity::default();
        self.read_link(now, &mut activity);
        self.read_gcs(now, &mut activity);
        self.bridge.poll(now, &mut self.out);
        self.flush(now, &mut activity);
        activity
    }

    fn read_link(&mut self, now: Instant, activity: &mut Activity) {
        for _ in 0..MAX_READS_PER_PASS {
            match self.link_rx.recv_message::<Downlink>(&mut self.buf) {
                Ok(Some(msg)) => {
                    activity.downlinks += 1;
                    self.bridge.on_downlink(&msg, now, &mut self.out);
                }
                Ok(None) => break,
                Err(LinkError::Decode(e)) => {
                    activity.dropped += 1;
                    debug!("dropped link frame: {e}");
                }
                Err(e) => {
                    activity.failures += 1;
                    self.link_errors
                        .report(now, format_args!("link receive failed: {e}"));
                    break;
                }
            }
        }
    }

    fn read_gcs(&mut self, now: Instant, activity: &mut Activity) {
        for _ in 0..MAX_READS_PER_PASS {
            match self.gcs.try_recv() {
                Ok((sender, msg)) => {
                    activity.mavlink += 1;
                    self.bridge.on_mavlink(&sender, &msg, now, &mut self.out);
                }
                Err(MessageReadError::Io(e)) if e.kind() == ErrorKind::WouldBlock => break,
                Err(MessageReadError::Parse(e)) => {
                    activity.dropped += 1;
                    debug!("dropped MAVLink message: {e}");
                }
                Err(e) => {
                    activity.failures += 1;
                    self.gcs_errors
                        .report(now, format_args!("ground station receive failed: {e}"));
                    break;
                }
            }
        }
    }

    fn flush(&mut self, now: Instant, activity: &mut Activity) {
        for msg in self.out.to_fc.drain(..) {
            if let Err(e) = self.link_tx.send_message(&msg, &mut self.buf) {
                activity.failures += 1;
                self.link_errors
                    .report(now, format_args!("link send failed: {e}"));
            }
        }
        for msg in self.out.to_gcs.drain(..) {
            if let Err(e) = self.gcs.send(&self.header, &msg) {
                activity.failures += 1;
                self.gcs_errors
                    .report(now, format_args!("ground station send failed: {e}"));
            }
        }
    }
}

/// Logs a failure at most once per [`ERROR_REPORT_INTERVAL`], counting the ones it skips, so
/// a dead peer doesn't flood the log.
#[derive(Default)]
struct ErrorReport {
    last: Option<Instant>,
    suppressed: u32,
}

impl ErrorReport {
    /// Logs `message` unless a report went out within the interval; it is formatted only when
    /// logged.
    fn report(&mut self, now: Instant, message: fmt::Arguments<'_>) {
        if self
            .last
            .is_some_and(|last| now.saturating_duration_since(last) < ERROR_REPORT_INTERVAL)
        {
            self.suppressed = self.suppressed.saturating_add(1);
            return;
        }
        warn!(
            "{message} ({} earlier failures suppressed)",
            self.suppressed
        );
        self.last = Some(now);
        self.suppressed = 0;
    }
}
