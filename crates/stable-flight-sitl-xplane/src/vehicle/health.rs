//! Stream bookkeeping for [`XPlaneVehicle`](super::XPlaneVehicle): when X-Plane has gone
//! quiet, when to resubscribe, and when to log a failure. Time is the tick's nominal `dt`, so
//! there is no clock here and the tests are deterministic.

use core::fmt;

use log::error;

#[cfg(test)]
mod tests;

/// No frame for this long reads as a silence: X-Plane stalled, restarted, reloaded, or gone.
/// Several frames at X-Plane's slowest useful rate (60 fps); a reload is 2–11 s of silence
/// (xplane-bridge `design.md`, Spike findings: Reloads).
pub(super) const SILENCE_S: f32 = 0.5;
/// Shortest gap between two resubscribes, as [`wait_ready`](crate::wait_ready)'s.
pub(super) const RESUBSCRIBE_INTERVAL_S: f32 = 0.25;
/// Shortest gap between two logged failures.
pub(super) const REPORT_INTERVAL_S: f32 = 1.0;

/// Whether X-Plane is streaming, and when to resubscribe.
#[derive(Debug, Default)]
pub(super) struct Liveness {
    /// Time since the last frame.
    since_frame_s: f32,
    /// Time since the last resubscribe; `None` before the first.
    since_resubscribe_s: Option<f32>,
    /// A silence was reported and hasn't ended.
    silent: bool,
    errors: ErrorReport,
}

impl Liveness {
    /// Advances the timers by one tick.
    pub(super) fn advance(&mut self, dt_s: f32) {
        self.since_frame_s += dt_s;
        if let Some(since) = &mut self.since_resubscribe_s {
            *since += dt_s;
        }
        self.errors.advance(dt_s);
    }

    /// Records a frame. Returns how long the silence it ends lasted, if it ends one.
    pub(super) fn frame_arrived(&mut self) -> Option<f32> {
        let silence_s = core::mem::take(&mut self.since_frame_s);
        core::mem::take(&mut self.silent).then_some(silence_s)
    }

    /// Whether no frame arrived for [`SILENCE_S`].
    pub(super) fn is_silent(&self) -> bool {
        self.since_frame_s >= SILENCE_S
    }

    /// True once when a silence starts, to log it.
    pub(super) fn silence_started(&mut self) -> bool {
        if !self.is_silent() || self.silent {
            return false;
        }
        self.silent = true;
        true
    }

    /// Whether to resubscribe now: `needed`, and none within [`RESUBSCRIBE_INTERVAL_S`].
    /// Counts the resubscribe when it says yes.
    pub(super) fn resubscribe_due(&mut self, needed: bool) -> bool {
        if !needed
            || self
                .since_resubscribe_s
                .is_some_and(|since| since < RESUBSCRIBE_INTERVAL_S)
        {
            return false;
        }
        self.since_resubscribe_s = Some(0.0);
        true
    }

    /// Logs a failure, unless one was logged within [`REPORT_INTERVAL_S`]; `message` is
    /// formatted only when logged. Returns whether it was.
    pub(super) fn report(&mut self, message: fmt::Arguments<'_>) -> bool {
        let Some(suppressed) = self.errors.report() else {
            return false;
        };
        error!("{message} ({suppressed} earlier failures suppressed)");
        true
    }
}

/// Lets a failure through at most once per [`REPORT_INTERVAL_S`], counting the ones it skips,
/// so a dead socket doesn't flood the log at the loop rate. `stable-flight-sitl-realflight`'s
/// `LinkHealth` does the same.
#[derive(Debug, Default)]
struct ErrorReport {
    /// Time since the last logged failure; `None` before the first.
    since_last_s: Option<f32>,
    suppressed: u32,
}

impl ErrorReport {
    fn advance(&mut self, dt_s: f32) {
        if let Some(since) = &mut self.since_last_s {
            *since += dt_s;
        }
    }

    /// `Some(n)` to log this failure, with the `n` skipped since the last logged one; `None` to
    /// skip it.
    fn report(&mut self) -> Option<u32> {
        if self
            .since_last_s
            .is_some_and(|since| since < REPORT_INTERVAL_S)
        {
            self.suppressed = self.suppressed.saturating_add(1);
            return None;
        }
        self.since_last_s = Some(0.0);
        Some(core::mem::take(&mut self.suppressed))
    }
}
