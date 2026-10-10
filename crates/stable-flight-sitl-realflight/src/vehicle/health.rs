//! Connection bookkeeping for [`RealFlightVehicle`](super::RealFlightVehicle): when to retry a
//! lost connection, and when to log a failure. Time is the tick's nominal `dt`, so there is no
//! clock here and the tests are deterministic.

#[cfg(test)]
mod tests;

/// Wait after a lost connection before the first reconnect attempt.
pub(super) const INITIAL_BACKOFF_S: f32 = 0.5;
/// Longest wait between reconnect attempts; each failed attempt doubles the wait up to this.
pub(super) const MAX_BACKOFF_S: f32 = 4.0;
/// Shortest gap between two logged failures.
pub(super) const REPORT_INTERVAL_S: f32 = 1.0;

/// Whether the connection is usable, and if not, when to try again.
#[derive(Debug, Default)]
pub(super) struct LinkHealth {
    down: Option<Down>,
    report: ErrorReport,
}

#[derive(Debug)]
struct Down {
    /// Time left until the next reconnect attempt.
    retry_in_s: f32,
    /// The wait that `retry_in_s` started from.
    backoff_s: f32,
    /// Reconnect attempts that failed since the connection was lost.
    failed_attempts: u32,
}

impl LinkHealth {
    /// Advances the retry and report timers by one tick.
    pub(super) fn advance(&mut self, dt_s: f32) {
        if let Some(down) = &mut self.down {
            down.retry_in_s -= dt_s;
        }
        self.report.advance(dt_s);
    }

    /// Whether the connection is lost and waiting for a reconnect.
    pub(super) fn is_down(&self) -> bool {
        self.down.is_some()
    }

    /// Whether a reconnect attempt is due.
    pub(super) fn retry_due(&self) -> bool {
        self.down
            .as_ref()
            .is_some_and(|down| down.retry_in_s <= 0.0)
    }

    /// Marks the connection lost; the first attempt is due after [`INITIAL_BACKOFF_S`].
    pub(super) fn connection_lost(&mut self) {
        self.down = Some(Down {
            retry_in_s: INITIAL_BACKOFF_S,
            backoff_s: INITIAL_BACKOFF_S,
            failed_attempts: 0,
        });
    }

    /// Records a failed attempt and doubles the wait, up to [`MAX_BACKOFF_S`]. Returns the wait
    /// until the next attempt (zero, and no effect, while connected).
    pub(super) fn reconnect_failed(&mut self) -> f32 {
        let Some(down) = &mut self.down else {
            return 0.0;
        };
        down.backoff_s = (down.backoff_s * 2.0).min(MAX_BACKOFF_S);
        down.retry_in_s = down.backoff_s;
        down.failed_attempts += 1;
        down.backoff_s
    }

    /// Marks the connection usable again. Returns how many attempts failed before this one.
    pub(super) fn reconnected(&mut self) -> u32 {
        self.down.take().map_or(0, |down| down.failed_attempts)
    }

    /// Whether to log a failure now: `Some(n)` with the `n` failures skipped since the last
    /// logged one, or `None` to skip this one.
    pub(super) fn report(&mut self) -> Option<u32> {
        self.report.report()
    }
}

/// Logs a failure at most once per [`REPORT_INTERVAL_S`], counting the ones it skips, so a dead
/// connection doesn't flood the log at the loop rate. The bridge's `ErrorReport`
/// (`stable-flight-bridge`) does the same against a wall clock.
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
