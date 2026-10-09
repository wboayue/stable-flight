//! A periodic timer on a caller-supplied clock: see [`Interval`].

use std::time::{Duration, Instant};

#[cfg(test)]
mod tests;

/// Fires every `period` once started, on the `now` its caller passes in. Drift-free: the next
/// firing is one period after the due time, not after the poll that noticed it, so polling
/// jitter doesn't stretch the rate. A poll more than a period late fires once and reschedules
/// from `now`, rather than bursting to catch up.
#[derive(Debug)]
pub(crate) struct Interval {
    period: Duration,
    /// When it next fires; `None` until started.
    next: Option<Instant>,
}

impl Interval {
    /// A stopped timer with `period`.
    pub(crate) fn new(period: Duration) -> Self {
        Self { period, next: None }
    }

    /// Fires at `now`, then every period from it.
    pub(crate) fn start(&mut self, now: Instant) {
        self.next = Some(now);
    }

    /// Whether it fires at `now`; if so, schedules the next firing.
    pub(crate) fn due(&mut self, now: Instant) -> bool {
        let Some(next) = self.next.filter(|&next| now >= next) else {
            return false;
        };
        let after = next + self.period;
        self.next = Some(if after > now {
            after
        } else {
            now + self.period
        });
        true
    }
}
