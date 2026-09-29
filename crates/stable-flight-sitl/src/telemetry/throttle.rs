//! Rate-limited warning for per-tick telemetry failures.
//!
//! Telemetry runs inside the flight loop, so a persistent failure (log file deleted, UDP peer
//! gone) would otherwise emit a warning every tick — 250 lines/s at a typical loop rate.

use std::fmt::Display;

use log::warn;

/// Warn on every `interval`-th failure, starting with the first.
const DEFAULT_INTERVAL: u32 = 250;

/// Counts failures and reports only the first of every `interval` of them.
#[derive(Debug)]
pub(crate) struct ErrorThrottle {
    interval: u32,
    suppressed: u32,
}

impl Default for ErrorThrottle {
    fn default() -> Self {
        Self::with_interval(DEFAULT_INTERVAL)
    }
}

impl ErrorThrottle {
    pub(crate) fn with_interval(interval: u32) -> Self {
        ErrorThrottle {
            interval: interval.max(1),
            suppressed: 0,
        }
    }

    /// Records a failure and logs it at `warn` if it is due. Returns whether it was logged.
    pub(crate) fn warn(&mut self, context: &str, error: impl Display) -> bool {
        let due = self.suppressed == 0;
        if due {
            warn!("{context}: {error}");
        }
        self.suppressed = (self.suppressed + 1) % self.interval;
        due
    }
}

#[cfg(test)]
mod tests {
    use assert2::check;

    use super::*;

    #[test]
    fn logs_first_failure_then_one_per_interval() {
        // (interval, failures, expected logged indices)
        let cases: [(u32, usize, &[usize]); 4] = [
            (3, 7, &[0, 3, 6]),
            (1, 3, &[0, 1, 2]),
            (0, 2, &[0, 1]), // zero clamps to one: never suppress
            (250, 2, &[0]),
        ];

        for (interval, failures, expected) in cases {
            let mut throttle = ErrorThrottle::with_interval(interval);
            let logged: Vec<usize> = (0..failures)
                .filter(|_| throttle.warn("ctx", "boom"))
                .collect();
            check!(logged == expected, "interval {interval}");
        }
    }
}
