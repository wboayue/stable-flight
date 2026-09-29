//! Rate limiting for per-tick telemetry failure reports.
//!
//! Telemetry runs inside the flight loop, so a persistent failure (log file deleted, UDP peer
//! gone) would otherwise be reported on every failed send. The throttle is count-based: it
//! reports the first failure of an episode, then one per `interval` consecutive failures, so
//! the time between reports depends on how often a publisher fails. For example, a dead
//! MAVLink link sending ~50 messages/s reports about every 5 s at the default interval.
//! A success ends the episode, so the next failure is reported immediately.
//!
//! The throttle only decides; callers log, so each report carries the caller's module as its
//! log target and the throttle has no logging dependency.

/// Report one failure in this many consecutive failures.
const DEFAULT_INTERVAL: u32 = 250;

/// Decides which failures in a run of consecutive failures should be reported.
#[derive(Debug)]
pub(crate) struct ErrorThrottle {
    interval: u32,
    /// Failures since the last report, counting the reported one; 0 when no episode is open.
    since_report: u32,
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
            since_report: 0,
        }
    }

    /// Records a failure. Returns `Some(suppressed)` when this failure should be reported,
    /// where `suppressed` is how many failures were skipped since the previous report.
    pub(crate) fn record(&mut self) -> Option<u32> {
        if self.since_report == 0 || self.since_report >= self.interval {
            let suppressed = self.since_report.saturating_sub(1);
            self.since_report = 1;
            return Some(suppressed);
        }
        self.since_report += 1;
        None
    }

    /// Records a success, ending the current failure episode.
    pub(crate) fn reset(&mut self) {
        self.since_report = 0;
    }
}

#[cfg(test)]
mod tests {
    use assert2::check;

    use super::*;

    /// (interval, events: F = failure / S = success, expected (event index, suppressed) reports)
    type Case = (u32, &'static str, &'static [(usize, u32)]);

    #[test]
    fn reports_first_failure_then_one_per_interval_and_restarts_after_success() {
        let cases: [Case; 6] = [
            (3, "FFFFFFF", &[(0, 0), (3, 2), (6, 2)]),
            (1, "FFF", &[(0, 0), (1, 0), (2, 0)]),
            (0, "FF", &[(0, 0), (1, 0)]), // zero clamps to one: never suppress
            (250, "FF", &[(0, 0)]),
            (3, "FFSF", &[(0, 0), (3, 0)]), // success ends the episode: next failure reports
            (3, "SSFS", &[(2, 0)]),
        ];

        for (interval, events, expected) in cases {
            let mut throttle = ErrorThrottle::with_interval(interval);
            let mut reports = Vec::new();
            for (i, event) in events.chars().enumerate() {
                match event {
                    'F' => {
                        if let Some(suppressed) = throttle.record() {
                            reports.push((i, suppressed));
                        }
                    }
                    _ => throttle.reset(),
                }
            }
            check!(reports == expected, "interval {interval}, events {events}");
        }
    }
}
