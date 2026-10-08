//! Fixed-rate scheduling for telemetry streams driven by the flight loop's `dt`.
//!
//! A tick rarely divides a stream's period, so [`Every`] carries the leftover time into the
//! next period instead of discarding it: a 10 Hz stream on a 60 Hz loop sends 10 times per
//! second, not 60 / 7 ≈ 8.6. A gap longer than a period (a stalled tick) fires once and
//! drops the backlog rather than bursting.

use std::time::Duration;

/// Fires on the first call, then once per `interval` of accumulated `dt`.
#[derive(Debug)]
pub(crate) struct Every {
    interval: Duration,
    /// Time accumulated toward the next firing; always below `interval` between calls.
    elapsed: Duration,
}

impl Every {
    /// Fires on the first call, then once per `interval`.
    pub(crate) fn new(interval: Duration) -> Self {
        Every {
            interval,
            elapsed: interval,
        }
    }

    /// Advances by `dt`. Returns `true` when a period has elapsed since the last firing.
    pub(crate) fn ready(&mut self, dt: Duration) -> bool {
        self.elapsed += dt;
        if self.elapsed < self.interval {
            return false;
        }
        self.elapsed -= self.interval;
        if self.elapsed >= self.interval {
            self.elapsed = Duration::ZERO;
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use assert2::check;

    use super::*;

    #[test]
    fn fires_first_then_once_per_interval_without_drift() {
        let ms = Duration::from_millis;
        // (interval, dt, ticks, expected firings: the first call plus one per interval)
        let cases = [
            (ms(100), Duration::from_nanos(16_666_667), 60, 11), // 60 Hz loop, 10 Hz stream
            (ms(100), ms(4), 250, 11),                           // 250 Hz loop
            (ms(100), ms(30), 10, 4),                            // ticks 1, 4, 7, 10
            (ms(100), ms(100), 5, 5),                            // dt equals interval
            (ms(100), Duration::ZERO, 100, 1),                   // only the first call
            (ms(100), ms(1000), 3, 3),                           // stalled ticks: once each
        ];

        for (interval, dt, ticks, expected) in cases {
            let mut every = Every::new(interval);
            let fired = (0..ticks).filter(|_| every.ready(dt)).count();
            check!(fired == expected, "interval {interval:?}, dt {dt:?}");
        }
    }

    #[test]
    fn stalled_tick_drops_the_backlog() {
        let mut every = Every::new(Duration::from_millis(100));
        check!(every.ready(Duration::ZERO)); // the first call
        check!(every.ready(Duration::from_millis(350)));
        // Leftover is dropped, not carried: the next 99 ms don't fire.
        check!(!every.ready(Duration::from_millis(99)));
        check!(every.ready(Duration::from_millis(1)));
    }
}
