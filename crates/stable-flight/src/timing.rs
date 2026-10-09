//! Fixed-rate scheduling driven by the flight loop's `dt`, for streams (telemetry, heartbeats)
//! that run slower than the loop.
//!
//! A tick rarely divides a stream's period, so [`Every`] carries the leftover time into the
//! next period instead of discarding it: a 10 Hz stream on a 60 Hz loop sends 10 times per
//! second, not 60 / 7 ≈ 8.6. A gap longer than a period (a stalled tick) fires once and
//! drops the backlog rather than bursting.

use core::time::Duration;

#[cfg(test)]
mod tests;

/// Fires on the first call, then once per `interval` of accumulated `dt`.
///
/// # Examples
///
/// ```
/// use core::time::Duration;
/// use stable_flight::timing::Every;
///
/// // A 10 Hz stream on a 250 Hz loop.
/// let mut heartbeat = Every::new(Duration::from_millis(100));
/// let dt = Duration::from_millis(4);
/// let fired = (0..250).filter(|_| heartbeat.ready(dt)).count();
/// assert_eq!(fired, 11); // the first call, then 10 per second
/// ```
#[derive(Debug, Clone)]
pub struct Every {
    interval: Duration,
    /// Time accumulated toward the next firing; always below `interval` between calls.
    elapsed: Duration,
}

impl Every {
    /// Fires on the first call, then once per `interval`.
    pub fn new(interval: Duration) -> Self {
        Every {
            interval,
            elapsed: interval,
        }
    }

    /// Advances by `dt`. Returns `true` when a period has elapsed since the last firing.
    pub fn ready(&mut self, dt: Duration) -> bool {
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
