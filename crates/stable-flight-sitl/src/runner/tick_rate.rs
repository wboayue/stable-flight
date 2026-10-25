//! [`TickRate`]: whether a [`RealTimeRunner`](super::RealTimeRunner) keeps its rate.

use std::collections::VecDeque;
use std::time::Duration;

use thiserror::Error;

use super::RunStats;

#[cfg(test)]
mod tests;

/// Returned by [`TickRate::new`] for a setting out of range, naming it.
#[derive(Debug, Clone, Copy, PartialEq, Error)]
#[error("invalid TickRate {field}: must be {requirement}")]
pub struct InvalidTickRate {
    /// The setting out of range.
    pub field: &'static str,
    /// What it must be.
    pub requirement: &'static str,
}

/// Watches the rate a [`RealTimeRunner`](super::RealTimeRunner) achieves, over the last
/// `window` of ticks, and reports it slow while that rate stays under `min_share` of nominal.
///
/// The runner gives every tick the nominal `dt`, so while the host runs slow (RealFlight
/// dropping to about 120 Hz for stretches, #438) every `dt`-based clock runs slow in wall
/// time: a step held for its ticks lasts twice as long. Feed it each tick's [`RunStats`] from
/// [`run_with`](super::RealTimeRunner::run_with)'s hook. A one-off stall shows too, for as long
/// as it stays in the window. Until the window has filled it reports the host keeping time.
/// It allocates once, at construction.
///
/// # Examples
///
/// ```
/// use std::time::Duration;
/// use stable_flight_sitl::runner::{RunStats, TickRate};
///
/// let mut rate = TickRate::new(250.0, Duration::from_millis(500), 0.9).expect("valid");
/// // Ticks 8 ms apart: 125 Hz, half of nominal.
/// for tick in 0..200u64 {
///     let stats = RunStats {
///         ticks: tick,
///         elapsed: Duration::from_millis(8 * tick),
///         overruns: 0,
///         skipped_ticks: 0,
///     };
///     rate.observe(&stats);
/// }
/// assert!(rate.is_slow());
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct TickRate {
    /// Wall-clock time at each of the last ticks observed, oldest first; at most `span + 1`.
    elapsed: VecDeque<Duration>,
    /// Ticks the window spans: `nominal_hz · window`.
    span: usize,
    /// The rate under which the host counts as slow, in Hz.
    min_hz: f64,
    slow: bool,
}

impl TickRate {
    /// A watch over `window` of ticks at `nominal_hz`, slow under `min_share · nominal_hz`.
    ///
    /// # Errors
    ///
    /// [`InvalidTickRate`] unless `nominal_hz` is positive and finite, `window` spans at least
    /// two ticks at it, and `min_share` is in `(0, 1]`.
    pub fn new(nominal_hz: f64, window: Duration, min_share: f64) -> Result<Self, InvalidTickRate> {
        if !(nominal_hz.is_finite() && nominal_hz > 0.0) {
            return Err(InvalidTickRate {
                field: "nominal_hz",
                requirement: "positive and finite",
            });
        }
        let span = (nominal_hz * window.as_secs_f64()).round();
        if !(span >= 2.0 && span <= usize::MAX as f64) {
            return Err(InvalidTickRate {
                field: "window",
                requirement: "at least two ticks at nominal_hz",
            });
        }
        if !(min_share > 0.0 && min_share <= 1.0) {
            return Err(InvalidTickRate {
                field: "min_share",
                requirement: "in (0, 1]",
            });
        }
        let span = span as usize;
        Ok(TickRate {
            elapsed: VecDeque::with_capacity(span + 1),
            span,
            min_hz: min_share * nominal_hz,
            slow: false,
        })
    }

    /// Records a tick's `stats` and returns whether the host now runs slow.
    pub fn observe(&mut self, stats: &RunStats) -> bool {
        if self.elapsed.len() > self.span {
            self.elapsed.pop_front();
        }
        self.elapsed.push_back(stats.elapsed);
        self.slow = self.rate_hz().is_some_and(|hz| hz < self.min_hz);
        self.slow
    }

    /// Whether the host ran slow as of the last [`observe`](Self::observe).
    pub fn is_slow(&self) -> bool {
        self.slow
    }

    /// The rate over the window, in Hz; `None` until it has filled (or if no time passed).
    pub fn rate_hz(&self) -> Option<f64> {
        let (Some(oldest), Some(newest)) = (self.elapsed.front(), self.elapsed.back()) else {
            return None;
        };
        let spanned = newest.saturating_sub(*oldest).as_secs_f64();
        (self.elapsed.len() > self.span && spanned > 0.0).then(|| self.span as f64 / spanned)
    }
}
