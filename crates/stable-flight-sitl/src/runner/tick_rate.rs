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
/// `window` of ticks: the host turns slow once that rate falls under `slow_share` of nominal,
/// and keeps time again once it is back over `resume_share`, so a host hovering at the
/// threshold doesn't flip on every tick.
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
/// let mut rate = TickRate::new(250.0, Duration::from_millis(500), 0.9, 0.96).expect("valid");
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
    /// The rate under which the host turns slow, in Hz.
    slow_hz: f64,
    /// The rate over which a slow host keeps time again, in Hz.
    resume_hz: f64,
    slow: bool,
}

impl TickRate {
    /// A watch over `window` of ticks at `nominal_hz`: slow under `slow_share · nominal_hz`,
    /// keeping time again over `resume_share · nominal_hz`.
    ///
    /// # Errors
    ///
    /// [`InvalidTickRate`] unless `nominal_hz` is positive and finite, `window` spans at least
    /// two ticks at it, `slow_share` is in `(0, 1]`, and `resume_share` in `[slow_share, 1]`.
    pub fn new(
        nominal_hz: f64,
        window: Duration,
        slow_share: f64,
        resume_share: f64,
    ) -> Result<Self, InvalidTickRate> {
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
        if !(slow_share > 0.0 && slow_share <= 1.0) {
            return Err(InvalidTickRate {
                field: "slow_share",
                requirement: "in (0, 1]",
            });
        }
        if !(resume_share >= slow_share && resume_share <= 1.0) {
            return Err(InvalidTickRate {
                field: "resume_share",
                requirement: "in [slow_share, 1]",
            });
        }
        let span = span as usize;
        Ok(TickRate {
            elapsed: VecDeque::with_capacity(span + 1),
            span,
            slow_hz: slow_share * nominal_hz,
            resume_hz: resume_share * nominal_hz,
            slow: false,
        })
    }

    /// Records a tick's `stats` and returns whether the host now runs slow.
    pub fn observe(&mut self, stats: &RunStats) -> bool {
        if self.elapsed.len() > self.span {
            self.elapsed.pop_front();
        }
        self.elapsed.push_back(stats.elapsed);
        let threshold_hz = if self.slow {
            self.resume_hz
        } else {
            self.slow_hz
        };
        self.slow = self.rate_hz().is_some_and(|hz| hz < threshold_hz);
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
