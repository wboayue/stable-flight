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

/// The window [`TickRate::standard`] judges the rate over: 125 ticks at 250 Hz, half an autotune
/// rate step's 63-tick window, so a slow stretch shows before a step held across it ends.
const STANDARD_WINDOW: Duration = Duration::from_millis(500);
/// The share of nominal under which [`TickRate::standard`] turns slow: 225 Hz at 250 Hz.
/// RealFlight's slow mode runs at about 120 Hz; at full rate it holds about 240 Hz or more per
/// second (#438).
const STANDARD_SLOW_SHARE: f64 = 0.9;
/// The share of nominal a slow [`TickRate::standard`] must regain to keep time again: 240 Hz at
/// 250 Hz. The gap keeps a host near 225 Hz from flipping mid-step: one flight turned slow at
/// 223–225 Hz and back at 226 Hz (#438).
const STANDARD_RESUME_SHARE: f64 = 0.96;

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
/// It allocates once, at construction. It also counts the slow stretches and the time spent in
/// them, for a run's summary.
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
///         host_slow: false,
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
    /// Wall-clock time of the last tick observed.
    last_elapsed: Duration,
    /// When the current slow stretch began (the tick that turned slow), if slow.
    slow_since: Option<Duration>,
    /// Slow stretches begun.
    stretches: u64,
    /// Time spent in slow stretches that have ended.
    ended_slow_time: Duration,
    /// How long the last ended slow stretch lasted.
    last_stretch: Duration,
}

impl TickRate {
    /// A watch over 0.5 s at `nominal_hz`, slow under 90% of it, keeping time again over 96%
    /// (225 and 240 Hz at 250 Hz): the thresholds a
    /// [`RealTimeRunner`](super::RealTimeRunner) logs slow stretches by, and the autotune waits
    /// on.
    ///
    /// # Errors
    ///
    /// [`InvalidTickRate`] as [`new`](Self::new): `nominal_hz` not positive and finite, or under
    /// 3 Hz, where the window spans fewer than two ticks.
    pub fn standard(nominal_hz: f64) -> Result<Self, InvalidTickRate> {
        Self::new(
            nominal_hz,
            STANDARD_WINDOW,
            STANDARD_SLOW_SHARE,
            STANDARD_RESUME_SHARE,
        )
    }

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
            last_elapsed: Duration::ZERO,
            slow_since: None,
            stretches: 0,
            ended_slow_time: Duration::ZERO,
            last_stretch: Duration::ZERO,
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
        self.last_elapsed = stats.elapsed;
        match (self.slow, self.slow_since) {
            (true, None) => {
                self.slow_since = Some(stats.elapsed);
                self.stretches += 1;
            }
            (false, Some(since)) => {
                self.last_stretch = stats.elapsed.saturating_sub(since);
                self.ended_slow_time += self.last_stretch;
                self.slow_since = None;
            }
            _ => {}
        }
        self.slow
    }

    /// Whether the host ran slow as of the last [`observe`](Self::observe).
    pub fn is_slow(&self) -> bool {
        self.slow
    }

    /// When the current slow stretch began, as the runner's elapsed time; `None` while keeping
    /// time.
    pub fn slow_since(&self) -> Option<Duration> {
        self.slow_since
    }

    /// Slow stretches begun so far, the current one included.
    pub fn slow_stretches(&self) -> u64 {
        self.stretches
    }

    /// How long the last ended slow stretch lasted; zero before one ends.
    pub fn last_slow_stretch(&self) -> Duration {
        self.last_stretch
    }

    /// Time spent slow so far: the ended stretches, plus the current one up to the last tick
    /// observed.
    pub fn slow_time(&self) -> Duration {
        let current = self.slow_since.map_or(Duration::ZERO, |since| {
            self.last_elapsed.saturating_sub(since)
        });
        self.ended_slow_time + current
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
