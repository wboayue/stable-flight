//! Drives a [`FlightLoop`] in real time on the host's wall clock.

use std::thread;
use std::time::{Duration, Instant};

use log::{debug, info};
use stable_flight::{
    Commander, FlightLoop, Guidance, Mixer, Stabilizer, TelemetryPublisher, Vehicle,
};
use thiserror::Error;

use crate::ShutdownSignal;

#[cfg(test)]
mod tests;

/// Log the achieved loop rate every this many ticks (at `debug`).
const RATE_LOG_INTERVAL: u64 = 100;

/// How far behind schedule the loop may fall and still catch up by running ticks
/// back-to-back. Above it (or one period, if longer) it resyncs instead. Simulator round-trip
/// jitter (up to ~20 ms with RealFlight, #76) stays below it, so `dt` summed over ticks keeps
/// tracking wall time; a reconnect (up to 250 ms in the RealFlight examples) exceeds it.
const MAX_LAG: Duration = Duration::from_millis(100);

/// Returned by [`RealTimeRunner::new`] for a frequency that isn't positive and finite, or
/// whose period isn't representable as a non-zero [`Duration`] (e.g. 1e-30 Hz, 1e10 Hz).
#[derive(Debug, Clone, Copy, Error)]
#[error("target frequency must be positive, finite, and give a non-zero period, got {0} Hz")]
pub struct InvalidFrequency(pub f64);

/// What a [`RealTimeRunner::run`] achieved, or (passed to
/// [`run_with`](RealTimeRunner::run_with)'s hook) has achieved so far.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RunStats {
    /// Ticks executed.
    pub ticks: u64,
    /// Wall-clock time from priming to shutdown.
    pub elapsed: Duration,
    /// Stalls: times the loop fell more than 100 ms (or a period) behind and resynced.
    pub overruns: u64,
    /// Deadlines dropped by those resyncs rather than replayed.
    pub skipped_ticks: u64,
}

impl RunStats {
    /// Achieved loop rate in Hz, or `None` if nothing ran.
    pub fn average_hz(&self) -> Option<f64> {
        (self.ticks > 0 && !self.elapsed.is_zero())
            .then(|| self.ticks as f64 / self.elapsed.as_secs_f64())
    }
}

/// Runs a [`FlightLoop`] at a fixed wall-clock rate until a [`ShutdownSignal`] is set.
///
/// For simulators that run in real time on their own clock, such as RealFlight. Each period
/// it calls [`FlightLoop::step`] with the nominal period, then sleeps until the next deadline.
/// Deadlines accumulate, so jitter of up to 100 ms is caught up by running ticks back-to-back,
/// keeping the average rate (and the sum of nominal `dt`s) on wall time. After a longer stall
/// (e.g. a simulator reconnect), the missed deadlines are dropped, not replayed: the next tick
/// runs at once and the schedule restarts from it. [`RunStats`] counts the overruns and
/// skipped ticks.
/// Tick ordering is `FlightLoop`'s concern; see its docs.
///
/// # Examples
///
/// ```no_run
/// # use std::time::Duration;
/// # use stable_flight::{FlightLoop, mixers::QuadCopterMixer};
/// # use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
/// # use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
/// # use stable_flight::testing::{ConstantCommander, NullVehicle, RecordingTelemetry};
/// use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
///
/// # let commander = ConstantCommander::default();
/// # let vehicle = NullVehicle::new();
/// # let guidance = CopterGuidance::new(CopterGuidanceConfig::default());
/// # let stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
/// # let mixer = QuadCopterMixer::default();
/// # let telemetry = RecordingTelemetry::<1>::new();
/// let mut flight_loop = FlightLoop::new(commander, vehicle, guidance, stabilizer, mixer, telemetry);
/// let shutdown = ShutdownSignal::install().expect("install Ctrl-C handler");
///
/// let stats = RealTimeRunner::new(250.0)
///     .expect("valid frequency")
///     .run(&mut flight_loop, &shutdown);
/// println!("{} ticks", stats.ticks);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RealTimeRunner {
    period: Duration,
}

impl RealTimeRunner {
    /// A runner ticking at `frequency_hz`.
    ///
    /// # Errors
    ///
    /// Returns [`InvalidFrequency`] unless `frequency_hz` is positive and finite and its
    /// period fits a non-zero [`Duration`]. A zero period would spin with `dt = 0`.
    pub fn new(frequency_hz: f64) -> Result<Self, InvalidFrequency> {
        if !(frequency_hz.is_finite() && frequency_hz > 0.0) {
            return Err(InvalidFrequency(frequency_hz));
        }
        match Duration::try_from_secs_f64(frequency_hz.recip()) {
            Ok(period) if !period.is_zero() => Ok(RealTimeRunner { period }),
            _ => Err(InvalidFrequency(frequency_hz)),
        }
    }

    /// The nominal loop period passed to each [`FlightLoop::step`].
    pub fn period(&self) -> Duration {
        self.period
    }

    /// Primes `flight_loop`, then ticks it every period until `shutdown` is set.
    ///
    /// `shutdown` is checked before each tick, so an already-set signal runs no ticks.
    pub fn run<C, V, G, S, M, T>(
        &self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        shutdown: &ShutdownSignal,
    ) -> RunStats
    where
        C: Commander,
        V: Vehicle,
        G: Guidance,
        S: Stabilizer,
        M: Mixer,
        T: TelemetryPublisher,
    {
        self.run_with(flight_loop, shutdown, |_, _| {})
    }

    /// [`run`](Self::run), calling `before_step` with the loop and the [`RunStats`] so far
    /// before each tick: the place to apply changes that need `&mut` to the loop, such as
    /// param writes from a link (`FlightLoop::set_param`) or config swaps, and to react to
    /// stalls (`skipped_ticks` growing). Its time counts toward the tick's period, so keep it
    /// bounded.
    ///
    /// With `flight_loop` and `shutdown` as in the [type-level example](Self):
    ///
    /// ```ignore
    /// RealTimeRunner::new(250.0)?.run_with(&mut flight_loop, &shutdown, |flight_loop, _stats| {
    ///     param_server.poll(flight_loop); // stable-flight-link
    /// });
    /// ```
    pub fn run_with<C, V, G, S, M, T>(
        &self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        shutdown: &ShutdownSignal,
        mut before_step: impl FnMut(&mut FlightLoop<C, V, G, S, M, T>, &RunStats),
    ) -> RunStats
    where
        C: Commander,
        V: Vehicle,
        G: Guidance,
        S: Stabilizer,
        M: Mixer,
        T: TelemetryPublisher,
    {
        let start = Instant::now();
        let mut next_deadline = start;
        let mut ticks: u64 = 0;
        let mut overruns: u64 = 0;
        let mut skipped_ticks: u64 = 0;
        let max_lag = MAX_LAG.max(self.period);

        flight_loop.prime();

        while !shutdown.is_set() {
            let stats = RunStats {
                ticks,
                elapsed: start.elapsed(),
                overruns,
                skipped_ticks,
            };
            before_step(flight_loop, &stats);
            flight_loop.step(self.period);
            ticks += 1;

            if ticks.is_multiple_of(RATE_LOG_INTERVAL) {
                debug!(
                    "Tick {ticks}: {:.2} Hz",
                    ticks as f64 / start.elapsed().as_secs_f64()
                );
            }

            let now = Instant::now();
            let skipped = advance(&mut next_deadline, now, self.period, max_lag);
            if skipped > 0 {
                overruns += 1;
                skipped_ticks = skipped_ticks.saturating_add(skipped);
                debug!("Tick {ticks}: stalled, resyncing ({skipped} ticks skipped)");
            }
            thread::sleep(next_deadline.saturating_duration_since(now));
        }

        let stats = RunStats {
            ticks,
            elapsed: start.elapsed(),
            overruns,
            skipped_ticks,
        };
        info!("Shutdown requested; stopping flight loop");
        if let Some(hz) = stats.average_hz() {
            info!("Ran {ticks} ticks at an average {hz:.2} Hz");
        }
        if overruns > 0 {
            info!("{overruns} overruns, {skipped_ticks} ticks skipped");
        }
        stats
    }
}

/// Advances `next_deadline` one `period`. If `now` is more than `max_lag` (at least `period`)
/// past the new deadline, resyncs it to `now` and returns the whole periods skipped; otherwise
/// returns 0 (a smaller lag is caught up by short or skipped sleeps).
fn advance(next_deadline: &mut Instant, now: Instant, period: Duration, max_lag: Duration) -> u64 {
    *next_deadline += period;
    let late = now.saturating_duration_since(*next_deadline);
    if late <= max_lag {
        return 0;
    }
    *next_deadline = now;
    // `period` is non-zero (`RealTimeRunner::new`); saturate a ludicrous stall.
    u64::try_from(late.as_nanos() / period.as_nanos()).unwrap_or(u64::MAX)
}
