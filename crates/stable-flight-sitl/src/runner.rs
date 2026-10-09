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

/// Returned by [`RealTimeRunner::new`] for a frequency that isn't positive and finite, or
/// whose period isn't representable as a non-zero [`Duration`] (e.g. 1e-30 Hz, 1e10 Hz).
#[derive(Debug, Clone, Copy, Error)]
#[error("target frequency must be positive, finite, and give a non-zero period, got {0} Hz")]
pub struct InvalidFrequency(pub f64);

/// What a [`RealTimeRunner::run`] achieved.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RunStats {
    /// Ticks executed.
    pub ticks: u64,
    /// Wall-clock time from priming to shutdown.
    pub elapsed: Duration,
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
/// it calls [`FlightLoop::step`] with the nominal period, then sleeps until the next deadline
/// (deadlines accumulate, so a slow tick is caught up rather than shifting the schedule).
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
        self.run_with(flight_loop, shutdown, |_| {})
    }

    /// [`run`](Self::run), calling `before_step` with the loop before each tick: the place to
    /// apply changes that need `&mut` to the loop, such as param writes from a link
    /// (`FlightLoop::set_param`) or config swaps. Its time counts toward the tick's period,
    /// so keep it bounded.
    ///
    /// ```no_run
    /// # use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
    /// # use stable_flight::mixers::QuadCopterMixer;
    /// # use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
    /// # use stable_flight::testing::{ConstantCommander, NullVehicle, RecordingTelemetry};
    /// # use stable_flight::FlightLoop;
    /// use stable_flight::params::Params;
    /// use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
    ///
    /// # let mut flight_loop = FlightLoop::new(
    /// #     ConstantCommander::default(),
    /// #     NullVehicle::new(),
    /// #     CopterGuidance::new(CopterGuidanceConfig::default()),
    /// #     CopterStabilizer::new(CopterStabilizerConfig::default()),
    /// #     QuadCopterMixer::default(),
    /// #     RecordingTelemetry::<1>::new(),
    /// # );
    /// let shutdown = ShutdownSignal::install().expect("install Ctrl-C handler");
    /// RealTimeRunner::new(250.0)
    ///     .expect("valid frequency")
    ///     .run_with(&mut flight_loop, &shutdown, |flight_loop| {
    ///         // e.g. `param_server.poll(flight_loop)` (stable-flight-link)
    ///         let _ = flight_loop.set_param("STAB_RATE_P", 0.12);
    ///     });
    /// ```
    pub fn run_with<C, V, G, S, M, T>(
        &self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        shutdown: &ShutdownSignal,
        mut before_step: impl FnMut(&mut FlightLoop<C, V, G, S, M, T>),
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

        flight_loop.prime();

        while !shutdown.is_set() {
            before_step(flight_loop);
            flight_loop.step(self.period);
            ticks += 1;

            if ticks.is_multiple_of(RATE_LOG_INTERVAL) {
                debug!(
                    "Tick {ticks}: {:.2} Hz",
                    ticks as f64 / start.elapsed().as_secs_f64()
                );
            }

            next_deadline += self.period;
            thread::sleep(next_deadline.saturating_duration_since(Instant::now()));
        }

        let stats = RunStats {
            ticks,
            elapsed: start.elapsed(),
        };
        info!("Shutdown requested; stopping flight loop");
        if let Some(hz) = stats.average_hz() {
            info!("Ran {ticks} ticks at an average {hz:.2} Hz");
        }
        stats
    }
}
