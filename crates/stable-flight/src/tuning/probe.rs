//! What an autotune stage runs inside the tick: see [`Probe`].

use super::{RelayAbort, RelayConfig, RelayResult, RelayStatus, RelayTuner};
use crate::controllers::InvalidConfig;

/// How to excite and measure one loop: what a [`Probe`] runs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ProbeConfig {
    /// Relay feedback: the relay replaces the loop's output and measures its ultimate point.
    Relay(RelayConfig),
}

impl ProbeConfig {
    /// Checks the probe's config ([`RelayConfig::validate`]).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::autotune::COPTER_RATE_RELAY;
    /// use stable_flight::tuning::{ProbeConfig, RelayConfig};
    ///
    /// assert!(ProbeConfig::Relay(COPTER_RATE_RELAY).validate().is_ok());
    /// let bad = RelayConfig { cycles: 0, ..COPTER_RATE_RELAY };
    /// assert!(ProbeConfig::Relay(bad).validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        match self {
            Self::Relay(config) => config.validate(),
        }
    }
}

/// Where a running probe stands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Progress {
    /// The relay's current amplitude and the consistent cycles in hand.
    Relay {
        /// Relay amplitude `h`, in the loop's output units.
        relay: f32,
        /// Measured cycles in the window.
        cycles: usize,
    },
}

/// What a finished probe measured.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Measurement {
    /// The relay's ultimate point and the cycle it came from.
    Relay(RelayResult),
}

/// Why a probe gave up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProbeAbort {
    /// The relay tuner gave up.
    Relay(RelayAbort),
}

/// Where a probe stands, as its host reports it ([`Tunable::probe`](super::Tunable::probe)).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ProbeStatus {
    /// The probe is flying the loop.
    Running(Progress),
    /// Measured; the loop is back with its controller.
    Done(Measurement),
    /// The probe gave up; the loop is back with its controller.
    Aborted(ProbeAbort),
    /// The host idled the loop under test, e.g. a
    /// [`CopterStabilizer`](crate::stabilizers::copter::CopterStabilizer) angle or heading loop
    /// on a [`Setpoint::Rate`](crate::Setpoint::Rate) tick (the pilot switched to acro). The
    /// loop is back with its controller; the measured trim may not carry over.
    Interrupted,
}

impl ProbeStatus {
    /// Whether the probe still flies the loop.
    pub fn is_running(&self) -> bool {
        matches!(self, Self::Running(_))
    }
}

impl From<RelayStatus> for ProbeStatus {
    fn from(status: RelayStatus) -> Self {
        match status {
            RelayStatus::Running { relay, cycles } => {
                Self::Running(Progress::Relay { relay, cycles })
            }
            RelayStatus::Done(result) => Self::Done(Measurement::Relay(result)),
            RelayStatus::Aborted(abort) => Self::Aborted(ProbeAbort::Relay(abort)),
        }
    }
}

/// One loop's excitation and measurement, stepped by its host inside the tick: the host feeds
/// it the loop's error and applies the output it returns in place of the loop's controller.
///
/// The host owns the loop and the handoff; the probe owns the signal and what it measures.
/// Plain data, no allocation.
///
/// # Examples
///
/// ```
/// use stable_flight::tuning::autotune::COPTER_RATE_RELAY;
/// use stable_flight::tuning::{Probe, ProbeConfig, ProbeStatus};
///
/// let mut probe = Probe::new(ProbeConfig::Relay(COPTER_RATE_RELAY), 0.0);
/// // Rolling right faster than wanted: the relay pushes left.
/// assert_eq!(probe.step(0.004, -0.5), -COPTER_RATE_RELAY.initial_relay);
/// assert!(probe.status().is_running());
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct Probe {
    kind: Kind,
}

#[derive(Clone, Debug, PartialEq)]
enum Kind {
    Relay(RelayTuner),
}

impl Probe {
    /// A probe ready to run, from the loop's current `trim` (its I term, in output units).
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`ProbeConfig::validate`]).
    pub fn new(config: ProbeConfig, trim: f32) -> Self {
        let kind = match config {
            ProbeConfig::Relay(config) => Kind::Relay(RelayTuner::new(config, trim)),
        };
        Self { kind }
    }

    /// Advances the probe by `dt` seconds with the loop's `error` (setpoint − measurement) and
    /// returns the output to apply in place of the loop's controller.
    ///
    /// # Panics
    ///
    /// If `dt` is not positive and finite.
    pub fn step(&mut self, dt: f32, error: f32) -> f32 {
        match &mut self.kind {
            Kind::Relay(tuner) => tuner.step(dt, error),
        }
    }

    /// Where the probe stands; never [`ProbeStatus::Interrupted`], which only a host reports.
    pub fn status(&self) -> ProbeStatus {
        match &self.kind {
            Kind::Relay(tuner) => tuner.status().into(),
        }
    }

    /// The trim to hand the loop back with, as its I term: the relay's bias.
    pub fn trim(&self) -> f32 {
        match &self.kind {
            Kind::Relay(tuner) => tuner.bias(),
        }
    }
}
