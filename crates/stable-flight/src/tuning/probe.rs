//! What an autotune stage runs inside the tick: see [`Probe`].

use super::{
    RelayAbort, RelayConfig, RelayResult, RelayStatus, RelayTuner, StepAbort, StepConfig,
    StepMetrics, StepProbe, StepStatus,
};
use crate::controllers::InvalidConfig;

/// How to excite and measure one loop: what a [`Probe`] runs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ProbeConfig {
    /// Relay feedback: the relay replaces the loop's output and measures its ultimate point.
    Relay(RelayConfig),
    /// A step: the loop's setpoint is held at a step and its response measured.
    Step(StepConfig),
}

impl ProbeConfig {
    /// Checks the probe's config ([`RelayConfig::validate`], [`StepConfig::validate`]).
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
            Self::Step(config) => config.validate(),
        }
    }
}

/// What the host tells a probe about its loop each tick, before the loop's controller runs.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LoopSignals {
    /// The loop's setpoint this tick, as its controller would see it (a saturated angle, a
    /// heading).
    pub setpoint: f32,
    /// The error the loop acts on: setpoint − measurement (wrapped for a heading).
    pub error: f32,
    /// Whether the loop's output, or the loop under it, was clipped on the last tick.
    pub saturated: bool,
}

impl LoopSignals {
    /// The measurement: `setpoint − error`, consistent with the setpoint even for a wrapped
    /// heading.
    pub fn measurement(&self) -> f32 {
        self.setpoint - self.error
    }
}

/// What a probe asks its host to do with the loop this tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Injection {
    /// Use this in place of the loop's controller output (relay).
    Output(f32),
    /// Run the loop's controller on this setpoint instead of its own (step).
    Setpoint(f32),
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
    /// Time into the step.
    Step {
        /// Seconds since the step.
        elapsed_s: f32,
    },
}

/// What a finished probe measured.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Measurement {
    /// The relay's ultimate point and the cycle it came from.
    Relay(RelayResult),
    /// The step response.
    Step(StepMetrics),
}

/// Why a probe gave up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProbeAbort {
    /// The relay tuner gave up.
    Relay(RelayAbort),
    /// The step response ran the wrong way or away.
    Step(StepAbort),
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

impl From<StepStatus> for ProbeStatus {
    fn from(status: StepStatus) -> Self {
        match status {
            StepStatus::Running { elapsed_s } => Self::Running(Progress::Step { elapsed_s }),
            StepStatus::Done(metrics) => Self::Done(Measurement::Step(metrics)),
            StepStatus::Aborted(abort) => Self::Aborted(ProbeAbort::Step(abort)),
        }
    }
}

/// One loop's excitation and measurement, stepped by its host inside the tick: the host feeds
/// it the loop's [`LoopSignals`] before the loop's controller runs and applies the
/// [`Injection`] it returns.
///
/// The host owns the loop and the handoff; the probe owns the signal and what it measures.
/// Plain data, no allocation.
///
/// # Examples
///
/// ```
/// use stable_flight::tuning::autotune::COPTER_RATE_RELAY;
/// use stable_flight::tuning::{Injection, LoopSignals, Probe, ProbeConfig};
///
/// let mut probe = Probe::new(ProbeConfig::Relay(COPTER_RATE_RELAY), 0.0);
/// // Rolling right faster than wanted: the relay pushes left.
/// let signals = LoopSignals { setpoint: 0.0, error: -0.5, saturated: false };
/// assert_eq!(probe.step(0.004, &signals), Injection::Output(-COPTER_RATE_RELAY.initial_relay));
/// assert!(probe.status().is_running());
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct Probe {
    kind: Kind,
}

#[derive(Clone, Debug, PartialEq)]
enum Kind {
    Relay(RelayTuner),
    Step(StepProbe),
}

impl Probe {
    /// A probe ready to run, from the loop's current `trim` (its I term, in output units; the
    /// relay starts its bias there).
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`ProbeConfig::validate`]).
    pub fn new(config: ProbeConfig, trim: f32) -> Self {
        let kind = match config {
            ProbeConfig::Relay(config) => Kind::Relay(RelayTuner::new(config, trim)),
            ProbeConfig::Step(config) => Kind::Step(StepProbe::new(config)),
        };
        Self { kind }
    }

    /// Advances the probe by `dt` seconds with the loop's `signals` and returns what to do
    /// with the loop this tick.
    ///
    /// # Panics
    ///
    /// If `dt` is not positive and finite.
    pub fn step(&mut self, dt: f32, signals: &LoopSignals) -> Injection {
        match &mut self.kind {
            Kind::Relay(tuner) => Injection::Output(tuner.step(dt, signals.error)),
            Kind::Step(step) => {
                Injection::Setpoint(step.step(dt, signals.measurement(), signals.saturated))
            }
        }
    }

    /// Where the probe stands; never [`ProbeStatus::Interrupted`], which only a host reports.
    pub fn status(&self) -> ProbeStatus {
        match &self.kind {
            Kind::Relay(tuner) => tuner.status().into(),
            Kind::Step(step) => step.status().into(),
        }
    }

    /// The trim to hand the loop back with, as its I term: the relay's bias. `None` for a
    /// step, which leaves the loop's integral as it is.
    pub fn trim(&self) -> Option<f32> {
        match &self.kind {
            Kind::Relay(tuner) => Some(tuner.bias()),
            Kind::Step(_) => None,
        }
    }
}
