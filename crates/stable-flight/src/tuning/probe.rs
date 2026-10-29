//! What an autotune stage runs inside the tick: see [`Probe`].

use super::{
    IdentifyAbort, IdentifyConfig, IdentifyProbe, IdentifyStatus, RateModel, StepAbort, StepConfig,
    StepMetrics, StepProbe, StepStatus,
};
use crate::controllers::InvalidConfig;

/// How to excite and measure one loop: what a [`Probe`] runs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ProbeConfig {
    /// A step: the loop's setpoint is held at a step and its response measured.
    Step(StepConfig),
    /// Identification: a square wave on the loop's setpoint while its axis's rate and moment
    /// are recorded and fitted. Its amplitude is in the target loop's setpoint units;
    /// [`COPTER_IDENTIFY`](crate::tuning::autotune::COPTER_IDENTIFY) is for the roll and pitch
    /// angle loops.
    Identify(IdentifyConfig),
}

impl ProbeConfig {
    /// Checks the probe's config ([`StepConfig::validate`], [`IdentifyConfig::validate`]).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::autotune::COPTER_RATE_STEP;
    /// use stable_flight::tuning::{ProbeConfig, StepConfig};
    ///
    /// assert!(ProbeConfig::Step(COPTER_RATE_STEP).validate().is_ok());
    /// let bad = StepConfig { duration_s: 0.0, ..COPTER_RATE_STEP };
    /// assert!(ProbeConfig::Step(bad).validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        match self {
            Self::Step(config) => config.validate(),
            Self::Identify(config) => config.validate(),
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
    /// The loop's axis's body rate this tick, in rad/s: the measurement itself on a rate loop,
    /// so through the gyro low-pass on a copter; unfiltered on an angle or heading loop, where
    /// the identify probe fits the airframe from it.
    pub rate_radps: f32,
    /// The loop's axis's moment command on the last tick, normalized to `[-1, 1]`: as
    /// commanded, before the mixer; a clip shows in [`saturated`](Self::saturated).
    pub last_moment: f32,
}

impl LoopSignals {
    /// The measurement: `setpoint − error`, consistent with the setpoint even for a wrapped
    /// heading.
    pub fn measurement(&self) -> f32 {
        self.setpoint - self.error
    }
}

/// Where a running probe stands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Progress {
    /// Time into the step.
    Step {
        /// Seconds since the step.
        elapsed_s: f32,
    },
    /// Time into the excitation.
    Identify {
        /// Seconds since the excitation began.
        elapsed_s: f32,
    },
}

/// What a finished probe measured.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Measurement {
    /// The step response.
    Step(StepMetrics),
    /// The axis's fitted model.
    Identify(RateModel),
}

/// Why a probe gave up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProbeAbort {
    /// The step response ran the wrong way or away.
    Step(StepAbort),
    /// The fit couldn't be solved.
    Identify(IdentifyAbort),
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
    /// loop is back with its controller, and the rate tick clears the idle angle loops' I terms.
    Interrupted,
}

impl ProbeStatus {
    /// Whether the probe still flies the loop.
    pub fn is_running(&self) -> bool {
        matches!(self, Self::Running(_))
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

impl From<IdentifyStatus> for ProbeStatus {
    fn from(status: IdentifyStatus) -> Self {
        match status {
            IdentifyStatus::Running { elapsed_s } => {
                Self::Running(Progress::Identify { elapsed_s })
            }
            IdentifyStatus::Done(model) => Self::Done(Measurement::Identify(model)),
            IdentifyStatus::Aborted(abort) => Self::Aborted(ProbeAbort::Identify(abort)),
        }
    }
}

/// One loop's excitation and measurement, stepped by its host inside the tick: the host feeds
/// it the loop's [`LoopSignals`] before the loop's controller runs and runs the controller on
/// the setpoint it returns.
///
/// The host owns the loop and the handoff; the probe owns the signal and what it measures.
/// Plain data, no allocation.
///
/// # Examples
///
/// ```
/// use stable_flight::tuning::autotune::COPTER_RATE_STEP;
/// use stable_flight::tuning::{LoopSignals, Probe, ProbeConfig};
///
/// let mut probe = Probe::new(ProbeConfig::Step(COPTER_RATE_STEP));
/// // Rolling right at 0.5 rad/s: the step is 1 rad/s above where it started.
/// let signals = LoopSignals { setpoint: 0.0, error: -0.5, ..LoopSignals::default() };
/// assert_eq!(probe.step(0.004, &signals), 0.5 + COPTER_RATE_STEP.amplitude);
/// assert!(probe.status().is_running());
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct Probe {
    kind: Kind,
}

// The identification's least-squares sums (about 1.7 KB) make its variant the largest; boxing
// it would allocate, which core doesn't, and a host holds one probe at a time.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
enum Kind {
    Step(StepProbe),
    Identify(IdentifyProbe),
}

impl Probe {
    /// A probe ready to run.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`ProbeConfig::validate`]).
    pub fn new(config: ProbeConfig) -> Self {
        let kind = match config {
            ProbeConfig::Step(config) => Kind::Step(StepProbe::new(config)),
            ProbeConfig::Identify(config) => Kind::Identify(IdentifyProbe::new(config)),
        };
        Self { kind }
    }

    /// Advances the probe by `dt` seconds with the loop's `signals` and returns the setpoint
    /// the loop's controller runs on this tick, in place of its own.
    ///
    /// # Panics
    ///
    /// If `dt` is not positive and finite.
    pub fn step(&mut self, dt: f32, signals: &LoopSignals) -> f32 {
        match &mut self.kind {
            Kind::Step(step) => step.step(dt, signals.measurement(), signals.saturated),
            Kind::Identify(identify) => {
                let offset = identify.step(dt, signals.rate_radps, signals.last_moment);
                signals.setpoint + offset
            }
        }
    }

    /// Where the probe stands; never [`ProbeStatus::Interrupted`], which only a host reports.
    pub fn status(&self) -> ProbeStatus {
        match &self.kind {
            Kind::Step(step) => step.status().into(),
            Kind::Identify(identify) => identify.status().into(),
        }
    }
}
