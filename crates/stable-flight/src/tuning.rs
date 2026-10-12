//! PID autotune: measure a loop, turn the measurement into gains, and run it per loop.
//!
//! The relay is feedback (Åström & Hägglund, 1984). A [`RelayTuner`]
//! replaces one loop's controller output with `bias ± h`, switching on the sign of the error
//! (with hysteresis). The plant settles into a limit cycle whose period `Tu` and amplitude `a`
//! give the ultimate gain `Ku = 4h / (π·√(a² − ε²))`: the point where a proportional
//! controller would oscillate. [`gains_from_ultimate`] maps that [`UltimatePoint`] to
//! [`Gains`](crate::controllers::Gains) with a [`TuningRule`].
//!
//! The relay never uses the loop's own gains, so a bad starting tune doesn't matter for the axis
//! under test.
//!
//! - [`Probe`]: one loop's excitation and measurement, run inside the tick: the relay, or a
//!   [`StepProbe`] whose [`StepMetrics`] describe the loop's step response.
//! - [`Tunable`]: a stabilizer that hosts a probe on one of its loops
//!   ([`CopterStabilizer`](crate::stabilizers::copter::CopterStabilizer)).
//! - [`autotune::Autotune`]: the copter supervisor, run between ticks: executes each stage's
//!   procedure (which probes to run, what gains follow), guards the envelope, and commits or
//!   reverts gains.
//!
//! Method and rules: `architecture.md#autotune`.
//!
//! # Examples
//!
//! ```
//! use stable_flight::tuning::{ControllerForm, TuningRule, UltimatePoint, gains_from_ultimate};
//!
//! // A measured rate loop: Ku 3.5, Tu 0.2 s.
//! let point = UltimatePoint { ku: 3.5, tu_s: 0.2 };
//! let gains = gains_from_ultimate(&point, TuningRule::TyreusLuyben, ControllerForm::Pid);
//! assert!((gains.p - 3.5 / 2.2).abs() < 1e-6);
//! ```

pub mod autotune;
pub mod probe;
pub mod relay;
pub mod rules;
pub mod step;

#[cfg(test)]
pub(crate) mod plant;

pub use probe::{
    Injection, LoopSignals, Measurement, Probe, ProbeAbort, ProbeConfig, ProbeStatus, Progress,
};
pub use relay::{MAX_CYCLES, RelayAbort, RelayConfig, RelayResult, RelayStatus, RelayTuner};
pub use rules::{ControllerForm, TuningRule, gains_from_ultimate};
pub use step::{Stat, StepAbort, StepConfig, StepMetrics, StepProbe, StepStatus, StepSummary};

/// A stabilizer that hosts autotune stages: one of its loops handed to a [`Probe`] inside the
/// tick, then handed back. [`FlightLoop`](crate::FlightLoop) forwards these between ticks
/// ([`FlightLoop::start_probe`](crate::FlightLoop::start_probe) and friends), so a supervisor
/// never needs `&mut` access to the stabilizer itself.
///
/// Implemented by [`CopterStabilizer`](crate::stabilizers::copter::CopterStabilizer), whose
/// targets are [`TunedLoop`](crate::stabilizers::copter::TunedLoop)s.
pub trait Tunable {
    /// Names a loop a stage can replace.
    type Target: Copy;

    /// Starts a probe on `target` with `config`, from the next tick. A running probe is handed
    /// back first.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`ProbeConfig::validate`]).
    fn start_probe(&mut self, target: Self::Target, config: ProbeConfig);

    /// The current probe, if any: running, or finished and not yet stopped.
    fn probe(&self) -> Option<(Self::Target, ProbeStatus)>;

    /// Ends the probe, handing the loop back to its controller if the probe still flies it,
    /// and returns its last status.
    fn stop_probe(&mut self) -> Option<(Self::Target, ProbeStatus)>;
}

/// A loop's ultimate point: the proportional gain at which it would oscillate, and the period
/// it would oscillate at. A [`RelayTuner`] measures it; [`gains_from_ultimate`] turns it into
/// gains.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UltimatePoint {
    /// Ultimate gain `Ku`, in the loop's output units per error unit.
    pub ku: f32,
    /// Ultimate period `Tu`, in seconds.
    pub tu_s: f32,
}
