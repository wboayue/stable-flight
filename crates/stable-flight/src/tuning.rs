//! PID autotune: measure a loop, turn the measurement into gains, and run it per loop.
//!
//! - [`Probe`]: one loop's excitation and measurement, run inside the tick: a
//!   [`StepProbe`] whose [`StepMetrics`] describe the loop's step response, or an
//!   [`IdentifyProbe`] that fits its axis's [`RateModel`].
//! - [`design_rate`] and [`design_angle_p`]: gains from a [`RateModel`] by loop shaping.
//! - [`Tunable`]: a stabilizer that hosts a probe on one of its loops
//!   ([`CopterStabilizer`](crate::stabilizers::copter::CopterStabilizer),
//!   [`PlaneStabilizer`](crate::stabilizers::plane::PlaneStabilizer)).
//! - [`autotune::Autotune`]: the copter supervisor, run between ticks: executes each stage's
//!   procedure (which probes to run, what gains follow), guards the envelope, and commits or
//!   reverts gains.
//!
//! Method and rules: `architecture.md#autotune`.
//!
//! # Examples
//!
//! ```
//! use stable_flight::tuning::autotune::COPTER_RATE_STEP;
//! use stable_flight::tuning::{LoopSignals, Probe, ProbeConfig};
//!
//! // A 1 rad/s step on a rate loop at rest: the loop runs on the stepped setpoint.
//! let mut probe = Probe::new(ProbeConfig::Step(COPTER_RATE_STEP));
//! let setpoint = probe.step(0.004, &LoopSignals::default());
//! assert_eq!(setpoint, COPTER_RATE_STEP.amplitude);
//! ```

use nalgebra::UnitQuaternion;

pub mod autotune;
pub mod design;
pub mod identify;
pub mod probe;
pub mod step;

#[cfg(test)]
pub(crate) mod plant;

pub use design::{DesignError, LoopShapingConfig, design_angle_p, design_rate};
pub use identify::{
    IdentifyAbort, IdentifyConfig, IdentifyProbe, IdentifyStatus, MAX_DELAY_TICKS, RateModel,
};
pub use probe::{LoopSignals, Measurement, Probe, ProbeAbort, ProbeConfig, ProbeStatus, Progress};
pub use step::{Stat, StepAbort, StepConfig, StepMetrics, StepProbe, StepStatus, StepSummary};

/// A stabilizer that hosts autotune stages: one of its loops handed to a [`Probe`] inside the
/// tick, then handed back. [`FlightLoop`](crate::FlightLoop) forwards these between ticks
/// ([`FlightLoop::start_probe`](crate::FlightLoop::start_probe) and friends), so a supervisor
/// never needs `&mut` access to the stabilizer itself.
///
/// `L` names the loops a stage can replace: a stabilizer with two sets of loops (a VTOL's
/// hover and plane loops) implements it once for each.
///
/// Implemented by [`CopterStabilizer`](crate::stabilizers::copter::CopterStabilizer) for
/// [`TunedLoop`](crate::stabilizers::copter::TunedLoop)s and by
/// [`PlaneStabilizer`](crate::stabilizers::plane::PlaneStabilizer) for
/// [`PlaneLoop`](crate::stabilizers::plane::PlaneLoop)s.
pub trait Tunable<L: Copy> {
    /// Starts a probe on `target` with `config`, from the next tick. A running probe is handed
    /// back first.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`ProbeConfig::validate`]).
    fn start_probe(&mut self, target: L, config: ProbeConfig);

    /// The current probe, if any: running, or finished and not yet stopped.
    fn probe(&self) -> Option<(L, ProbeStatus)>;

    /// Ends the probe, handing the loop back to its controller if the probe still flies it,
    /// and returns its last status.
    fn stop_probe(&mut self) -> Option<(L, ProbeStatus)>;

    /// The frame the probed loops fly in, expressed in the body frame: identity for loops that
    /// fly the body frame, a [`View`](crate::view::View)'s rotation for loops run in it (nose-up
    /// hover reads level in a tailsitter's view). The autotune supervisor reads tilt and
    /// per-axis rates in this frame, so its envelope and settle judge the axes the loops fly.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
    /// use stable_flight::tuning::Tunable;
    /// use stable_flight::view::{View, tailsitter_rotation};
    ///
    /// let hover = View::tailsitter(CopterStabilizer::new(CopterStabilizerConfig::default()));
    /// assert_eq!(hover.probe_frame(), tailsitter_rotation());
    /// ```
    fn probe_frame(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::identity()
    }
}
