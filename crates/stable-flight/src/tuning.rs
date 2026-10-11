//! PID autotune (#149): measure a loop, turn the measurement into gains, and run it per loop.
//!
//! Phase 1 of the autotune is relay feedback (Åström & Hägglund, 1984). A [`RelayTuner`]
//! replaces one loop's controller output with `bias ± h`, switching on the sign of the error
//! (with hysteresis). The plant settles into a limit cycle whose period `Tu` and amplitude `a`
//! give the ultimate gain `Ku = 4h / (π·√(a² − ε²))`: the point where a proportional
//! controller would oscillate. [`gains_from_ultimate`] maps that [`UltimatePoint`] to
//! [`Gains`](crate::controllers::Gains) with a [`TuningRule`].
//!
//! The relay never uses the loop's own gains, so a bad starting tune doesn't matter for the axis
//! under test.
//!
//! - [`Tunable`]: a stabilizer that hosts a stage, one loop handed to a tuner inside the tick
//!   ([`CopterStabilizer`](crate::stabilizers::copter::CopterStabilizer)).
//! - [`autotune::Autotune`]: the copter supervisor, run between ticks: sequences the stages,
//!   guards the envelope, and commits or reverts gains.
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
pub mod relay;
pub mod rules;

#[cfg(test)]
pub(crate) mod plant;

pub use relay::{MAX_CYCLES, RelayAbort, RelayConfig, RelayResult, RelayStatus, RelayTuner};
pub use rules::{ControllerForm, TuningRule, gains_from_ultimate};

/// A stabilizer that hosts autotune stages: one of its loops handed to a [`RelayTuner`] inside
/// the tick, then handed back. [`FlightLoop`](crate::FlightLoop) forwards these between ticks
/// ([`FlightLoop::start_tuning`](crate::FlightLoop::start_tuning) and friends), so a
/// supervisor never needs `&mut` access to the stabilizer itself.
///
/// Implemented by [`CopterStabilizer`](crate::stabilizers::copter::CopterStabilizer), whose
/// targets are [`TunedLoop`](crate::stabilizers::copter::TunedLoop)s.
pub trait Tunable {
    /// Names a loop a stage can replace.
    type Target: Copy;

    /// Starts relay tuning on `target` with `config`, from the next tick. A running stage is
    /// handed back first.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`RelayConfig::validate`]).
    fn start_tuning(&mut self, target: Self::Target, config: RelayConfig);

    /// The current stage, if any: running, or finished and not yet stopped.
    fn tuning(&self) -> Option<(Self::Target, TuningStatus)>;

    /// Ends the stage, handing the loop back to its controller if the relay still flies it,
    /// and returns its last status.
    fn stop_tuning(&mut self) -> Option<(Self::Target, TuningStatus)>;
}

/// Where a tuning stage stands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TuningStatus {
    /// The relay is flying the loop: its current amplitude and the consistent cycles in hand.
    Running {
        /// Relay amplitude `h`, in the loop's output units.
        relay: f32,
        /// Measured cycles in the window.
        cycles: usize,
    },
    /// Measured; the loop is back with its controller.
    Done(RelayResult),
    /// The tuner gave up; the loop is back with its controller.
    Aborted(RelayAbort),
    /// The host idled the loop under test, e.g. a
    /// [`CopterStabilizer`](crate::stabilizers::copter::CopterStabilizer) angle or heading loop
    /// on a [`Setpoint::Rate`](crate::Setpoint::Rate) tick (the pilot switched to acro). The
    /// loop is back with its controller; the measured trim may not carry over.
    Interrupted,
}

impl From<RelayStatus> for TuningStatus {
    fn from(status: RelayStatus) -> Self {
        match status {
            RelayStatus::Running { relay, cycles } => Self::Running { relay, cycles },
            RelayStatus::Done(result) => Self::Done(result),
            RelayStatus::Aborted(abort) => Self::Aborted(abort),
        }
    }
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
