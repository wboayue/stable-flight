//! PID autotuning building blocks (#149): measure a loop, then turn the measurement into gains.
//!
//! Phase 1 of the autotune is relay feedback (Åström & Hägglund, 1984). A [`RelayTuner`]
//! replaces one loop's controller output with `bias ± h`, switching on the sign of the error
//! (with hysteresis). The plant settles into a limit cycle whose period `Tu` and amplitude `a`
//! give the ultimate gain `Ku = 4h / (π·√(a² − ε²))`: the point where a proportional
//! controller would oscillate. [`gains_from_ultimate`] maps that [`UltimatePoint`] to
//! [`Gains`](crate::controllers::Gains) with a [`TuningRule`].
//!
//! The relay never uses the loop's own gains, so a bad starting tune doesn't matter for the axis
//! under test. Hosting the tuner in a loop, sequencing axes, and the safety envelope belong to
//! the stabilizer and the autotune supervisor, not to this module.
//!
//! # Examples
//!
//! ```
//! use stable_flight::tuning::{ControllerForm, TuningRule, UltimatePoint, gains_from_ultimate};
//!
//! // A measured rate loop: Ku 3.5, Tu 0.2 s.
//! let point = UltimatePoint { ku: 3.5, tu_s: 0.2, relay: 0.1, cycle_amplitude: 0.036, bias: 0.0 };
//! let gains = gains_from_ultimate(&point, TuningRule::TyreusLuyben, ControllerForm::Pid);
//! assert!((gains.p - 3.5 / 2.2).abs() < 1e-6);
//! ```

pub mod relay;
pub mod rules;

#[cfg(test)]
pub(crate) mod plant;

pub use relay::{MAX_CYCLES, RelayAbort, RelayConfig, RelayStatus, RelayTuner, UltimatePoint};
pub use rules::{ControllerForm, TuningRule, gains_from_ultimate};
