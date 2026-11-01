//! The copter's gyro filter chain, a notch then a low-pass: see [`GyroFilters`].

use nalgebra::Complex;

use super::NotchConfig;
use super::low_pass::LowPass;
use super::notch::Notch;

/// The filters the body rates pass through before the rate loop, as
/// [`CopterStabilizerConfig`](crate::stabilizers::copter::CopterStabilizerConfig) sets them:
/// a notch for one vibration tone, then the low-pass. The low-pass after the notch cleans up
/// what it leaves, as ArduPilot and PX4 order them (`apply_gyro_filters`,
/// `FilterAngularVelocity`). The autotune's design models both, since the rate loop's P and
/// D act through them.
///
/// # Examples
///
/// ```
/// use stable_flight::controllers::GyroFilters;
///
/// let filters = GyroFilters::low_pass(40.0);
/// assert!(!filters.notch.is_on());
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GyroFilters {
    /// The low-pass cutoff, in Hz; 0, or at or past Nyquist, passes the rates through.
    pub cutoff_hz: f32,
    /// The notch; off when its center is 0.
    pub notch: NotchConfig,
}

impl GyroFilters {
    /// A low-pass at `cutoff_hz` alone, the notch off.
    pub fn low_pass(cutoff_hz: f32) -> Self {
        Self {
            cutoff_hz,
            notch: NotchConfig::default(),
        }
    }

    /// The chain's response at `unit_delay` for samples `dt_s` apart: the notch's times the
    /// low-pass's.
    pub fn response(&self, dt_s: f32, unit_delay: Complex<f32>) -> Complex<f32> {
        self.notch.response(dt_s, unit_delay) * LowPass::response(self.cutoff_hz, dt_s, unit_delay)
    }
}

/// One axis's [`GyroFilters`] state.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct GyroFilter {
    notch: Notch,
    low_pass: LowPass,
}

impl GyroFilter {
    /// `rate_radps`, sampled `dt` s after the last, through `filters`' notch, then its
    /// low-pass.
    pub(crate) fn apply(&mut self, rate_radps: f32, dt: f32, filters: &GyroFilters) -> f32 {
        let notched = self.notch.apply(rate_radps, dt, &filters.notch);
        self.low_pass.apply(notched, dt, filters.cutoff_hz)
    }
}

#[cfg(test)]
mod tests;
