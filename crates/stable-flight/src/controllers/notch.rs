//! Notch filter for the copter's gyro path: see [`NotchConfig`].

use core::f32::consts::PI;

use nalgebra::Complex;

use super::{InvalidConfig, check};

/// Highest center a notch takes, as a share of the sample rate: past it the filter passes the
/// signal through (ArduPilot's `HARMONIC_NYQUIST_CUTOFF`).
const MAX_CENTER_SHARE: f32 = 0.48;

/// Lowest center a notch takes, as a share of its bandwidth: a lower one is raised to it, so
/// the band's lower edge stays above 0 Hz, as ArduPilot's `HarmonicNotchFilter::init` does.
const MIN_CENTER_PER_BANDWIDTH: f32 = 0.52;

/// Where a gyro notch cuts and how deep: one vibration tone, such as a motor's, removed from
/// the body rates before the low-pass, so the rate loop's D doesn't amplify it into the
/// moments. A low-pass alone barely touches a tone just above its cutoff.
///
/// ArduPilot's harmonic notch in its fixed mode (`INS_HNTCH_FREQ`, `INS_HNTCH_BW`,
/// `INS_HNTCH_ATT`, `NotchFilter`), described, not copied: a biquad whose band edges sit
/// geometrically about the center, `center − bandwidth/2` below it, with a finite depth.
///
/// # Examples
///
/// ```
/// use stable_flight::controllers::NotchConfig;
///
/// assert!(!NotchConfig::default().is_on());
/// let notch = NotchConfig { center_hz: 90.0, bandwidth_hz: 45.0, attenuation_db: 40.0 };
/// assert!(notch.validate().is_ok());
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NotchConfig {
    /// The tone cut, in Hz; 0 turns the notch off, and so does one at or past 0.48 of the
    /// sample rate. One under 0.52 of the bandwidth cuts at 0.52 of it. Non-negative and
    /// finite.
    pub center_hz: f32,
    /// Width of the band cut, in Hz: wider takes a tone that wanders, at more lag below the
    /// center (ArduPilot suggests half the center); 0 cuts nothing. Non-negative and finite.
    pub bandwidth_hz: f32,
    /// How deep the notch cuts at its center, in dB. Positive and finite: past 40 dB it cuts
    /// hard rather than attenuating (ArduPilot's `INS_HNTCH_ATT` default is 40).
    pub attenuation_db: f32,
}

impl Default for NotchConfig {
    /// Off, with ArduPilot's width and depth (`INS_HNTCH_BW` 40 Hz, `INS_HNTCH_ATT` 40 dB)
    /// ready for a center.
    fn default() -> Self {
        Self {
            center_hz: 0.0,
            bandwidth_hz: 40.0,
            attenuation_db: 40.0,
        }
    }
}

impl NotchConfig {
    /// Whether the notch cuts anything: a positive center.
    pub fn is_on(&self) -> bool {
        self.center_hz > 0.0
    }

    /// Checks each field's stated requirement; each stands alone, so a param can change any
    /// one of them.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check(
            "center_hz",
            self.center_hz >= 0.0 && self.center_hz.is_finite(),
            "non-negative and finite",
        )?;
        check(
            "bandwidth_hz",
            self.bandwidth_hz >= 0.0 && self.bandwidth_hz.is_finite(),
            "non-negative and finite",
        )?;
        check(
            "attenuation_db",
            self.attenuation_db > 0.0 && self.attenuation_db.is_finite(),
            "positive and finite",
        )
    }

    /// The notch's response at `unit_delay` for samples `dt_s` apart, `(b₀ + b₁·z⁻¹ + b₂·z⁻²)
    /// / (a₀ + a₁·z⁻¹ + a₂·z⁻²)`: 1 where the filter passes the signal through. The
    /// autotune's design models it, as it does the low-pass.
    pub fn response(&self, dt_s: f32, unit_delay: Complex<f32>) -> Complex<f32> {
        let one = Complex::new(1.0, 0.0);
        let Some(Coefficients { b, a }) = Coefficients::new(self, dt_s) else {
            return one;
        };
        let z1 = unit_delay;
        (one * b[0] + z1 * b[1] + z1 * z1 * b[2]) / (one * a[0] + z1 * a[1] + z1 * z1 * a[2])
    }
}

/// Cuts one tone out of a signal ([`NotchConfig`]); the first sample fills the history, so the
/// output starts at the input. A non-finite sample passes through and leaves the history as it
/// was.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Notch {
    /// The coefficients and the config and `dt` they were computed for.
    coefficients: Option<(Coefficients, NotchConfig, f32)>,
    /// The last two inputs and outputs, newest first (direct form I); `None` before the first
    /// sample.
    history: Option<([f32; 2], [f32; 2])>,
}

impl Notch {
    /// Filters `value`, sampled `dt` s after the last, through `config`'s notch. The
    /// coefficients are recomputed only when `config` or `dt` changes.
    pub(crate) fn apply(&mut self, value: f32, dt: f32, config: &NotchConfig) -> f32 {
        if !value.is_finite() {
            return value;
        }
        let Some(Coefficients { b, a }) = self.coefficients(dt, config) else {
            self.history = None;
            return value;
        };
        let (inputs, outputs) = *self.history.get_or_insert(([value; 2], [value; 2]));
        let output = (b[0] * value + b[1] * inputs[0] + b[2] * inputs[1]
            - a[1] * outputs[0]
            - a[2] * outputs[1])
            / a[0];
        self.history = Some(([value, inputs[0]], [output, outputs[0]]));
        output
    }

    fn coefficients(&mut self, dt: f32, config: &NotchConfig) -> Option<Coefficients> {
        match self.coefficients {
            Some((coefficients, cached, cached_dt)) if cached == *config && cached_dt == dt => {
                Some(coefficients)
            }
            _ => {
                let coefficients = Coefficients::new(config, dt)?;
                self.coefficients = Some((coefficients, *config, dt));
                Some(coefficients)
            }
        }
    }
}

/// A [`Notch`]'s weights: numerator `b` and denominator `a`, each for `z⁰`, `z⁻¹`, `z⁻²`.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Coefficients {
    b: [f32; 3],
    a: [f32; 3],
}

impl Coefficients {
    /// `None` when the notch passes the signal through: off, no width, at or past
    /// [`MAX_CENTER_SHARE`] of the sample rate, or not a valid config. A center under
    /// [`MIN_CENTER_PER_BANDWIDTH`] of the bandwidth is raised to it.
    ///
    /// The band's lower edge is `center − bandwidth/2`; its width in octaves, `n = 2·log₂(center
    /// / lower)`, gives `Q = √(2ⁿ) / (2ⁿ − 1)`, and with `ω = 2π·center·dt`, `α = sin ω / 2Q`
    /// and the depth `g = 10^(−attenuation/20)`: `b = [1 + α·g, −2·cos ω, 1 − α·g]`,
    /// `a = [1 + α, −2·cos ω, 1 − α]`. At the center the gain is `g`; far from it, 1.
    fn new(config: &NotchConfig, dt_s: f32) -> Option<Self> {
        let center_hz = config
            .center_hz
            .max(MIN_CENTER_PER_BANDWIDTH * config.bandwidth_hz);
        let center_share = center_hz * dt_s;
        let cuts = config.is_on()
            && config.bandwidth_hz > 0.0
            && center_share < MAX_CENTER_SHARE
            && config.validate().is_ok();
        if !cuts {
            return None;
        }
        let lower_hz = center_hz - 0.5 * config.bandwidth_hz;
        let octaves = 2.0 * libm::log2f(center_hz / lower_hz);
        let ratio = libm::exp2f(octaves);
        let q = libm::sqrtf(ratio) / (ratio - 1.0);
        let omega = 2.0 * PI * center_share;
        let alpha = libm::sinf(omega) / (2.0 * q);
        let depth = libm::powf(10.0, -config.attenuation_db / 20.0);
        let cos_omega = libm::cosf(omega);
        Some(Self {
            b: [1.0 + alpha * depth, -2.0 * cos_omega, 1.0 - alpha * depth],
            a: [1.0 + alpha, -2.0 * cos_omega, 1.0 - alpha],
        })
    }
}

#[cfg(test)]
mod tests;
