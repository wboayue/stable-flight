//! Two-pole low-pass filter shared by the copter's gyro path and the autotune: see [`LowPass`].

use core::f32::consts::{PI, SQRT_2};

use nalgebra::Complex;

/// Smooths one signal: passes what changes slower than its cutoff and cuts what changes faster
/// (gyro noise, motor vibration), at the cost of lag, about 5.6 ms at 40 Hz.
///
/// A second-order Butterworth low-pass by the bilinear transform with its cutoff prewarped:
/// with `K = tan(π·cutoff·dt)`, `y[k] = b₀·(x[k] + 2·x[k−1] + x[k−2]) − a₁·y[k−1] − a₂·y[k−2]`.
/// The first sample fills the history, so the output starts at the input instead of ramping
/// up from 0. A cutoff of 0 or less, or at or past Nyquist (`0.5/dt`), passes the signal
/// through. A non-finite sample passes through and leaves the history as it was.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct LowPass {
    /// The coefficients and the `(cutoff_hz, dt)` they were computed for.
    coefficients: Option<(Coefficients, f32, f32)>,
    /// The last two inputs and outputs (direct form I), so a new cutoff or tick reuses them
    /// as they are; `None` before the first sample.
    history: Option<History>,
}

impl LowPass {
    /// Filters `value`, sampled `dt` s after the last, at `cutoff_hz`. The coefficients are
    /// recomputed only when `cutoff_hz` or `dt` changes.
    pub(crate) fn apply(&mut self, value: f32, dt: f32, cutoff_hz: f32) -> f32 {
        if !value.is_finite() {
            return value;
        }
        let Some(coefficients) = self.coefficients(dt, cutoff_hz) else {
            self.history = None;
            return value;
        };
        let Coefficients { b0, a1, a2 } = coefficients;
        let History { inputs, outputs } = *self.history.get_or_insert(History {
            inputs: [value; 2],
            outputs: [value; 2],
        });
        let output = b0 * (value + 2.0 * inputs[0] + inputs[1]) - a1 * outputs[0] - a2 * outputs[1];
        self.history = Some(History {
            inputs: [value, inputs[0]],
            outputs: [output, outputs[0]],
        });
        output
    }

    /// The filter's response at `w_radps` for samples `dt_s` apart: gain and phase as a complex
    /// number, `b₀·(1 + 2·z⁻¹ + z⁻²) / (1 + a₁·z⁻¹ + a₂·z⁻²)` at `z⁻¹ = unit_delay`. 1 where
    /// [`apply`](Self::apply) passes the signal through.
    pub(crate) fn response(cutoff_hz: f32, dt_s: f32, unit_delay: Complex<f32>) -> Complex<f32> {
        let one = Complex::new(1.0, 0.0);
        let Some(Coefficients { b0, a1, a2 }) = Coefficients::new(cutoff_hz, dt_s) else {
            return one;
        };
        let z1 = unit_delay;
        (one + z1 * 2.0 + z1 * z1) * b0 / (one + z1 * a1 + z1 * z1 * a2)
    }

    fn coefficients(&mut self, dt: f32, cutoff_hz: f32) -> Option<Coefficients> {
        match self.coefficients {
            Some((coefficients, cached_hz, cached_dt))
                if cached_hz == cutoff_hz && cached_dt == dt =>
            {
                Some(coefficients)
            }
            _ => {
                let coefficients = Coefficients::new(cutoff_hz, dt)?;
                self.coefficients = Some((coefficients, cutoff_hz, dt));
                Some(coefficients)
            }
        }
    }
}

/// A [`LowPass`]'s past samples, newest first.
#[derive(Clone, Copy, Debug, PartialEq)]
struct History {
    inputs: [f32; 2],
    outputs: [f32; 2],
}

/// A [`LowPass`]'s weights; `b₁ = 2·b₀` and `b₂ = b₀`.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Coefficients {
    b0: f32,
    a1: f32,
    a2: f32,
}

impl Coefficients {
    /// `None` when the filter passes the signal through: a cutoff of 0 or less, at or past
    /// Nyquist, or not finite.
    fn new(cutoff_hz: f32, dt_s: f32) -> Option<Self> {
        let passes = !(cutoff_hz > 0.0 && cutoff_hz * dt_s < 0.5 && cutoff_hz.is_finite());
        if passes {
            return None;
        }
        let k = libm::tanf(PI * cutoff_hz * dt_s);
        let k_squared = k * k;
        let norm = 1.0 / (1.0 + SQRT_2 * k + k_squared);
        Some(Self {
            b0: k_squared * norm,
            a1: 2.0 * (k_squared - 1.0) * norm,
            a2: (1.0 - SQRT_2 * k + k_squared) * norm,
        })
    }
}

#[cfg(test)]
mod tests;
