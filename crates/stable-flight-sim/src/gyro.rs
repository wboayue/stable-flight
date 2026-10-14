//! The error a [`GyroNoise`] puts on the reported body rates: see [`Gyro`].

use core::f32::consts::TAU;

use stable_flight::Vec3;

use crate::perturbations::GyroNoise;

#[cfg(test)]
mod tests;

/// [`GyroNoise`] with its generator's state.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Gyro {
    noise: GyroNoise,
    rng: SplitMix64,
}

impl Gyro {
    pub(crate) fn new(noise: GyroNoise) -> Self {
        Gyro {
            rng: SplitMix64(noise.seed),
            noise,
        }
    }

    /// The rate error at `time_s` seconds of simulated time, in rad/s: white noise from the
    /// next three draws plus the vibration tone. `None` when the noise is off, so a clean run
    /// draws nothing.
    pub(crate) fn error_radps(&mut self, time_s: f64) -> Option<Vec3> {
        let rms_radps = self.noise.rms_radps;
        let white_radps = (rms_radps != Vec3::zeros()).then(|| {
            Vec3::new(self.rng.normal(), self.rng.normal(), self.rng.normal())
                .component_mul(&rms_radps)
        });
        let tone_radps = self.noise.vibration.map(|vibration| {
            // The phase in cycles, wrapped in f64 so a long run keeps the tone's precision.
            let cycles = f64::from(vibration.frequency_hz) * time_s;
            let phase = (cycles - libm::floor(cycles)) as f32;
            vibration.amplitude_radps * libm::sinf(TAU * phase)
        });
        match (white_radps, tone_radps) {
            (None, None) => None,
            (white, tone) => Some(white.unwrap_or_default() + tone.unwrap_or_default()),
        }
    }
}

/// SplitMix64: a small, seedable generator whose every seed (zero included) gives a full
/// sequence. Statistical quality is ample for sensor noise; it isn't for cryptography.
#[derive(Debug, Clone, PartialEq)]
struct SplitMix64(u64);

impl SplitMix64 {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    /// Uniform in (0, 1]: the top 24 bits, so every value is exact in `f32` and never 0.
    fn uniform(&mut self) -> f32 {
        ((self.next_u64() >> 40) + 1) as f32 / (1u32 << 24) as f32
    }

    /// Standard normal, by Box–Muller (the cosine half).
    fn normal(&mut self) -> f32 {
        let radius = libm::sqrtf(-2.0 * libm::logf(self.uniform()));
        radius * libm::cosf(TAU * self.uniform())
    }
}
