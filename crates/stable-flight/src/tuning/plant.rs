//! A simulated loop for tuning tests: `K / (s(τ₁s+1)(τ₂s+1))` with input dead time, a constant
//! input offset, and measurement noise.
//!
//! Models a rate axis: the normalized moment goes through motor lag (`τ₁`) and a second lag
//! (`τ₂`), then integrates to a body rate (`K`, rad/s² per unit moment). Dead time stands in
//! for link latency; the offset for a CG offset or yaw drag.

use std::collections::VecDeque;
use std::f32::consts::PI;

use nalgebra::UnitQuaternion;

use crate::{Vec3, VehicleState};

/// Integration sub-steps per control tick.
const SUBSTEPS: usize = 40;

#[derive(Clone, Copy, Debug)]
pub(crate) struct PlantParams {
    /// Integrator gain `K`.
    pub gain: f32,
    /// First lag `τ₁`, in seconds.
    pub lag1_s: f32,
    /// Second lag `τ₂`, in seconds.
    pub lag2_s: f32,
    /// Input dead time, in seconds; rounded to whole control ticks.
    pub delay_s: f32,
    /// Added to every input, in input units.
    pub offset: f32,
    /// Peak of the uniform measurement noise, in output units.
    pub noise: f32,
}

impl PlantParams {
    /// A rate-axis-like plant: `K` 20, `τ₁` 20 ms, `τ₂` 50 ms, no delay, offset, or noise.
    pub(crate) fn rate_axis() -> Self {
        Self {
            gain: 20.0,
            lag1_s: 0.02,
            lag2_s: 0.05,
            delay_s: 0.0,
            offset: 0.0,
            noise: 0.0,
        }
    }
}

pub(crate) struct Plant {
    params: PlantParams,
    dt: f32,
    /// Outputs of the two lags, and the integrated output.
    lag1: f32,
    lag2: f32,
    output: f32,
    /// Inputs still in flight through the dead time.
    queue: VecDeque<f32>,
    /// xorshift32 state for the noise.
    rng: u32,
}

impl Plant {
    /// A plant at rest, stepped every `dt` seconds.
    pub(crate) fn new(params: PlantParams, dt: f32) -> Self {
        let delay_ticks = (params.delay_s / dt).round() as usize;
        Self {
            params,
            dt,
            lag1: 0.0,
            lag2: 0.0,
            output: 0.0,
            queue: VecDeque::from(vec![0.0; delay_ticks]),
            rng: 0x2545_f491,
        }
    }

    /// The measured output (with noise).
    pub(crate) fn measure(&mut self) -> f32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        let unit = self.rng as f32 / u32::MAX as f32 * 2.0 - 1.0;
        self.output + self.params.noise * unit
    }

    /// The true output, without noise.
    pub(crate) fn output(&self) -> f32 {
        self.output
    }

    /// Holds `input` for one control tick.
    pub(crate) fn step(&mut self, input: f32) {
        self.queue.push_back(input);
        let applied = self.queue.pop_front().unwrap_or(input) + self.params.offset;
        let h = self.dt / SUBSTEPS as f32;
        let p = &self.params;
        for _ in 0..SUBSTEPS {
            self.lag1 += h * (applied - self.lag1) / p.lag1_s;
            self.lag2 += h * (self.lag1 - self.lag2) / p.lag2_s;
            self.output += h * p.gain * self.lag2;
        }
    }

    /// The exact ultimate point `(Ku, Tu)` of the plant as a sampled loop: dead time plus one
    /// half-tick (the zero-order hold).
    pub(crate) fn ultimate_point(&self) -> (f32, f32) {
        let p = &self.params;
        let delay_s = (p.delay_s / self.dt).round() * self.dt + 0.5 * self.dt;
        let phase =
            |w: f32| -PI / 2.0 - (w * p.lag1_s).atan() - (w * p.lag2_s).atan() - w * delay_s;
        // Phase falls monotonically from −90°: bisect for −180°.
        let (mut low, mut high) = (1e-3_f32, 1e4_f32);
        for _ in 0..200 {
            let mid = 0.5 * (low + high);
            if phase(mid) > -PI {
                low = mid;
            } else {
                high = mid;
            }
        }
        let w = 0.5 * (low + high);
        let magnitude = p.gain
            / (w * (1.0 + (w * p.lag1_s).powi(2)).sqrt() * (1.0 + (w * p.lag2_s).powi(2)).sqrt());
        (1.0 / magnitude, 2.0 * PI / w)
    }
}

/// Three decoupled rate axes (roll, pitch, yaw), each a [`Plant`] from moment to body rate,
/// with their angles integrated: a small-angle vehicle for stabilizer tuning tests.
pub(crate) struct Axes {
    rates: [Plant; 3],
    /// Roll, pitch, and yaw, in rad.
    angles_rad: Vec3,
    dt: f32,
}

impl Axes {
    /// Axes at `angles_rad`, at rest, stepped every `dt` seconds.
    pub(crate) fn new(params: [PlantParams; 3], angles_rad: Vec3, dt: f32) -> Self {
        Self {
            rates: params.map(|p| Plant::new(p, dt)),
            angles_rad,
            dt,
        }
    }

    /// Roll, pitch, and yaw, in rad.
    pub(crate) fn angles_rad(&self) -> Vec3 {
        self.angles_rad
    }

    /// Sets one angle (0 roll, 1 pitch, 2 yaw), in rad: a disturbance.
    pub(crate) fn tilt(&mut self, axis: usize, angle_rad: f32) {
        self.angles_rad[axis] = angle_rad;
    }

    /// The state the stabilizer sees: attitude from the angles, body rates as measured.
    pub(crate) fn state(&mut self) -> VehicleState {
        let a = self.angles_rad;
        VehicleState {
            attitude: UnitQuaternion::from_euler_angles(a.x, a.y, a.z),
            angular_velocity_radps: Vec3::from_fn(|axis, _| self.rates[axis].measure()),
            roll_rad: a.x,
            pitch_rad: a.y,
            heading_rad: a.z,
            ..VehicleState::default()
        }
    }

    /// Applies `moments` (roll, pitch, yaw) for one tick.
    pub(crate) fn step(&mut self, moments: Vec3) {
        for (axis, plant) in self.rates.iter_mut().enumerate() {
            plant.step(moments[axis]);
            self.angles_rad[axis] += self.dt * plant.output();
        }
    }
}
