//! A simulated loop for tuning tests: `K / (s(τ₁s+1)(τ₂s+1))` with input dead time, a constant
//! input offset, and measurement noise.
//!
//! Models a rate axis: the normalized moment goes through motor lag (`τ₁`) and a second lag
//! (`τ₂`), then integrates to a body rate (`K`, rad/s² per unit moment). Dead time stands in
//! for link latency; the offset for a CG offset or yaw drag.

use std::collections::VecDeque;

use nalgebra::UnitQuaternion;

use crate::controllers::{PlaneRateControllerConfig, RateGains};
use crate::stabilizers::plane::PlaneStabilizerConfig;
use crate::{ActuatorCommand, ControlOutput, Mixer, Vec2, Vec3, VehicleState};

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
    /// Peak of a uniform disturbance added to the input each tick, in input units: turbulence
    /// and unmodeled dynamics that make each limit cycle a little different.
    pub disturbance: f32,
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
            disturbance: 0.0,
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
        self.output + self.params.noise * self.uniform()
    }

    /// A uniform sample in [-1, 1] (xorshift32).
    fn uniform(&mut self) -> f32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        self.rng as f32 / u32::MAX as f32 * 2.0 - 1.0
    }

    /// The true output, without noise.
    pub(crate) fn output(&self) -> f32 {
        self.output
    }

    /// Holds `input` for one control tick.
    pub(crate) fn step(&mut self, input: f32) {
        self.queue.push_back(input);
        let disturbance = self.params.disturbance * self.uniform();
        let applied = self.queue.pop_front().unwrap_or(input) + self.params.offset + disturbance;
        let h = self.dt / SUBSTEPS as f32;
        let p = &self.params;
        for _ in 0..SUBSTEPS {
            self.lag1 += h * (applied - self.lag1) / p.lag1_s;
            self.lag2 += h * (self.lag1 - self.lag2) / p.lag2_s;
            self.output += h * p.gain * self.lag2;
        }
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

/// A wing's roll and pitch, each a rate driven by its moment against its damping: rate
/// acceleration = control × moment − damping × rate, angles integrated from the rates (small
/// angles). Gentle enough for the default gains to fly.
#[derive(Clone, Debug, Default)]
pub(crate) struct Wing {
    pub(crate) angles_rad: Vec2,
    pub(crate) rates_radps: Vec2,
}

/// Rate acceleration per unit moment, rad/s².
pub(crate) const CONTROL_RADPS2: f32 = 40.0;
/// Rate damping, 1/s.
pub(crate) const DAMPING_PER_S: f32 = 8.0;

impl Wing {
    pub(crate) fn state(&self) -> VehicleState {
        VehicleState {
            attitude: UnitQuaternion::from_euler_angles(self.angles_rad.x, self.angles_rad.y, 0.0),
            angular_velocity_radps: Vec3::new(self.rates_radps.x, self.rates_radps.y, 0.0),
            ..Default::default()
        }
    }

    pub(crate) fn step(&mut self, dt: f32, moments: Vec3) {
        for axis in 0..2 {
            let accel = CONTROL_RADPS2 * moments[axis] - DAMPING_PER_S * self.rates_radps[axis];
            self.rates_radps[axis] += accel * dt;
            self.angles_rad[axis] += self.rates_radps[axis] * dt;
        }
    }
}

/// Plane gains that fly the [`Wing`]: feedforward matching its damping, a little P and I.
pub(crate) fn wing_config() -> PlaneStabilizerConfig {
    let rate = RateGains {
        p: 0.1,
        i: 0.1,
        ff: DAMPING_PER_S / CONTROL_RADPS2,
    };
    PlaneStabilizerConfig {
        rate: PlaneRateControllerConfig {
            roll: rate,
            pitch: rate,
            ..Default::default()
        },
        ..Default::default()
    }
}

/// Passes the stabilizer's moments straight to a test plant, in `servos[0..3]`, and its thrust
/// in `motors[0]`.
pub(crate) struct MomentMixer;

impl Mixer for MomentMixer {
    fn mix(&mut self, controls: &ControlOutput) -> ActuatorCommand {
        let mut command = ActuatorCommand::default();
        command.servos[..3].copy_from_slice(controls.moments.as_slice());
        command.motors[0] = controls.thrust;
        command
    }
}
