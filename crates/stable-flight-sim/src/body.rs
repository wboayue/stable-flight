//! Rigid-body motion in the NED world frame and FRD body frame: see [`Body`].

use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::{GRAVITY, Vec3};

#[cfg(test)]
mod tests;

/// The simulated body's state, in aerospace frames: world NED (`z` down), body FRD.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Body {
    /// World position (north, east, down), in m; the ground is `z = 0`.
    pub position_ned_m: Vec3,
    /// World velocity (north, east, down), in m/s.
    pub velocity_ned_mps: Vec3,
    /// World acceleration (north, east, down) over the last tick, in m/s².
    pub acceleration_ned_mps2: Vec3,
    /// Rotation from NED world to FRD body: `attitude * v_body` is `v_body` in world axes.
    pub attitude: UnitQuaternion<f32>,
    /// FRD body rates (roll, pitch, yaw), in rad/s.
    pub angular_velocity_radps: Vec3,
}

impl Default for Body {
    /// At rest on the ground at the origin, level, nose north.
    fn default() -> Self {
        Body {
            position_ned_m: Vec3::zeros(),
            velocity_ned_mps: Vec3::zeros(),
            acceleration_ned_mps2: Vec3::zeros(),
            attitude: UnitQuaternion::identity(),
            angular_velocity_radps: Vec3::zeros(),
        }
    }
}

/// A body's mass and principal moments of inertia.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct MassProperties {
    /// Total mass, in kg.
    pub mass_kg: f32,
    /// Principal moments of inertia about the FRD body axes (roll, pitch, yaw), in kg·m².
    pub inertia_kgm2: Vec3,
}

/// The forces and torques acting on the body during one integration step, gravity aside.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Loads {
    /// Force (north, east, down), in N: thrust, drag, and lift.
    pub force_ned_n: Vec3,
    /// Torque on the FRD body axes, in N·m.
    pub torque_nm: Vec3,
    /// Rotational damping torque opposing the body rates, in N·m: subtracted from the torque.
    pub damping_nm: Vec3,
}

impl Body {
    /// Advances `h` seconds under `loads` and gravity with semi-implicit Euler: velocities
    /// first, then position and attitude from the new velocities. Rotation follows Euler's
    /// equations for a diagonal inertia, gyroscopic term included.
    pub(crate) fn step(&mut self, mass: &MassProperties, loads: &Loads, h: f32) {
        let acceleration_mps2 = loads.force_ned_n / mass.mass_kg + Vec3::new(0.0, 0.0, GRAVITY);
        self.velocity_ned_mps += acceleration_mps2 * h;
        self.position_ned_m += self.velocity_ned_mps * h;

        let rates = self.angular_velocity_radps;
        let momentum = mass.inertia_kgm2.component_mul(&rates);
        let net_torque_nm = loads.torque_nm - rates.cross(&momentum) - loads.damping_nm;
        self.angular_velocity_radps += net_torque_nm.component_div(&mass.inertia_kgm2) * h;
        // Body rates turn the body frame, so the increment composes on the right.
        self.attitude *= UnitQuaternion::from_scaled_axis(self.angular_velocity_radps * h);
        self.attitude.renormalize_fast();
    }

    /// The body's velocity through the air (north, east, down), in m/s, in `wind_ned_mps`.
    pub(crate) fn air_relative_velocity_ned_mps(&self, wind_ned_mps: Vec3) -> Vec3 {
        self.velocity_ned_mps - wind_ned_mps
    }
}

/// A world vector between NED and the contract's north-east-up axes: `z` negated, so the
/// conversion is its own inverse.
pub(crate) fn flip_z(v: Vec3) -> Vec3 {
    Vec3::new(v.x, v.y, -v.z)
}
