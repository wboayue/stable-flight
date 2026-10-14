//! The ground plane at `z = 0`: see [`contact`].

use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::{GRAVITY, Vec3, euler_angles};

use crate::body::Body;

#[cfg(test)]
mod tests;

/// What the ground needs to know about a body resting on it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Ground {
    /// The body's mass, in kg.
    pub mass_kg: f32,
    /// Friction coefficient against the ground (Coulomb, unitless).
    pub friction: f32,
    /// The body's attitude resting on the ground, relative to level with its heading kept:
    /// identity on landing gear, nose up for a tailsitter on its tail.
    pub rest: UnitQuaternion<f32>,
}

/// Keeps `body` from sinking below the ground after a step, and returns whether it rests on
/// it.
///
/// Touching the ground stops any sinking. While `lift_n`, the upward force from thrust and
/// lift, is less than the weight, the body rests: its landing gear (or tail) holds it at
/// `ground.rest` (heading kept, as ArduPilot's tailsitter ground model does,
/// `SIM_Aircraft.cpp:745-765`), it stops turning, and Coulomb friction (`ground.friction` times the weight the ground
/// carries) slows its sliding over the step `h`. Once the lift carries the weight it lifts off.
///
/// The leveling is instant, not a modeled tip-over: the simulator aims at plausible landings
/// and takeoffs, not at gear dynamics.
pub(crate) fn contact(body: &mut Body, ground: &Ground, lift_n: f32, h: f32) -> bool {
    // NED: above the ground is z < 0.
    if body.position_ned_m.z < 0.0 {
        return false;
    }
    body.position_ned_m.z = 0.0;
    body.velocity_ned_mps.z = body.velocity_ned_mps.z.min(0.0);

    let normal_n = ground.mass_kg * GRAVITY - lift_n;
    if normal_n <= 0.0 {
        return false;
    }

    // The heading of the rest frame: a tailsitter's nose-up body has none.
    let (_, _, heading_rad) = euler_angles(&(body.attitude * ground.rest.inverse()));
    body.attitude = UnitQuaternion::from_euler_angles(0.0, 0.0, heading_rad) * ground.rest;
    body.angular_velocity_radps = Vec3::zeros();
    body.velocity_ned_mps.z = 0.0;

    let sliding = body.velocity_ned_mps;
    let speed_mps = sliding.norm();
    if speed_mps > 0.0 {
        let slowdown_mps = ground.friction * normal_n / ground.mass_kg * h;
        body.velocity_ned_mps = sliding * ((speed_mps - slowdown_mps).max(0.0) / speed_mps);
    }
    true
}
