//! The ground plane at `z = 0`: see [`contact`].

use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::{GRAVITY, Vec3, euler_angles};

use crate::body::Body;
use crate::model::QuadModel;

#[cfg(test)]
mod tests;

/// Keeps `body` from sinking below the ground after a step, and returns whether it rests on
/// it.
///
/// Touching the ground stops any sinking. While the thrust's upward part is less than the
/// weight, the body rests: its landing gear holds it level (heading kept), it stops turning,
/// and Coulomb friction (`model.ground_friction` times the weight the ground carries) slows its
/// sliding over the step `h`. Once the thrust carries the weight it lifts off.
///
/// The leveling is instant, not a modeled tip-over: the simulator aims at plausible landings
/// and takeoffs, not at gear dynamics.
pub(crate) fn contact(body: &mut Body, model: &QuadModel, thrust_n: f32, h: f32) -> bool {
    // NED: above the ground is z < 0.
    if body.position_ned_m.z < 0.0 {
        return false;
    }
    body.position_ned_m.z = 0.0;
    body.velocity_ned_mps.z = body.velocity_ned_mps.z.min(0.0);

    let lift_n = -(body.attitude * Vec3::new(0.0, 0.0, -thrust_n)).z;
    let normal_n = model.mass_kg * GRAVITY - lift_n;
    if normal_n <= 0.0 {
        return false;
    }

    let (_, _, heading_rad) = euler_angles(&body.attitude);
    body.attitude = UnitQuaternion::from_euler_angles(0.0, 0.0, heading_rad);
    body.angular_velocity_radps = Vec3::zeros();
    body.velocity_ned_mps.z = 0.0;

    let sliding = body.velocity_ned_mps;
    let speed_mps = sliding.norm();
    if speed_mps > 0.0 {
        let slowdown_mps = model.ground_friction * normal_n / model.mass_kg * h;
        body.velocity_ned_mps = sliding * ((speed_mps - slowdown_mps).max(0.0) / speed_mps);
    }
    true
}
