//! Small numeric helpers shared by guidance, stabilizers, controllers, and mixers: [`saturate_unit`],
//! [`saturate_unit_vec2`], [`saturate_throttle`], [`wrap_angle`], and the per-component vector
//! helpers [`clamp_each`] and [`all_finite`], and [`euler_angles`].
//!
//! They implement the per-tick input policy (saturate, NaN → neutral, never let a non-finite
//! error into controller state, wrap angles), which is specified in `architecture.md`,
//! "Input Policy (no panics in the tick)".

use core::f32::consts::{PI, TAU};

use nalgebra::{SVector, UnitQuaternion};

use crate::Vec2;

#[cfg(test)]
mod tests;

/// Saturates a normalized command to [-1, 1]; NaN becomes 0 (neutral).
pub(crate) fn saturate_unit(value: f32) -> f32 {
    if value.is_nan() {
        0.0
    } else {
        value.clamp(-1.0, 1.0)
    }
}

/// [`saturate_unit`] applied to each component.
pub(crate) fn saturate_unit_vec2(value: Vec2<f32>) -> Vec2<f32> {
    Vec2::new(saturate_unit(value.x), saturate_unit(value.y))
}

/// `f32::clamp` applied to each component. NaN components stay NaN, like `f32::clamp`.
pub(crate) fn clamp_each<const D: usize>(
    value: SVector<f32, D>,
    min: f32,
    max: f32,
) -> SVector<f32, D> {
    value.map(|component| component.clamp(min, max))
}

/// True when every component is finite.
pub(crate) fn all_finite<const D: usize>(value: &SVector<f32, D>) -> bool {
    value.iter().all(|component| component.is_finite())
}

/// ZYX Euler angles `(roll, pitch, yaw)` of `attitude`, the same convention as
/// `UnitQuaternion::euler_angles`.
///
/// Closed form on the quaternion rather than nalgebra's, which goes through a rotation matrix
/// and `cos` (more flash on target) and whose gimbal-lock branch reports a finite pitch of
/// -π/2 and yaw 0 for a NaN attitude. Here NaN in gives NaN out, which the controllers treat as
/// neutral.
pub(crate) fn euler_angles(attitude: &UnitQuaternion<f32>) -> (f32, f32, f32) {
    let (w, x, y, z) = (attitude.w, attitude.i, attitude.j, attitude.k);
    let roll = libm::atan2f(2.0 * (y * z + w * x), w * w - x * x - y * y + z * z);
    let pitch = libm::asinf(-2.0 * (x * z - w * y));
    let yaw = libm::atan2f(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z);
    (roll, pitch, yaw)
}

/// Saturates a throttle or thrust command to [0, 1]; NaN becomes 0 (no thrust).
pub(crate) fn saturate_throttle(value: f32) -> f32 {
    if value.is_nan() {
        0.0
    } else {
        value.clamp(0.0, 1.0)
    }
}

/// Wraps an angle in radians to [-π, π). Non-finite input stays non-finite.
pub(crate) fn wrap_angle(angle: f32) -> f32 {
    rem_euclid(angle + PI, TAU) - PI
}

/// `f32::rem_euclid` for a positive `divisor`, which `core` lacks: the remainder in
/// [0, divisor). `%` truncates toward zero, so a negative remainder is shifted up.
fn rem_euclid(value: f32, divisor: f32) -> f32 {
    let remainder = value % divisor;
    if remainder < 0.0 {
        remainder + divisor
    } else {
        remainder
    }
}
