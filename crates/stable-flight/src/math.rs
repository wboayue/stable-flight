//! Small numeric helpers shared by guidance, stabilizers, controllers, and mixers: [`saturate_unit`],
//! [`saturate_unit_vec2`], [`saturate_throttle`], [`wrap_angle`], [`desaturate`], [`cap_magnitude`], the per-component vector
//! helpers [`clamp_each`] and [`all_finite`], and [`euler_angles`] (public at the crate root).
//!
//! They implement the per-tick input policy (saturate, NaN → neutral, never let a non-finite
//! error into controller state, wrap angles), which is specified in `architecture.md`,
//! "Input Policy (no panics in the tick)".

use core::f32::consts::{PI, TAU};

use nalgebra::{SVector, UnitQuaternion};

use crate::Vec2;

#[cfg(test)]
mod tests;

/// Fits mixer outputs into `[0, 1]`, keeping their differences when it can: if any output
/// exceeds 1, all are lowered by the excess (attitude over thrust), then each is clamped to
/// `[0, 1]`. Only the top is shifted, so an output below 0 is still clipped.
pub(crate) fn desaturate(outputs: &mut [f32]) {
    let excess = outputs.iter().fold(0.0_f32, |m, &o| m.max(o - 1.0));
    for output in outputs.iter_mut() {
        *output = (*output - excess).clamp(0.0, 1.0);
    }
}

/// Scales `v` down, keeping its direction, so its length is at most `max`. Unlike per-axis
/// saturation, a diagonal request stays diagonal.
pub(crate) fn cap_magnitude(v: Vec2<f32>, max: f32) -> Vec2<f32> {
    let length = v.norm();
    if length > max { v * (max / length) } else { v }
}

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
    value.map(saturate_unit)
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

/// ZYX Euler angles `(roll, pitch, yaw)` in radians of an attitude quaternion, in the
/// [frame contract](crate::VehicleState#frame-contract)'s convention (the same as
/// `UnitQuaternion::euler_angles`). Use this rather than nalgebra's method to read Euler angles
/// from [`VehicleState::attitude`](crate::VehicleState::attitude).
///
/// It is a closed form on the quaternion. nalgebra's goes through a rotation matrix and `cos`
/// (more flash on target), and its gimbal-lock branch reports a finite pitch of -π/2 and yaw 0
/// for a NaN attitude. Here NaN in gives NaN out, which the controllers treat as neutral. Near
/// ±90° pitch the `asin` argument is clamped, so float rounding can't turn a valid attitude
/// into NaN.
///
/// # Examples
///
/// ```
/// use stable_flight::euler_angles;
/// use stable_flight::nalgebra::UnitQuaternion;
///
/// let attitude = UnitQuaternion::from_euler_angles(0.1, -0.2, 1.5);
/// let (roll, pitch, yaw) = euler_angles(&attitude);
/// assert!((roll - 0.1).abs() < 1e-6 && (pitch + 0.2).abs() < 1e-6 && (yaw - 1.5).abs() < 1e-6);
/// ```
pub fn euler_angles(attitude: &UnitQuaternion<f32>) -> (f32, f32, f32) {
    let (w, x, y, z) = (attitude.w, attitude.i, attitude.j, attitude.k);
    let roll = libm::atan2f(2.0 * (y * z + w * x), w * w - x * x - y * y + z * z);
    // `clamp` passes NaN through, so a NaN attitude still gives NaN pitch.
    let pitch = libm::asinf((-2.0 * (x * z - w * y)).clamp(-1.0, 1.0));
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
