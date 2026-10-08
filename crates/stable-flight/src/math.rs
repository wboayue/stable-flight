//! Small numeric helpers shared by guidance, stabilizers, controllers, and mixers:
//! [`saturate_unit`], [`saturate_symmetric`], [`saturate_throttle`], [`desaturate`],
//! [`cap_magnitude`], [`clamp_symmetric`] and its per-component [`clamp_symmetric_each`],
//! [`limit_or_zero`], [`all_finite`], [`assert_dt`], [`tilt_cosine`], and [`wrap_angle`] and
//! [`euler_angles`] (both public at the crate root).
//!
//! They implement the per-tick input policy (saturate, NaN → neutral, never let a non-finite
//! error into controller state, wrap angles), which is specified in `architecture.md`,
//! "Input Policy (no panics in the tick)". [`assert_dt`] is the policy's one per-tick panic.

use core::f32::consts::{PI, TAU};

use nalgebra::{SVector, UnitQuaternion};

use crate::Vec2;

#[cfg(test)]
mod tests;

/// Panics unless `dt` is positive and finite: the one per-tick assert the input policy allows.
/// Runners guarantee it; a zero, negative, NaN, or infinite `dt` is a caller bug, and an
/// infinite one would poison integrals (`0 · ∞ = NaN`).
#[track_caller]
pub(crate) fn assert_dt(dt: f32) {
    assert!(dt > 0.0 && dt.is_finite(), "dt must be positive and finite");
}

/// Fits mixer outputs into `[0, 1]`, keeping the moments (attitude over thrust), like PX4's
/// airmode. Each output is `collective` plus its moment share.
///
/// 1. If the outputs span more than 1, the moment shares are scaled down about `collective`
///    until they span exactly 1, keeping their ratios.
/// 2. All outputs then shift by the smallest amount that fits them in `[0, 1]`: down if one
///    is above 1, up if one is below 0. Collective thrust gives way.
///
/// A `collective` of 0 or less (throttle fully down) never shifts up: the shares below 0 are
/// clipped instead, so an armed vehicle at zero throttle (on the ground, say) isn't lifted by
/// its moments alone. Outputs are then clamped to `[0, 1]`. Inputs are expected finite (the
/// mixer saturates them); a NaN output stays NaN.
pub(crate) fn desaturate(outputs: &mut [f32], collective: f32) {
    let (low, high) = outputs
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), &o| {
            (lo.min(o), hi.max(o))
        });
    let spread = high - low;
    let scale = if spread > 1.0 { 1.0 / spread } else { 1.0 };
    let (low, high) = (
        collective + scale * (low - collective),
        collective + scale * (high - collective),
    );

    let shift = if high > 1.0 {
        1.0 - high
    } else if low < 0.0 && collective > 0.0 {
        -low
    } else {
        0.0
    };

    for output in outputs.iter_mut() {
        *output = (collective + scale * (*output - collective) + shift).clamp(0.0, 1.0);
    }
}

/// Scales `v` down, keeping its direction, so its length is at most `max`. Unlike per-axis
/// saturation, a diagonal request stays diagonal. A non-finite `v` has no direction, so it
/// becomes zero (neutral), per the input policy.
pub(crate) fn cap_magnitude(v: Vec2, max: f32) -> Vec2 {
    let length = v.norm();
    if !length.is_finite() {
        Vec2::zeros()
    } else if length > max {
        v * (max / length)
    } else {
        v
    }
}

/// Saturates a normalized command to [-1, 1]; NaN becomes 0 (neutral).
pub(crate) fn saturate_unit(value: f32) -> f32 {
    saturate_symmetric(value, 1.0)
}

/// Saturates `value` to `[-limit, limit]`; NaN becomes 0 (neutral), and a negative or NaN
/// `limit` acts as 0, so neither a bad input nor a bad config panics the loop.
pub(crate) fn saturate_symmetric(value: f32, limit: f32) -> f32 {
    if value.is_nan() {
        0.0
    } else {
        clamp_symmetric(value, limit)
    }
}

/// Clamps `value` to `[-limit, limit]`; NaN `value` stays NaN, like `f32::clamp`.
///
/// Unlike `f32::clamp`, it never panics: a negative or NaN `limit` acts as 0 (neutral output).
/// Limits come from config passed in every tick, so a bad config must not panic the loop.
pub(crate) fn clamp_symmetric(value: f32, limit: f32) -> f32 {
    let limit = limit_or_zero(limit);
    value.clamp(-limit, limit)
}

/// A config limit as given when it is non-negative, else 0 (neutral); NaN also becomes 0.
pub(crate) fn limit_or_zero(limit: f32) -> f32 {
    // `>=` is false for NaN.
    if limit >= 0.0 { limit } else { 0.0 }
}

/// [`clamp_symmetric`] applied to each component.
pub(crate) fn clamp_symmetric_each<const D: usize>(
    value: SVector<f32, D>,
    limit: f32,
) -> SVector<f32, D> {
    value.map(|component| clamp_symmetric(component, limit))
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

/// Floor on [`tilt_cosine`]: past 60° of tilt the thrust boost stops growing, so a
/// near-inverted estimate can't demand unbounded thrust.
const MIN_TILT_COSINE: f32 = 0.5;

/// Cosine of the tilt from level, `cos(roll) · cos(pitch)` (yaw doesn't change it), floored at
/// [`MIN_TILT_COSINE`]: what dividing a level thrust by gives the thrust that holds the same
/// vertical component. A non-finite attitude counts as level (1, no boost).
///
/// Read from the quaternion directly (`1 − 2(x² + y²)`, the rotation matrix's `(2, 2)` entry):
/// no trig, no matrix.
pub(crate) fn tilt_cosine(attitude: &UnitQuaternion<f32>) -> f32 {
    let (x, y) = (attitude.i, attitude.j);
    let cosine = 1.0 - 2.0 * (x * x + y * y);
    if cosine.is_finite() {
        cosine.max(MIN_TILT_COSINE)
    } else {
        1.0
    }
}

/// Saturates a throttle or thrust command to [0, 1]; NaN becomes 0 (no thrust).
pub(crate) fn saturate_throttle(value: f32) -> f32 {
    if value.is_nan() {
        0.0
    } else {
        value.clamp(0.0, 1.0)
    }
}

/// Wraps an angle in radians to [-π, π); non-finite input gives NaN.
///
/// Float rounding can return exactly `+π` for inputs just below an odd multiple of π, so
/// treat the range as closed, [-π, π], when that matters.
///
/// # Examples
///
/// ```
/// use core::f32::consts::{FRAC_PI_2, PI};
/// use stable_flight::wrap_angle;
///
/// assert!((wrap_angle(3.0 * FRAC_PI_2) + FRAC_PI_2).abs() < 1e-6); // 270° → -90°
/// assert_eq!(wrap_angle(PI), -PI);
/// assert!(wrap_angle(f32::NAN).is_nan());
/// ```
pub fn wrap_angle(angle: f32) -> f32 {
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
