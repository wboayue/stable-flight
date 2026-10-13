//! Small numeric helpers shared by guidance, stabilizers, controllers, and mixers:
//! [`saturate_unit`], [`saturate_symmetric`], [`saturate_throttle`], [`desaturate`],
//! [`cap_magnitude`], [`clamp_symmetric`] and its per-component [`clamp_symmetric_each`],
//! [`limit_or_zero`], [`all_finite`], [`assert_dt`], [`tilt_boost`], and [`wrap_angle`] and
//! [`euler_angles`] (both public at the crate root).
//!
//! They implement the per-tick input policy (saturate, NaN → neutral, never let a non-finite
//! error into controller state, wrap angles), which is specified in `architecture.md`,
//! "Input Policy (no panics in the tick)". [`assert_dt`] is the policy's one per-tick panic.

use core::f32::consts::{PI, TAU};

use nalgebra::{SVector, UnitQuaternion};

use crate::{Vec2, Vec3};

#[cfg(test)]
mod tests;

/// Panics unless `dt` is positive and finite: the one per-tick assert the input policy allows.
/// Runners guarantee it; a zero, negative, NaN, or infinite `dt` is a caller bug, and an
/// infinite one would poison integrals (`0 · ∞ = NaN`).
#[track_caller]
pub(crate) fn assert_dt(dt: f32) {
    assert!(dt > 0.0 && dt.is_finite(), "dt must be positive and finite");
}

/// How far [`desaturate`] may shift outputs up, per unit of collective: up to twice the
/// collective. Keeps nearly the full moment split at flying throttle (a hover collective of
/// ~0.55 allows 1.1, more than any split needs) while fading airmode out toward idle.
const MAX_RAISE_PER_COLLECTIVE: f32 = 2.0;

/// Fits mixer outputs into `[0, 1]`, keeping the moments (attitude over thrust), like PX4's
/// airmode. Each output is `collective` plus its moment share.
///
/// 1. If the outputs span more than 1, the moment shares are scaled down about `collective`
///    until they span exactly 1, keeping their ratios.
/// 2. All outputs then shift by the smallest amount that fits them in `[0, 1]`: down if one
///    is above 1, up if one is below 0. Collective thrust gives way.
///
/// Shifting up is limited to [`MAX_RAISE_PER_COLLECTIVE`] times `collective`, so it fades in
/// from zero throttle instead of switching on at the first bit of it: at zero (or negative)
/// collective nothing rises and the shares below 0 are clipped, so an armed vehicle at idle
/// on the ground isn't lifted, or tipped, by wound-up moments. Outputs are then clamped to
/// `[0, 1]`. Inputs are expected finite (the mixer saturates them); a NaN output stays NaN.
///
/// Returns what it gave up, for saturation reporting.
pub(crate) fn desaturate(outputs: &mut [f32], collective: f32) -> Desaturation {
    let (low, high) = outputs
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), &o| {
            (lo.min(o), hi.max(o))
        });
    let spread = high - low;
    let scale = if spread > 1.0 { 1.0 / spread } else { 1.0 };
    let scaled_low = collective + scale * (low - collective);
    let scaled_high = collective + scale * (high - collective);

    let shift = if scaled_high > 1.0 {
        1.0 - scaled_high
    } else if scaled_low < 0.0 {
        (-scaled_low).min(MAX_RAISE_PER_COLLECTIVE * limit_or_zero(collective))
    } else {
        0.0
    };

    for output in outputs.iter_mut() {
        *output = (collective + scale * (*output - collective) + shift).clamp(0.0, 1.0);
    }

    Desaturation {
        lowered: shift < -CLIP_TOLERANCE,
        // Scaled down, or still outside [0, 1] after the shift (a limited raise, or zero
        // collective) and clipped. The tolerance absorbs rounding in a shift that lands an
        // output exactly on a bound.
        moments_lost: scale < 1.0
            || scaled_low + shift < -CLIP_TOLERANCE
            || scaled_high + shift > 1.0 + CLIP_TOLERANCE,
    }
}

/// How far past `[0, 1]` an output may land after [`desaturate`]'s shift before it counts as
/// clipped rather than float rounding.
const CLIP_TOLERANCE: f32 = 1e-6;

/// What [`desaturate`] gave up to fit the outputs.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Desaturation {
    /// Outputs were shifted down: collective thrust gave way to keep the moments.
    pub(crate) lowered: bool,
    /// Moments weren't delivered in full: scaled down, or an output clipped.
    pub(crate) moments_lost: bool,
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
    let pitch = libm::asinf(nose_up_cosine(attitude).clamp(-1.0, 1.0));
    let yaw = libm::atan2f(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z);
    (roll, pitch, yaw)
}

/// Floor on the tilt cosine [`tilt_boost`] divides by: past 60° of tilt the boost stops
/// growing (2×), so a steep estimate can't demand unbounded thrust.
const MIN_TILT_COSINE: f32 = 0.5;

/// Tilt cosine below which [`tilt_boost`] fades to zero (about 84°–90° of tilt), so an
/// inverted vehicle gets no thrust from the boost, as ArduPilot's angle boost does.
const INVERTED_FADE_COSINE: f32 = 0.1;

/// Thrust multiplier that keeps a level thrust's vertical component while tilted:
/// `1 / cos(tilt)`, with `cos(tilt) = cos(roll) · cos(pitch)` (yaw doesn't change it).
///
/// Floored at [`MIN_TILT_COSINE`] (at most 2× past 60°), and faded linearly to 0 as the
/// cosine falls from [`INVERTED_FADE_COSINE`] to 0: at 90° and beyond (inverted) the
/// multiplier is 0, so thrust never pushes an upside-down vehicle toward the ground. A
/// non-finite attitude counts as level (1, no boost).
///
/// The cosine is read from the quaternion directly (`1 − 2(x² + y²)`, the rotation matrix's
/// `(2, 2)` entry): no trig, no matrix.
pub(crate) fn tilt_boost(attitude: &UnitQuaternion<f32>) -> f32 {
    let (x, y) = (attitude.i, attitude.j);
    let cosine = 1.0 - 2.0 * (x * x + y * y);
    if !cosine.is_finite() {
        return 1.0;
    }
    let upright = (cosine / INVERTED_FADE_COSINE).clamp(0.0, 1.0);
    upright / cosine.max(MIN_TILT_COSINE)
}

/// The cosine of the nose's (body `x`) tilt from vertical: `1` pointing straight up, `0`
/// level, `−1` straight down; the sine of the pitch, read from the quaternion directly (the
/// rotation matrix's `(2, 0)` entry, negated for NED's down `z`), so it stays well defined at
/// ±90° pitch where the Euler angles don't. NaN for a non-finite attitude.
pub(crate) fn nose_up_cosine(attitude: &UnitQuaternion<f32>) -> f32 {
    let (w, x, y, z) = (attitude.w, attitude.i, attitude.j, attitude.k);
    2.0 * (w * y - x * z)
}

/// The attitude error a roll/pitch/heading setpoint asks of `current`, in `current`'s body
/// axes (`x` roll, `y` pitch, `z` yaw), in radians: what the attitude loops act on.
///
/// Tilt first, as PX4's multicopter attitude control does (`mc_att_control`,
/// `AttitudeControl.cpp`, reduced attitude): roll and pitch are taken at the vehicle's current
/// heading, as guidance and pilots set them, and `x`/`y` are the shortest rotation that turns
/// the body `z` (thrust) axis onto the setpoint's. `z` is the heading error, wrapped to the
/// short way round, weighted by the cosine of the current tilt (0 past 90°): the yaw loop turns
/// about the thrust axis, and far over that axis is nearly horizontal, where a turn about it
/// tips the vehicle more than it changes heading. Near level each component is the Euler
/// difference of its axis, but as body rates, so the loops' outputs are body rates at any
/// attitude; a setpoint equal to the current Euler angles gives zero.
///
/// The heading is the frame contract's ([`euler_angles`]), so it matches guidance's heading
/// targets. It degenerates only at exactly ±90° pitch, where the weighting has already zeroed
/// the heading term and a tailsitter's transition doesn't reach (its ramp stops short of it).
///
/// A non-finite `current` gives NaN components; a NaN `heading_rad` gives a NaN `z` only.
pub(crate) fn attitude_error(
    current: &UnitQuaternion<f32>,
    roll_rad: f32,
    pitch_rad: f32,
    heading_rad: f32,
) -> Vec3 {
    if !current.coords.iter().all(|c| c.is_finite()) {
        return Vec3::repeat(f32::NAN);
    }
    let rotation = current.to_rotation_matrix();
    let m = rotation.matrix();

    // The current heading's cosine and sine from the matrix (no trig); at exactly ±90° pitch
    // there is none, and any will do.
    let (north, east) = (m[(0, 0)], m[(1, 0)]);
    let current_heading_rad = libm::atan2f(east, north);
    let horizontal = libm::sqrtf(north * north + east * east);
    let (cos_h, sin_h) = if horizontal > 1e-6 {
        (north / horizontal, east / horizontal)
    } else {
        (1.0, 0.0)
    };

    // The setpoint's body z axis in the world (the third column of the ZYX rotation at the
    // current heading), then seen from the current body.
    let (sin_r, cos_r) = (libm::sinf(roll_rad), libm::cosf(roll_rad));
    let (sin_p, cos_p) = (libm::sinf(pitch_rad), libm::cosf(pitch_rad));
    let desired_z = Vec3::new(
        cos_h * sin_p * cos_r + sin_h * sin_r,
        sin_h * sin_p * cos_r - cos_h * sin_r,
        cos_p * cos_r,
    );
    let z = m.tr_mul(&desired_z);

    // The rotation onto it is about z × it, by the angle between them.
    let axis = Vec2::new(-z.y, z.x);
    let sine = axis.norm();
    // Below this the cross product is the angle itself to f32 precision (sin x ≈ x).
    let tilt = if sine < 1e-6 {
        axis
    } else {
        axis * (libm::atan2f(sine, z.z) / sine)
    };

    // The matrix's (2, 2) entry: the body z axis's share of the vertical.
    let upright = m[(2, 2)].max(0.0);
    Vec3::new(
        tilt.x,
        tilt.y,
        wrap_angle(heading_rad - current_heading_rad) * upright,
    )
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
