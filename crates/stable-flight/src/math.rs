//! Small numeric helpers shared by guidance, stabilizers, controllers, and mixers: [`saturate_unit`],
//! [`saturate_unit_vec2`], [`saturate_throttle`], and [`wrap_angle`].
//!
//! They implement the per-tick input policy (saturate, NaN → neutral, never let a non-finite
//! error into controller state, wrap angles), which is specified in `architecture.md`,
//! "Input Policy (no panics in the tick)".

use core::f32::consts::{PI, TAU};

use robomath::{Vec2, vec2};

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
    vec2(saturate_unit(value.x), saturate_unit(value.y))
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
