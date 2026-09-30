//! Small numeric helpers shared by stabilizers and mixers.
//!
//! # Out-of-range and non-finite input policy
//!
//! Nothing that runs every tick may panic on its inputs. Controllers and mixers apply one rule:
//!
//! - **Out-of-range normalized commands saturate** to their valid range: [`saturate_unit`] for
//!   [-1, 1] (roll/pitch/yaw, moments) and [`saturate_throttle`] for [0, 1]. A controller that
//!   *produces* a normalized command saturates its output; one that *consumes* it saturates its
//!   input too, defensively.
//! - **NaN commands become neutral** (0: level, no rate, no thrust).
//! - **A non-finite error (from a NaN/∞ setpoint or estimate) yields zero output** and leaves
//!   the controller's integral and derivative history untouched; the first finite tick after
//!   the gap has no derivative kick. `NaN` survives `f32::clamp`, so it must never enter
//!   controller state.
//! - **Angles wrap** to [-π, π) with [`wrap_angle`] rather than being range-checked.
//!
//! Only documented construction preconditions (config validation) and `dt > 0` (guaranteed by
//! the runners) are asserted.

use core::f32::consts::PI;

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
///
/// The only `rem_euclid` in the core, which PR 4 replaces with a `libm` form for `no_std`.
pub(crate) fn wrap_angle(angle: f32) -> f32 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}
