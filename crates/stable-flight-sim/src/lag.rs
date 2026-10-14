//! First-order lag, shared by the motors and servos: see [`factor`].

use core::ops::RangeInclusive;

/// The step factor of a first-order lag with time constant `time_constant_s` over a step of
/// `h` seconds: `1 − e^(−h/τ)`, the fraction of the gap closed per step.
pub(crate) fn factor(h: f32, time_constant_s: f32) -> f32 {
    1.0 - libm::expf(-h / time_constant_s)
}

/// Moves `value` toward `target` by `alpha` ([`factor`]) of the gap.
pub(crate) fn approach(value: &mut f32, target: f32, alpha: f32) {
    *value += (target - *value) * alpha;
}

/// Moves `value` toward `command` saturated to `range`, by `alpha` ([`factor`]) of the gap; a
/// NaN command moves it toward `neutral`.
pub(crate) fn approach_saturated(
    value: &mut f32,
    command: f32,
    range: &RangeInclusive<f32>,
    neutral: f32,
    alpha: f32,
) {
    let target = if command.is_nan() {
        neutral
    } else {
        command.clamp(*range.start(), *range.end())
    };
    approach(value, target, alpha);
}
