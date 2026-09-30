//! Control algorithms that maintain desired flight dynamics by processing commands and vehicle state feedback.

use robomath::{Vec3, vec3};

pub mod copter;

pub mod controllers;

/// PID gains for a single control loop.
#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct Gains {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

/// Saturates a normalized command to [-1, 1]; NaN becomes 0 (neutral).
///
/// Controllers call this on normalized setpoints instead of asserting the range, so an
/// out-of-range or non-finite request degrades to the nearest valid command rather than
/// panicking the flight loop.
pub fn saturate_unit(value: f32) -> f32 {
    if value.is_nan() {
        0.0
    } else {
        value.clamp(-1.0, 1.0)
    }
}

// alpha 0.1 is a good starting point for most applications
pub fn low_pass_filter_vec3(prev: Vec3<f32>, new: Vec3<f32>, alpha: f32) -> Vec3<f32> {
    vec3(
        alpha * new.x + (1.0 - alpha) * prev.x,
        alpha * new.y + (1.0 - alpha) * prev.y,
        alpha * new.z + (1.0 - alpha) * prev.z,
    )
}
