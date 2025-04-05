//! Control algorithms that maintain desired flight dynamics by processing commands and vehicle state feedback.

use robomath::{vec3, Vec3};

pub mod copter;
pub mod vtol;

pub mod controllers;

/// PID gains for a single control loop.
#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct Gains {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

// alpha 0.1 is a good starting point for most applications
pub fn low_pass_filter_vec3(prev: Vec3<f32>, new: Vec3<f32>, alpha: f32) -> Vec3<f32> {
    vec3(
        alpha * new.x + (1.0 - alpha) * prev.x,
        alpha * new.y + (1.0 - alpha) * prev.y,
        alpha * new.z + (1.0 - alpha) * prev.z,
    )
}
