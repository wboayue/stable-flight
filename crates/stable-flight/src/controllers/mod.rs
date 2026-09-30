//! PID building blocks shared by guidance (altitude, lateral) and stabilizers (attitude, yaw,
//! body rate).

use robomath::{Vec3, vec3};

pub mod altitude;
pub mod attitude;
pub mod body_rate;
pub mod lateral;
pub mod yaw;

pub use altitude::{AltitudeController, AltitudeControllerConfig};
pub use attitude::{AttitudeController, AttitudeControllerConfig};
pub use body_rate::{BodyRateController, BodyRateControllerConfig};
pub use lateral::{LateralController, LateralControllerConfig};
pub use yaw::{YawController, YawControllerConfig};

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
