#![doc = include_str!("README.md")]

use core::fmt;

use crate::Vec3;

pub mod altitude;
pub mod attitude;
pub mod body_rate;
pub mod lateral;
pub mod yaw;

pub use altitude::{AltitudeController, AltitudeControllerConfig, DEFAULT_HOVER_THRUST};
pub use attitude::{AttitudeController, AttitudeControllerConfig};
pub use body_rate::{BodyRateController, BodyRateControllerConfig};
pub use lateral::{LateralController, LateralControllerConfig};
pub use yaw::{YawController, YawControllerConfig};

#[cfg(test)]
mod tests;

/// A controller config value that [`validate`](BodyRateControllerConfig::validate) rejected.
///
/// Controllers assume a valid config: check it once at construction and on every live change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InvalidConfig {
    /// The offending field, e.g. `"max_integral"`.
    pub field: &'static str,
    /// What the field must satisfy, e.g. `"non-negative and finite"`.
    pub requirement: &'static str,
}

impl fmt::Display for InvalidConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} must be {}", self.field, self.requirement)
    }
}

impl core::error::Error for InvalidConfig {}

const NON_NEGATIVE_FINITE: &str = "non-negative and finite";

/// `Ok` when `value` is non-negative and finite, the rule for every controller limit.
fn check_limit(field: &'static str, value: f32) -> Result<(), InvalidConfig> {
    if value >= 0.0 && value.is_finite() {
        Ok(())
    } else {
        Err(InvalidConfig {
            field,
            requirement: NON_NEGATIVE_FINITE,
        })
    }
}

/// PID gains for a single control loop.
#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct Gains {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

// alpha 0.1 is a good starting point for most applications
pub fn low_pass_filter_vec3(prev: Vec3, new: Vec3, alpha: f32) -> Vec3 {
    Vec3::new(
        alpha * new.x + (1.0 - alpha) * prev.x,
        alpha * new.y + (1.0 - alpha) * prev.y,
        alpha * new.z + (1.0 - alpha) * prev.z,
    )
}
