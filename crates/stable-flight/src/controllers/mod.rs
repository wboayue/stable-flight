#![doc = include_str!("README.md")]

use core::fmt;

use crate::Vec3;

pub mod altitude;
pub mod attitude;
mod biquad;
pub mod body_rate;
mod derivative;
pub mod gyro;
pub mod lateral;
pub(crate) mod low_pass;
pub mod notch;
pub mod plane_rate;
pub mod yaw;

pub use altitude::{
    AltitudeController, AltitudeControllerConfig, AltitudeEstimate, AltitudeTarget,
    DEFAULT_HOVER_THRUST, DEFAULT_INTEGRATION_BAND_M,
};
pub use attitude::{AttitudeController, AttitudeControllerConfig};
pub use body_rate::{BodyRateController, BodyRateControllerConfig, RATE_I_PER_P, YAW_RATE_I_PER_P};
pub use gyro::GyroFilters;
pub use lateral::{
    DEFAULT_BRAKING_SHARE, LateralController, LateralControllerConfig, LateralEstimate,
    LateralTarget,
};
pub use notch::NotchConfig;
pub use plane_rate::{PlaneRateController, PlaneRateControllerConfig, RateGains};
pub use yaw::{YawController, YawControllerConfig};

#[cfg(test)]
mod tests;

/// A controller config value that [`validate`](BodyRateControllerConfig::validate) rejected.
///
/// Controllers assume a valid config: check it once at construction and on every live change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct InvalidConfig {
    /// The loop the field belongs to when a composite config (e.g.
    /// [`CopterStabilizerConfig`](crate::stabilizers::copter::CopterStabilizerConfig))
    /// reported it, e.g. `Some("body_rate")`; `None` from a controller config itself.
    pub section: Option<&'static str>,
    /// The offending field, e.g. `"max_integral"`.
    pub field: &'static str,
    /// What the field must satisfy, e.g. `"non-negative and finite"`.
    pub requirement: &'static str,
}

impl InvalidConfig {
    pub(crate) fn new(field: &'static str, requirement: &'static str) -> Self {
        Self {
            section: None,
            field,
            requirement,
        }
    }

    /// The same error, attributed to `section` of a composite config.
    pub(crate) fn in_section(self, section: &'static str) -> Self {
        Self {
            section: Some(section),
            ..self
        }
    }
}

impl fmt::Display for InvalidConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(section) = self.section {
            write!(f, "{section}.")?;
        }
        write!(f, "{} must be {}", self.field, self.requirement)
    }
}

impl core::error::Error for InvalidConfig {}

/// `Ok` when `value` is non-negative and finite, the rule for every controller limit.
pub(crate) fn check_limit(field: &'static str, value: f32) -> Result<(), InvalidConfig> {
    check(
        field,
        value >= 0.0 && value.is_finite(),
        "non-negative and finite",
    )
}

/// `Ok` when `value` is positive and finite.
pub(crate) fn check_positive(field: &'static str, value: f32) -> Result<(), InvalidConfig> {
    check(
        field,
        value > 0.0 && value.is_finite(),
        "positive and finite",
    )
}

/// `Ok` when `value` is in `(0, 1]` (a fraction of full scale that must not be zero).
pub(crate) fn check_unit_interval(field: &'static str, value: f32) -> Result<(), InvalidConfig> {
    check(field, value > 0.0 && value <= 1.0, "in (0, 1]")
}

/// `Ok` when `value` is a tilt angle strictly between level and vertical, `(0, π/2)` rad.
pub(crate) fn check_tilt(field: &'static str, value: f32) -> Result<(), InvalidConfig> {
    check(
        field,
        value > 0.0 && value < core::f32::consts::FRAC_PI_2,
        "in (0, π/2) rad",
    )
}

/// `Ok` when `ok`, otherwise an [`InvalidConfig`] naming `field` and its `requirement`.
pub(crate) fn check(
    field: &'static str,
    ok: bool,
    requirement: &'static str,
) -> Result<(), InvalidConfig> {
    if ok {
        Ok(())
    } else {
        Err(InvalidConfig::new(field, requirement))
    }
}

/// The integrated error that keeps the I term `i · integral` unchanged when the integral gain
/// goes from `old_i` to `new_i`, so a retune doesn't bump the output (the policy in
/// `architecture.md#config-ownership`).
///
/// Unchanged when the gain is. Zero when either gain is zero (an integral accumulated under a
/// zero gain was never applied, so carrying it in would bump) or the ratio isn't finite.
pub(crate) fn rescale_integral(integral: f32, old_i: f32, new_i: f32) -> f32 {
    if old_i == new_i {
        return integral;
    }
    if old_i == 0.0 || new_i == 0.0 {
        return 0.0;
    }
    let rescaled = integral * (old_i / new_i);
    if rescaled.is_finite() { rescaled } else { 0.0 }
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
