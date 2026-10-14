//! The physical parameters of a simulated multirotor: see [`MultirotorModel`].

use stable_flight::Vec3;
use thiserror::Error;

#[cfg(test)]
mod tests;

/// Physical parameters of a simulated multirotor.
///
/// Geometry isn't listed here: [`SimCopter`](crate::SimCopter) takes each motor's torque
/// directions from the mixer's rows, so a row factor of 1 sits [`moment_arm_m`](Self::moment_arm_m)
/// from the roll or pitch axis and a yaw factor of ±1 gives
/// [`yaw_torque_per_thrust_m`](Self::yaw_torque_per_thrust_m) of reaction torque.
///
/// # Examples
///
/// ```
/// use stable_flight_sim::MultirotorModel;
///
/// let heavier = MultirotorModel { mass_kg: 1.5, ..stable_flight_sim::airframes::quad_x().model };
/// assert!(heavier.validate().is_ok());
/// assert!(MultirotorModel { mass_kg: 0.0, ..heavier }.validate().is_err());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MultirotorModel {
    /// Total mass, in kg.
    pub mass_kg: f32,
    /// Principal moments of inertia about the FRD body axes (roll, pitch, yaw), in kg·m².
    pub inertia_kgm2: Vec3,
    /// Distance from the roll or pitch axis of a motor whose row factor is 1, in m (half the
    /// motor spacing on an X quad: arm length / √2).
    pub moment_arm_m: f32,
    /// Thrust of one motor at full command, in N. Thrust is `max_thrust_n · command²` (prop
    /// thrust goes with the square of its speed, and speed with the command).
    pub max_thrust_n: f32,
    /// First-order lag from a motor's command to the command it runs at, in s.
    pub motor_time_constant_s: f32,
    /// Prop drag torque about the yaw axis per newton of thrust, in N·m/N (m).
    pub yaw_torque_per_thrust_m: f32,
    /// Linear drag on the velocity through the air, in N per m/s.
    pub drag_n_per_mps: f32,
    /// Rotational damping on the body rates (roll, pitch, yaw), in N·m per rad/s.
    pub angular_damping_nm_per_radps: Vec3,
    /// Friction coefficient against the ground while resting on it (Coulomb, unitless).
    pub ground_friction: f32,
    /// Longest integration step, in s: each [`state`](stable_flight::StateSource::state) call
    /// splits its `dt` into the fewest equal steps no longer than this, so accuracy doesn't
    /// depend on the loop rate.
    pub max_step_s: f32,
}

/// Why a [`MultirotorModel`], [`Perturbations`](crate::Perturbations), or a
/// [`SimCopter`](crate::SimCopter)'s mixer was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
#[non_exhaustive]
pub enum ModelError {
    /// A parameter that must be positive and finite isn't.
    #[error("{0} must be positive and finite")]
    NotPositive(&'static str),
    /// A parameter that must be non-negative and finite isn't.
    #[error("{0} must be non-negative and finite")]
    Negative(&'static str),
    /// A parameter that must be finite isn't.
    #[error("{0} must be finite")]
    NotFinite(&'static str),
    /// The mixer drives a servo; the simulator models motors only.
    #[error("the mixer drives servo {0}, which the simulator doesn't model")]
    Servo(u8),
    /// A motor slot's [`Perturbations::motor_thrust_scale`](crate::Perturbations::motor_thrust_scale)
    /// isn't positive and finite.
    #[error("motor_thrust_scale[{0}] must be positive and finite")]
    ThrustScale(u8),
    /// [`Perturbations::actuator_delay_s`](crate::Perturbations::actuator_delay_s) is over
    /// [`MAX_ACTUATOR_DELAY_S`](crate::MAX_ACTUATOR_DELAY_S).
    #[error("actuator_delay_s must be at most {max_s} s", max_s = crate::MAX_ACTUATOR_DELAY_S)]
    DelayTooLong,
}

impl MultirotorModel {
    /// Checks every parameter's range.
    ///
    /// # Errors
    ///
    /// [`ModelError::NotPositive`] for a mass, inertia, moment arm, max thrust, motor time
    /// constant, or max step that isn't positive and finite; [`ModelError::Negative`] for a yaw
    /// torque, drag, damping, or friction that is negative or not finite.
    pub fn validate(&self) -> Result<(), ModelError> {
        let positive = [
            ("mass_kg", self.mass_kg),
            ("inertia_kgm2.x", self.inertia_kgm2.x),
            ("inertia_kgm2.y", self.inertia_kgm2.y),
            ("inertia_kgm2.z", self.inertia_kgm2.z),
            ("moment_arm_m", self.moment_arm_m),
            ("max_thrust_n", self.max_thrust_n),
            ("motor_time_constant_s", self.motor_time_constant_s),
            ("max_step_s", self.max_step_s),
        ];
        check_positive(&positive)?;
        let non_negative = [
            ("yaw_torque_per_thrust_m", self.yaw_torque_per_thrust_m),
            ("drag_n_per_mps", self.drag_n_per_mps),
            (
                "angular_damping_nm_per_radps.x",
                self.angular_damping_nm_per_radps.x,
            ),
            (
                "angular_damping_nm_per_radps.y",
                self.angular_damping_nm_per_radps.y,
            ),
            (
                "angular_damping_nm_per_radps.z",
                self.angular_damping_nm_per_radps.z,
            ),
            ("ground_friction", self.ground_friction),
        ];
        check_non_negative(&non_negative)
    }
}

/// [`ModelError::NotPositive`] naming the first of `params` that isn't positive and finite.
pub(crate) fn check_positive(params: &[(&'static str, f32)]) -> Result<(), ModelError> {
    first_failing(params, |value| value > 0.0)
        .map_or(Ok(()), |name| Err(ModelError::NotPositive(name)))
}

/// [`ModelError::Negative`] naming the first of `params` that is negative or not finite.
pub(crate) fn check_non_negative(params: &[(&'static str, f32)]) -> Result<(), ModelError> {
    first_failing(params, |value| value >= 0.0)
        .map_or(Ok(()), |name| Err(ModelError::Negative(name)))
}

/// The name of the first of `params` that isn't finite or fails `in_range`.
fn first_failing(
    params: &[(&'static str, f32)],
    in_range: impl Fn(f32) -> bool,
) -> Option<&'static str> {
    params
        .iter()
        .find(|(_, value)| !(value.is_finite() && in_range(*value)))
        .map(|(name, _)| *name)
}
