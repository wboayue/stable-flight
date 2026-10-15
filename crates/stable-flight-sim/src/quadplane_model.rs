//! The physical parameters of a simulated lift+cruise VTOL: see [`QuadPlaneModel`].

use crate::fixed_wing::FixedWingModel;
use crate::model::{ModelError, check_non_negative, check_positive};
use crate::motors::MotorLayout;

#[cfg(test)]
mod tests;

/// Physical parameters of a simulated quadplane: an airplane ([`FixedWingModel`]) with lift
/// rotors thrusting along body `−z` and a pusher along body `x`, one rigid body.
///
/// The lift rotors are laid out like [`MultirotorModel`](crate::MultirotorModel)'s, from the
/// mixer's motor rows: a roll or pitch factor of 1 sits [`LiftRotors::moment_arm_m`] from the
/// axis, a yaw factor of ±1 gives [`LiftRotors::yaw_torque_per_thrust_m`]. The mixer's
/// [`pusher`](stable_flight::mixers::MixRow::pusher) row is the pusher;
/// [`wing`](Self::wing)'s `max_thrust_n` and motor lag are the pusher's.
///
/// # Examples
///
/// ```
/// use stable_flight_sim::{LiftRotors, QuadPlaneModel};
///
/// let model = stable_flight_sim::airframes::quadplane().model;
/// assert!(model.validate().is_ok());
/// let no_thrust = LiftRotors { max_thrust_n: 0.0, ..model.lift };
/// assert!(QuadPlaneModel { lift: no_thrust, ..model }.validate().is_err());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadPlaneModel {
    /// The airframe as an airplane: mass, inertia, wing, aerodynamics, surfaces, and the
    /// pusher's thrust and lag. Its parasitic drag carries the booms and the stopped lift
    /// rotors.
    pub wing: FixedWingModel,
    /// The lift rotors.
    pub lift: LiftRotors,
}

/// A quadplane's lift rotors: thrust along body `−z`, laid out by the mixer's motor rows.
///
/// Spinning edgewise to the air, they drag in their plane (H-force): a linear drag on the
/// air's velocity in the body `x`–`y` plane, [`drag_n_per_mps`](Self::drag_n_per_mps) with
/// every rotor at full thrust, scaled down with their thrust, so it is gone with the rotors
/// stopped and never more than `drag_n_per_mps` times the airspeed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LiftRotors {
    /// Distance from the roll or pitch axis of a rotor whose row factor is 1, in m.
    pub moment_arm_m: f32,
    /// Thrust of one rotor at full command, in N, `max_thrust_n · command²`.
    pub max_thrust_n: f32,
    /// First-order lag from a rotor's command to the command it runs at, in s.
    pub motor_time_constant_s: f32,
    /// Prop drag torque about the yaw axis per newton of thrust, in N·m/N (m).
    pub yaw_torque_per_thrust_m: f32,
    /// In-plane drag on the air's velocity with every rotor at full thrust, in N per m/s.
    pub drag_n_per_mps: f32,
}

impl LiftRotors {
    /// The rotors' torque arms and strength.
    pub(crate) fn motor_layout(&self) -> MotorLayout {
        MotorLayout {
            moment_arm_m: self.moment_arm_m,
            yaw_torque_per_thrust_m: self.yaw_torque_per_thrust_m,
            max_thrust_n: self.max_thrust_n,
        }
    }
}

impl QuadPlaneModel {
    /// Checks every parameter's range.
    ///
    /// # Errors
    ///
    /// [`FixedWingModel::validate`]'s errors; [`ModelError::NotPositive`] for a lift moment
    /// arm, max thrust, or time constant that isn't positive and finite;
    /// [`ModelError::Negative`] for a yaw torque or drag that is negative or not finite.
    pub fn validate(&self) -> Result<(), ModelError> {
        self.wing.validate()?;
        let lift = &self.lift;
        check_positive(&[
            ("lift.moment_arm_m", lift.moment_arm_m),
            ("lift.max_thrust_n", lift.max_thrust_n),
            ("lift.motor_time_constant_s", lift.motor_time_constant_s),
        ])?;
        check_non_negative(&[
            ("lift.yaw_torque_per_thrust_m", lift.yaw_torque_per_thrust_m),
            ("lift.drag_n_per_mps", lift.drag_n_per_mps),
        ])
    }
}
