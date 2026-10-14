//! The physical parameters of a simulated airplane: see [`FixedWingModel`].

use stable_flight::Vec3;

use crate::model::{ModelError, check_finite, check_non_negative, check_positive};

#[cfg(test)]
mod tests;

/// Physical parameters of a simulated airplane: a rigid body with a wing, three control
/// surfaces, and a tractor prop along body `x`.
///
/// The surfaces aren't named by servo slot: [`SimPlane`](crate::SimPlane) reads each servo's
/// share of roll, pitch, and yaw from the mixer's rows, so a moment the mixer sends comes back
/// as the deflection it meant. Aerodynamics are in [`AeroCoefficients`].
///
/// # Examples
///
/// ```
/// use stable_flight_sim::FixedWingModel;
///
/// let heavier = FixedWingModel { mass_kg: 2.5, ..stable_flight_sim::airframes::plane().model };
/// assert!(heavier.validate().is_ok());
/// assert!(FixedWingModel { span_m: 0.0, ..heavier }.validate().is_err());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FixedWingModel {
    /// Total mass, in kg.
    pub mass_kg: f32,
    /// Principal moments of inertia about the FRD body axes (roll, pitch, yaw), in kg·m².
    pub inertia_kgm2: Vec3,
    /// Wing area, in m².
    pub wing_area_m2: f32,
    /// Wing span, in m.
    pub span_m: f32,
    /// Mean aerodynamic chord, in m.
    pub chord_m: f32,
    /// Lift, drag, side force, and moment coefficients.
    pub aero: AeroCoefficients,
    /// Deflection at full servo command of the aileron (`x`), elevator (`y`), and rudder (`z`)
    /// axes, in rad.
    pub max_deflection_rad: Vec3,
    /// First-order lag from a servo's command to its position, in s.
    pub servo_time_constant_s: f32,
    /// Each motor's thrust at full command, in N, along body `x`. Thrust is
    /// `max_thrust_n · command²`, as on the multirotor, and doesn't fall off with airspeed.
    pub max_thrust_n: f32,
    /// First-order lag from the motor's command to the command it runs at, in s.
    pub motor_time_constant_s: f32,
    /// Friction coefficient against the ground while resting on it (Coulomb, unitless).
    pub ground_friction: f32,
    /// Longest integration step, in s (see [`MultirotorModel::max_step_s`](crate::MultirotorModel::max_step_s)).
    pub max_step_s: f32,
}

/// Dimensionless aerodynamic coefficients, in the stability-derivative form of Beard and
/// McLain, *Small Unmanned Aircraft* (2012), ch. 4.
///
/// Angles (α angle of attack, β sideslip, δ deflections) are in rad. Rate derivatives take the
/// rate normalized by airspeed (`p·b/2V`, `q·c/2V`, `r·b/2V`). Forces scale by dynamic
/// pressure times wing area; roll and yaw moments also by span, pitch by chord.
///
/// Lift is linear in α up to [`alpha_stall_rad`](Self::alpha_stall_rad), then blends over a
/// sigmoid of [`stall_sharpness`](Self::stall_sharpness) into a flat plate's
/// (`2·sign α·sin²α·cos α`), so it falls off past the stall. Drag is parasitic plus induced,
/// `c_drag_0 + CL²/(π·e·AR)` with the linear lift, blended by the same sigmoid into a flat
/// plate's `c_drag_0 + 2·sin²α`. Static pitch stability acts on `sin α`. So forces and moments
/// stay bounded all the way round, broadside (α ±90°, a tailsitter's hover) and backward alike.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroCoefficients {
    /// Lift at zero α.
    pub c_lift_0: f32,
    /// Lift slope, per rad of α.
    pub c_lift_alpha: f32,
    /// Angle of attack where lift stalls, in rad (either way).
    pub alpha_stall_rad: f32,
    /// How abruptly lift blends into the flat plate's at the stall, per rad.
    pub stall_sharpness: f32,
    /// Parasitic drag.
    pub c_drag_0: f32,
    /// Oswald efficiency of the induced drag.
    pub oswald: f32,
    /// Side force per rad of β.
    pub c_side_beta: f32,
    /// Side force per rad of rudder.
    pub c_side_rudder: f32,
    /// Roll moment per rad of β (dihedral effect).
    pub c_roll_beta: f32,
    /// Roll moment per normalized roll rate (roll damping).
    pub c_roll_p: f32,
    /// Roll moment per normalized yaw rate.
    pub c_roll_r: f32,
    /// Roll moment per rad of aileron.
    pub c_roll_aileron: f32,
    /// Roll moment per rad of rudder.
    pub c_roll_rudder: f32,
    /// Pitch moment at zero α.
    pub c_pitch_0: f32,
    /// Pitch moment per unit of `sin α`, per rad at small α (static stability; negative is
    /// stable).
    pub c_pitch_alpha: f32,
    /// Pitch moment per normalized pitch rate (pitch damping).
    pub c_pitch_q: f32,
    /// Pitch moment per rad of elevator.
    pub c_pitch_elevator: f32,
    /// Yaw moment per rad of β (weathercock stability).
    pub c_yaw_beta: f32,
    /// Yaw moment per normalized roll rate.
    pub c_yaw_p: f32,
    /// Yaw moment per normalized yaw rate (yaw damping).
    pub c_yaw_r: f32,
    /// Yaw moment per rad of rudder.
    pub c_yaw_rudder: f32,
}

impl FixedWingModel {
    /// Wing aspect ratio: span² over area.
    pub(crate) fn aspect_ratio(&self) -> f32 {
        self.span_m * self.span_m / self.wing_area_m2
    }

    /// Checks every parameter's range.
    ///
    /// # Errors
    ///
    /// [`ModelError::NotPositive`] for a mass, inertia, wing area, span, chord, stall angle,
    /// stall sharpness, Oswald efficiency, max thrust, time constant, or max step that isn't
    /// positive and finite; [`ModelError::Negative`] for a max deflection, parasitic drag, or
    /// friction that is negative or not finite; [`ModelError::NotFinite`] for any other
    /// coefficient that isn't finite.
    pub fn validate(&self) -> Result<(), ModelError> {
        let aero = &self.aero;
        check_positive(&[
            ("mass_kg", self.mass_kg),
            ("inertia_kgm2.x", self.inertia_kgm2.x),
            ("inertia_kgm2.y", self.inertia_kgm2.y),
            ("inertia_kgm2.z", self.inertia_kgm2.z),
            ("wing_area_m2", self.wing_area_m2),
            ("span_m", self.span_m),
            ("chord_m", self.chord_m),
            ("aero.alpha_stall_rad", aero.alpha_stall_rad),
            ("aero.stall_sharpness", aero.stall_sharpness),
            ("aero.oswald", aero.oswald),
            ("servo_time_constant_s", self.servo_time_constant_s),
            ("max_thrust_n", self.max_thrust_n),
            ("motor_time_constant_s", self.motor_time_constant_s),
            ("max_step_s", self.max_step_s),
        ])?;
        check_non_negative(&[
            ("max_deflection_rad.x", self.max_deflection_rad.x),
            ("max_deflection_rad.y", self.max_deflection_rad.y),
            ("max_deflection_rad.z", self.max_deflection_rad.z),
            ("aero.c_drag_0", aero.c_drag_0),
            ("ground_friction", self.ground_friction),
        ])?;
        check_finite(&[
            ("aero.c_lift_0", aero.c_lift_0),
            ("aero.c_lift_alpha", aero.c_lift_alpha),
            ("aero.c_side_beta", aero.c_side_beta),
            ("aero.c_side_rudder", aero.c_side_rudder),
            ("aero.c_roll_beta", aero.c_roll_beta),
            ("aero.c_roll_p", aero.c_roll_p),
            ("aero.c_roll_r", aero.c_roll_r),
            ("aero.c_roll_aileron", aero.c_roll_aileron),
            ("aero.c_roll_rudder", aero.c_roll_rudder),
            ("aero.c_pitch_0", aero.c_pitch_0),
            ("aero.c_pitch_alpha", aero.c_pitch_alpha),
            ("aero.c_pitch_q", aero.c_pitch_q),
            ("aero.c_pitch_elevator", aero.c_pitch_elevator),
            ("aero.c_yaw_beta", aero.c_yaw_beta),
            ("aero.c_yaw_p", aero.c_yaw_p),
            ("aero.c_yaw_r", aero.c_yaw_r),
            ("aero.c_yaw_rudder", aero.c_yaw_rudder),
        ])
    }
}
