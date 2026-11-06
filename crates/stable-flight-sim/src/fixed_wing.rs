//! The physical parameters of a simulated airplane: see [`FixedWingModel`].

use stable_flight::Vec3;

use crate::model::{ModelError, check_finite, check_fraction, check_non_negative, check_positive};
use crate::perturbations::MAX_ACTUATOR_DELAY_S;

#[cfg(test)]
mod tests;

/// How a prop's thrust falls as the air comes through it faster ([`FixedWingModel`]'s
/// motors along body `x`), by forward airspeed `u` (body `x`, positive).
///
/// # Examples
///
/// ```
/// use stable_flight_sim::{FixedWingModel, ThrustFalloff};
///
/// let model = FixedWingModel {
///     thrust_falloff: ThrustFalloff::PitchSpeed { full_mps: 30.0 },
///     ..stable_flight_sim::airframes::plane().model
/// };
/// assert!(model.validate().is_ok());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ThrustFalloff {
    /// Each m/s takes `n_per_mps` N off the full-command thrust, scaled by the command as the
    /// thrust is: a power-limited prop that stops pushing at the same speed at any throttle.
    /// 0 holds static thrust at any speed. Non-negative.
    Linear {
        /// Thrust lost per m/s at full command, in N per m/s.
        n_per_mps: f32,
    },
    /// A fixed-pitch prop stops pushing once the air comes through it at the speed its blades
    /// screw forward, and that speed falls with the motor's: thrust is
    /// `static · (1 − u / (full_mps · √thrust_share))`, `thrust_share` the motor's static
    /// thrust over full (thrust goes with rpm²). So at part throttle each m/s takes a larger
    /// share of the thrust than at full.
    PitchSpeed {
        /// Airspeed at which full-command thrust reaches 0, in m/s. Positive.
        full_mps: f32,
    },
}

/// Where a wing's zero-α pitch moment ([`c_pitch_0`](AeroCoefficients::c_pitch_0)) leaves
/// dynamic pressure: below `knee_mps` it grows as `V²`, past it as `V^exponent`. With an
/// exponent under 2 the elevator that trims it (its authority still growing as `V²`) holds less
/// the faster the wing flies; over 2, more.
///
/// # Examples
///
/// ```
/// use stable_flight_sim::{AeroCoefficients, FixedWingModel, PitchTrimKnee};
///
/// let plane = stable_flight_sim::airframes::plane().model;
/// let model = FixedWingModel {
///     aero: AeroCoefficients {
///         c_pitch_0_knee: Some(PitchTrimKnee { knee_mps: 14.0, exponent: 1.0 }),
///         ..plane.aero
///     },
///     ..plane
/// };
/// assert!(model.validate().is_ok());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PitchTrimKnee {
    /// Airspeed past which the moment grows as `V^exponent`, in m/s. Positive.
    pub knee_mps: f32,
    /// How the moment grows with airspeed past the knee: as `V^exponent` (2 is dynamic
    /// pressure's). Finite.
    pub exponent: f32,
}

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
    /// Each motor's thrust at full command and no airspeed, in N, along body `x`. Thrust is
    /// `max_thrust_n · command^thrust_exponent`, less what
    /// [`thrust_falloff`](Self::thrust_falloff) takes at forward airspeed, and never below 0.
    pub max_thrust_n: f32,
    /// How thrust follows the command: 2 for a fixed-pitch prop at a set voltage; 1 where the
    /// command sets the motor's power and the prop runs near its design speed, as X-Plane's
    /// Alia 250 pusher (#392). Positive.
    pub thrust_exponent: f32,
    /// How the thrust falls as the air comes through the prop faster.
    pub thrust_falloff: ThrustFalloff,
    /// How far below the center of gravity the thrust line runs, in m: thrust there pitches the
    /// nose up by `thrust · thrust_line_below_cg_m`. Negative above the center of gravity.
    pub thrust_line_below_cg_m: f32,
    /// First-order lag from the motor's command to the command it runs at, in s.
    pub motor_time_constant_s: f32,
    /// Pure delay from the command of a motor along body `x` to the start of its lag, in s, on
    /// top of any [`actuator_delay_s`](crate::Perturbations::actuator_delay_s): a prop slow to
    /// spin up, whose thrust holds still a while before following the command. The surfaces
    /// and any lift rotors don't wait it out. At most
    /// [`MAX_ACTUATOR_DELAY_S`](crate::MAX_ACTUATOR_DELAY_S).
    pub motor_delay_s: f32,
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
/// plate's `c_drag_0 + 2·sin²α`. Static pitch stability acts on `sin α`; past the stall the
/// pitch moment can blend into a flat plate's
/// ([`post_stall_pitch_share`](Self::post_stall_pitch_share)) and the surfaces can lose the
/// free stream ([`surface_stall_share`](Self::surface_stall_share)). So forces and moments
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
    /// Where [`c_pitch_0`](Self::c_pitch_0)'s moment leaves dynamic pressure; `None` grows with
    /// it at every airspeed.
    pub c_pitch_0_knee: Option<PitchTrimKnee>,
    /// Pitch moment per unit of `sin α`, per rad at small α (static stability; negative is
    /// stable).
    pub c_pitch_alpha: f32,
    /// Share of a flat plate's pitch moment past the stall, in `[0, 1]`; the rest stays
    /// [`c_pitch_alpha`](Self::c_pitch_alpha)'s `sin α`. A flat plate's normal force (its lift
    /// and drag past the stall, `2·sin²α` broadside) acts at a center of pressure that moves
    /// back from the quarter chord at α 0 to mid-chord broadside and the three-quarter chord
    /// with the flow reversed, about a center of gravity ahead of the quarter chord by the
    /// static margin (`−c_pitch_alpha / c_lift_alpha` chords). So broadside it pushes the nose
    /// down far harder than a small static margin's `sin α` does, and tail first it turns the
    /// nose away from the flow (unstable). 0 for a wing whose stall moment the sim doesn't
    /// claim.
    pub post_stall_pitch_share: f32,
    /// Pitch moment per normalized pitch rate (pitch damping).
    pub c_pitch_q: f32,
    /// Pitch moment per rad of elevator.
    pub c_pitch_elevator: f32,
    /// Share of the free stream's dynamic pressure the surfaces lose past the stall, in
    /// `[0, 1]`: blended by the stall sigmoid, so 1 leaves them working on the free stream only
    /// in attached flow from ahead (broadside and tail first it separates or reverses over
    /// them). Prop wash over them ([`PropWash`](crate::PropWash)) still works them.
    pub surface_stall_share: f32,
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

    /// Each motor's thrust at full command and `airspeed_mps` forward (body `x`; none counted
    /// below 0), in N: `max_thrust_n` less the falloff, never below 0.
    pub(crate) fn full_thrust_n(&self, airspeed_mps: f32) -> f32 {
        self.max_thrust_n * self.falloff_share(1.0, airspeed_mps)
    }

    /// The share of a motor's static thrust left at `airspeed_mps` forward (body `x`; none
    /// counted below 0), in `[0, 1]`, for a motor at `thrust_share` of full static thrust.
    pub(crate) fn falloff_share(&self, thrust_share: f32, airspeed_mps: f32) -> f32 {
        let airspeed_mps = airspeed_mps.max(0.0);
        match self.thrust_falloff {
            ThrustFalloff::Linear { n_per_mps } => {
                // Exactly 1 with no falloff (`x / x`), so airframes without it fly unchanged.
                (self.max_thrust_n - n_per_mps * airspeed_mps).max(0.0) / self.max_thrust_n
            }
            ThrustFalloff::PitchSpeed { full_mps } => {
                if airspeed_mps == 0.0 {
                    return 1.0;
                }
                // Thrust goes with rpm², so the motor runs at the root of its thrust share.
                let zero_thrust_mps = full_mps * libm::sqrtf(thrust_share.max(0.0));
                // A stopped motor (`zero_thrust_mps` 0) divides to −∞ and clamps to 0.
                (1.0 - airspeed_mps / zero_thrust_mps).max(0.0)
            }
        }
    }

    /// Checks every parameter's range.
    ///
    /// # Errors
    ///
    /// [`ModelError::NotPositive`] for a mass, inertia, wing area, span, chord, stall angle,
    /// stall sharpness, Oswald efficiency, max thrust, time constant, or max step that isn't
    /// positive and finite; [`ModelError::Negative`] for a max deflection, parasitic drag,
    /// friction, or motor delay that is negative or not finite; [`ModelError::DelayTooLong`] for
    /// a motor delay over [`MAX_ACTUATOR_DELAY_S`]; [`ModelError::NotFraction`] for a stall
    /// share outside `[0, 1]`; [`ModelError::NotFinite`] for any other coefficient that isn't
    /// finite (a [`PitchTrimKnee`]'s knee must be positive).
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
            ("thrust_exponent", self.thrust_exponent),
            ("motor_time_constant_s", self.motor_time_constant_s),
            ("max_step_s", self.max_step_s),
        ])?;
        check_non_negative(&[
            ("max_deflection_rad.x", self.max_deflection_rad.x),
            ("max_deflection_rad.y", self.max_deflection_rad.y),
            ("max_deflection_rad.z", self.max_deflection_rad.z),
            ("aero.c_drag_0", aero.c_drag_0),
            ("ground_friction", self.ground_friction),
            ("motor_delay_s", self.motor_delay_s),
        ])?;
        match self.thrust_falloff {
            ThrustFalloff::Linear { n_per_mps } => {
                check_non_negative(&[("thrust_falloff.n_per_mps", n_per_mps)])?
            }
            ThrustFalloff::PitchSpeed { full_mps } => {
                check_positive(&[("thrust_falloff.full_mps", full_mps)])?
            }
        }
        if let Some(knee) = aero.c_pitch_0_knee {
            check_positive(&[("aero.c_pitch_0_knee.knee_mps", knee.knee_mps)])?;
            check_finite(&[("aero.c_pitch_0_knee.exponent", knee.exponent)])?;
        }
        if self.motor_delay_s > MAX_ACTUATOR_DELAY_S {
            return Err(ModelError::DelayTooLong("motor_delay_s"));
        }
        check_fraction(&[
            ("aero.post_stall_pitch_share", aero.post_stall_pitch_share),
            ("aero.surface_stall_share", aero.surface_stall_share),
        ])?;
        check_finite(&[
            ("thrust_line_below_cg_m", self.thrust_line_below_cg_m),
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
