//! The physical parameters of a simulated tailsitter: see [`TailsitterModel`].

use crate::fixed_wing::FixedWingModel;
use crate::model::{ModelError, check_fraction, check_positive};

#[cfg(test)]
mod tests;

/// Physical parameters of a simulated tailsitter: an airplane ([`FixedWingModel`]) with its
/// motors spread along the wing and its surfaces in their wash.
///
/// Every motor thrusts along body `x`, so the tailsitter hovers nose up and flies forward on
/// the same thrust. The mixer's yaw factor on a motor yaws it through
/// [`motor_arm_m`](Self::motor_arm_m) (differential thrust); its roll and pitch factors turn
/// nothing. [`wing`](Self::wing)'s `max_thrust_n` and lag are each motor's.
///
/// # Examples
///
/// ```
/// use stable_flight_sim::TailsitterModel;
///
/// let model = stable_flight_sim::airframes::tailsitter().model;
/// assert!(model.validate().is_ok());
/// assert!(TailsitterModel { motor_arm_m: 0.0, ..model }.validate().is_err());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TailsitterModel {
    /// The airframe as an airplane: mass, inertia, wing, aerodynamics, surfaces, and each
    /// motor's thrust and lag.
    pub wing: FixedWingModel,
    /// Distance from the centerline of a motor whose mixer yaw factor is 1, in m.
    pub motor_arm_m: f32,
    /// The props' wash over the surfaces.
    pub prop_wash: PropWash,
}

/// The props' slipstream over the control surfaces, by actuator-disc momentum theory: the
/// far wake carries `thrust / disc area` more dynamic pressure than the free stream.
///
/// The surfaces see [`surface_fraction`](Self::surface_fraction) of it on top of the free
/// stream's, so they keep authority in a hover; the wing's lift, drag, and damping see the
/// free stream only. The total thrust over the total disc area washes every surface evenly.
///
/// # Examples
///
/// ```
/// use stable_flight_sim::PropWash;
///
/// // Two 10-inch props.
/// let wash = PropWash::props(2, 0.254, 1.0);
/// assert!((wash.disc_area_m2 - 0.1013).abs() < 1e-4);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropWash {
    /// Total disc area of the props, in m².
    pub disc_area_m2: f32,
    /// Share of the wake's added dynamic pressure the surfaces see, in `[0, 1]`: 1 for surfaces
    /// wholly inside the wash.
    pub surface_fraction: f32,
}

impl PropWash {
    /// `count` props of `diameter_m`, with `surface_fraction` of their wake over the surfaces.
    pub fn props(count: u8, diameter_m: f32, surface_fraction: f32) -> Self {
        let radius_m = diameter_m / 2.0;
        PropWash {
            disc_area_m2: f32::from(count) * core::f32::consts::PI * radius_m * radius_m,
            surface_fraction,
        }
    }

    /// Dynamic pressure the wash adds over the surfaces at `thrust_n`, in Pa.
    pub(crate) fn pressure_pa(&self, thrust_n: f32) -> f32 {
        self.surface_fraction * thrust_n / self.disc_area_m2
    }
}

impl TailsitterModel {
    /// Checks every parameter's range.
    ///
    /// # Errors
    ///
    /// [`FixedWingModel::validate`]'s errors; [`ModelError::NotPositive`] for a motor arm or
    /// disc area that isn't positive and finite; [`ModelError::NotFraction`] for a surface
    /// fraction outside `[0, 1]`.
    pub fn validate(&self) -> Result<(), ModelError> {
        self.wing.validate()?;
        check_positive(&[
            ("motor_arm_m", self.motor_arm_m),
            ("prop_wash.disc_area_m2", self.prop_wash.disc_area_m2),
        ])?;
        check_fraction(&[(
            "prop_wash.surface_fraction",
            self.prop_wash.surface_fraction,
        )])
    }
}
