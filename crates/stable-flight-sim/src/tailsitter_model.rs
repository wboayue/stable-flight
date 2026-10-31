//! The physical parameters of a simulated tailsitter: see [`TailsitterModel`].

use crate::aero::{AIR_DENSITY_KGPM3, Wash};
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

/// The props' slipstream over the control surfaces, by actuator-disc momentum theory along the
/// thrust axis.
///
/// Flying nose first (or still), the far wake carries `thrust / disc area` more dynamic
/// pressure than the free stream. Sinking tail first, the air meets the wash head on: the
/// flow over the surfaces is the static jet's speed less the sink, so the wash's share shrinks
/// and, sinking as fast as the jet, is gone (the eXplora's elevons lose the nose sinking a few
/// m/s, #123). Vortex-ring flow isn't modeled.
///
/// The surfaces see [`surface_fraction`](Self::surface_fraction) of it on top of the free
/// stream's, so they keep authority in a hover. The part of the span in the wash is damped by
/// it as by the free stream, [`damping_fraction`](Self::damping_fraction) of the wing's rate
/// damping; the wing's lift and drag see the free stream only. The total thrust over the total disc area washes every surface evenly.
///
/// # Examples
///
/// ```
/// use stable_flight_sim::PropWash;
///
/// // Two 10-inch props, 0.325 m out on a 1.4 m wing.
/// let wash = PropWash::props(2, 0.254, 1.0, PropWash::span_damping_share(0.325, 0.254, 1.4));
/// assert!((wash.disc_area_m2 - 0.1013).abs() < 1e-4);
/// assert!((wash.damping_fraction - 0.24).abs() < 0.01);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropWash {
    /// Total disc area of the props, in m².
    pub disc_area_m2: f32,
    /// Share of the wake's added dynamic pressure the surfaces see, in `[0, 1]`: 1 for surfaces
    /// wholly inside the wash.
    pub surface_fraction: f32,
    /// Share of the wing's rate damping inside the wash, in `[0, 1]`: the wash damps that part
    /// of the span at its jet's speed ([`span_damping_share`](Self::span_damping_share)).
    pub damping_fraction: f32,
}

impl PropWash {
    /// `count` props of `diameter_m`, with `surface_fraction` of their wake over the surfaces
    /// and `damping_fraction` of the wing's rate damping in it.
    pub fn props(count: u8, diameter_m: f32, surface_fraction: f32, damping_fraction: f32) -> Self {
        let radius_m = diameter_m / 2.0;
        PropWash {
            disc_area_m2: f32::from(count) * core::f32::consts::PI * radius_m * radius_m,
            surface_fraction,
            damping_fraction,
        }
    }

    /// The share of a wing's rate damping inside the wash of a prop `motor_arm_m` out each side,
    /// `diameter_m` across, on a wing of `span_m`: by strip theory each strip of span damps a
    /// roll as its distance from the center squared, so the share is `∫y²` over the washed
    /// strip (the arm ± the radius, clipped to the half-span) over `∫y²` to the tip.
    pub fn span_damping_share(motor_arm_m: f32, diameter_m: f32, span_m: f32) -> f32 {
        let half_span_m = span_m / 2.0;
        let radius_m = diameter_m / 2.0;
        let inner_m = (motor_arm_m - radius_m).clamp(0.0, half_span_m);
        let outer_m = (motor_arm_m + radius_m).clamp(0.0, half_span_m);
        let cube = |y_m: f32| y_m * y_m * y_m;
        (cube(outer_m) - cube(inner_m)) / cube(half_span_m)
    }

    /// The wash at `thrust_n` while moving through the air at `axial_mps` along the thrust
    /// axis (negative sinking tail first): the pressure it adds over the surfaces, its
    /// [`surface_fraction`](Self::surface_fraction) of [`added_pa`](Self::added_pa), and the speed it damps the wing at, its jet's
    /// (less any sink) times [`damping_fraction`](Self::damping_fraction), 0 where it takes
    /// from the free stream.
    pub(crate) fn wash(&self, thrust_n: f32, axial_mps: f32) -> Wash {
        let added_pa = self.added_pa(thrust_n, axial_mps);
        let flow_mps = libm::sqrtf(2.0 * added_pa.max(0.0) / AIR_DENSITY_KGPM3);
        Wash {
            pressure_pa: self.surface_fraction * added_pa,
            damping_mps: self.damping_fraction * flow_mps,
        }
    }

    /// Dynamic pressure the wake adds where it flows, before the surfaces' share, at `thrust_n`
    /// while moving through the air at `axial_mps`, in Pa; negative where it takes away from
    /// the free stream's.
    ///
    /// Nose first, `T/A`. Tail first at speed `V` against a static jet of speed
    /// `v = √(2T/ρA)`, the surfaces see `½ρ(v − V)²` in place of the free stream's `½ρV²`:
    /// `T/A − ρ·v·V`.
    fn added_pa(&self, thrust_n: f32, axial_mps: f32) -> f32 {
        let disc_pa = thrust_n / self.disc_area_m2;
        if axial_mps < 0.0 {
            let jet_mps = libm::sqrtf(2.0 * disc_pa / AIR_DENSITY_KGPM3);
            disc_pa - AIR_DENSITY_KGPM3 * jet_mps * -axial_mps
        } else {
            disc_pa
        }
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
        check_fraction(&[
            (
                "prop_wash.surface_fraction",
                self.prop_wash.surface_fraction,
            ),
            (
                "prop_wash.damping_fraction",
                self.prop_wash.damping_fraction,
            ),
        ])
    }
}
