//! [`ActuatorMapping`]: which X-Plane engine each [`ActuatorCommand`] motor slot drives, and
//! which wing surfaces each servo slot deflects.

use stable_flight::ActuatorCommand;
use thiserror::Error;
use xplane_bridge::datarefs::wing;
use xplane_bridge::{DatarefArray, Writable};

#[cfg(test)]
mod tests;

/// Number of X-Plane engines (`sim/flightmodel/engine/ENGN_thro_use[16]`).
pub const ENGINE_COUNT: usize = 16;

/// Number of X-Plane wings carrying control surfaces (`sim/flightmodel2/wing/*_deg[48]`).
pub const WING_COUNT: usize = 48;

/// A kind of control surface, and the `sim/flightmodel2/wing` datarefs it writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Surface {
    /// `aileron1_deg` and `aileron2_deg`.
    Aileron,
    /// `elevator1_deg` and `elevator2_deg`.
    Elevator,
    /// `rudder1_deg` and `rudder2_deg`.
    Rudder,
}

impl Surface {
    /// Every kind, in [`ActuatorMapping::neutral_wings`]' write order.
    pub const ALL: [Surface; 3] = [Surface::Aileron, Surface::Elevator, Surface::Rudder];

    /// Both of its per-wing arrays: a `.acf` puts a surface on set 1, set 2, or both (the
    /// Alia's ailerons on both), and X-Plane ignores the set a wing doesn't carry.
    pub(crate) fn datarefs(self) -> [DatarefArray<f32, Writable, WING_COUNT>; 2] {
        match self {
            Surface::Aileron => [wing::AILERON1_DEG, wing::AILERON2_DEG],
            Surface::Elevator => [wing::ELEVATOR1_DEG, wing::ELEVATOR2_DEG],
            Surface::Rudder => [wing::RUDDER1_DEG, wing::RUDDER2_DEG],
        }
    }
}

/// One wing's surface driven by one [`ActuatorCommand`] servo slot.
///
/// The servo's `[-1, 1]` maps linearly on each side of 0: `+1` deflects it `at_plus_deg`,
/// `-1` `at_minus_deg`, 0 holds it at 0°. Degrees are X-Plane's (ailerons and elevators
/// positive trailing edge down; rudders mirrored between left and right fins), so a link's
/// two ends carry both the sign that makes the mixer's moment and the surface's travel each
/// way (ailerons that rise further than they drop).
///
/// # Examples
///
/// ```
/// use stable_flight_sitl_xplane::{Surface, SurfaceLink};
///
/// // A left aileron: rolling right (servo +1) drops it 10°, rolling left raises it 20°.
/// let left = SurfaceLink::new(0, Surface::Aileron, 2, 10.0, -20.0);
/// assert_eq!(left.deflection_deg(0.5), 5.0);
/// assert_eq!(left.deflection_deg(-0.5), -10.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceLink {
    /// The servo slot (`0..ActuatorCommand::SLOTS`).
    pub servo: usize,
    /// Which surface on the wing.
    pub surface: Surface,
    /// The `.acf`'s wing number (`0..WING_COUNT`).
    pub wing: usize,
    /// Deflection at servo `+1`, in degrees.
    pub at_plus_deg: f32,
    /// Deflection at servo `-1`, in degrees.
    pub at_minus_deg: f32,
}

impl SurfaceLink {
    /// `wing`'s `surface` on servo slot `servo`, deflected `at_plus_deg` at servo `+1` and
    /// `at_minus_deg` at `-1`.
    pub const fn new(
        servo: usize,
        surface: Surface,
        wing: usize,
        at_plus_deg: f32,
        at_minus_deg: f32,
    ) -> Self {
        SurfaceLink {
            servo,
            surface,
            wing,
            at_plus_deg,
            at_minus_deg,
        }
    }

    /// The deflection for a servo command: saturated to `[-1, 1]`, NaN as 0 (input policy).
    pub fn deflection_deg(&self, servo: f32) -> f32 {
        if servo.is_nan() {
            return 0.0;
        }
        let servo = servo.clamp(-1.0, 1.0);
        if servo >= 0.0 {
            servo * self.at_plus_deg
        } else {
            -servo * self.at_minus_deg
        }
    }
}

/// Maps [`ActuatorCommand`] motor slots to X-Plane engines and servo slots to wing surfaces.
///
/// `motors[i]` names the `ENGN_thro_use` element (0–15) the mixer's motor `i` drives, 0.0 to
/// 1.0 as the mixer outputs it; `None` leaves the slot unused. Under the throttle override
/// X-Plane holds whatever was last written to an engine, so `idle_engines` lists the ones the
/// mixer doesn't drive (a pusher in copter flight), written 0.0.
///
/// `surfaces` drives wing surfaces from the servo slots ([`SurfaceLink`]); `neutral_wings`
/// lists wings (the `.acf`'s wing numbers, 0–47) whose ailerons, elevators, and rudders are
/// held at 0°, so X-Plane's joystick can't move them under a hover. Either one takes the
/// surfaces from X-Plane (`override_control_surfaces`); both empty leave them to X-Plane.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl_xplane::{ActuatorMapping, Surface, SurfaceLink};
///
/// // An elevator: servo +1 raises its trailing edge 20° (X-Plane's negative).
/// const SURFACES: &[SurfaceLink] = &[SurfaceLink::new(1, Surface::Elevator, 8, -20.0, 20.0)];
///
/// let mapping = ActuatorMapping {
///     motors: [Some(1), Some(0), Some(2), Some(3), Some(4), None, None, None],
///     surfaces: SURFACES,
///     ..Default::default()
/// };
/// assert!(mapping.validate().is_ok());
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ActuatorMapping {
    /// X-Plane engine for each motor slot.
    pub motors: [Option<usize>; ActuatorCommand::SLOTS],
    /// Engines held at 0.0.
    pub idle_engines: &'static [usize],
    /// Wing surfaces driven by the servo slots.
    pub surfaces: &'static [SurfaceLink],
    /// Wings whose surfaces are held at 0°.
    pub neutral_wings: &'static [usize],
}

/// Why an [`ActuatorMapping`] is invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum MappingError {
    /// An engine X-Plane doesn't have.
    #[error("engine {engine} is out of range (X-Plane has {ENGINE_COUNT})")]
    EngineOutOfRange {
        /// The offending engine.
        engine: usize,
    },
    /// An engine both driven and idled, or driven twice.
    #[error("engine {engine} is mapped twice")]
    DuplicateEngine {
        /// The offending engine.
        engine: usize,
    },
    /// A wing X-Plane doesn't have.
    #[error("wing {wing} is out of range (X-Plane has {WING_COUNT})")]
    WingOutOfRange {
        /// The offending wing.
        wing: usize,
    },
    /// A surface link names a servo slot the command doesn't have.
    #[error(
        "servo {servo} is out of range (commands have {})",
        ActuatorCommand::SLOTS
    )]
    ServoOutOfRange {
        /// The offending servo slot.
        servo: usize,
    },
    /// A surface link's deflection isn't finite.
    #[error("wing {wing}'s {surface:?} deflection is not finite")]
    InvalidDeflection {
        /// The link's wing.
        wing: usize,
        /// The link's surface.
        surface: Surface,
    },
    /// A wing surface linked twice, or both linked and held neutral.
    #[error("wing {wing}'s {surface:?} is driven twice")]
    DuplicateSurface {
        /// The offending wing.
        wing: usize,
        /// The offending surface.
        surface: Surface,
    },
}

impl ActuatorMapping {
    /// Checks that every engine, wing, and servo exists, that each deflection is finite, and
    /// that no engine or wing surface is used twice.
    ///
    /// # Errors
    ///
    /// The first [`MappingError`] found: motors, then idle engines, then neutral wings, then
    /// surface links.
    pub fn validate(&self) -> Result<(), MappingError> {
        let mut used = [false; ENGINE_COUNT];
        for &engine in self.motors.iter().flatten().chain(self.idle_engines) {
            let slot = used
                .get_mut(engine)
                .ok_or(MappingError::EngineOutOfRange { engine })?;
            if *slot {
                return Err(MappingError::DuplicateEngine { engine });
            }
            *slot = true;
        }
        if let Some(&wing) = self.neutral_wings.iter().find(|&&w| w >= WING_COUNT) {
            return Err(MappingError::WingOutOfRange { wing });
        }
        for (i, link) in self.surfaces.iter().enumerate() {
            let SurfaceLink { wing, surface, .. } = *link;
            if link.servo >= ActuatorCommand::SLOTS {
                return Err(MappingError::ServoOutOfRange { servo: link.servo });
            }
            if wing >= WING_COUNT {
                return Err(MappingError::WingOutOfRange { wing });
            }
            if !(link.at_plus_deg.is_finite() && link.at_minus_deg.is_finite()) {
                return Err(MappingError::InvalidDeflection { wing, surface });
            }
            let linked_before = self.surfaces[..i]
                .iter()
                .any(|other| other.wing == wing && other.surface == surface);
            if linked_before || self.neutral_wings.contains(&wing) {
                return Err(MappingError::DuplicateSurface { wing, surface });
            }
        }
        Ok(())
    }

    /// Whether the mapping takes the control surfaces from X-Plane.
    pub(crate) fn overrides_surfaces(&self) -> bool {
        !(self.surfaces.is_empty() && self.neutral_wings.is_empty())
    }

    /// Each engine's throttle for `command`: mapped motors saturated to [0, 1] (NaN → 0), idle
    /// engines 0.0.
    pub(crate) fn throttles(
        &self,
        command: &ActuatorCommand,
    ) -> impl Iterator<Item = (usize, f32)> + '_ {
        let driven = self
            .motors
            .iter()
            .zip(command.motors)
            .filter_map(|(engine, throttle)| Some(((*engine)?, saturate(throttle))));
        let idle = self.idle_engines.iter().map(|&engine| (engine, 0.0));
        driven.chain(idle)
    }

    /// Every engine the mapping writes.
    pub(crate) fn engines(&self) -> impl Iterator<Item = usize> + '_ {
        self.motors
            .iter()
            .flatten()
            .chain(self.idle_engines)
            .copied()
    }
}

/// A throttle in [0, 1]; NaN reads as 0 (input policy).
fn saturate(throttle: f32) -> f32 {
    if throttle.is_nan() {
        0.0
    } else {
        throttle.clamp(0.0, 1.0)
    }
}
