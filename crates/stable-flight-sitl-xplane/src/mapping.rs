//! [`ActuatorMapping`]: which X-Plane engine each [`ActuatorCommand`] motor slot drives.

use stable_flight::ActuatorCommand;
use thiserror::Error;

#[cfg(test)]
mod tests;

/// Number of X-Plane engines (`sim/flightmodel/engine/ENGN_thro_use[16]`).
pub const ENGINE_COUNT: usize = 16;

/// Number of X-Plane wings carrying control surfaces (`sim/flightmodel2/wing/*_deg[48]`).
pub const WING_COUNT: usize = 48;

/// Maps [`ActuatorCommand`] motor slots to X-Plane engines.
///
/// `motors[i]` names the `ENGN_thro_use` element (0–15) the mixer's motor `i` drives, 0.0 to
/// 1.0 as the mixer outputs it; `None` leaves the slot unused. Under the throttle override
/// X-Plane holds whatever was last written to an engine, so `idle_engines` lists the ones the
/// mixer doesn't drive (a pusher in copter flight), written 0.0 every tick.
///
/// `neutral_wings` lists the wings (the `.acf`'s wing numbers, 0–47) whose ailerons,
/// elevators, and rudders are taken from X-Plane and held at 0° (`override_control_surfaces`),
/// so X-Plane's joystick can't move them under a hover. Empty leaves the surfaces to X-Plane.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl_xplane::ActuatorMapping;
///
/// let mapping = ActuatorMapping {
///     motors: [Some(1), Some(0), Some(2), Some(3), None, None, None, None],
///     idle_engines: &[4],
///     neutral_wings: &[2, 3],
/// };
/// assert!(mapping.validate().is_ok());
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct ActuatorMapping {
    /// X-Plane engine for each motor slot.
    pub motors: [Option<usize>; ActuatorCommand::SLOTS],
    /// Engines held at 0.0.
    pub idle_engines: &'static [usize],
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
}

impl ActuatorMapping {
    /// Checks that every engine and wing exists and that no engine is used twice.
    ///
    /// # Errors
    ///
    /// The first [`MappingError`] found: motors, then idle engines, then wings.
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
        Ok(())
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
