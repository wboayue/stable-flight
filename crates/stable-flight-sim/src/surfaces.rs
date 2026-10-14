//! Servo-driven control surfaces: see [`Surfaces`].

use stable_flight::mixers::{MixRow, Slot};
use stable_flight::{ActuatorCommand, Vec3};

use crate::lag;

#[cfg(test)]
mod tests;

const SLOTS: usize = ActuatorCommand::SLOTS;

/// The servos a mixer drives, each lagging its command, read back as aileron, elevator, and
/// rudder deflections.
///
/// Each servo moves the axes its mixer row names: a row's roll, pitch, and yaw factors. An
/// axis's deflection is the servos' least-squares fit to those factors, so a moment sent
/// through the mixer reads back as that moment: one aileron servo at `0.3` is `0.3`, and two
/// elevons both at `0.3` pitch `0.3`, not `0.6`. Multiplied by the full-command deflection.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Surfaces {
    /// Each servo slot's roll, pitch, and yaw factors over the axis's sum of squared factors
    /// (zero for an axis no servo moves).
    weights: [Vec3; SLOTS],
    /// Each servo's position, in `[-1, 1]`.
    running: [f32; SLOTS],
}

impl Surfaces {
    /// The servo rows of `rows`; motor rows are skipped. Servos start centered.
    pub(crate) fn new<const N: usize>(rows: &[MixRow; N]) -> Self {
        let mut factors = [Vec3::zeros(); SLOTS];
        for row in rows {
            if let Slot::Servo(index) = row.slot()
                && let Some(factor) = factors.get_mut(usize::from(index))
            {
                *factor = Vec3::new(row.roll(), row.pitch(), row.yaw());
            }
        }
        let squares: Vec3 = factors.iter().map(|f| f.component_mul(f)).sum();
        let inverse = squares.map(|sum| if sum > 0.0 { 1.0 / sum } else { 0.0 });
        Surfaces {
            weights: factors.map(|f| f.component_mul(&inverse)),
            running: [0.0; SLOTS],
        }
    }

    /// Moves each servo toward its command by `alpha` ([`lag::factor`]). A command outside
    /// `[-1, 1]` saturates; NaN centers the servo.
    pub(crate) fn move_to(&mut self, commands: &[f32; SLOTS], alpha: f32) {
        for (running, command) in self.running.iter_mut().zip(commands) {
            lag::approach_saturated(running, *command, &(-1.0..=1.0), 0.0, alpha);
        }
    }

    /// Aileron (`x`), elevator (`y`), and rudder (`z`) deflections, in rad, given each axis's
    /// deflection at full command.
    pub(crate) fn deflection_rad(&self, max_deflection_rad: &Vec3) -> Vec3 {
        let unit: Vec3 = self
            .weights
            .iter()
            .zip(&self.running)
            .map(|(weight, running)| weight * *running)
            .sum();
        unit.component_mul(max_deflection_rad)
    }
}
