//! Motor spin-up and the thrust and torque the motors put on the body.

use stable_flight::mixers::{MixRow, Slot};
use stable_flight::{ActuatorCommand, Vec3};

use crate::model::{ModelError, QuadModel};

#[cfg(test)]
mod tests;

const SLOTS: usize = ActuatorCommand::SLOTS;

/// The motors a mixer drives: each one's lagged command and the body torque its thrust makes.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Motors {
    /// Motor slots in use (`0..count`).
    count: usize,
    /// FRD body torque per newton of each motor's thrust, in N·m/N (roll, pitch, yaw).
    torque_per_thrust_m: [Vec3; SLOTS],
    /// The command each motor runs at, after its lag, in `[0, 1]`.
    running: [f32; SLOTS],
}

impl Motors {
    /// Motors laid out by `rows`: a roll or pitch factor of 1 acts at `model.moment_arm_m`,
    /// a yaw factor of 1 gives `model.yaw_torque_per_thrust_m`. Motors start stopped.
    ///
    /// The signs follow the mixer's: a row's positive roll factor means its thrust rolls right,
    /// so the simulated vehicle answers each moment the way the mixer expects.
    pub(crate) fn new<const N: usize>(
        rows: &[MixRow; N],
        model: &QuadModel,
    ) -> Result<Self, ModelError> {
        let mut motors = Motors {
            count: 0,
            torque_per_thrust_m: [Vec3::zeros(); SLOTS],
            running: [0.0; SLOTS],
        };
        for row in rows {
            let index = match row.slot() {
                Slot::Motor(index) => usize::from(index),
                Slot::Servo(index) => return Err(ModelError::Servo(index)),
            };
            // A valid `TableMixer` keeps motor slots below `SLOTS` and contiguous from 0.
            if let Some(torque) = motors.torque_per_thrust_m.get_mut(index) {
                *torque = Vec3::new(
                    row.roll() * model.moment_arm_m,
                    row.pitch() * model.moment_arm_m,
                    row.yaw() * model.yaw_torque_per_thrust_m,
                );
                motors.count = motors.count.max(index + 1);
            }
        }
        Ok(motors)
    }

    /// Moves each motor toward its command by `alpha` (the first-order lag's step factor,
    /// `1 − e^(−h/τ)`). A command outside `[0, 1]` saturates; NaN stops the motor.
    pub(crate) fn spin(&mut self, commands: &[f32; SLOTS], alpha: f32) {
        for (running, command) in self.running.iter_mut().zip(commands).take(self.count) {
            let target = if command.is_nan() {
                0.0
            } else {
                command.clamp(0.0, 1.0)
            };
            *running += (target - *running) * alpha;
        }
    }

    /// Each motor's thrust, in N: `max_thrust_n · running²`.
    pub(crate) fn thrusts_n(&self, model: &QuadModel) -> [f32; SLOTS] {
        self.running
            .map(|running| model.max_thrust_n * running * running)
    }

    /// Total thrust along body −z, in N, and the FRD body torque, in N·m.
    pub(crate) fn wrench(&self, model: &QuadModel) -> (f32, Vec3) {
        let thrusts_n = self.thrusts_n(model);
        let torque_nm = thrusts_n
            .iter()
            .zip(&self.torque_per_thrust_m)
            .map(|(thrust_n, torque)| torque * *thrust_n)
            .sum();
        (thrusts_n.iter().sum(), torque_nm)
    }

    /// Motor slots in use.
    #[cfg(test)]
    pub(crate) fn count(&self) -> usize {
        self.count
    }
}
