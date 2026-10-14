//! Motor spin-up and the thrust and torque the motors put on the body.

use stable_flight::mixers::{MixRow, Slot};
use stable_flight::{ActuatorCommand, Vec3};

use crate::lag;

#[cfg(test)]
mod tests;

const SLOTS: usize = ActuatorCommand::SLOTS;

/// How the motors' thrust turns into body torque, and how strong they are.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct MotorLayout {
    /// Distance from the roll or pitch axis of a motor whose row factor is 1, in m.
    pub moment_arm_m: f32,
    /// Yaw reaction torque per newton of thrust for a yaw factor of 1, in m.
    pub yaw_torque_per_thrust_m: f32,
    /// Thrust of one motor at full command, in N.
    pub max_thrust_n: f32,
}

/// The motors a mixer drives: each one's lagged command and the body torque its thrust makes.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Motors {
    max_thrust_n: f32,
    /// Motor slots in use (`0..count`).
    count: usize,
    /// FRD body torque per newton of each motor's thrust, in N·m/N (roll, pitch, yaw).
    torque_per_thrust_m: [Vec3; SLOTS],
    /// The command each motor runs at, after its lag, in `[0, 1]`.
    running: [f32; SLOTS],
    /// Each motor's thrust as a fraction of the model's.
    thrust_scale: [f32; SLOTS],
}

impl Motors {
    /// The motor rows of `rows`, laid out by `layout`: a roll or pitch factor of 1 acts at
    /// `moment_arm_m`, a yaw factor of 1 gives `yaw_torque_per_thrust_m`. Servo rows are
    /// skipped. Motors start stopped.
    ///
    /// The signs follow the mixer's: a row's positive roll factor means its thrust rolls right,
    /// so the simulated vehicle answers each moment the way the mixer expects.
    pub(crate) fn new<const N: usize>(rows: &[MixRow; N], layout: &MotorLayout) -> Self {
        let mut motors = Motors {
            max_thrust_n: layout.max_thrust_n,
            count: 0,
            torque_per_thrust_m: [Vec3::zeros(); SLOTS],
            running: [0.0; SLOTS],
            thrust_scale: [1.0; SLOTS],
        };
        for row in rows {
            let Slot::Motor(index) = row.slot() else {
                continue;
            };
            let index = usize::from(index);
            // A valid `TableMixer` keeps motor slots below `SLOTS` and contiguous from 0.
            if let Some(torque) = motors.torque_per_thrust_m.get_mut(index) {
                *torque = Vec3::new(
                    row.roll() * layout.moment_arm_m,
                    row.pitch() * layout.moment_arm_m,
                    row.yaw() * layout.yaw_torque_per_thrust_m,
                );
                motors.count = motors.count.max(index + 1);
            }
        }
        motors
    }

    /// Moves each motor toward its command by `alpha` ([`lag::factor`]). A command outside
    /// `[0, 1]` saturates; NaN stops the motor.
    pub(crate) fn spin(&mut self, commands: &[f32; SLOTS], alpha: f32) {
        for (running, command) in self.running.iter_mut().zip(commands).take(self.count) {
            lag::approach_saturated(running, *command, &(0.0..=1.0), 0.0, alpha);
        }
    }

    /// Scales each motor's thrust (mismatched motors); all 1 at construction.
    pub(crate) fn set_thrust_scale(&mut self, thrust_scale: [f32; SLOTS]) {
        self.thrust_scale = thrust_scale;
    }

    /// Each motor's thrust, in N: `thrust_scale · max_thrust_n · running²`.
    pub(crate) fn thrusts_n(&self) -> [f32; SLOTS] {
        let mut thrusts_n = [0.0; SLOTS];
        for ((thrust_n, running), scale) in thrusts_n
            .iter_mut()
            .zip(&self.running)
            .zip(&self.thrust_scale)
        {
            *thrust_n = scale * self.max_thrust_n * running * running;
        }
        thrusts_n
    }

    /// Total thrust, in N, and the FRD body torque, in N·m.
    pub(crate) fn wrench(&self) -> (f32, Vec3) {
        let thrusts_n = self.thrusts_n();
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
