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
    /// Which motor slots this group drives.
    used: [bool; SLOTS],
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
        Self::selected(rows, layout, |_| true)
    }

    /// As [`new`](Self::new), without motor slot `skipped` (another group drives it).
    pub(crate) fn excluding<const N: usize>(
        rows: &[MixRow; N],
        layout: &MotorLayout,
        skipped: u8,
    ) -> Self {
        Self::selected(rows, layout, |index| index != skipped)
    }

    /// One motor on slot `index`, `max_thrust_n` at full command, turning nothing: whether
    /// or not a mixer row drives it. An index of [`SLOTS`] or more drives nothing.
    pub(crate) fn single(index: u8, max_thrust_n: f32) -> Self {
        let mut motors = Self::stopped(max_thrust_n);
        if let Some(used) = motors.used.get_mut(usize::from(index)) {
            *used = true;
        }
        motors
    }

    fn stopped(max_thrust_n: f32) -> Self {
        Motors {
            max_thrust_n,
            used: [false; SLOTS],
            torque_per_thrust_m: [Vec3::zeros(); SLOTS],
            running: [0.0; SLOTS],
            thrust_scale: [1.0; SLOTS],
        }
    }

    /// The motor rows of `rows` whose slot `include` accepts, laid out by `layout`.
    fn selected<const N: usize>(
        rows: &[MixRow; N],
        layout: &MotorLayout,
        include: impl Fn(u8) -> bool,
    ) -> Self {
        let mut motors = Self::stopped(layout.max_thrust_n);
        for row in rows {
            let Slot::Motor(index) = row.slot() else {
                continue;
            };
            if !include(index) {
                continue;
            }
            let index = usize::from(index);
            // A valid `TableMixer` keeps motor slots below `SLOTS`.
            if let (Some(torque), Some(used)) = (
                motors.torque_per_thrust_m.get_mut(index),
                motors.used.get_mut(index),
            ) {
                *torque = Vec3::new(
                    row.roll() * layout.moment_arm_m,
                    row.pitch() * layout.moment_arm_m,
                    row.yaw() * layout.yaw_torque_per_thrust_m,
                );
                *used = true;
            }
        }
        motors
    }

    /// Moves each motor toward its command by `alpha` ([`lag::factor`]). A command outside
    /// `[0, 1]` saturates; NaN stops the motor.
    pub(crate) fn spin(&mut self, commands: &[f32; SLOTS], alpha: f32) {
        for ((running, command), used) in self.running.iter_mut().zip(commands).zip(&self.used) {
            if *used {
                lag::approach_saturated(running, *command, &(0.0..=1.0), 0.0, alpha);
            }
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

    /// Total thrust at full command on every motor, in N, before mismatch.
    pub(crate) fn max_total_thrust_n(&self) -> f32 {
        self.count() as f32 * self.max_thrust_n
    }

    /// Motor slots in use.
    pub(crate) fn count(&self) -> usize {
        self.used.iter().filter(|used| **used).count()
    }
}
