//! Autotune stages hosted by the [`CopterStabilizer`]: one loop's output replaced by a
//! [`RelayTuner`] (#149). See the "Tuning" section on [`CopterStabilizer`].

use crate::tuning::{RelayAbort, RelayConfig, RelayResult, RelayStatus, RelayTuner};

use super::CopterStabilizer;

#[cfg(test)]
mod tests;

/// A loop of the [`CopterStabilizer`] that a tuning stage can replace.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TunedLoop {
    /// Roll rate loop: the relay sets the roll moment.
    RollRate,
    /// Pitch rate loop: the relay sets the pitch moment.
    PitchRate,
    /// Yaw rate loop: the relay sets the yaw moment.
    YawRate,
    /// Roll angle loop: the relay sets the roll rate setpoint, in rad/s.
    Roll,
    /// Pitch angle loop: the relay sets the pitch rate setpoint, in rad/s.
    Pitch,
    /// Heading loop: the relay sets the yaw rate setpoint, in rad/s.
    Heading,
}

/// Where a tuning stage stands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TuningStatus {
    /// The relay is flying the loop: its current amplitude and the consistent cycles in hand.
    Running {
        /// Relay amplitude `h`, in the loop's output units.
        relay: f32,
        /// Measured cycles in the window.
        cycles: usize,
    },
    /// Measured; the loop is back with its controller.
    Done(RelayResult),
    /// The tuner gave up; the loop is back with its controller.
    Aborted(RelayAbort),
    /// A [`Setpoint::Rate`](crate::Setpoint::Rate) tick idled the angle or heading loop under
    /// test (the pilot switched to acro); the loop is back with its controller.
    Interrupted,
}

impl From<RelayStatus> for TuningStatus {
    fn from(status: RelayStatus) -> Self {
        match status {
            RelayStatus::Running { relay, cycles } => Self::Running { relay, cycles },
            RelayStatus::Done(result) => Self::Done(result),
            RelayStatus::Aborted(abort) => Self::Aborted(abort),
        }
    }
}

/// A tuning stage: the loop under test and its tuner.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct Stage {
    pub(super) target: TunedLoop,
    pub(super) tuner: RelayTuner,
    /// The loop has been handed back (finished, interrupted, or stopped): no more override.
    handed_back: bool,
    interrupted: bool,
}

impl Stage {
    /// Whether the relay still flies the loop.
    pub(super) fn is_running(&self) -> bool {
        !self.handed_back
    }

    fn status(&self) -> TuningStatus {
        if self.interrupted {
            TuningStatus::Interrupted
        } else {
            self.tuner.status().into()
        }
    }
}

impl CopterStabilizer {
    /// Starts relay tuning on `target`: from the next [`stabilize`](crate::Stabilizer::stabilize),
    /// a [`RelayTuner`] with `config` sets that loop's output. Its bias starts at the loop's
    /// current I term (its trim). A running stage is handed back first.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`RelayConfig::validate`]).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::{
    ///     CopterStabilizer, CopterStabilizerConfig, TunedLoop, TuningStatus,
    /// };
    /// use stable_flight::tuning::RelayConfig;
    /// use stable_flight::{RateSetpoint, Setpoint, Stabilizer, VehicleState};
    ///
    /// let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
    /// let relay = RelayConfig {
    ///     initial_relay: 0.05,
    ///     max_relay: 0.3,
    ///     relay_step: 1.5,
    ///     hysteresis: 0.002,
    ///     min_cycle_amplitude: 0.05,
    ///     max_cycle_amplitude: 2.0,
    ///     max_half_cycle_s: 0.3,
    ///     max_bias: 0.3,
    ///     cycles: 4,
    ///     tolerance: 0.1,
    ///     timeout_s: 20.0,
    /// };
    /// stabilizer.start_tuning(TunedLoop::RollRate, relay);
    ///
    /// // Rolling right at 0.5 rad/s with a zero rate setpoint: the relay pushes left.
    /// let mut state = VehicleState::default();
    /// state.angular_velocity_radps.x = 0.5;
    /// let output = stabilizer.stabilize(0.004, &Setpoint::Rate(RateSetpoint::default()), &state);
    /// assert_eq!(output.moments.x, -0.05);
    ///
    /// assert!(matches!(stabilizer.tuning(), Some((TunedLoop::RollRate, TuningStatus::Running { .. }))));
    /// stabilizer.stop_tuning(); // hands the roll rate loop back
    /// assert!(stabilizer.tuning().is_none());
    /// ```
    pub fn start_tuning(&mut self, target: TunedLoop, config: RelayConfig) {
        self.stop_tuning();
        let trim = self.integral_term(target);
        self.stage = Some(Stage {
            target,
            tuner: RelayTuner::new(config, trim),
            handed_back: false,
            interrupted: false,
        });
    }

    /// The current stage, if any: running, or finished and not yet stopped.
    pub fn tuning(&self) -> Option<(TunedLoop, TuningStatus)> {
        self.stage
            .as_ref()
            .map(|stage| (stage.target, stage.status()))
    }

    /// Ends the stage, handing the loop back to its controller if the relay still flies it,
    /// and returns its last status.
    pub fn stop_tuning(&mut self) -> Option<(TunedLoop, TuningStatus)> {
        self.hand_back();
        self.stage
            .take()
            .map(|stage| (stage.target, stage.status()))
    }

    /// Hands a running stage's loop back to its controller: the loop restarts with the tuner's
    /// bias as its I term and no derivative history, so the trim carries over. A no-op when no
    /// stage is running.
    pub(super) fn hand_back(&mut self) {
        let Some(stage) = &mut self.stage else {
            return;
        };
        if stage.handed_back {
            return;
        }
        stage.handed_back = true;
        let (target, trim) = (stage.target, stage.tuner.bias());
        let config = &self.config;
        match target {
            TunedLoop::RollRate => self.body_rate.hand_back(&config.body_rate, 0, trim),
            TunedLoop::PitchRate => self.body_rate.hand_back(&config.body_rate, 1, trim),
            TunedLoop::YawRate => self.body_rate.hand_back(&config.body_rate, 2, trim),
            TunedLoop::Roll => self.attitude.hand_back(&config.attitude, 0, trim),
            TunedLoop::Pitch => self.attitude.hand_back(&config.attitude, 1, trim),
            TunedLoop::Heading => self.yaw.hand_back(&config.yaw, trim),
        }
    }

    /// Ends a running angle or heading stage as interrupted (its loop is idle on a rate tick).
    pub(super) fn interrupt_attitude_stage(&mut self) {
        let Some(stage) = &mut self.stage else {
            return;
        };
        let angle_loop = matches!(
            stage.target,
            TunedLoop::Roll | TunedLoop::Pitch | TunedLoop::Heading
        );
        if angle_loop && stage.is_running() {
            stage.interrupted = true;
            self.hand_back();
        }
    }

    /// Steps the running stage on `target` with `error` and returns the relay output, or None
    /// when no stage runs on `target`. Hands the loop back once the tuner finishes.
    pub(super) fn step_stage(&mut self, target: TunedLoop, dt: f32, error: f32) -> Option<f32> {
        let stage = self.stage.as_mut()?;
        if stage.target != target || !stage.is_running() {
            return None;
        }
        let output = stage.tuner.step(dt, error);
        if !matches!(stage.tuner.status(), RelayStatus::Running { .. }) {
            self.hand_back();
        }
        Some(output)
    }

    /// The I term of `target`'s controller: its trim.
    fn integral_term(&self, target: TunedLoop) -> f32 {
        let config = &self.config;
        match target {
            TunedLoop::RollRate => self.body_rate.integral_term(&config.body_rate, 0),
            TunedLoop::PitchRate => self.body_rate.integral_term(&config.body_rate, 1),
            TunedLoop::YawRate => self.body_rate.integral_term(&config.body_rate, 2),
            TunedLoop::Roll => self.attitude.integral_term(&config.attitude, 0),
            TunedLoop::Pitch => self.attitude.integral_term(&config.attitude, 1),
            TunedLoop::Heading => self.yaw.integral_term(&config.yaw),
        }
    }
}
