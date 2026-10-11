//! Autotune stages hosted by the [`CopterStabilizer`]: one loop's output replaced by a
//! [`RelayTuner`] (#149). See the "Tuning" section on [`CopterStabilizer`].

use crate::tuning::{RelayConfig, RelayStatus, RelayTuner, Tunable, TuningStatus};

use super::{CopterStabilizer, CopterStabilizerConfig};
use crate::controllers::Gains;

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

/// Where a [`TunedLoop`] sits in the cascade: the controller that owns it and its axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Location {
    /// The body-rate loop, axis 0 roll, 1 pitch, 2 yaw.
    Rate(usize),
    /// The attitude loop, axis 0 roll, 1 pitch.
    Angle(usize),
    /// The heading loop.
    Heading,
}

impl TunedLoop {
    /// The gains this loop flies in `config`.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::{CopterStabilizerConfig, TunedLoop};
    ///
    /// let mut config = CopterStabilizerConfig::default();
    /// TunedLoop::Heading.gains_mut(&mut config).p = 4.0;
    /// assert_eq!(config.yaw.gains.p, 4.0);
    /// assert_eq!(TunedLoop::Heading.gains(&config).p, 4.0);
    /// ```
    pub fn gains(self, config: &CopterStabilizerConfig) -> Gains {
        let mut config = *config;
        *self.gains_mut(&mut config)
    }

    /// The gains this loop flies in `config`, to retune them.
    pub fn gains_mut(self, config: &mut CopterStabilizerConfig) -> &mut Gains {
        match self {
            Self::RollRate => &mut config.body_rate.roll_gains,
            Self::PitchRate => &mut config.body_rate.pitch_gains,
            Self::YawRate => &mut config.body_rate.yaw_gains,
            Self::Roll => &mut config.attitude.roll_gains,
            Self::Pitch => &mut config.attitude.pitch_gains,
            Self::Heading => &mut config.yaw.gains,
        }
    }

    pub(super) fn location(self) -> Location {
        match self {
            Self::RollRate => Location::Rate(0),
            Self::PitchRate => Location::Rate(1),
            Self::YawRate => Location::Rate(2),
            Self::Roll => Location::Angle(0),
            Self::Pitch => Location::Angle(1),
            Self::Heading => Location::Heading,
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

impl Tunable for CopterStabilizer {
    type Target = TunedLoop;

    /// Starts relay tuning on `target`: from the next [`stabilize`](crate::Stabilizer::stabilize),
    /// a [`RelayTuner`] with `config` sets that loop's output (see the "Tuning" section on
    /// [`CopterStabilizer`]). Its bias starts at the loop's current I term (its trim).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::{
    ///     CopterStabilizer, CopterStabilizerConfig, TunedLoop,
    /// };
    /// use stable_flight::tuning::{RelayConfig, Tunable, TuningStatus};
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
    ///     bias_gain: 0.2,
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
    fn start_tuning(&mut self, target: TunedLoop, config: RelayConfig) {
        self.stop_tuning();
        let trim = self.integral_term(target);
        self.stage = Some(Stage {
            target,
            tuner: RelayTuner::new(config, trim),
            handed_back: false,
            interrupted: false,
        });
    }

    fn tuning(&self) -> Option<(TunedLoop, TuningStatus)> {
        self.stage
            .as_ref()
            .map(|stage| (stage.target, stage.status()))
    }

    fn stop_tuning(&mut self) -> Option<(TunedLoop, TuningStatus)> {
        self.hand_back();
        self.stage
            .take()
            .map(|stage| (stage.target, stage.status()))
    }
}

impl CopterStabilizer {
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
        let (location, trim) = (stage.target.location(), stage.tuner.bias());
        let config = &self.config;
        match location {
            Location::Rate(axis) => self.body_rate.hand_back(&config.body_rate, axis, trim),
            Location::Angle(axis) => self.attitude.hand_back(&config.attitude, axis, trim),
            Location::Heading => self.yaw.hand_back(&config.yaw, trim),
        }
    }

    /// Ends a running angle or heading stage as interrupted (its loop is idle on a rate tick).
    pub(super) fn interrupt_attitude_stage(&mut self) {
        let Some(stage) = &mut self.stage else {
            return;
        };
        let angle_loop = !matches!(stage.target.location(), Location::Rate(_));
        if angle_loop && stage.is_running() {
            stage.interrupted = true;
            self.hand_back();
        }
    }

    /// Where the running stage replaces a value, or None when no stage runs: the one check an
    /// idle tick pays.
    pub(super) fn running_stage(&self) -> Option<Location> {
        self.stage
            .as_ref()
            .filter(|stage| stage.is_running())
            .map(|stage| stage.target.location())
    }

    /// Steps the running stage with its loop's `error` and returns the relay output (0 when no
    /// stage runs). Hands the loop back once the tuner finishes.
    pub(super) fn step_stage(&mut self, dt: f32, error: f32) -> f32 {
        let Some(stage) = self.stage.as_mut().filter(|stage| stage.is_running()) else {
            return 0.0;
        };
        let output = stage.tuner.step(dt, error);
        if !matches!(stage.tuner.status(), RelayStatus::Running { .. }) {
            self.hand_back();
        }
        output
    }

    /// The I term of `target`'s controller: its trim.
    fn integral_term(&self, target: TunedLoop) -> f32 {
        let config = &self.config;
        match target.location() {
            Location::Rate(axis) => self.body_rate.integral_term(&config.body_rate, axis),
            Location::Angle(axis) => self.attitude.integral_term(&config.attitude, axis),
            Location::Heading => self.yaw.integral_term(&config.yaw),
        }
    }
}
