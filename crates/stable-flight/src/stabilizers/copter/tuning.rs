//! Autotune stages hosted by the [`CopterStabilizer`]: one loop's output replaced by a
//! [`Probe`]. See the "Tuning" section on [`CopterStabilizer`].

use crate::Vec3;
use crate::controllers::AttitudeController;
use crate::controllers::yaw::angle_diff;
use crate::math::saturate_symmetric;
use crate::tuning::{Injection, LoopSignals, Probe, ProbeConfig, ProbeStatus, Tunable};

use super::{CopterStabilizer, CopterStabilizerConfig};
use crate::controllers::Gains;

#[cfg(test)]
mod tests;

/// A loop of the [`CopterStabilizer`] that a tuning stage can replace.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TunedLoop {
    /// Roll rate loop: a probe sets the roll moment.
    RollRate,
    /// Pitch rate loop: a probe sets the pitch moment.
    PitchRate,
    /// Yaw rate loop: a probe sets the yaw moment.
    YawRate,
    /// Roll angle loop: a probe sets the roll rate setpoint, in rad/s.
    Roll,
    /// Pitch angle loop: a probe sets the pitch rate setpoint, in rad/s.
    Pitch,
    /// Heading loop: a probe sets the yaw rate setpoint, in rad/s.
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

    /// Whether this is a rate loop (the others are angle and heading loops).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    ///
    /// assert!(TunedLoop::YawRate.is_rate());
    /// assert!(!TunedLoop::Heading.is_rate());
    /// ```
    pub fn is_rate(self) -> bool {
        matches!(self.location(), Location::Rate(_))
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

/// A tuning stage: the loop under test and its probe.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct Stage {
    pub(super) target: TunedLoop,
    pub(super) probe: Probe,
    /// The loop has been handed back (finished, interrupted, or stopped): no more override.
    handed_back: bool,
    interrupted: bool,
}

impl Stage {
    /// Whether the probe still flies the loop.
    pub(super) fn is_running(&self) -> bool {
        !self.handed_back
    }

    fn status(&self) -> ProbeStatus {
        if self.interrupted {
            ProbeStatus::Interrupted
        } else {
            self.probe.status()
        }
    }
}

impl Tunable for CopterStabilizer {
    type Target = TunedLoop;

    /// Starts a probe on `target`: from the next [`stabilize`](crate::Stabilizer::stabilize),
    /// a [`Probe`] with `config` sets that loop's output or setpoint (see the "Tuning" section
    /// on [`CopterStabilizer`]). A relay starts from the loop's current I term (its trim).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::{
    ///     CopterStabilizer, CopterStabilizerConfig, TunedLoop,
    /// };
    /// use stable_flight::tuning::autotune::COPTER_RATE_RELAY;
    /// use stable_flight::tuning::{ProbeConfig, ProbeStatus, Tunable};
    /// use stable_flight::{RateSetpoint, Setpoint, Stabilizer, VehicleState};
    ///
    /// let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
    /// stabilizer.start_probe(TunedLoop::RollRate, ProbeConfig::Relay(COPTER_RATE_RELAY));
    ///
    /// // Rolling right at 0.5 rad/s with a zero rate setpoint: the relay pushes left.
    /// let mut state = VehicleState::default();
    /// state.angular_velocity_radps.x = 0.5;
    /// let output = stabilizer.stabilize(0.004, &Setpoint::Rate(RateSetpoint::default()), &state);
    /// assert_eq!(output.moments.x, -COPTER_RATE_RELAY.initial_relay);
    ///
    /// assert!(matches!(stabilizer.probe(), Some((TunedLoop::RollRate, ProbeStatus::Running(_)))));
    /// stabilizer.stop_probe(); // hands the roll rate loop back
    /// assert!(stabilizer.probe().is_none());
    /// ```
    fn start_probe(&mut self, target: TunedLoop, config: ProbeConfig) {
        self.stop_probe();
        let trim = self.integral_term(target);
        self.stage = Some(Stage {
            target,
            probe: Probe::new(config, trim),
            handed_back: false,
            interrupted: false,
        });
    }

    fn probe(&self) -> Option<(TunedLoop, ProbeStatus)> {
        self.stage
            .as_ref()
            .map(|stage| (stage.target, stage.status()))
    }

    fn stop_probe(&mut self) -> Option<(TunedLoop, ProbeStatus)> {
        self.hand_back();
        self.stage
            .take()
            .map(|stage| (stage.target, stage.status()))
    }
}

impl CopterStabilizer {
    /// Hands a running stage's loop back to its controller: the loop restarts with no
    /// derivative history and the probe's trim as its I term (a step has none, so the I term
    /// stays). A no-op when no stage is running.
    pub(super) fn hand_back(&mut self) {
        let Some(stage) = &mut self.stage else {
            return;
        };
        if stage.handed_back {
            return;
        }
        stage.handed_back = true;
        let (target, trim) = (stage.target, stage.probe.trim());
        let location = target.location();
        let trim = trim.unwrap_or_else(|| self.integral_term(target));
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

    /// Steps the running stage with its loop's `signals` and returns what to do with the loop
    /// (`None` when no stage runs). The loop is handed back at the end of the tick
    /// ([`finish_stage`](Self::finish_stage)), after its controller ran.
    fn step_stage(&mut self, dt: f32, signals: &LoopSignals) -> Option<Injection> {
        let stage = self.stage.as_mut().filter(|stage| stage.is_running())?;
        Some(stage.probe.step(dt, signals))
    }

    /// Hands the loop back once its probe has finished; call it at the end of the tick.
    pub(super) fn finish_stage(&mut self) {
        let finished = self
            .stage
            .as_ref()
            .is_some_and(|stage| stage.is_running() && !stage.probe.status().is_running());
        if finished {
            self.hand_back();
        }
    }

    /// Steps a probe on an angle or heading loop before the controllers run. `setpoints` are
    /// the roll, pitch, and heading setpoints, `current` the estimate: a step replaces one
    /// setpoint; a relay's rate output comes back as `(axis, rate)` (axis 2: heading) for the
    /// caller to apply after the controllers.
    pub(super) fn probe_attitude(
        &mut self,
        dt: f32,
        setpoints: &mut Vec3,
        current: &Vec3,
    ) -> Option<(usize, f32)> {
        let location = self.running_stage()?;
        let saturated = self.loop_saturated(location);
        let (axis, signals) = match location {
            Location::Angle(axis) => {
                let attitude = &self.config.attitude;
                let error = AttitudeController::angle_error(attitude, setpoints.xy(), current.xy());
                let setpoint = saturate_symmetric(setpoints[axis], attitude.max_angles_rad[axis]);
                (
                    axis,
                    LoopSignals {
                        setpoint,
                        error: error[axis],
                        saturated,
                    },
                )
            }
            Location::Heading => {
                let error = angle_diff(setpoints.z, current.z);
                (
                    2,
                    LoopSignals {
                        setpoint: setpoints.z,
                        error,
                        saturated,
                    },
                )
            }
            Location::Rate(_) => return None,
        };
        self.inject(dt, axis, &signals, setpoints)
    }

    /// Steps a probe on a rate loop before the rate controller runs: a step replaces that
    /// axis's rate in `setpoints`; a relay's moment comes back as `(axis, moment)` for the
    /// caller to apply after it.
    pub(super) fn probe_rates(
        &mut self,
        dt: f32,
        setpoints: &mut Vec3,
        current: &Vec3,
    ) -> Option<(usize, f32)> {
        let location @ Location::Rate(axis) = self.running_stage()? else {
            return None;
        };
        let signals = LoopSignals {
            setpoint: setpoints[axis],
            error: setpoints[axis] - current[axis],
            saturated: self.loop_saturated(location),
        };
        self.inject(dt, axis, &signals, setpoints)
    }

    /// Steps the running probe with `signals` for `axis` of `setpoints`: a step's setpoint
    /// replaces `setpoints[axis]`; a relay's output comes back as `(axis, output)`.
    fn inject(
        &mut self,
        dt: f32,
        axis: usize,
        signals: &LoopSignals,
        setpoints: &mut Vec3,
    ) -> Option<(usize, f32)> {
        match self.step_stage(dt, signals)? {
            Injection::Setpoint(setpoint) => {
                setpoints[axis] = setpoint;
                None
            }
            Injection::Output(output) => Some((axis, output)),
        }
    }

    /// Whether `location`'s loop, or the loop under it, clipped on the last tick: a rate
    /// loop's moment; an angle or heading loop's rate setpoint or that axis's moment.
    fn loop_saturated(&self, location: Location) -> bool {
        let clipped = &self.clipped;
        match location {
            Location::Rate(axis) => clipped.moments[axis],
            Location::Angle(axis) => clipped.rates[axis] || clipped.moments[axis],
            Location::Heading => clipped.rates[2] || clipped.moments[2],
        }
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
