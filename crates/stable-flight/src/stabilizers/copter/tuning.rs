//! Autotune stages hosted by the [`CopterStabilizer`]: one loop's setpoint replaced by a
//! [`Probe`]. See the "Tuning" section on [`CopterStabilizer`].

use crate::Vec3;
use crate::tuning::{LoopSignals, Probe, ProbeConfig, ProbeStatus, Tunable};

use super::{CopterStabilizer, CopterStabilizerConfig};
use crate::controllers::Gains;

#[cfg(test)]
mod tests;

/// A loop of the [`CopterStabilizer`] that a tuning stage can replace.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TunedLoop {
    /// Roll rate loop: a probe sets the roll rate setpoint, in rad/s.
    RollRate,
    /// Pitch rate loop: a probe sets the pitch rate setpoint, in rad/s.
    PitchRate,
    /// Yaw rate loop: a probe sets the yaw rate setpoint, in rad/s.
    YawRate,
    /// Roll angle loop: a probe sets the roll angle setpoint, in rad.
    Roll,
    /// Pitch angle loop: a probe sets the pitch angle setpoint, in rad.
    Pitch,
    /// Heading loop: a probe sets the heading setpoint, in rad.
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

    /// The rate loop under this angle or heading loop, which flies its rate setpoint; `None`
    /// for a rate loop.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    ///
    /// assert_eq!(TunedLoop::Roll.inner(), Some(TunedLoop::RollRate));
    /// assert_eq!(TunedLoop::RollRate.inner(), None);
    /// ```
    pub fn inner(self) -> Option<Self> {
        match self {
            Self::Roll => Some(Self::RollRate),
            Self::Pitch => Some(Self::PitchRate),
            Self::Heading => Some(Self::YawRate),
            Self::RollRate | Self::PitchRate | Self::YawRate => None,
        }
    }

    /// The angle or heading loop over this rate loop, which sets its setpoint; `None` for an
    /// angle or heading loop.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    ///
    /// assert_eq!(TunedLoop::PitchRate.outer(), Some(TunedLoop::Pitch));
    /// assert_eq!(TunedLoop::Pitch.outer(), None);
    /// ```
    pub fn outer(self) -> Option<Self> {
        match self {
            Self::RollRate => Some(Self::Roll),
            Self::PitchRate => Some(Self::Pitch),
            Self::YawRate => Some(Self::Heading),
            Self::Roll | Self::Pitch | Self::Heading => None,
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
    /// a [`Probe`] with `config` sets that loop's setpoint (see the "Tuning" section on
    /// [`CopterStabilizer`]).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::{
    ///     CopterStabilizer, CopterStabilizerConfig, TunedLoop,
    /// };
    /// use stable_flight::tuning::autotune::COPTER_RATE_STEP;
    /// use stable_flight::tuning::{ProbeConfig, ProbeStatus, Tunable};
    /// use stable_flight::{RateSetpoint, Setpoint, Stabilizer, VehicleState};
    ///
    /// let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
    /// stabilizer.start_probe(TunedLoop::RollRate, ProbeConfig::Step(COPTER_RATE_STEP));
    ///
    /// // At rest with a zero rate setpoint: the step asks for 1 rad/s of roll, so the loop
    /// // pushes right.
    /// let state = VehicleState::default();
    /// let output = stabilizer.stabilize(0.004, &Setpoint::Rate(RateSetpoint::default()), &state);
    /// assert!(output.moments.x > 0.0);
    ///
    /// assert!(matches!(stabilizer.probe(), Some((TunedLoop::RollRate, ProbeStatus::Running(_)))));
    /// stabilizer.stop_probe(); // hands the roll rate loop back
    /// assert!(stabilizer.probe().is_none());
    /// ```
    fn start_probe(&mut self, target: TunedLoop, config: ProbeConfig) {
        self.stop_probe();
        self.stage = Some(Stage {
            target,
            probe: Probe::new(config),
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
    /// derivative history, so the setpoint's jump back doesn't kick it, and keeps its I term.
    /// A no-op when no stage is running.
    pub(super) fn hand_back(&mut self) {
        let Some(stage) = &mut self.stage else {
            return;
        };
        if stage.handed_back {
            return;
        }
        stage.handed_back = true;
        match stage.target.location() {
            Location::Rate(axis) => self.body_rate.hand_back(axis),
            Location::Angle(axis) => self.attitude.hand_back(axis),
            Location::Heading => self.yaw.hand_back(),
        }
    }

    /// Which attitude loops keep their integral this tick (roll, pitch, heading): the one over
    /// a rate loop a step flies. Integrating the error the step tilts the vehicle into would
    /// wind that loop up and kick the vehicle on the handoff; at angle P 2 and I 0.25 the sim's
    /// `quad_x` then didn't settle within 10 s after a large rate step (#579).
    pub(super) fn held_integrals(&self) -> [bool; 3] {
        let stepped_axis = self
            .stage
            .as_ref()
            .filter(|stage| stage.is_running() && stage.probe.replaces_setpoint())
            .and_then(|stage| match stage.target.location() {
                Location::Rate(axis) => Some(axis),
                Location::Angle(_) | Location::Heading => None,
            });
        [0, 1, 2].map(|axis| stepped_axis == Some(axis))
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

    /// Steps the running stage with its loop's `signals` and returns the setpoint the loop
    /// runs on (`None` when no stage runs). The loop is handed back at the end of the tick
    /// ([`finish_stage`](Self::finish_stage)), after its controller ran.
    fn step_stage(&mut self, dt: f32, signals: &LoopSignals) -> Option<f32> {
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
    /// the saturated roll, pitch, and heading setpoints, `error` the body-frame attitude error
    /// the loops act on, `rates_radps` the body rates: the probe replaces one setpoint.
    pub(super) fn probe_attitude(
        &mut self,
        dt: f32,
        setpoints: &mut Vec3,
        error: &Vec3,
        rates_radps: &Vec3,
    ) {
        let Some(location) = self.running_stage() else {
            return;
        };
        let saturated = self.loop_saturated(location);
        let axis = match location {
            Location::Angle(axis) => axis,
            Location::Heading => 2,
            Location::Rate(_) => return,
        };
        let signals = LoopSignals {
            setpoint: setpoints[axis],
            error: error[axis],
            saturated,
            rate_radps: rates_radps[axis],
            last_moment: self.last_moments[axis],
        };
        self.inject(dt, axis, &signals, setpoints)
    }

    /// Steps a probe on a rate loop before the rate controller runs: the probe replaces that
    /// axis's rate in `setpoints`. `filtered_rates_radps` holds the filtered rates the loop
    /// runs on, `raw_rates_radps` the unfiltered ones the identify fit models the airframe from.
    pub(super) fn probe_rates(
        &mut self,
        dt: f32,
        setpoints: &mut Vec3,
        filtered_rates_radps: &Vec3,
        raw_rates_radps: &Vec3,
    ) {
        let Some(location @ Location::Rate(axis)) = self.running_stage() else {
            return;
        };
        let signals = LoopSignals {
            setpoint: setpoints[axis],
            error: setpoints[axis] - filtered_rates_radps[axis],
            saturated: self.loop_saturated(location),
            rate_radps: raw_rates_radps[axis],
            last_moment: self.last_moments[axis],
        };
        self.inject(dt, axis, &signals, setpoints)
    }

    /// Steps the running probe with `signals` for `axis` of `setpoints`: its setpoint
    /// replaces `setpoints[axis]`.
    fn inject(&mut self, dt: f32, axis: usize, signals: &LoopSignals, setpoints: &mut Vec3) {
        if let Some(setpoint) = self.step_stage(dt, signals) {
            setpoints[axis] = setpoint;
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
}
