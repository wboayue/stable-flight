//! Autotune stages hosted by the [`PlaneStabilizer`]: one loop's setpoint replaced by a
//! [`Probe`]. See the "Tuning" section on the [module docs](super).

use crate::VehicleState;
use crate::controllers::RateGains;
use crate::tuning::{LoopSignals, Probe, ProbeConfig, ProbeStatus, Tunable};

use super::{PlaneStabilizer, PlaneStabilizerConfig};

#[cfg(test)]
mod tests;

/// A loop of the [`PlaneStabilizer`] that a tuning stage can replace.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlaneLoop {
    /// Roll rate loop: a probe sets the roll rate setpoint, in rad/s.
    RollRate,
    /// Pitch rate loop: a probe sets the pitch rate setpoint, in rad/s.
    PitchRate,
    /// Roll angle loop: a probe sets the bank setpoint, in rad.
    Roll,
    /// Pitch angle loop: a probe sets the pitch setpoint, in rad.
    Pitch,
}

/// One axis's gains on the [`PlaneStabilizer`]: its rate loop's, and its attitude loop's time
/// constant. A loop's stage reads and sets its whole axis: on a plane the rate loop's
/// feedforward and the attitude loop's time constant are tuned together, against one attitude
/// step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlaneAxisGains {
    /// The rate loop's gains.
    pub rate: RateGains,
    /// The attitude loop's time constant, in s: the rate target is the error divided by it.
    pub time_constant_s: f32,
}

impl PlaneLoop {
    /// The body axis this loop flies: 0 roll, 1 pitch.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::plane::PlaneLoop;
    ///
    /// assert_eq!(PlaneLoop::Pitch.axis(), 1);
    /// ```
    pub fn axis(self) -> usize {
        match self {
            Self::RollRate | Self::Roll => 0,
            Self::PitchRate | Self::Pitch => 1,
        }
    }

    /// Whether this is a rate loop (the others are angle loops).
    pub fn is_rate(self) -> bool {
        matches!(self, Self::RollRate | Self::PitchRate)
    }

    /// The rate loop under this angle loop; `None` for a rate loop.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::plane::PlaneLoop;
    ///
    /// assert_eq!(PlaneLoop::Roll.inner(), Some(PlaneLoop::RollRate));
    /// assert_eq!(PlaneLoop::RollRate.inner(), None);
    /// ```
    pub fn inner(self) -> Option<Self> {
        match self {
            Self::Roll => Some(Self::RollRate),
            Self::Pitch => Some(Self::PitchRate),
            Self::RollRate | Self::PitchRate => None,
        }
    }

    /// The gains of this loop's axis in `config`.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::plane::{PlaneLoop, PlaneStabilizerConfig};
    ///
    /// let mut config = PlaneStabilizerConfig::default();
    /// let mut gains = PlaneLoop::Pitch.gains(&config);
    /// gains.time_constant_s = 0.6;
    /// PlaneLoop::PitchRate.set_gains(&mut config, gains);
    /// assert_eq!(config.time_constant_s.y, 0.6);
    /// ```
    pub fn gains(self, config: &PlaneStabilizerConfig) -> PlaneAxisGains {
        match self.axis() {
            0 => PlaneAxisGains {
                rate: config.rate.roll,
                time_constant_s: config.time_constant_s.x,
            },
            _ => PlaneAxisGains {
                rate: config.rate.pitch,
                time_constant_s: config.time_constant_s.y,
            },
        }
    }

    /// Sets the gains of this loop's axis in `config`.
    pub fn set_gains(self, config: &mut PlaneStabilizerConfig, gains: PlaneAxisGains) {
        match self.axis() {
            0 => {
                config.rate.roll = gains.rate;
                config.time_constant_s.x = gains.time_constant_s;
            }
            _ => {
                config.rate.pitch = gains.rate;
                config.time_constant_s.y = gains.time_constant_s;
            }
        }
    }
}

/// A tuning stage: the loop under test and its probe.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct Stage {
    target: PlaneLoop,
    probe: Probe,
    /// The loop has been handed back (finished, interrupted, or stopped): no more override.
    handed_back: bool,
    interrupted: bool,
}

impl Stage {
    fn status(&self) -> ProbeStatus {
        if self.interrupted {
            ProbeStatus::Interrupted
        } else {
            self.probe.status()
        }
    }
}

impl Tunable<PlaneLoop> for PlaneStabilizer {
    /// Starts a probe on `target`: from the next [`stabilize`](crate::Stabilizer::stabilize),
    /// a [`Probe`] with `config` sets that loop's setpoint (see the "Tuning" section on the
    /// [module docs](super)).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::plane::{PlaneLoop, PlaneStabilizer, PlaneStabilizerConfig};
    /// use stable_flight::tuning::{ProbeConfig, ProbeStatus, StepConfig, Tunable};
    /// use stable_flight::{PlaneSetpoint, Setpoint, Stabilizer, VehicleState};
    ///
    /// let mut stabilizer = PlaneStabilizer::new(PlaneStabilizerConfig::default());
    /// let step = StepConfig { amplitude: 0.3, duration_s: 1.0, settle_band: 0.1, filter_hz: 0.0 };
    /// stabilizer.start_probe(PlaneLoop::Roll, ProbeConfig::Step(step));
    ///
    /// // Level, with a level setpoint: the step asks for 0.3 rad of bank, so the wing rolls right.
    /// let level = Setpoint::Plane(PlaneSetpoint { thrust: 0.6, ..Default::default() });
    /// let output = stabilizer.stabilize(0.004, &level, &VehicleState::default());
    /// assert!(output.moments.x > 0.0);
    ///
    /// assert!(matches!(stabilizer.probe(), Some((PlaneLoop::Roll, ProbeStatus::Running(_)))));
    /// stabilizer.stop_probe(); // hands the roll loop back
    /// assert!(Tunable::<PlaneLoop>::probe(&stabilizer).is_none());
    /// ```
    fn start_probe(&mut self, target: PlaneLoop, config: ProbeConfig) {
        self.stop_probe();
        self.stage = Some(Stage {
            target,
            probe: Probe::new(config),
            handed_back: false,
            interrupted: false,
        });
    }

    fn probe(&self) -> Option<(PlaneLoop, ProbeStatus)> {
        self.stage
            .as_ref()
            .map(|stage| (stage.target, stage.status()))
    }

    fn stop_probe(&mut self) -> Option<(PlaneLoop, ProbeStatus)> {
        self.stage
            .take()
            .map(|stage| (stage.target, stage.status()))
    }
}

impl PlaneStabilizer {
    /// Ends a running stage as interrupted: its loop is idle, or the setpoint stopped flying
    /// the plane side (another setpoint, a transition). The loops have no derivative to reset,
    /// so handing back only stops the override; the rate integral stays.
    pub(crate) fn interrupt_stage(&mut self) {
        if let Some(stage) = self.stage.as_mut().filter(|stage| !stage.handed_back) {
            stage.handed_back = true;
            stage.interrupted = true;
        }
    }

    /// Ends a running angle stage as interrupted: a rate setpoint idles the attitude loops.
    pub(super) fn interrupt_angle_stage(&mut self) {
        if self.running_stage().is_some_and(|target| !target.is_rate()) {
            self.interrupt_stage();
        }
    }

    /// The loop the running stage replaces a setpoint on, or `None` when no stage runs.
    fn running_stage(&self) -> Option<PlaneLoop> {
        self.stage
            .as_ref()
            .filter(|stage| !stage.handed_back)
            .map(|stage| stage.target)
    }

    /// Steps a probe on an angle loop before its rate target is computed. `angles_rad` are the
    /// saturated bank and pitch setpoints, `error_rad` the body-frame attitude error the loops
    /// act on, `rates_radps` the body rates: the probe replaces its axis's angle.
    pub(super) fn probe_angles(
        &mut self,
        dt: f32,
        angles_rad: &mut [f32; 2],
        error_rad: [f32; 2],
        rates_radps: &crate::Vec3,
    ) {
        let Some(target) = self.running_stage().filter(|target| !target.is_rate()) else {
            return;
        };
        let axis = target.axis();
        let signals = LoopSignals {
            setpoint: angles_rad[axis],
            error: error_rad[axis],
            saturated: self.moment_clipped(axis),
            rate_radps: rates_radps[axis],
            last_moment: self.last_moments[axis],
            alpha_rad: 0.0,
        };
        if let Some(angle_rad) = self.step_stage(dt, &signals) {
            angles_rad[axis] = angle_rad;
        }
    }

    /// Steps a probe on a rate loop before the rate controller runs: the probe replaces its
    /// axis's rate in `desired_radps`, read against `state`'s body rates; a pitch probe also
    /// reads the angle of attack.
    pub(super) fn probe_rates(
        &mut self,
        dt: f32,
        desired_radps: &mut crate::Vec3,
        state: &VehicleState,
    ) {
        let Some(target) = self.running_stage().filter(|target| target.is_rate()) else {
            return;
        };
        let axis = target.axis();
        let rates_radps = &state.angular_velocity_radps;
        let alpha_rad = match target {
            PlaneLoop::PitchRate => self.angle_of_attack_rad(state),
            _ => 0.0,
        };
        let signals = LoopSignals {
            setpoint: desired_radps[axis],
            error: desired_radps[axis] - rates_radps[axis],
            saturated: self.moment_clipped(axis),
            rate_radps: rates_radps[axis],
            last_moment: self.last_moments[axis],
            alpha_rad,
        };
        if let Some(rate_radps) = self.step_stage(dt, &signals) {
            desired_radps[axis] = rate_radps;
        }
    }

    /// Hands the loop back once its probe has finished; call it at the end of the tick, after
    /// the loop's controller ran on the probe's last setpoint.
    pub(super) fn finish_stage(&mut self) {
        if let Some(stage) = self.stage.as_mut()
            && !stage.handed_back
            && !stage.probe.status().is_running()
        {
            stage.handed_back = true;
        }
    }

    fn step_stage(&mut self, dt: f32, signals: &LoopSignals) -> Option<f32> {
        let stage = self.stage.as_mut().filter(|stage| !stage.handed_back)?;
        Some(stage.probe.step(dt, signals))
    }

    /// Whether `axis`'s moment was at full deflection on the last tick.
    fn moment_clipped(&self, axis: usize) -> bool {
        self.last_moments[axis].abs() >= 1.0
    }
}
