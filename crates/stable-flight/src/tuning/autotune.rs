//! The copter autotune supervisor: see [`Autotune`].

use core::fmt;

use crate::controllers::{Gains, InvalidConfig, check, check_positive};
use crate::math::euler_angles;
use crate::stabilizers::copter::{CopterStabilizerConfig, TunedLoop};
use crate::{
    Commander, Configurable, FlightLoop, Guidance, Mixer, Stabilizer, TelemetryPublisher, Vehicle,
    VehicleState,
};

use super::{
    ControllerForm, RelayAbort, RelayConfig, RelayResult, Tunable, TuningRule, TuningStatus,
    gains_from_ultimate,
};

#[cfg(test)]
mod tests;

/// The most stages a plan may hold: one per [`TunedLoop`].
pub const MAX_STAGES: usize = 6;

/// Relay for a copter rate loop: normalized moment out, rad/s in.
pub const COPTER_RATE_RELAY: RelayConfig = RelayConfig {
    initial_relay: 0.05,
    max_relay: 0.4,
    relay_step: 1.5,
    hysteresis: 0.005,
    min_cycle_amplitude: 0.1,
    max_cycle_amplitude: 2.0,
    max_half_cycle_s: 0.3,
    max_bias: 0.5,
    cycles: 4,
    tolerance: 0.1,
    timeout_s: 20.0,
};

/// Relay for a copter angle or heading loop: rad/s out, rad in.
pub const COPTER_ANGLE_RELAY: RelayConfig = RelayConfig {
    initial_relay: 0.2,
    max_relay: 2.0,
    relay_step: 1.5,
    hysteresis: 0.002,
    min_cycle_amplitude: 0.04,
    max_cycle_amplitude: 0.35,
    max_half_cycle_s: 1.0,
    max_bias: 1.0,
    cycles: 4,
    tolerance: 0.1,
    timeout_s: 30.0,
};

/// The order [`StagePlan::copter_sequence`] tunes in: innermost loops first, so each stage
/// flies on the gains the earlier ones found.
pub const COPTER_SEQUENCE: [TunedLoop; MAX_STAGES] = [
    TunedLoop::RollRate,
    TunedLoop::PitchRate,
    TunedLoop::YawRate,
    TunedLoop::Roll,
    TunedLoop::Pitch,
    TunedLoop::Heading,
];

/// One stage of an autotune: which loop, how to excite it, and how to turn the measurement
/// into gains.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StagePlan {
    /// The loop to tune.
    pub target: TunedLoop,
    /// The relay that measures it.
    pub relay: RelayConfig,
    /// The tuning rule for its gains.
    pub rule: TuningRule,
    /// Which terms its gains use.
    pub form: ControllerForm,
}

impl StagePlan {
    /// The copter defaults for `target`, Tyreus–Luyben throughout: roll and pitch rate PID
    /// ([`COPTER_RATE_RELAY`]), yaw rate PI (its D is 0 in hand tunes), and the angle and
    /// heading loops PD ([`COPTER_ANGLE_RELAY`]).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    /// use stable_flight::tuning::ControllerForm;
    /// use stable_flight::tuning::autotune::StagePlan;
    ///
    /// assert_eq!(StagePlan::copter(TunedLoop::YawRate).form, ControllerForm::Pi);
    /// ```
    pub fn copter(target: TunedLoop) -> Self {
        let (relay, form) = match target {
            TunedLoop::RollRate | TunedLoop::PitchRate => (COPTER_RATE_RELAY, ControllerForm::Pid),
            TunedLoop::YawRate => (COPTER_RATE_RELAY, ControllerForm::Pi),
            TunedLoop::Roll | TunedLoop::Pitch | TunedLoop::Heading => {
                (COPTER_ANGLE_RELAY, ControllerForm::Pd)
            }
        };
        Self {
            target,
            relay,
            rule: TuningRule::TyreusLuyben,
            form,
        }
    }

    /// Every loop, in [`COPTER_SEQUENCE`] order, with the [`copter`](Self::copter) defaults.
    pub fn copter_sequence() -> [Self; MAX_STAGES] {
        COPTER_SEQUENCE.map(Self::copter)
    }
}

/// Limits that end a stage, and how still the vehicle must be before one starts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Envelope {
    /// Roll or pitch beyond this aborts the stage, in rad.
    pub max_tilt_rad: f32,
    /// Any body rate beyond this aborts the stage, in rad/s.
    pub max_rate_radps: f32,
    /// Sinking more than this below the altitude at [`Autotune::start`] aborts, in m.
    pub max_altitude_loss_m: f32,
    /// Level enough to start a stage: roll and pitch within this, in rad.
    pub settle_tilt_rad: f32,
    /// Still enough to start a stage: body rates within this, in rad/s.
    pub settle_rate_radps: f32,
    /// How long the vehicle must stay level and still before a stage starts, in s.
    pub settle_s: f32,
    /// How long to wait for it before failing, in s.
    pub settle_timeout_s: f32,
}

impl Default for Envelope {
    /// Copter limits: 35° tilt, 4 rad/s, 3 m sink; settle within 6° and 0.3 rad/s for 1 s,
    /// waiting up to 10 s.
    fn default() -> Self {
        Self {
            max_tilt_rad: 0.6,
            max_rate_radps: 4.0,
            max_altitude_loss_m: 3.0,
            settle_tilt_rad: 0.1,
            settle_rate_radps: 0.3,
            settle_s: 1.0,
            settle_timeout_s: 10.0,
        }
    }
}

impl Envelope {
    /// Checks that every limit and time is positive and finite, and that settling is
    /// tighter than aborting.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_positive("max_tilt_rad", self.max_tilt_rad)?;
        check_positive("max_rate_radps", self.max_rate_radps)?;
        check_positive("max_altitude_loss_m", self.max_altitude_loss_m)?;
        check(
            "settle_tilt_rad",
            self.settle_tilt_rad > 0.0 && self.settle_tilt_rad < self.max_tilt_rad,
            "positive and below max_tilt_rad",
        )?;
        check(
            "settle_rate_radps",
            self.settle_rate_radps > 0.0 && self.settle_rate_radps < self.max_rate_radps,
            "positive and below max_rate_radps",
        )?;
        check_positive("settle_s", self.settle_s)?;
        check(
            "settle_timeout_s",
            self.settle_timeout_s > self.settle_s && self.settle_timeout_s.is_finite(),
            "finite and longer than settle_s",
        )
    }

    /// The limit `state` breaks, if any. NaN never counts as a breach.
    fn breach(&self, state: &VehicleState, start_altitude_m: f32) -> Option<AutotuneAbort> {
        let (roll, pitch, _) = euler_angles(&state.attitude);
        if roll.abs() > self.max_tilt_rad || pitch.abs() > self.max_tilt_rad {
            return Some(AutotuneAbort::Tilt);
        }
        if state.angular_velocity_radps.amax() > self.max_rate_radps {
            return Some(AutotuneAbort::Rate);
        }
        if start_altitude_m - state.position_m.z > self.max_altitude_loss_m {
            return Some(AutotuneAbort::AltitudeLoss);
        }
        None
    }

    /// Whether `state` is level and still enough to start a stage.
    fn settled(&self, state: &VehicleState) -> bool {
        let (roll, pitch, _) = euler_angles(&state.attitude);
        roll.abs() < self.settle_tilt_rad
            && pitch.abs() < self.settle_tilt_rad
            && state.angular_velocity_radps.amax() < self.settle_rate_radps
    }
}

/// Why a stage failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AutotuneAbort {
    /// Roll or pitch passed [`Envelope::max_tilt_rad`].
    Tilt,
    /// A body rate passed [`Envelope::max_rate_radps`].
    Rate,
    /// The vehicle sank more than [`Envelope::max_altitude_loss_m`].
    AltitudeLoss,
    /// The vehicle didn't settle within [`Envelope::settle_timeout_s`].
    NotSettled,
    /// The relay tuner gave up.
    Relay(RelayAbort),
    /// The stabilizer idled the loop under test (e.g. a rate setpoint), or the stage was
    /// stopped from outside.
    Interrupted,
    /// The stabilizer rejected the tuned config.
    Rejected,
}

/// What an [`Autotune::update`] did, for the caller to log.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AutotuneEvent {
    /// A stage started.
    StageStarted(TunedLoop),
    /// A stage measured its loop and committed the gains.
    StageDone {
        /// The tuned loop.
        target: TunedLoop,
        /// What the relay measured.
        result: RelayResult,
        /// The gains committed for it.
        gains: Gains,
    },
    /// A stage failed: its loop keeps its gains, earlier stages' stay, and the run ends.
    StageFailed {
        /// The loop the stage was for.
        target: TunedLoop,
        /// Why.
        reason: AutotuneAbort,
    },
    /// Every stage committed; the run is over and its gains stay.
    Complete,
    /// Disarmed before completion: every stage reverted to the config at
    /// [`Autotune::start`].
    Reverted,
}

/// Where a run stands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AutotuneStatus {
    /// Not started.
    Idle,
    /// Waiting for the vehicle to settle before stage `stage` (an index into the plan).
    Settling {
        /// The stage about to start.
        stage: usize,
    },
    /// Stage `stage` is measuring `target`.
    Running {
        /// The stage index.
        stage: usize,
        /// Its loop.
        target: TunedLoop,
    },
    /// Every stage committed.
    Complete,
    /// Stage `stage` failed; earlier stages stay committed.
    Failed {
        /// The failed stage's index.
        stage: usize,
        /// Why.
        reason: AutotuneAbort,
    },
    /// Disarmed before completion; everything reverted.
    Reverted,
}

/// Why [`Autotune::start`] refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartError {
    /// The last tick wasn't armed.
    NotArmed,
    /// A run is already in progress.
    Running,
}

impl fmt::Display for StartError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::NotArmed => "not armed",
            Self::Running => "already running",
        })
    }
}

impl core::error::Error for StartError {}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    Idle,
    Settling {
        stage: usize,
        level_s: f32,
        waited_s: f32,
    },
    Running {
        stage: usize,
    },
    /// The last stage committed; the next update reports `Complete`.
    Finishing,
    Complete,
    Failed {
        stage: usize,
        reason: AutotuneAbort,
    },
    Reverted,
}

/// Runs a copter autotune between ticks (#149): each [`StagePlan`] in turn hands one loop to
/// a relay ([`Tunable::start_tuning`]), and its measurement becomes that loop's gains.
///
/// - **Start** ([`start`](Self::start)) when armed and hovering; the caller owns the trigger.
///   The config and altitude at start are the baseline.
/// - **Settle:** before each stage, the vehicle must stay within the [`Envelope`]'s settle
///   limits for `settle_s`.
/// - **Measure:** the stage runs until the relay finishes. On `Done`, the plan's rule turns
///   the [`RelayResult`] into gains, committed through
///   [`set_stabilizer_config`](FlightLoop::set_stabilizer_config); the next stage flies them.
/// - **Fail:** an envelope breach, a relay abort, or an interrupt stops the stage and ends the
///   run. A stage commits only when it finishes, so the failed loop keeps its gains and the
///   earlier stages' stay.
/// - **Disarm** before completion reverts every stage to the baseline.
/// - **Complete:** the gains stay; read them from the stabilizer's config.
///
/// [`update`](Self::update) returns an [`AutotuneEvent`] when something happens, for the
/// caller to log; the supervisor has no clock (the caller passes `dt`) and does no I/O.
///
/// # Examples
///
/// ```no_run
/// # use stable_flight::FlightLoop;
/// # use stable_flight::stabilizers::copter::CopterStabilizer;
/// # use stable_flight::guidance::copter::CopterGuidance;
/// # use stable_flight::mixers::QuadCopterMixer;
/// # use stable_flight::testing::{ConstantCommander, NullVehicle};
/// use stable_flight::tuning::autotune::{Autotune, AutotuneEvent, Envelope, StagePlan};
///
/// # fn fly(flight_loop: &mut FlightLoop<ConstantCommander, NullVehicle, CopterGuidance, CopterStabilizer, QuadCopterMixer, ()>) {
/// let mut autotune = Autotune::new(&StagePlan::copter_sequence(), Envelope::default())
///     .expect("valid plan");
/// autotune.start(flight_loop).expect("armed");
/// loop {
///     flight_loop.step(std::time::Duration::from_millis(4));
///     if let Some(event) = autotune.update(flight_loop, 0.004) {
///         println!("{event:?}");
///         if matches!(event, AutotuneEvent::Complete | AutotuneEvent::Reverted) {
///             break;
///         }
///     }
/// }
/// # }
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct Autotune {
    plan: [Option<StagePlan>; MAX_STAGES],
    stages: usize,
    envelope: Envelope,
    phase: Phase,
    baseline: Option<CopterStabilizerConfig>,
    start_altitude_m: f32,
}

impl Autotune {
    /// A supervisor for `plan` (1 to [`MAX_STAGES`] stages) within `envelope`.
    ///
    /// # Errors
    ///
    /// When the plan is empty or too long (`plan`), a stage's relay is invalid
    /// (`plan.relay`), or the envelope is (`envelope`).
    pub fn new(plan: &[StagePlan], envelope: Envelope) -> Result<Self, InvalidConfig> {
        check(
            "stages",
            (1..=MAX_STAGES).contains(&plan.len()),
            "1 to MAX_STAGES",
        )
        .map_err(|e| e.in_section("plan"))?;
        for stage in plan {
            stage
                .relay
                .validate()
                .map_err(|e| e.in_section("plan.relay"))?;
        }
        envelope.validate().map_err(|e| e.in_section("envelope"))?;

        let mut stages = [None; MAX_STAGES];
        for (slot, stage) in stages.iter_mut().zip(plan) {
            *slot = Some(*stage);
        }
        Ok(Self {
            plan: stages,
            stages: plan.len(),
            envelope,
            phase: Phase::Idle,
            baseline: None,
            start_altitude_m: 0.0,
        })
    }

    /// Starts a run: records the stabilizer's config and the altitude as the baseline, then
    /// settles for the first stage. A finished run can be started again.
    ///
    /// # Errors
    ///
    /// [`StartError::NotArmed`] unless the last tick was armed; [`StartError::Running`]
    /// while a run is in progress.
    pub fn start<C, V, G, S, M, T>(
        &mut self,
        flight_loop: &FlightLoop<C, V, G, S, M, T>,
    ) -> Result<(), StartError>
    where
        C: Commander,
        V: Vehicle,
        G: Guidance,
        S: Stabilizer + Tunable<Target = TunedLoop> + Configurable<Config = CopterStabilizerConfig>,
        M: Mixer,
        T: TelemetryPublisher,
    {
        if self.is_running() {
            return Err(StartError::Running);
        }
        if !flight_loop.armed() {
            return Err(StartError::NotArmed);
        }
        self.baseline = Some(*flight_loop.stabilizer().config());
        self.start_altitude_m = flight_loop.state().position_m.z;
        self.phase = Self::settling(0);
        Ok(())
    }

    /// Advances the run by `dt` seconds; call it between ticks (after
    /// [`step`](FlightLoop::step)). Returns what happened, if anything.
    pub fn update<C, V, G, S, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        dt: f32,
    ) -> Option<AutotuneEvent>
    where
        C: Commander,
        V: Vehicle,
        G: Guidance,
        S: Stabilizer + Tunable<Target = TunedLoop> + Configurable<Config = CopterStabilizerConfig>,
        M: Mixer,
        T: TelemetryPublisher,
    {
        if !self.is_running() {
            return None;
        }
        if self.phase == Phase::Finishing {
            self.phase = Phase::Complete;
            return Some(AutotuneEvent::Complete);
        }
        if !flight_loop.armed() {
            flight_loop.stop_tuning();
            if let Some(baseline) = self.baseline {
                restore(flight_loop, baseline);
            }
            self.phase = Phase::Reverted;
            return Some(AutotuneEvent::Reverted);
        }
        if let Some(reason) = self
            .envelope
            .breach(flight_loop.state(), self.start_altitude_m)
        {
            return Some(self.fail(flight_loop, reason));
        }

        match self.phase {
            Phase::Settling {
                stage,
                level_s,
                waited_s,
            } => {
                let waited_s = waited_s + dt;
                let level_s = if self.envelope.settled(flight_loop.state()) {
                    level_s + dt
                } else {
                    0.0
                };
                if level_s >= self.envelope.settle_s {
                    let plan = self.stage(stage)?;
                    flight_loop.start_tuning(plan.target, plan.relay);
                    self.phase = Phase::Running { stage };
                    return Some(AutotuneEvent::StageStarted(plan.target));
                }
                if waited_s >= self.envelope.settle_timeout_s {
                    return Some(self.fail(flight_loop, AutotuneAbort::NotSettled));
                }
                self.phase = Phase::Settling {
                    stage,
                    level_s,
                    waited_s,
                };
                None
            }
            Phase::Running { stage } => match flight_loop.tuning() {
                Some((_, TuningStatus::Running { .. })) => None,
                Some((_, TuningStatus::Done(result))) => {
                    Some(self.commit(flight_loop, stage, result))
                }
                Some((_, TuningStatus::Aborted(abort))) => {
                    Some(self.fail(flight_loop, AutotuneAbort::Relay(abort)))
                }
                Some((_, TuningStatus::Interrupted)) | None => {
                    Some(self.fail(flight_loop, AutotuneAbort::Interrupted))
                }
            },
            Phase::Idle
            | Phase::Finishing
            | Phase::Complete
            | Phase::Failed { .. }
            | Phase::Reverted => None,
        }
    }

    /// Where the run stands.
    pub fn status(&self) -> AutotuneStatus {
        match self.phase {
            Phase::Idle => AutotuneStatus::Idle,
            Phase::Settling { stage, .. } => AutotuneStatus::Settling { stage },
            Phase::Running { stage } => AutotuneStatus::Running {
                stage,
                target: self.target(stage),
            },
            Phase::Finishing | Phase::Complete => AutotuneStatus::Complete,
            Phase::Failed { stage, reason } => AutotuneStatus::Failed { stage, reason },
            Phase::Reverted => AutotuneStatus::Reverted,
        }
    }

    /// Whether a run is in progress (started, not yet complete, failed, or reverted).
    pub fn is_running(&self) -> bool {
        matches!(
            self.phase,
            Phase::Settling { .. } | Phase::Running { .. } | Phase::Finishing
        )
    }

    fn settling(stage: usize) -> Phase {
        Phase::Settling {
            stage,
            level_s: 0.0,
            waited_s: 0.0,
        }
    }

    fn stage(&self, stage: usize) -> Option<StagePlan> {
        self.plan.get(stage).copied().flatten()
    }

    /// The target of `stage`; `stage` is always in the plan (the phases only hold indices
    /// below `stages`).
    fn target(&self, stage: usize) -> TunedLoop {
        self.stage(stage)
            .map_or(TunedLoop::RollRate, |plan| plan.target)
    }

    /// Commits a finished stage's gains and moves on.
    fn commit<C, V, G, S, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
        result: RelayResult,
    ) -> AutotuneEvent
    where
        C: Commander,
        V: Vehicle,
        G: Guidance,
        S: Stabilizer + Tunable<Target = TunedLoop> + Configurable<Config = CopterStabilizerConfig>,
        M: Mixer,
        T: TelemetryPublisher,
    {
        let Some(plan) = self.stage(stage) else {
            return self.fail(flight_loop, AutotuneAbort::Interrupted);
        };
        let gains = gains_from_ultimate(&result.point, plan.rule, plan.form);
        let mut config = *flight_loop.stabilizer().config();
        *plan.target.gains_mut(&mut config) = gains;
        flight_loop.stop_tuning();
        if flight_loop.set_stabilizer_config(config).is_err() {
            return self.fail(flight_loop, AutotuneAbort::Rejected);
        }
        self.phase = if stage + 1 < self.stages {
            Self::settling(stage + 1)
        } else {
            Phase::Finishing
        };
        AutotuneEvent::StageDone {
            target: plan.target,
            result,
            gains,
        }
    }

    /// Ends the run at the current stage, stopping it if it runs. A stage commits only when it
    /// finishes, so a failed one leaves the config as it found it.
    fn fail<C, V, G, S, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        reason: AutotuneAbort,
    ) -> AutotuneEvent
    where
        C: Commander,
        V: Vehicle,
        G: Guidance,
        S: Stabilizer + Tunable<Target = TunedLoop> + Configurable<Config = CopterStabilizerConfig>,
        M: Mixer,
        T: TelemetryPublisher,
    {
        let stage = match self.phase {
            Phase::Settling { stage, .. } => stage,
            Phase::Running { stage } => {
                flight_loop.stop_tuning();
                stage
            }
            _ => 0,
        };
        self.phase = Phase::Failed { stage, reason };
        AutotuneEvent::StageFailed {
            target: self.target(stage),
            reason,
        }
    }
}

/// Puts back a config the stabilizer flew before. It was valid then and validation doesn't
/// change between calls, so it can't be rejected.
fn restore<C, V, G, S, M, T>(
    flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
    config: CopterStabilizerConfig,
) where
    C: Commander,
    V: Vehicle,
    G: Guidance,
    S: Stabilizer + Configurable<Config = CopterStabilizerConfig>,
    M: Mixer,
    T: TelemetryPublisher,
{
    let _ = flight_loop.set_stabilizer_config(config);
}
