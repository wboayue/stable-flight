//! The copter autotune supervisor: see [`Autotune`].

use core::fmt;

use crate::controllers::{Gains, InvalidConfig, check, check_positive};
use crate::math::euler_angles;
use crate::stabilizers::copter::{CopterStabilizerConfig, TunedLoop};
use crate::{Configurable, FlightLoop, VehicleState};

use super::{
    ControllerForm, Measurement, ProbeAbort, ProbeConfig, ProbeStatus, RelayConfig, RelayResult,
    Tunable, TuningRule, gains_from_ultimate,
};

use procedure::{Action, Input, Procedure};

mod procedure;
#[cfg(test)]
mod tests;

/// The most stages a plan may hold: one per [`TunedLoop`].
pub const MAX_STAGES: usize = 6;

/// Relay for a copter rate loop: normalized moment out, rad/s in. RealFlight's QuadCopterX
/// cycles in about 70 ms (17 ticks) with cycle-to-cycle jitter, so it averages 8 cycles
/// within 25% and moves the bias a fifth of the way per cycle.
pub const COPTER_RATE_RELAY: RelayConfig = RelayConfig {
    initial_relay: 0.05,
    max_relay: 0.4,
    relay_step: 1.5,
    hysteresis: 0.005,
    min_cycle_amplitude: 0.1,
    max_cycle_amplitude: 2.0,
    max_half_cycle_s: 0.3,
    max_bias: 0.5,
    bias_gain: 0.2,
    cycles: 8,
    tolerance: 0.25,
    timeout_s: 20.0,
};

/// Relay for a copter angle or heading loop: rad/s out, rad in; averaging as
/// [`COPTER_RATE_RELAY`]. The relay stays small (at most 0.6 rad/s) so the rate loop under it
/// doesn't saturate its moment: a saturated inner loop doesn't grow its cycle with the relay,
/// which reads `Ku` high (on QuadCopterX a 1.5 rad/s relay gave angle P 15, which oscillated).
/// Cycles of half a degree count.
pub const COPTER_ANGLE_RELAY: RelayConfig = RelayConfig {
    initial_relay: 0.1,
    max_relay: 0.6,
    relay_step: 1.5,
    hysteresis: 0.0005,
    min_cycle_amplitude: 0.01,
    max_cycle_amplitude: 0.35,
    max_half_cycle_s: 1.0,
    max_bias: 0.3,
    bias_gain: 0.2,
    cycles: 8,
    tolerance: 0.25,
    timeout_s: 30.0,
};

/// The loops [`StagePlan::copter_sequence`] tunes: roll and pitch rate, the loops a bad start
/// breaks. Flights on QuadCopterX (#295) left the rest out:
///
/// - Yaw rate: the relay can't measure it within the yaw authority the mixer leaves at hover
///   (#296).
/// - Roll and pitch angle: the relay measures a high ultimate gain (about 60: an integrator
///   around a fast rate loop), and the rule's P oscillates once a few degrees of error saturate
///   the rate setpoint and the moments. Step-response tuning (phase 2) sees the saturation.
/// - Heading: both of the above.
///
/// Their stages still run when planned explicitly (`StagePlan::copter`).
pub const COPTER_SEQUENCE: [TunedLoop; 2] = [TunedLoop::RollRate, TunedLoop::PitchRate];

/// One stage of an autotune: which loop, and the method that tunes it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StagePlan {
    /// The loop to tune.
    pub target: TunedLoop,
    /// How to tune it.
    pub method: Method,
}

impl StagePlan {
    /// The copter defaults for `target`: relay feedback, [`RelayPlan::copter`].
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    /// use stable_flight::tuning::autotune::{Method, RelayPlan, StagePlan};
    ///
    /// let plan = StagePlan::copter(TunedLoop::RollRate);
    /// assert_eq!(plan.method, Method::Relay(RelayPlan::copter(TunedLoop::RollRate)));
    /// ```
    pub fn copter(target: TunedLoop) -> Self {
        Self {
            target,
            method: Method::Relay(RelayPlan::copter(target)),
        }
    }

    /// The loops in [`COPTER_SEQUENCE`], in order, with the [`copter`](Self::copter) defaults.
    pub fn copter_sequence() -> [Self; COPTER_SEQUENCE.len()] {
        COPTER_SEQUENCE.map(Self::copter)
    }
}

/// How a stage tunes its loop: which probes it runs and what gains follow from them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Method {
    /// One relay measurement, turned into gains by a tuning rule.
    Relay(RelayPlan),
}

impl Method {
    /// Checks the method's probe configs.
    ///
    /// # Errors
    ///
    /// When the relay is invalid ([`RelayConfig::validate`]).
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        match self {
            Self::Relay(plan) => plan.relay.validate(),
        }
    }

    /// The section [`Autotune::new`] reports this method's errors in.
    fn plan_section(&self) -> &'static str {
        match self {
            Self::Relay(_) => "plan.relay",
        }
    }
}

/// Relay feedback: measure the loop's ultimate point, then set the gains for the terms `form`
/// tunes by `rule`; the other terms keep their current gains.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RelayPlan {
    /// The relay that measures the loop.
    pub relay: RelayConfig,
    /// The tuning rule for its gains.
    pub rule: TuningRule,
    /// Which terms its gains set.
    pub form: ControllerForm,
}

impl RelayPlan {
    /// The copter defaults for `target`: Tyreus–Luyben PD throughout, so the integral keeps its
    /// current gain. A rate loop's I holds trim against steady disturbances rather than
    /// shaping its response, and the rule's integral time (`2.2·Tu`) is a fraction of a second
    /// on these fast loops: about 30× a flown hand tune on QuadCopterX. Rate loops use
    /// [`COPTER_RATE_RELAY`], angle and heading loops [`COPTER_ANGLE_RELAY`].
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::stabilizers::copter::TunedLoop;
    /// use stable_flight::tuning::ControllerForm;
    /// use stable_flight::tuning::autotune::RelayPlan;
    ///
    /// assert_eq!(RelayPlan::copter(TunedLoop::RollRate).form, ControllerForm::Pd);
    /// ```
    pub fn copter(target: TunedLoop) -> Self {
        let relay = match target {
            TunedLoop::RollRate | TunedLoop::PitchRate | TunedLoop::YawRate => COPTER_RATE_RELAY,
            TunedLoop::Roll | TunedLoop::Pitch | TunedLoop::Heading => COPTER_ANGLE_RELAY,
        };
        Self {
            relay,
            rule: TuningRule::TyreusLuyben,
            form: ControllerForm::Pd,
        }
    }

    /// The gains a measurement sets: the rule's for the terms the form tunes, `current` for
    /// the rest.
    fn gains(&self, result: &RelayResult, current: Gains) -> Gains {
        let tuned = gains_from_ultimate(&result.point, self.rule, self.form);
        Gains {
            p: tuned.p,
            i: if self.form.has_integral() {
                tuned.i
            } else {
                current.i
            },
            d: if self.form.has_derivative() {
                tuned.d
            } else {
                current.d
            },
        }
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

    /// The limit `motion` breaks, if any. NaN never counts as a breach.
    fn breach(&self, motion: &Motion, start_altitude_m: f32) -> Option<AutotuneAbort> {
        if motion.tilt_rad > self.max_tilt_rad {
            Some(AutotuneAbort::Tilt)
        } else if motion.rate_radps > self.max_rate_radps {
            Some(AutotuneAbort::Rate)
        } else if start_altitude_m - motion.altitude_m > self.max_altitude_loss_m {
            Some(AutotuneAbort::AltitudeLoss)
        } else {
            None
        }
    }

    /// Whether `motion` is level and still enough to start a stage.
    fn settled(&self, motion: &Motion) -> bool {
        motion.tilt_rad < self.settle_tilt_rad && motion.rate_radps < self.settle_rate_radps
    }
}

/// What the envelope checks, read once per update from the state.
struct Motion {
    /// The larger of |roll| and |pitch|, in rad.
    tilt_rad: f32,
    /// The largest body rate magnitude, in rad/s.
    rate_radps: f32,
    altitude_m: f32,
}

impl Motion {
    fn of(state: &VehicleState) -> Self {
        let (roll, pitch, _) = euler_angles(&state.attitude);
        Self {
            tilt_rad: roll.abs().max(pitch.abs()),
            rate_radps: state.angular_velocity_radps.amax(),
            altitude_m: state.position_m.z,
        }
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
    /// The probe gave up.
    Probe(ProbeAbort),
    /// The stabilizer idled the loop under test (e.g. a rate setpoint), or the probe was
    /// stopped from outside.
    Interrupted,
    /// The stabilizer rejected the tuned config.
    Rejected,
}

/// What an [`Autotune::update`] did, for the caller to log.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AutotuneEvent {
    /// A stage started: it settles, then probes its loop.
    StageStarted(TunedLoop),
    /// A probe on the stage's loop finished.
    Measured {
        /// The probed loop.
        target: TunedLoop,
        /// What the probe measured.
        measurement: Measurement,
    },
    /// A stage finished and committed its loop's gains.
    StageDone {
        /// The tuned loop.
        target: TunedLoop,
        /// The gains committed for it.
        gains: Gains,
    },
    /// A stage failed: the config went back to the last committed one (earlier stages stay),
    /// and the run ends.
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
    /// Stage `stage` (an index into the plan) is waiting for the vehicle to settle before its
    /// next probe.
    Settling {
        /// The stage.
        stage: usize,
    },
    /// Stage `stage` is probing `target`, or deciding on what a probe measured.
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
    /// Stage `stage` starts on the next update.
    Starting {
        stage: usize,
    },
    /// The stage's procedure decides what follows `measurement` (`None`: the stage's start).
    Deciding {
        stage: usize,
        measurement: Option<Measurement>,
    },
    /// Waiting for the vehicle to settle before running `probe`.
    Settling {
        stage: usize,
        probe: ProbeConfig,
        level_s: f32,
        waited_s: f32,
    },
    Probing {
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

/// Runs a copter autotune between ticks (#149, #298): each [`StagePlan`]'s procedure decides
/// which probes to run on its loop ([`Tunable::start_probe`]) and what gains follow from
/// them; the supervisor executes it.
///
/// - **Start** ([`start`](Self::start)) when armed and hovering; the caller owns the trigger.
///   The config and altitude at start are the baseline.
/// - **Settle:** before each probe, the vehicle must stay within the [`Envelope`]'s settle
///   limits for `settle_s`.
/// - **Probe and decide:** a finished probe's [`Measurement`] goes to the stage's procedure,
///   which asks for another probe or finishes with gains for its loop, committed through
///   [`set_stabilizer_config`](FlightLoop::set_stabilizer_config); the next stage flies them.
///   With [`Method::Relay`], one relay measurement and its rule's gains.
/// - **Fail:** an envelope breach, a probe abort, an interrupt, or a rejected config stops the
///   stage, puts back the last committed config (the earlier stages' gains stay), and ends the
///   run.
/// - **Disarm** before completion reverts every stage to the baseline.
/// - **Complete:** the gains stay; read them from the stabilizer's config.
///
/// The procedures are pure (they see measurements and gains, never the loop), so the
/// supervisor alone flies, guards, and commits. [`update`](Self::update) returns an
/// [`AutotuneEvent`] when something happens, for the caller to log; the supervisor has no
/// clock (the caller passes `dt`) and does no I/O.
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
    /// The current stage's procedure.
    procedure: Procedure,
    /// The config at start: what a disarm reverts to.
    baseline: Option<CopterStabilizerConfig>,
    /// The config as of the last commit: what a failure reverts to.
    last_good: Option<CopterStabilizerConfig>,
    start_altitude_m: f32,
}

/// What the supervisor needs from the stabilizer: copter probes and config.
pub trait CopterTunable:
    Tunable<Target = TunedLoop> + Configurable<Config = CopterStabilizerConfig>
{
}

impl<S> CopterTunable for S where
    S: Tunable<Target = TunedLoop> + Configurable<Config = CopterStabilizerConfig>
{
}

impl Autotune {
    /// A supervisor for `plan` (1 to [`MAX_STAGES`] stages) within `envelope`.
    ///
    /// # Errors
    ///
    /// When the plan is empty or too long (`plan`), a stage's method is invalid
    /// (`plan.<method>`, e.g. `plan.relay`; [`Method::validate`]), or the envelope is
    /// (`envelope`).
    pub fn new(plan: &[StagePlan], envelope: Envelope) -> Result<Self, InvalidConfig> {
        check(
            "stages",
            (1..=MAX_STAGES).contains(&plan.len()),
            "1 to MAX_STAGES",
        )
        .map_err(|e| e.in_section("plan"))?;
        let Some(first) = plan.first() else {
            unreachable!("the plan was checked non-empty above");
        };
        for stage in plan {
            stage
                .method
                .validate()
                .map_err(|e| e.in_section(stage.method.plan_section()))?;
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
            procedure: Procedure::new(first.method),
            baseline: None,
            last_good: None,
            start_altitude_m: 0.0,
        })
    }

    /// Starts a run: records the stabilizer's config and the altitude as the baseline; the
    /// first stage starts on the next update. A finished run can be started again.
    ///
    /// # Errors
    ///
    /// [`StartError::NotArmed`] unless the last tick was armed; [`StartError::Running`]
    /// while a run is in progress.
    pub fn start<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &FlightLoop<C, V, G, S, M, T>,
    ) -> Result<(), StartError> {
        if self.is_running() {
            return Err(StartError::Running);
        }
        if !flight_loop.armed() {
            return Err(StartError::NotArmed);
        }
        let config = *flight_loop.stabilizer().config();
        self.baseline = Some(config);
        self.last_good = Some(config);
        self.start_altitude_m = flight_loop.state().position_m.z;
        self.phase = Phase::Starting { stage: 0 };
        Ok(())
    }

    /// Advances the run by `dt` seconds; call it between ticks (after
    /// [`step`](FlightLoop::step)). Returns what happened, if anything: at most one event per
    /// call.
    ///
    /// In order: a finished run reports [`Complete`](AutotuneEvent::Complete); a disarm
    /// reverts everything; an envelope breach fails the stage; then the run starts a stage,
    /// lets its procedure decide, settles, or polls the running probe.
    pub fn update<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        dt: f32,
    ) -> Option<AutotuneEvent> {
        if !self.is_running() {
            return None;
        }
        if self.phase == Phase::Finishing {
            self.phase = Phase::Complete;
            return Some(AutotuneEvent::Complete);
        }
        if !flight_loop.armed() {
            flight_loop.stop_probe();
            if let Some(baseline) = self.baseline {
                restore(flight_loop, baseline);
            }
            self.phase = Phase::Reverted;
            return Some(AutotuneEvent::Reverted);
        }
        let motion = Motion::of(flight_loop.state());
        if let Some(reason) = self.envelope.breach(&motion, self.start_altitude_m) {
            return Some(self.fail(flight_loop, reason));
        }
        match self.phase {
            Phase::Starting { stage } => self.begin(stage),
            Phase::Deciding { stage, measurement } => self.decide(flight_loop, stage, measurement),
            Phase::Settling {
                stage,
                probe,
                level_s,
                waited_s,
            } => self.settle(
                flight_loop,
                &motion,
                (stage, probe),
                level_s + dt,
                waited_s + dt,
            ),
            Phase::Probing { stage } => self.poll_probe(flight_loop, stage),
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
            Phase::Starting { stage }
            | Phase::Settling { stage, .. }
            | Phase::Deciding {
                stage,
                measurement: None,
            } => AutotuneStatus::Settling { stage },
            Phase::Probing { stage }
            | Phase::Deciding {
                stage,
                measurement: Some(_),
            } => AutotuneStatus::Running {
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
        !matches!(
            self.phase,
            Phase::Idle | Phase::Complete | Phase::Failed { .. } | Phase::Reverted
        )
    }

    /// Starts `stage` with a fresh procedure, which decides on the next update.
    fn begin(&mut self, stage: usize) -> Option<AutotuneEvent> {
        let plan = self.stage(stage)?;
        self.procedure = Procedure::new(plan.method);
        self.phase = Phase::Deciding {
            stage,
            measurement: None,
        };
        Some(AutotuneEvent::StageStarted(plan.target))
    }

    /// Hands the procedure the stage's start or its last measurement, with the loop's current
    /// gains, and carries out what it asks for.
    fn decide<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
        measurement: Option<Measurement>,
    ) -> Option<AutotuneEvent> {
        let gains = self.target(stage).gains(flight_loop.stabilizer().config());
        let input = match measurement {
            None => Input::Start { gains },
            Some(measurement) => Input::Measured { measurement, gains },
        };
        match self.procedure.next(input) {
            Action::Probe(probe) => {
                self.phase = Phase::Settling {
                    stage,
                    probe,
                    level_s: 0.0,
                    waited_s: 0.0,
                };
                None
            }
            Action::Finish(gains) => Some(self.commit(flight_loop, stage, gains)),
        }
    }

    /// Waits for the vehicle to hold level and still for `settle_s` (`level_s` counts it,
    /// restarting when it moves), then starts `probe` for `stage`; fails after
    /// `settle_timeout_s`.
    fn settle<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        motion: &Motion,
        (stage, probe): (usize, ProbeConfig),
        level_s: f32,
        waited_s: f32,
    ) -> Option<AutotuneEvent> {
        let level_s = if self.envelope.settled(motion) {
            level_s
        } else {
            0.0
        };
        if level_s >= self.envelope.settle_s {
            flight_loop.start_probe(self.target(stage), probe);
            self.phase = Phase::Probing { stage };
            return None;
        }
        if waited_s >= self.envelope.settle_timeout_s {
            return Some(self.fail(flight_loop, AutotuneAbort::NotSettled));
        }
        self.phase = Phase::Settling {
            stage,
            probe,
            level_s,
            waited_s,
        };
        None
    }

    /// Checks the running probe: a measurement goes to the procedure on the next update; a
    /// stopped probe fails the run.
    fn poll_probe<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
    ) -> Option<AutotuneEvent> {
        let reason = match flight_loop.probe() {
            Some((_, ProbeStatus::Running(_))) => return None,
            Some((target, ProbeStatus::Done(measurement))) => {
                flight_loop.stop_probe();
                self.phase = Phase::Deciding {
                    stage,
                    measurement: Some(measurement),
                };
                return Some(AutotuneEvent::Measured {
                    target,
                    measurement,
                });
            }
            Some((_, ProbeStatus::Aborted(abort))) => AutotuneAbort::Probe(abort),
            Some((_, ProbeStatus::Interrupted)) | None => AutotuneAbort::Interrupted,
        };
        Some(self.fail(flight_loop, reason))
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
    fn commit<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        stage: usize,
        gains: Gains,
    ) -> AutotuneEvent {
        let target = self.target(stage);
        let mut config = *flight_loop.stabilizer().config();
        *target.gains_mut(&mut config) = gains;
        if flight_loop.set_stabilizer_config(config).is_err() {
            return self.fail(flight_loop, AutotuneAbort::Rejected);
        }
        self.last_good = Some(config);
        self.phase = if stage + 1 < self.stages {
            Phase::Starting { stage: stage + 1 }
        } else {
            Phase::Finishing
        };
        AutotuneEvent::StageDone { target, gains }
    }

    /// Ends the run at the current stage: stops its probe and puts back the last committed
    /// config.
    fn fail<C, V, G, S: CopterTunable, M, T>(
        &mut self,
        flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
        reason: AutotuneAbort,
    ) -> AutotuneEvent {
        let stage = match self.phase {
            Phase::Starting { stage }
            | Phase::Deciding { stage, .. }
            | Phase::Settling { stage, .. }
            | Phase::Probing { stage } => stage,
            _ => 0,
        };
        flight_loop.stop_probe();
        if let Some(last_good) = self.last_good {
            restore(flight_loop, last_good);
        }
        self.phase = Phase::Failed { stage, reason };
        AutotuneEvent::StageFailed {
            target: self.target(stage),
            reason,
        }
    }
}

/// Puts back a config the stabilizer flew before. It was valid then and validation doesn't
/// change between calls, so it can't be rejected.
fn restore<C, V, G, S: CopterTunable, M, T>(
    flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
    config: CopterStabilizerConfig,
) {
    let _ = flight_loop.set_stabilizer_config(config);
}
