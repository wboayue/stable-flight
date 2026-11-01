//! The loops a [`Supervisor`](super::Supervisor) tunes ([`Side`]: a copter's or a plane's),
//! what it reads from the vehicle each update ([`Motion`]), and what a stage's procedure
//! decides on ([`Decide`]).

use core::fmt;

use super::procedure::{Action, Input, Procedure, Verdict};
use super::{AutotuneAbort, Envelope, Method, Settle, YAW_AXIS, tilt_axis};
use crate::controllers::gyro::GyroFilter;
use crate::controllers::{Gains, InvalidConfig};
use crate::math::euler_angles;
use crate::stabilizers::copter::{CopterStabilizerConfig, TunedLoop};
use crate::tuning::{Measurement, ProbeAbort, ProbeConfig, Tunable};
use crate::{Configurable, Setpoint, Vec2, Vec3, VehicleState};
use nalgebra::UnitQuaternion;

/// One set of loops the [`Supervisor`](super::Supervisor) tunes, and how: a copter's
/// ([`CopterSide`]). The supervisor's phases (settle, probe, decide, commit, revert) are the
/// same on every side; a side names its loops, gains, config, methods, and envelope, and
/// supplies the procedures that decide what each stage flies. Sealed: the sides are core's.
pub trait Side: sealed::Sealed + Copy + fmt::Debug + PartialEq {
    /// The loops a stage tunes, and the stabilizer hosts probes on ([`Tunable`]).
    type Target: Copy + fmt::Debug + PartialEq;
    /// One target's gains.
    type Gains: Copy + fmt::Debug + PartialEq;
    /// The config of the stabilizer flying this side ([`SideConfig`]).
    type Config: Copy + fmt::Debug + PartialEq;
    /// How a stage tunes its target.
    type Method: Copy + fmt::Debug + PartialEq;
    /// The limits that end a stage, and the settle before each probe.
    type Envelope: Copy + fmt::Debug + PartialEq;
    /// A stage's method as a pure state machine (core's own).
    #[doc(hidden)]
    type Procedure: Copy + fmt::Debug + PartialEq;

    /// `target`'s gains in `config`.
    #[doc(hidden)]
    fn gains(target: Self::Target, config: &Self::Config) -> Self::Gains;
    /// Sets `target`'s gains in `config`.
    #[doc(hidden)]
    fn set_gains(target: Self::Target, config: &mut Self::Config, gains: Self::Gains);
    /// The loop `probe` runs on for a stage on `target`.
    #[doc(hidden)]
    fn probed_loop(target: Self::Target, probe: &ProbeConfig) -> Self::Target;

    /// Checks `method`, its error in the plan section it reports.
    #[doc(hidden)]
    fn validate_method(method: &Self::Method) -> Result<(), InvalidConfig>;
    /// The settle `method` asks for before each probe; `None` uses the envelope's.
    #[doc(hidden)]
    fn method_settle(method: &Self::Method) -> Option<Settle>;
    /// A fresh procedure for `method`, at a stage's start.
    #[doc(hidden)]
    fn procedure(method: Self::Method) -> Self::Procedure;
    /// What `procedure` does next on `on`, for the stage `stage` describes.
    #[doc(hidden)]
    fn decide(
        procedure: &mut Self::Procedure,
        on: Decide,
        stage: StageView<'_, Self>,
    ) -> Action<Self::Gains>;
    /// A search's verdict on the last decision, if any; taking it clears it.
    #[doc(hidden)]
    fn take_verdict(procedure: &mut Self::Procedure) -> Option<Verdict<Self::Gains>>;

    /// Checks `envelope`.
    #[doc(hidden)]
    fn validate_envelope(envelope: &Self::Envelope) -> Result<(), InvalidConfig>;
    /// Checks that `settle` is tighter than `envelope`'s limits.
    #[doc(hidden)]
    fn check_settle(envelope: &Self::Envelope, settle: &Settle) -> Result<(), InvalidConfig>;
    /// The settle and times `envelope` gives every side's phases.
    #[doc(hidden)]
    fn waits(envelope: &Self::Envelope) -> Waits;
    /// The limit `motion` breaks, if any, `start_altitude_m` the run's start.
    #[doc(hidden)]
    fn breach(
        envelope: &Self::Envelope,
        motion: &Motion,
        start_altitude_m: f32,
    ) -> Option<AutotuneAbort>;

    /// The body rates the settle reads, `dt` on: through the stabilizer's gyro filters, or raw.
    #[doc(hidden)]
    fn filter_rates(
        filters: &mut RateFilters,
        config: &Self::Config,
        rates_radps: Vec3,
        dt: f32,
    ) -> Vec3;
    /// The roll and pitch `setpoint` holds on this side, in rad.
    #[doc(hidden)]
    fn attitude_target(setpoint: &Setpoint) -> Vec2;
    /// Whether `motion` holds `settle`, within `envelope`, for a probe on `target` to start.
    #[doc(hidden)]
    fn settles(
        envelope: &Self::Envelope,
        settle: &Settle,
        motion: &Motion,
        target: Self::Target,
    ) -> bool;
    /// Whether `motion` holds `settle` about `target`'s axis on candidate gains.
    #[doc(hidden)]
    fn holds(settle: &Settle, motion: &Motion, target: Self::Target) -> bool;
}

mod sealed {
    pub trait Sealed {}
    impl Sealed for super::CopterSide {}
    impl Sealed for super::super::PlaneSide {}
}

/// A stabilizer config that holds a [`Side`]'s config: the copter's is its own.
pub trait SideConfig<K: Side> {
    /// The side's config.
    fn side(&self) -> &K::Config;
    /// The side's config, to change it.
    fn side_mut(&mut self) -> &mut K::Config;
}

/// What the [`Supervisor`](super::Supervisor) needs from a stabilizer to tune side `K`: probes
/// on its loops, and a config holding the side's.
pub trait SideTunable<K: Side>: Tunable<K::Target> + Configurable<Config: SideConfig<K>> {}

impl<K: Side, S> SideTunable<K> for S where
    S: Tunable<K::Target> + Configurable<Config: SideConfig<K>>
{
}

/// What every side's envelope gives the phases: the settle before a probe and how long the
/// phases wait. See [`Envelope`] for each.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Waits {
    pub(super) settle: Settle,
    pub(super) settle_timeout_s: f32,
    pub(super) recovery_s: f32,
    pub(super) max_stale_s: f32,
}

/// One gyro filter per body axis (roll, pitch, yaw), for the settle's rates.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RateFilters(pub(super) [GyroFilter; 3]);

/// What a procedure deciding for a stage may read: its target, the side's config as flown,
/// and as it was at the run's start.
pub struct StageView<'a, K: Side> {
    pub(super) target: K::Target,
    pub(super) config: &'a K::Config,
    pub(super) run_start: Option<&'a K::Config>,
}

/// A copter's loops ([`TunedLoop`]), tuned on a [`CopterStabilizerConfig`] by a [`Method`]
/// within an [`Envelope`]: the [`Autotune`](super::Autotune).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CopterSide;

impl SideConfig<CopterSide> for CopterStabilizerConfig {
    fn side(&self) -> &CopterStabilizerConfig {
        self
    }

    fn side_mut(&mut self) -> &mut CopterStabilizerConfig {
        self
    }
}

/// A copter [`Procedure`], behind a type the [`Side`] trait can name.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CopterProcedure(Procedure);

impl Side for CopterSide {
    type Target = TunedLoop;
    type Gains = Gains;
    type Config = CopterStabilizerConfig;
    type Method = Method;
    type Envelope = Envelope;
    type Procedure = CopterProcedure;

    fn gains(target: TunedLoop, config: &CopterStabilizerConfig) -> Gains {
        target.gains(config)
    }

    fn set_gains(target: TunedLoop, config: &mut CopterStabilizerConfig, gains: Gains) {
        *target.gains_mut(config) = gains;
    }

    /// An identification excites an angle loop's axis on the rate loop under it
    /// ([`DesignPlan`](super::DesignPlan)); every other probe runs on `target` itself.
    fn probed_loop(target: TunedLoop, probe: &ProbeConfig) -> TunedLoop {
        match probe {
            ProbeConfig::Identify(_) | ProbeConfig::IdentifyWing(_) => {
                target.inner().unwrap_or(target)
            }
            ProbeConfig::Step(_) => target,
        }
    }

    fn validate_method(method: &Method) -> Result<(), InvalidConfig> {
        method
            .validate()
            .map_err(|e| e.in_section(method.plan_section()))
    }

    fn method_settle(method: &Method) -> Option<Settle> {
        method.settle()
    }

    fn procedure(method: Method) -> CopterProcedure {
        CopterProcedure(Procedure::new(method))
    }

    fn decide(procedure: &mut CopterProcedure, on: Decide, stage: StageView<'_, Self>) -> Action {
        let StageView {
            target,
            config,
            run_start,
        } = stage;
        let gains = target.gains(config);
        let input = match on {
            // No copter method computes over updates.
            Decide::Computing => return Action::Fail(AutotuneAbort::Interrupted),
            Decide::Start => Input::Start {
                gains,
                inner: target.inner().map(|inner| inner.gains(config)),
                run_start: run_start.map_or(gains, |run_start| target.gains(run_start)),
                // The stage's own axis: yaw flies no notch.
                gyro: config.gyro_filters_on(tilt_axis(target).unwrap_or(YAW_AXIS)),
            },
            Decide::Measured {
                measurement,
                stalled,
            } => Input::Measured {
                measurement,
                gains,
                stalled,
            },
            Decide::Stalled => Input::Stalled { gains },
            Decide::Aborted { abort } => Input::Aborted { abort, gains },
            Decide::Breached { reason } => Input::Breached { reason, gains },
            Decide::Held { settled } => Input::Held { settled, gains },
            Decide::Unsettled => Input::Unsettled { gains },
        };
        procedure.0.next(input)
    }

    fn take_verdict(procedure: &mut CopterProcedure) -> Option<Verdict> {
        procedure.0.take_verdict()
    }

    fn validate_envelope(envelope: &Envelope) -> Result<(), InvalidConfig> {
        envelope.validate()
    }

    fn check_settle(envelope: &Envelope, settle: &Settle) -> Result<(), InvalidConfig> {
        envelope.check_settle(settle)
    }

    fn waits(envelope: &Envelope) -> Waits {
        Waits {
            settle: envelope.settle,
            settle_timeout_s: envelope.settle_timeout_s,
            recovery_s: envelope.recovery_s,
            max_stale_s: envelope.max_stale_s,
        }
    }

    fn breach(
        envelope: &Envelope,
        motion: &Motion,
        start_altitude_m: f32,
    ) -> Option<AutotuneAbort> {
        envelope.breach(motion, start_altitude_m)
    }

    /// Through the stabilizer's gyro filters on each axis, at their configured cutoffs.
    fn filter_rates(
        filters: &mut RateFilters,
        config: &CopterStabilizerConfig,
        rates_radps: Vec3,
        dt: f32,
    ) -> Vec3 {
        Vec3::from_fn(|axis, _| {
            let gyro = config.gyro_filters_on(axis);
            filters.0[axis].apply(rates_radps[axis], dt, &gyro)
        })
    }

    fn attitude_target(setpoint: &Setpoint) -> Vec2 {
        attitude_target(setpoint)
    }

    fn settles(_envelope: &Envelope, settle: &Settle, motion: &Motion, target: TunedLoop) -> bool {
        settle.holds_for(motion, target)
    }

    /// For a roll or pitch loop only its axis counts: the other may still ring on its start
    /// gains (pitch on `racer`'s high start with 25 ms of delay, #518). A yaw rate or heading
    /// loop needs both level and yaw still.
    fn holds(settle: &Settle, motion: &Motion, target: TunedLoop) -> bool {
        tilt_axis(target).map_or_else(
            || settle.holds_yaw(motion),
            |axis| settle.holds_axis(motion, axis),
        )
    }
}

/// What the envelope and the settle check, read once per update from the state. The envelope
/// reads tilt from level and the settle from the attitude target, so a hover leaning to hold
/// its position settles where it leans. The settle's rates are filtered through the
/// stabilizer's gyro low-pass, the rates its loops fly on: read raw, a still vehicle's gyro
/// noise and vibration hold them over the settle (0.2 rad/s on the sim's noise rows, the
/// settle 0.1), so no stage starts (#373). The envelope reads the raw rate, so a breach shows
/// on its first tick. Angles and rates are in the probed loops' frame
/// ([`Tunable::probe_frame`]): view axes for a tailsitter's hover, where nose-up reads level.
pub struct Motion {
    /// The larger of |roll| and |pitch|, from level, in rad.
    pub(super) tilt_rad: f32,
    /// |roll| from level, in rad.
    pub(super) bank_rad: f32,
    /// |pitch| from level, in rad.
    pub(super) pitch_rad: f32,
    /// The largest raw body rate magnitude, in rad/s.
    pub(super) rate_radps: f32,
    /// The |roll| and |pitch| errors from the attitude target, in rad.
    pub(super) axis_tilt_error_rad: [f32; 2],
    /// The horizontal ground speed, in m/s; 0 for a NaN velocity.
    pub(super) speed_mps: f32,
    /// The filtered |roll| and |pitch| rates, in rad/s.
    pub(super) axis_rate_radps: [f32; 2],
    /// The filtered |yaw| rate, in rad/s.
    pub(super) yaw_rate_radps: f32,
    pub(super) altitude_m: f32,
    /// The airspeed, in m/s, when the state has a valid one.
    pub(super) airspeed_mps: Option<f32>,
    /// How fast the airspeed changed since the last update, in m/s²; 0 without an airspeed
    /// then and now. The supervisor sets it: a single state has no change.
    pub(super) airspeed_change_mps2: f32,
}

impl Motion {
    /// `state`'s motion in `frame` (the probed loops' frame, in the body frame), with
    /// `filtered_radps` its body rates through the gyro low-pass and `target_rad` the roll
    /// and pitch the loops hold ([`attitude_target`]), in that frame.
    pub(super) fn of(
        state: &VehicleState,
        frame: &UnitQuaternion<f32>,
        filtered_radps: Vec3,
        target_rad: Vec2,
    ) -> Self {
        // The view as `VehicleState::rotate_body` builds it, without cloning the state.
        let (roll, pitch, _) = euler_angles(&(state.attitude * frame));
        let raw_radps = frame.inverse_transform_vector(&state.angular_velocity_radps);
        let filtered_radps = frame.inverse_transform_vector(&filtered_radps);
        let roll_error_rad = (roll - target_rad.x).abs();
        let pitch_error_rad = (pitch - target_rad.y).abs();
        Self {
            tilt_rad: roll.abs().max(pitch.abs()),
            bank_rad: roll.abs(),
            pitch_rad: pitch.abs(),
            rate_radps: raw_radps.amax(),
            axis_tilt_error_rad: [roll_error_rad, pitch_error_rad],
            speed_mps: zero_if_nan(state.velocity_mps.xy().norm()),
            axis_rate_radps: [filtered_radps.x.abs(), filtered_radps.y.abs()],
            yaw_rate_radps: filtered_radps.z.abs(),
            altitude_m: state.position_m.z,
            airspeed_mps: state.valid_airspeed_mps(),
            airspeed_change_mps2: 0.0,
        }
    }
}

/// The roll and pitch `setpoint` holds, in rad, in the stabilizer's frame: an attitude
/// setpoint's (a NaN axis is level, as the stabilizer reads it), a blend's hover side's, and
/// level for a rate or plane setpoint, which holds no hover attitude. Guidance's target,
/// before the stabilizer caps it at its maximum tilt: a target past the cap never settles, and
/// the stage times out.
pub(super) fn attitude_target(setpoint: &Setpoint) -> Vec2 {
    let attitude = match setpoint {
        Setpoint::Attitude(attitude) => attitude,
        Setpoint::Blend(blend) => &blend.hover,
        _ => return Vec2::zeros(),
    };
    Vec2::new(
        zero_if_nan(attitude.roll_rad),
        zero_if_nan(attitude.pitch_rad),
    )
}

/// `value`, or 0 for NaN: the neutral reading of a missing angle or speed.
pub(super) fn zero_if_nan(value: f32) -> f32 {
    if value.is_nan() { 0.0 } else { value }
}

/// What a stage's procedure decides on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Decide {
    /// The stage's start.
    Start,
    /// A finished probe's measurement, and whether the host stalled during it.
    Measured {
        measurement: Measurement,
        stalled: bool,
    },
    /// A probe stopped because the host turned slow, with no measurement.
    Stalled,
    /// A probe that gave up.
    Aborted { abort: ProbeAbort },
    /// A probe or a hold on candidate gains that took the vehicle past the envelope.
    Breached { reason: AutotuneAbort },
    /// A hold ended, `settled` on its candidate or timed out.
    Held { settled: bool },
    /// The settle before a probe timed out on the committed gains.
    Unsettled,
    /// An update after the procedure asked to compute ([`Action::Compute`]).
    Computing,
}
