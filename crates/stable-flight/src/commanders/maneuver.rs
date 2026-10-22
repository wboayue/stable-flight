//! Scripted maneuvers: a [`Maneuver`] is a short list of [`Segment`]s, each a body rate about
//! one axis held until an angle has turned or a time has passed, or a pause, and a
//! [`ManeuverRunner`] flies one from the attitude it starts at, then levels the wings and hands
//! back.
//!
//! | Maneuver | Segments |
//! |---|---|
//! | [`Maneuver::roll`] | Roll 360° at a set rate (negative: left) |
//! | [`Maneuver::loop_`] | Pitch 360° at `V / R`: a loop of radius `R` at speed `V` (negative: outside) |
//! | [`Maneuver::doublet`] | `+rate` then `−rate` about one axis, a half width each, then a [`DOUBLET_PAUSE_S`] pause: a step response in flight |
//!
//! A new maneuver is a new list of segments, up to [`MAX_SEGMENTS`]: an Immelmann is half a
//! loop then half a roll, a split-S the reverse, a hammerhead's pivot a yaw. [`Segment`] is
//! non-exhaustive, so segments that hold an attitude or a flight path (Aresti lines, point-roll
//! hesitations) can join it.
//!
//! The rates go out as [`ControlInput::AbsoluteRate`], open loop, each segment at its
//! [`Thrust`]: rolls and loops at [`ManeuverConfig::thrust`], doublets and pauses at
//! [`ManeuverConfig::cruise_thrust`], so a doublet measures the rate loop as it flies in
//! cruise (on a tailsitter the propwash over the elevons grows its authority with thrust). The
//! recovery is a level [`ControlInput::AbsoluteAttitude`] until the wings stay level and the
//! body rates small ([`RecoveryConfig`]). The shapes are ArduPilot's scripted aerobatics
//! (`libraries/AP_Scripting/applets/Aerobatics/FixedWing/plane_aerobatics.lua` at `3d24375`):
//! its loop is a vertical arc of a given radius (`:1456-1462`), its roll whole turns along a
//! straight (`:1465-1469`). Unlike ArduPilot, the runner flies rates rather than tracking a
//! path, and holds a fixed thrust where ArduPilot holds the speed by throttle (`:54`, `:561`).
//! So a roll holds no height: its lift tips over with it.
//!
//! The angle a segment has turned is read from the attitude, not Euler angles (which clamp at
//! ±90° in a loop): the twist about the segment's axis of the rotation since it began,
//! unwrapped tick to tick. A segment that can't read it (a non-finite attitude) ends at a
//! timeout instead, so nothing stalls a maneuver.

use nalgebra::UnitQuaternion;

use crate::controllers::{InvalidConfig, check, check_positive, check_unit_interval};
use crate::math::{DEG, assert_dt, euler_angles, twist_angle_rad, wrap_angle};
use crate::{AbsoluteAttitudeCommand, AbsoluteRateCommand, ControlInput, VehicleState};

mod segment;
#[cfg(test)]
mod tests;

pub use segment::{
    Axis, DEFAULT_DOUBLET_HALF_S, DOUBLET_PAUSE_S, MAX_SEGMENTS, Maneuver, ManeuverError, RateLaw,
    Segment, SegmentEnd, Thrust,
};
use segment::{UNUSED, positive};

/// How a [`ManeuverRunner`] levels the wings after a maneuver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RecoveryConfig {
    /// The pitch held level, in rad.
    pub pitch_rad: f32,
    /// Thrust while recovering, `[0, 1]`.
    pub thrust: f32,
    /// Bank, and pitch off [`pitch_rad`](Self::pitch_rad), within which the wings count as
    /// level, in rad.
    pub level_band_rad: f32,
    /// Body rate (magnitude) under which the wings count as settled, in rad/s.
    pub max_body_rate_radps: f32,
    /// How long the wings stay level and settled before the recovery ends, in s.
    pub settle_s: f32,
    /// Longest recovery, in s: past it the route takes over, level or not.
    pub timeout_s: f32,
}

impl Default for RecoveryConfig {
    /// Level within 10° and 20°/s for 0.5 s, at full thrust, given up after 10 s: placeholders
    /// until an airframe is tuned.
    fn default() -> Self {
        RecoveryConfig {
            pitch_rad: 0.0,
            thrust: 1.0,
            level_band_rad: 10.0 * DEG,
            max_body_rate_radps: 20.0 * DEG,
            settle_s: 0.5,
            timeout_s: 10.0,
        }
    }
}

/// [`ManeuverRunner`] settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ManeuverConfig {
    /// Thrust through [`Thrust::Maneuver`] segments (rolls, loops), `[0, 1]`.
    pub thrust: f32,
    /// Thrust through [`Thrust::Cruise`] segments (doublets and their pause), `[0, 1]`: the airframe's
    /// level-flight thrust.
    pub cruise_thrust: f32,
    /// An [`Angle`](SegmentEnd::Angle) segment ends after this many times the time its angle
    /// takes at the rate it began with (at least 1): a turn the vehicle can't fly, or can't
    /// measure, ends anyway.
    pub timeout_factor: f32,
    /// The recovery after the last segment.
    pub recovery: RecoveryConfig,
}

impl Default for ManeuverConfig {
    /// Full thrust for aerobatics (a loop climbs on it, and on a tailsitter a roll needs the
    /// propwash's authority), [`PlaneGuidanceConfig`](crate::guidance::plane::PlaneGuidanceConfig)'s
    /// default cruise thrust (0.5), a timeout at twice a segment's time, and the default recovery.
    fn default() -> Self {
        ManeuverConfig {
            thrust: 1.0,
            cruise_thrust: 0.5,
            timeout_factor: 2.0,
            recovery: RecoveryConfig::default(),
        }
    }
}

impl ManeuverConfig {
    /// `Ok` when the thrusts are in `[0, 1]`, the timeout factor at least 1, the level pitch
    /// in `[-π/2, π/2]`, and the band, rate, settle, and timeout positive and finite.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_unit_interval("thrust", self.thrust)?;
        check_unit_interval("cruise_thrust", self.cruise_thrust)?;
        check(
            "timeout_factor",
            self.timeout_factor >= 1.0 && self.timeout_factor.is_finite(),
            "at least 1 and finite",
        )?;
        let recovery = &self.recovery;
        let section = |e: InvalidConfig| e.in_section("recovery");
        check(
            "pitch_rad",
            (-core::f32::consts::FRAC_PI_2..=core::f32::consts::FRAC_PI_2)
                .contains(&recovery.pitch_rad),
            "in [-π/2, π/2]",
        )
        .map_err(section)?;
        check_unit_interval("thrust", recovery.thrust).map_err(section)?;
        check_positive("level_band_rad", recovery.level_band_rad).map_err(section)?;
        check_positive("max_body_rate_radps", recovery.max_body_rate_radps).map_err(section)?;
        check_positive("settle_s", recovery.settle_s).map_err(section)?;
        check_positive("timeout_s", recovery.timeout_s).map_err(section)
    }
}

/// Where a [`ManeuverRunner`] is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ManeuverPhase {
    /// Nothing started.
    Idle,
    /// Flying segment `segment`.
    Rotation {
        /// The segment's index.
        segment: usize,
    },
    /// Leveling the wings.
    Recovery,
    /// Finished: the caller flies on.
    Done,
}

/// Progress through the segment being flown.
#[derive(Clone, Copy, Debug)]
struct SegmentProgress {
    /// The inverse of the attitude it began at.
    start_inverse: UnitQuaternion<f32>,
    /// The last finite twist read since then, in rad.
    last_twist_rad: f32,
    /// The angle turned, unwrapped, in rad.
    turned_rad: f32,
    /// Time flown, in s.
    elapsed_s: f32,
    /// When it ends regardless of the attitude, in s: a duration, or an angle's timeout.
    limit_s: f32,
}

impl SegmentProgress {
    /// `segment` beginning at `state`.
    fn new(segment: &Segment, state: &VehicleState, timeout_factor: f32) -> Self {
        let limit_s = match *segment {
            Segment::Pause { duration_s, .. } => duration_s,
            Segment::Rotate {
                end: SegmentEnd::Duration { s },
                ..
            } => s,
            Segment::Rotate {
                rate,
                end: SegmentEnd::Angle { rad },
                ..
            } => {
                let radps = rate.radps(state).abs();
                // No rate to fly it at (no speed for a radius): end at once.
                if positive(radps) {
                    rad / radps * timeout_factor
                } else {
                    0.0
                }
            }
        };
        SegmentProgress {
            start_inverse: state.attitude.inverse(),
            last_twist_rad: 0.0,
            turned_rad: 0.0,
            elapsed_s: 0.0,
            limit_s,
        }
    }

    /// Whether `segment` has ended at `state`; reads the turn for an angle.
    fn ended(&mut self, segment: &Segment, state: &VehicleState) -> bool {
        if self.elapsed_s >= self.limit_s {
            return true;
        }
        let Segment::Rotate {
            axis,
            rate,
            end: SegmentEnd::Angle { rad },
            ..
        } = *segment
        else {
            return false;
        };
        let rotation = self.start_inverse * state.attitude;
        let twist_rad = twist_angle_rad(&rotation, &axis.unit());
        // Each tick turns far less than half a turn, so the wrapped step is the turn since
        // the last reading. A non-finite one is skipped: the timeout ends the segment.
        let step_rad = wrap_angle(twist_rad - self.last_twist_rad);
        if step_rad.is_finite() {
            self.turned_rad += step_rad;
            self.last_twist_rad = twist_rad;
        }
        self.turned_rad * rate.direction() >= rad
    }
}

/// Leveling after the last segment.
#[derive(Clone, Copy, Debug, Default)]
struct Recovery {
    /// Time recovering, in s.
    elapsed_s: f32,
    /// Time continuously level and settled, in s.
    level_s: f32,
}

impl Recovery {
    /// This tick's level attitude command, or `None` once level and settled for
    /// [`settle_s`](RecoveryConfig::settle_s) or past the timeout.
    fn next(
        &mut self,
        config: &RecoveryConfig,
        dt: f32,
        state: &VehicleState,
    ) -> Option<ControlInput> {
        let (roll_rad, pitch_rad, _) = euler_angles(&state.attitude);
        // `<=` is false for NaN: an unknown attitude or rate isn't level.
        let level = roll_rad.abs() <= config.level_band_rad
            && (pitch_rad - config.pitch_rad).abs() <= config.level_band_rad
            && state.angular_velocity_radps.norm() <= config.max_body_rate_radps;
        self.level_s = if level { self.level_s + dt } else { 0.0 };
        if self.level_s >= config.settle_s || self.elapsed_s >= config.timeout_s {
            return None;
        }
        self.elapsed_s += dt;
        Some(ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
            roll_rad: 0.0,
            pitch_rad: config.pitch_rad,
            heading_rad: f32::NAN,
            thrust: config.thrust,
        }))
    }
}

/// Flies a [`Maneuver`]: its segments in order from the attitude it starts at, then a recovery
/// to level ([`RecoveryConfig`]), then nothing, so the caller (a script, a route) flies on.
/// The caller decides where it starts: level, at the speed and height the maneuver needs.
///
/// # Examples
///
/// ```
/// use stable_flight::commanders::maneuver::{Maneuver, ManeuverPhase, ManeuverRunner};
/// use stable_flight::{ControlInput, VehicleState};
///
/// let mut runner = ManeuverRunner::default();
/// let level = VehicleState::default();
/// runner.start(Maneuver::roll(1.5).expect("a nonzero rate"), &level);
/// let Some(ControlInput::AbsoluteRate(rates)) = runner.next(0.004, &level) else {
///     unreachable!("a roll begins with a roll rate");
/// };
/// assert_eq!(rates.roll_radps, 1.5);
/// assert_eq!(runner.phase(), ManeuverPhase::Rotation { segment: 0 });
/// ```
#[derive(Clone, Debug)]
pub struct ManeuverRunner {
    config: ManeuverConfig,
    maneuver: Option<Maneuver>,
    phase: ManeuverPhase,
    progress: SegmentProgress,
    recovery: Recovery,
}

impl Default for ManeuverRunner {
    /// A runner with [`ManeuverConfig::default`].
    fn default() -> Self {
        ManeuverRunner::new(ManeuverConfig::default())
    }
}

impl ManeuverRunner {
    /// An idle runner with `config`.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`ManeuverConfig::validate`]).
    pub fn new(config: ManeuverConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid ManeuverConfig: {error}");
        }
        ManeuverRunner {
            config,
            maneuver: None,
            phase: ManeuverPhase::Idle,
            progress: SegmentProgress::new(&UNUSED, &VehicleState::default(), 1.0),
            recovery: Recovery::default(),
        }
    }

    /// The settings.
    pub fn config(&self) -> &ManeuverConfig {
        &self.config
    }

    /// Starts `maneuver` at `state`, replacing any in progress.
    pub fn start(&mut self, maneuver: Maneuver, state: &VehicleState) {
        self.maneuver = Some(maneuver);
        self.begin(0, state);
    }

    /// Cuts a maneuver's segments short and recovers; a recovery or a finished maneuver is
    /// left alone.
    pub fn abort(&mut self) {
        if matches!(self.phase, ManeuverPhase::Rotation { .. }) {
            self.recover();
        }
    }

    /// Where the runner is.
    pub fn phase(&self) -> ManeuverPhase {
        self.phase
    }

    /// True while a maneuver is flying or recovering.
    pub fn is_active(&self) -> bool {
        matches!(
            self.phase,
            ManeuverPhase::Rotation { .. } | ManeuverPhase::Recovery
        )
    }

    /// This tick's command: a segment's body rate or the recovery's level attitude, or `None`
    /// once done (or idle). A segment that ends this tick hands over to the next phase in the
    /// same tick.
    ///
    /// # Panics
    ///
    /// If `dt` is not positive and finite.
    pub fn next(&mut self, dt: f32, state: &VehicleState) -> Option<ControlInput> {
        assert_dt(dt);
        loop {
            match self.phase {
                ManeuverPhase::Idle | ManeuverPhase::Done => return None,
                ManeuverPhase::Rotation { segment: index } => {
                    let segment = self.segment(index)?;
                    if self.progress.ended(&segment, state) {
                        self.begin(index + 1, state);
                        continue;
                    }
                    self.progress.elapsed_s += dt;
                    return Some(self.rates(&segment, state));
                }
                ManeuverPhase::Recovery => {
                    let command = self.recovery.next(&self.config.recovery, dt, state);
                    if command.is_none() {
                        self.phase = ManeuverPhase::Done;
                    }
                    return command;
                }
            }
        }
    }

    /// Segment `index` of the maneuver being flown.
    fn segment(&self, index: usize) -> Option<Segment> {
        self.maneuver.as_ref()?.segments().get(index).copied()
    }

    /// Begins segment `index` at `state`, or the recovery past the last.
    fn begin(&mut self, index: usize, state: &VehicleState) {
        let Some(segment) = self.segment(index) else {
            self.recover();
            return;
        };
        self.phase = ManeuverPhase::Rotation { segment: index };
        self.progress = SegmentProgress::new(&segment, state, self.config.timeout_factor);
    }

    /// Begins the recovery.
    fn recover(&mut self) {
        self.phase = ManeuverPhase::Recovery;
        self.recovery = Recovery::default();
    }

    /// `segment`'s body rate at `state`, the other axes at zero, and its thrust.
    fn rates(&self, segment: &Segment, state: &VehicleState) -> ControlInput {
        let (axis, radps, thrust) = match *segment {
            Segment::Rotate {
                axis, rate, thrust, ..
            } => (Some(axis), rate.radps(state), thrust),
            Segment::Pause { thrust, .. } => (None, 0.0, thrust),
        };
        let rate = |about| if axis == Some(about) { radps } else { 0.0 };
        ControlInput::AbsoluteRate(AbsoluteRateCommand {
            roll_radps: rate(Axis::Roll),
            pitch_radps: rate(Axis::Pitch),
            yaw_radps: rate(Axis::Yaw),
            thrust: match thrust {
                Thrust::Maneuver => self.config.thrust,
                Thrust::Cruise => self.config.cruise_thrust,
            },
        })
    }
}
