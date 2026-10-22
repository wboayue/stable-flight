//! The data a [`ManeuverRunner`] flies: [`Maneuver`]s, their
//! [`Segment`]s, and the catalog.

use thiserror::Error;

use crate::{Vec3, VehicleState};

use super::Path;

#[cfg(doc)]
use super::{ManeuverConfig, ManeuverRunner};

/// Most segments a [`Maneuver`] holds: a square loop with point rolls on its lines needs more
/// than 8.
pub const MAX_SEGMENTS: usize = 16;

/// Seconds a [`Maneuver::doublet`] pauses after its two halves, so the free response is
/// recorded before the recovery takes over.
pub const DOUBLET_PAUSE_S: f32 = 1.0;

/// A [`Maneuver::doublet`]'s half width, in s, when a script gives none.
pub const DEFAULT_DOUBLET_HALF_S: f32 = 0.5;

/// A body axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    /// About body `x`, right wing down positive.
    Roll,
    /// About body `y`, nose up positive.
    Pitch,
    /// About body `z`, nose right positive.
    Yaw,
}

impl Axis {
    /// The axis as a unit vector in body axes.
    pub(super) fn unit(self) -> Vec3 {
        match self {
            Axis::Roll => Vec3::x(),
            Axis::Pitch => Vec3::y(),
            Axis::Yaw => Vec3::z(),
        }
    }
}

/// The body rate a [`Segment::Rotate`] holds, signed: the sign is the direction about the
/// axis (negative roll is left, negative pitch a push).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RateLaw {
    /// A fixed rate, in rad/s.
    Constant {
        /// The rate, in rad/s.
        radps: f32,
    },
    /// `V / radius_m` each tick, `V` the airspeed (the ground speed without one): the arc of a
    /// circle of that radius whatever the speed does over it. Negative curves the other way
    /// (about pitch, an outside loop).
    Radius {
        /// The arc's radius, in m.
        radius_m: f32,
    },
}

impl RateLaw {
    /// The rate for `state`, in rad/s; NaN without a speed.
    pub(super) fn radps(self, state: &VehicleState) -> f32 {
        match self {
            RateLaw::Constant { radps } => radps,
            RateLaw::Radius { radius_m } => {
                let speed_mps = state
                    .valid_airspeed_mps()
                    .unwrap_or_else(|| state.velocity_mps.norm());
                speed_mps / radius_m
            }
        }
    }

    /// The direction it turns about its axis, `±1`.
    pub(super) fn direction(self) -> f32 {
        match self {
            RateLaw::Constant { radps } => radps.signum(),
            RateLaw::Radius { radius_m } => radius_m.signum(),
        }
    }

    /// Its rate or radius: what must be finite and nonzero.
    fn value(self) -> f32 {
        match self {
            RateLaw::Constant { radps } => radps,
            RateLaw::Radius { radius_m } => radius_m,
        }
    }
}

/// When a [`Segment::Rotate`] ends.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SegmentEnd {
    /// Once it has turned this far in its rate's direction, in rad (positive).
    Angle {
        /// The angle, in rad.
        rad: f32,
    },
    /// After this long, in s, whatever the attitude.
    Duration {
        /// The time, in s.
        s: f32,
    },
}

/// The thrust a [`Segment`] holds, from the [`ManeuverConfig`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Thrust {
    /// [`ManeuverConfig::thrust`]: an aerobatic segment's.
    Maneuver,
    /// [`ManeuverConfig::cruise_thrust`]: a measurement's, flown as in level flight.
    Cruise,
}

/// One step of a [`Maneuver`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum Segment {
    /// A body rate about one axis until its end.
    Rotate {
        /// The axis it turns about.
        axis: Axis,
        /// The rate it holds.
        rate: RateLaw,
        /// When it ends.
        end: SegmentEnd,
        /// The thrust it holds.
        thrust: Thrust,
        /// The path it holds while it turns.
        path: Path,
    },
    /// Zero body rate for a time: the free response after a doublet.
    Pause {
        /// The time, in s.
        duration_s: f32,
        /// The thrust it holds.
        thrust: Thrust,
    },
}

impl Segment {
    /// `Err` with what's wrong: a rate or radius that isn't finite and nonzero, an angle or
    /// duration that isn't positive and finite, or a [`Path::Circle`] without a radius or
    /// about the roll axis (whose circle has no center off the nose).
    fn check(&self) -> Result<(), &'static str> {
        match *self {
            Segment::Rotate {
                axis,
                rate,
                end,
                path,
                ..
            } => {
                let value = rate.value();
                if !(value.is_finite() && value != 0.0) {
                    return Err(match rate {
                        RateLaw::Constant { .. } => "rate not finite and nonzero",
                        RateLaw::Radius { .. } => "radius not finite and nonzero",
                    });
                }
                if path == Path::Circle {
                    if !matches!(rate, RateLaw::Radius { .. }) {
                        return Err("circle without a radius");
                    }
                    if axis == Axis::Roll {
                        return Err("circle about the roll axis");
                    }
                }
                match end {
                    SegmentEnd::Angle { rad } if !positive(rad) => {
                        Err("angle not positive and finite")
                    }
                    SegmentEnd::Duration { s } if !positive(s) => {
                        Err("duration not positive and finite")
                    }
                    _ => Ok(()),
                }
            }
            Segment::Pause { duration_s, .. } if !positive(duration_s) => {
                Err("duration not positive and finite")
            }
            Segment::Pause { .. } => Ok(()),
        }
    }
}

/// Positive and finite.
pub(super) fn positive(value: f32) -> bool {
    value > 0.0 && value.is_finite()
}

/// A [`Maneuver`] that can't be flown.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
#[non_exhaustive]
pub enum ManeuverError {
    /// No segments.
    #[error("a maneuver needs a segment")]
    Empty,
    /// More than [`MAX_SEGMENTS`].
    #[error("a maneuver holds at most {max} segments, got {count}", max = MAX_SEGMENTS)]
    TooManySegments {
        /// The segments given.
        count: usize,
    },
    /// A segment out of range.
    #[error("segment {index}: {reason}")]
    InvalidSegment {
        /// Its index.
        index: usize,
        /// What's wrong with it.
        reason: &'static str,
    },
}

/// A sequence of [`Segment`]s flown in order by a [`ManeuverRunner`]. `Copy` and fixed-size, so
/// a script or a mission can hold one without allocating.
///
/// # Examples
///
/// ```
/// use stable_flight::commanders::maneuver::{
///     Axis, Maneuver, Path, RateLaw, Segment, SegmentEnd, Thrust,
/// };
///
/// let roll = Maneuver::roll(90.0_f32.to_radians()).expect("a nonzero rate");
/// assert_eq!(roll.segments().len(), 1);
///
/// // An Immelmann: half a loop of 25 m, then half a roll at 90°/s on the line it ends on.
/// let half_turn = SegmentEnd::Angle { rad: core::f32::consts::PI };
/// let immelmann = Maneuver::new(&[
///     Segment::Rotate {
///         axis: Axis::Pitch,
///         rate: RateLaw::Radius { radius_m: 25.0 },
///         end: half_turn,
///         thrust: Thrust::Maneuver,
///         path: Path::Circle,
///     },
///     Segment::Rotate {
///         axis: Axis::Roll,
///         rate: RateLaw::Constant { radps: 90.0_f32.to_radians() },
///         end: half_turn,
///         thrust: Thrust::Maneuver,
///         path: Path::Line,
///     },
/// ])
/// .expect("valid segments");
/// assert_eq!(immelmann.segments().len(), 2);
/// assert!(Maneuver::loop_(0.0).is_err());
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Maneuver {
    segments: [Segment; MAX_SEGMENTS],
    len: usize,
}

impl Maneuver {
    /// A maneuver flying `segments` in order.
    ///
    /// # Errors
    ///
    /// [`ManeuverError::Empty`] for none, [`ManeuverError::TooManySegments`] for more than
    /// [`MAX_SEGMENTS`], [`ManeuverError::InvalidSegment`] for the first segment out of range.
    pub fn new(segments: &[Segment]) -> Result<Self, ManeuverError> {
        let count = segments.len();
        if count == 0 {
            return Err(ManeuverError::Empty);
        }
        if count > MAX_SEGMENTS {
            return Err(ManeuverError::TooManySegments { count });
        }
        for (index, segment) in segments.iter().enumerate() {
            segment
                .check()
                .map_err(|reason| ManeuverError::InvalidSegment { index, reason })?;
        }
        let mut maneuver = Maneuver {
            segments: [UNUSED; MAX_SEGMENTS],
            len: count,
        };
        maneuver.segments[..count].copy_from_slice(segments);
        Ok(maneuver)
    }

    /// One full roll at `rate_radps` (negative rolls left), holding its line.
    ///
    /// # Errors
    ///
    /// [`ManeuverError::InvalidSegment`] for a zero or non-finite rate.
    pub fn roll(rate_radps: f32) -> Result<Self, ManeuverError> {
        Maneuver::new(&[Segment::Rotate {
            axis: Axis::Roll,
            rate: RateLaw::Constant { radps: rate_radps },
            end: SegmentEnd::Angle { rad: FULL_TURN_RAD },
            thrust: Thrust::Maneuver,
            path: Path::Line,
        }])
    }

    /// One loop of `radius_m`, holding its circle: inside, or outside for a negative radius.
    ///
    /// # Errors
    ///
    /// [`ManeuverError::InvalidSegment`] for a zero or non-finite radius.
    pub fn loop_(radius_m: f32) -> Result<Self, ManeuverError> {
        Maneuver::new(&[Segment::Rotate {
            axis: Axis::Pitch,
            rate: RateLaw::Radius { radius_m },
            end: SegmentEnd::Angle { rad: FULL_TURN_RAD },
            thrust: Thrust::Maneuver,
            path: Path::Circle,
        }])
    }

    /// A doublet about `axis`: `rate_radps` for `half_s`, `-rate_radps` for `half_s`, then a
    /// [`DOUBLET_PAUSE_S`] pause, all at cruise thrust and on rates alone ([`Path::Free`]), so
    /// it measures the rate loop. The two halves leave the attitude about where it began.
    ///
    /// # Errors
    ///
    /// [`ManeuverError::InvalidSegment`] for a zero or non-finite rate, or a half width that
    /// isn't positive and finite.
    pub fn doublet(axis: Axis, rate_radps: f32, half_s: f32) -> Result<Self, ManeuverError> {
        let half = |radps| Segment::Rotate {
            axis,
            rate: RateLaw::Constant { radps },
            end: SegmentEnd::Duration { s: half_s },
            thrust: Thrust::Cruise,
            path: Path::Free,
        };
        Maneuver::new(&[
            half(rate_radps),
            half(-rate_radps),
            Segment::Pause {
                duration_s: DOUBLET_PAUSE_S,
                thrust: Thrust::Cruise,
            },
        ])
    }

    /// The segments, in order.
    pub fn segments(&self) -> &[Segment] {
        &self.segments[..self.len]
    }
}

/// Fills a [`Maneuver`]'s unused slots.
pub(super) const UNUSED: Segment = Segment::Pause {
    duration_s: 0.0,
    thrust: Thrust::Cruise,
};

/// A full turn, in rad.
const FULL_TURN_RAD: f32 = core::f32::consts::TAU;
