//! The path a [`Segment::Rotate`] holds while it turns: see [`Path`].

use crate::controllers::{InvalidConfig, check_limit};
use crate::math::{all_finite, neu_to_ned};
use crate::{GRAVITY, Vec3, VehicleState};

use super::segment::{RateLaw, Segment, positive};

#[cfg(doc)]
use super::ManeuverRunner;

#[cfg(test)]
mod tests;

/// The path a [`Segment::Rotate`] holds while it turns, laid out from where the segment
/// begins. Flying rates alone, a roll sinks as its lift tips over and a loop stretches as it
/// speeds up coming down; holding a path, the [`ManeuverRunner`] steers back toward it with
/// pitch and yaw rate on top of the segment's own rate, harder the further off it is
/// ([`PathConfig`]).
///
/// This is ArduPilot's scripted aerobatics' correction
/// (`libraries/AP_Scripting/applets/Aerobatics/FixedWing/plane_aerobatics.lua:2615-2645` at
/// `3d24375`): only the error across the path counts, a PD turns it into an acceleration, and
/// the acceleration divided by the speed is the turn rate, in pitch and yaw, never roll.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Path {
    /// No path: the segment's rates alone, as a doublet measures them.
    Free,
    /// The straight line along the direction of travel at the start: a roll keeps its height
    /// and its line.
    Line,
    /// The circle its [`RateLaw::Radius`] turns about its axis, tangent to the nose at the
    /// start: a loop about pitch (center above the canopy, or below it for a negative radius),
    /// a flat circle about yaw. Needs a radius and a pitch or yaw axis.
    Circle,
}

/// How hard a [`ManeuverRunner`] steers back onto a segment's [`Path`]. All zero flies the
/// segments' rates alone.
///
/// On `stable-flight-sim`'s plane, tailsitter, and eXplora the defaults end loops of 10 and 25 m
/// within 1.1 m of their entry height and 3% of their radius, and rolls at 90 and 180°/s lose
/// at most 3.5 m and climb at most 2.6 m, where rates alone lost up to 37 m in a roll and 6.4 m
/// in a loop (#481).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathConfig {
    /// Acceleration back toward the path per meter off it, in m/s² per m (1/s²). Higher holds
    /// the path tighter; 4 (with a velocity gain of 4) balloons a 90°/s roll 4.5 m on the sim
    /// for no less sink (#481). ArduPilot's `AEROM_ERR_COR_P` (`plane_aerobatics.lua:91-95`).
    pub position_gain_per_s2: f32,
    /// Acceleration against the speed away from the path, in m/s² per m/s (1/s): damps the
    /// return. ArduPilot's `AEROM_ERR_COR_D` (`:98-102`).
    pub velocity_gain_per_s: f32,
    /// On a [`Path::Line`], the share of the weight the wing stops carrying as it rolls away
    /// from upright (none upright, all of it on knife edge, twice it inverted) steered against
    /// up front instead of waiting for the sink. Higher: less sink, more climb. 0 leaves it all
    /// to the gains, as ArduPilot does: on the sim rolls then lose up to 5.2 m (the
    /// tailsitter's at 180°/s) against 3.5 m at 0.5, which climbs up to 2.6 m (#481).
    pub line_gravity_share: f32,
}

impl Default for PathConfig {
    /// ArduPilot's gains, 2 and 2.8, and half the lost lift up front (#481).
    fn default() -> Self {
        PathConfig {
            position_gain_per_s2: 2.0,
            velocity_gain_per_s: 2.8,
            line_gravity_share: 0.5,
        }
    }
}

impl PathConfig {
    /// `Ok` when every field is non-negative and finite.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_limit("position_gain_per_s2", self.position_gain_per_s2)?;
        check_limit("velocity_gain_per_s", self.velocity_gain_per_s)?;
        check_limit("line_gravity_share", self.line_gravity_share)
    }
}

/// Below this speed, in m/s, a turn rate from an acceleration (`a / V`) runs away: no
/// correction.
const MIN_SPEED_MPS: f32 = 1.0;

/// A segment's [`Path`], laid out in NED where it began.
#[derive(Clone, Copy, Debug)]
pub(super) struct PathHold {
    path: Path,
    /// Where the segment began, in m.
    start_m: Vec3,
    /// The direction of travel there, a unit vector.
    direction: Vec3,
    /// A [`Path::Circle`]'s center, in m.
    center_m: Vec3,
    /// A [`Path::Circle`]'s axis (the segment's, at the start), a unit vector.
    normal: Vec3,
    /// A [`Path::Circle`]'s radius, in m (positive).
    radius_m: f32,
}

impl PathHold {
    /// `segment`'s path, beginning at `state`.
    pub(super) fn new(segment: &Segment, state: &VehicleState) -> Self {
        let start_m = neu_to_ned(state.position_m);
        let velocity_mps = neu_to_ned(state.velocity_mps);
        let nose = state.attitude * Vec3::x();
        let direction = if velocity_mps.norm() >= MIN_SPEED_MPS {
            velocity_mps.normalize()
        } else {
            nose
        };
        let mut hold = PathHold {
            path: Path::Free,
            start_m,
            direction,
            center_m: start_m,
            normal: Vec3::zeros(),
            radius_m: 0.0,
        };
        let Segment::Rotate {
            axis, rate, path, ..
        } = *segment
        else {
            return hold;
        };
        hold.path = path;
        if let (Path::Circle, RateLaw::Radius { radius_m }) = (path, rate) {
            // The center sits off the nose, square to it and to the axis, on the side the rate
            // turns toward: above the canopy for a loop (pitch: y × x = −z, body up), right of
            // it for a yaw circle (z × x = y).
            let toward_center = axis.unit().cross(&Vec3::x());
            hold.center_m = start_m + state.attitude * toward_center * radius_m;
            hold.normal = state.attitude * axis.unit();
            hold.radius_m = radius_m.abs();
        }
        hold
    }

    /// The body rates, in rad/s (FRD: `x` roll, always 0; `y` pitch, nose up; `z` yaw, nose
    /// right), that steer `state` back onto the path: zero on it, for a [`Path::Free`], without
    /// a speed, or for a non-finite state.
    ///
    /// The error across the path (`e`, m) and its rate (`ė`, m/s) give the acceleration back
    /// toward it, `a = −(k_p·e + k_d·ė)` (NED, m/s²); a line adds `share · (1 − up_body·up) · g`
    /// up. Turning the velocity at that acceleration takes the rate `v̂ × a / V`, rotated into
    /// the body.
    pub(super) fn correction_radps(&self, config: &PathConfig, state: &VehicleState) -> Vec3 {
        let speed_mps = state
            .valid_airspeed_mps()
            .unwrap_or_else(|| state.velocity_mps.norm());
        // An unknown speed corrects nothing.
        if self.path == Path::Free || speed_mps.is_nan() || speed_mps < MIN_SPEED_MPS {
            return Vec3::zeros();
        }
        let position_m = neu_to_ned(state.position_m);
        let velocity_mps = neu_to_ned(state.velocity_mps);
        let Some((error_m, error_rate_mps)) = self.error(position_m, velocity_mps) else {
            return Vec3::zeros();
        };
        let mut accel_mps2 =
            -(error_m * config.position_gain_per_s2 + error_rate_mps * config.velocity_gain_per_s);
        if self.path == Path::Line {
            // NED up; the body's up is its −z.
            let up = -Vec3::z();
            let body_up = state.attitude * -Vec3::z();
            let lift_lost = 1.0 - body_up.dot(&up);
            accel_mps2 += up * (config.line_gravity_share * lift_lost * GRAVITY);
        }
        let travel = if velocity_mps.norm() >= MIN_SPEED_MPS {
            velocity_mps.normalize()
        } else {
            self.direction
        };
        let turn_radps = travel.cross(&accel_mps2) / speed_mps;
        let body_radps = state.attitude.inverse() * turn_radps;
        let correction = Vec3::new(0.0, body_radps.y, body_radps.z);
        if all_finite(&correction) {
            correction
        } else {
            Vec3::zeros()
        }
    }

    /// The position off the path across it, in m, and its rate, in m/s (NED); `None` at a
    /// circle's center, which has no direction back to it.
    fn error(&self, position_m: Vec3, velocity_mps: Vec3) -> Option<(Vec3, Vec3)> {
        match self.path {
            Path::Free => None,
            Path::Line => {
                let across = |v: Vec3| v - self.direction * v.dot(&self.direction);
                Some((across(position_m - self.start_m), across(velocity_mps)))
            }
            Path::Circle => {
                let offset_m = position_m - self.center_m;
                let off_plane_m = self.normal * offset_m.dot(&self.normal);
                let in_plane_m = offset_m - off_plane_m;
                let distance_m = in_plane_m.norm();
                if !positive(distance_m) {
                    return None;
                }
                let outward = in_plane_m / distance_m;
                let error_m = outward * (distance_m - self.radius_m) + off_plane_m;
                let error_rate_mps = outward * velocity_mps.dot(&outward)
                    + self.normal * velocity_mps.dot(&self.normal);
                Some((error_m, error_rate_mps))
            }
        }
    }
}
