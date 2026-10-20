//! The heading along a waypoint leg or to a waypoint's heading, and its yield when the heading
//! is stuck.

use super::CopterGuidanceConfig;
use crate::math::all_finite;
use crate::{Vec2, VehicleState, WaypointCommand, wrap_angle};

/// Yaw rate toward the heading-hold target, in rad/s (about 6°/s), below which a heading off
/// it by more than [`heading_yield_rad`](CopterGuidanceConfig::heading_yield_rad) is stuck. A
/// vehicle turning to its target yaws toward it faster; one held off by its fin doesn't yaw,
/// or yaws away (#452).
const STUCK_YAW_RATE_RADPS: f32 = 0.1;

/// Turns the heading-hold target to a waypoint's heading or along its leg, and yields a leg's
/// to the vehicle's heading once the heading is stuck off it. Allocation-free.
///
/// The config is passed to each call rather than stored, so the owner's config stays the
/// single source of truth.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct LegHeading {
    /// The leg being flown; `None` outside waypoint flight, until a finite position starts one.
    leg: Option<Leg>,
    /// Seconds this leg's heading has been stuck off the heading-hold target.
    stuck_s: f32,
    /// This leg's heading-hold target follows the vehicle's heading.
    yielding: bool,
}

/// A waypoint leg, from where the vehicle was when it began to its waypoint.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Leg {
    /// The leg's waypoint, in meters (north, east).
    to_m: Vec2,
    /// The leg's direction, in radians from north, set when the leg begins; `None` on a leg no
    /// longer than [`waypoint_yaw_min_leg_m`](CopterGuidanceConfig::waypoint_yaw_min_leg_m) at
    /// that time, which keeps the heading.
    bearing_rad: Option<f32>,
}

impl LegHeading {
    /// The heading-hold target for this tick of the leg to `waypoint`, from the current
    /// `target_rad`: turned toward the waypoint's finite
    /// [`heading_rad`](WaypointCommand::heading_rad) by at most `heading_rate_radps · dt`, never
    /// yielded; without one, turned toward the leg's direction by at most
    /// `waypoint_yaw_radps · dt`, or the vehicle's heading once the leg has yielded
    /// ([`yielded`](Self::yielded)); otherwise `target_rad`. Turns go the shorter way round,
    /// unwrapped.
    ///
    /// A new waypoint starts a leg from the vehicle's position, with nothing stuck; a
    /// non-finite position starts none, so the next tick retries. A waypoint that moves every
    /// tick (one held on the vehicle, a streamed offboard target) starts a leg every tick:
    /// held when the vehicle is within `waypoint_yaw_min_leg_m` of it, turned toward it from
    /// the vehicle otherwise, and never yielded.
    pub(crate) fn target_rad(
        &mut self,
        dt: f32,
        config: &CopterGuidanceConfig,
        target_rad: f32,
        waypoint: &WaypointCommand,
        estimated_state: &VehicleState,
    ) -> f32 {
        let bearing_rad = self.bearing_rad(config, waypoint.position_m, estimated_state);
        if let Some(heading_rad) = waypoint.heading_rad.filter(|h| h.is_finite()) {
            return turned_toward(target_rad, heading_rad, config.heading_rate_radps * dt);
        }
        if self.yielded(dt, config, target_rad, estimated_state) {
            return estimated_state.heading_rad;
        }
        let rate_radps = config.waypoint_yaw_radps;
        let Some(bearing_rad) = bearing_rad.filter(|_| rate_radps > 0.0) else {
            return target_rad;
        };
        turned_toward(target_rad, bearing_rad, rate_radps * dt)
    }

    /// Forgets the leg and its yield.
    pub(crate) fn clear(&mut self) {
        *self = Self::default();
    }

    /// The direction of the leg to `waypoint_m`, starting a leg when the waypoint is new.
    fn bearing_rad(
        &mut self,
        config: &CopterGuidanceConfig,
        waypoint_m: Vec2,
        estimated_state: &VehicleState,
    ) -> Option<f32> {
        if let Some(leg) = self.leg
            && leg.to_m == waypoint_m
        {
            return leg.bearing_rad;
        }
        self.clear();
        let here_m = estimated_state.position_m.xy();
        if !all_finite(&here_m) {
            return None;
        }
        let leg_m = waypoint_m - here_m;
        // `>` is false for NaN: a non-finite leg holds.
        let bearing_rad =
            (leg_m.norm() > config.waypoint_yaw_min_leg_m).then(|| libm::atan2f(leg_m.y, leg_m.x));
        self.leg = Some(Leg {
            to_m: waypoint_m,
            bearing_rad,
        });
        bearing_rad
    }

    /// Whether this leg has yielded its heading, after the heading has been stuck (more than
    /// `heading_yield_rad` off `target_rad`, yawing toward it at under
    /// [`STUCK_YAW_RATE_RADPS`]) for `heading_yield_s`. The body yaw rate stands in for the
    /// heading's rate, as it does near level. Never with `heading_yield_s` at 0 or a
    /// non-finite heading.
    fn yielded(
        &mut self,
        dt: f32,
        config: &CopterGuidanceConfig,
        target_rad: f32,
        estimated_state: &VehicleState,
    ) -> bool {
        let heading_rad = estimated_state.heading_rad;
        if !(config.heading_yield_s > 0.0 && heading_rad.is_finite()) {
            return false;
        }
        if self.yielding {
            return true;
        }
        let error_rad = wrap_angle(target_rad - heading_rad);
        let toward_radps = error_rad.signum() * estimated_state.angular_velocity_radps.z;
        // False for a NaN error or rate: not stuck.
        let stuck =
            error_rad.abs() > config.heading_yield_rad && toward_radps < STUCK_YAW_RATE_RADPS;
        self.stuck_s = if stuck { self.stuck_s + dt } else { 0.0 };
        self.yielding = self.stuck_s > config.heading_yield_s;
        self.yielding
    }
}

/// `from_rad` turned toward `to_rad` the shorter way round by at most `step_rad`, unwrapped.
fn turned_toward(from_rad: f32, to_rad: f32, step_rad: f32) -> f32 {
    from_rad + wrap_angle(to_rad - from_rad).clamp(-step_rad, step_rad)
}
