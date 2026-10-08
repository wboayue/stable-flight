//! Follows a fixed list of waypoints, looping or holding at the end.

use core::fmt;

use crate::math::all_finite;
use crate::{Command, Commander, ControlInput, Vec2, VehicleState, WaypointCommand};

#[cfg(test)]
mod tests;

/// Default horizontal distance (meters) within which the active waypoint is considered
/// reached and the follower advances to the next one.
pub const DEFAULT_ACCEPTANCE_RADIUS_M: f32 = 3.0;

/// Returned by [`WaypointFollower::new`] when given no waypoints; there is nothing to fly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EmptyWaypoints;

impl fmt::Display for EmptyWaypoints {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("waypoint list is empty")
    }
}

impl core::error::Error for EmptyWaypoints {}

/// What a [`WaypointFollower`] does after reaching the last waypoint.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum RouteEnd {
    /// Wrap to the first waypoint and fly the route again, forever.
    #[default]
    Loop,
    /// Stop on the last waypoint and hover there;
    /// [`is_complete`](WaypointFollower::is_complete) turns true on reaching it.
    Hold,
}

/// A [`Commander`] that flies a sequence of waypoints.
///
/// Each tick it emits the active waypoint as an armed [`ControlInput::Waypoint`] (it never
/// disarms; landing is #66). When the vehicle is within the acceptance radius of it
/// (horizontal distance only), or has passed it along the leg that leads to it (missed it
/// wide), it advances to the next. After the last it follows its
/// [`RouteEnd`]: wrap to the first ([`RouteEnd::Loop`], the default) or hold the last
/// ([`RouteEnd::Hold`]). It knows nothing about where the waypoints came from or what shape
/// they trace.
///
/// So the vehicle flies each leg at cruise speed instead of slowing at every waypoint, a
/// waypoint without its own `velocity_mps` is emitted with one along the leg that leads to it:
/// from the previous waypoint, at the waypoint's `cruise_speed_mps`. The first waypoint
/// flown, a waypoint with no positive cruise speed, and the waypoint held at the route's end
/// get none, so the vehicle stops on them.
///
/// Generic over the storage: anything that is `AsRef<[WaypointCommand]>`. Firmware can use a
/// `[WaypointCommand; N]` or `&'static [WaypointCommand]`; a host can use a `Vec`. It never
/// allocates.
///
/// # Examples
///
/// ```
/// use stable_flight::Vec2;
/// use stable_flight::commanders::{RouteEnd, WaypointFollower};
/// use stable_flight::{Commander, ControlInput, VehicleState, WaypointCommand};
///
/// let square = [
///     WaypointCommand { position_m: Vec2::zeros(), altitude_m: 10.0, ..Default::default() },
///     WaypointCommand { position_m: Vec2::new(20.0, 0.0), altitude_m: 10.0, ..Default::default() },
///     WaypointCommand { position_m: Vec2::new(20.0, 20.0), altitude_m: 10.0, ..Default::default() },
///     WaypointCommand { position_m: Vec2::new(0.0, 20.0), altitude_m: 10.0, ..Default::default() },
/// ];
/// let mut follower = WaypointFollower::new(square)
///     .expect("non-empty")
///     .with_acceptance_radius(2.0)
///     .with_route_end(RouteEnd::Hold);
///
/// // Starting on the first waypoint, the follower advances to the second.
/// let command = follower.next_command(0.004, &VehicleState::default());
/// let ControlInput::Waypoint(target) = command.input else {
///     unreachable!("WaypointFollower only emits waypoint commands");
/// };
/// assert_eq!(target.position_m.x, 20.0);
/// assert!(!follower.is_complete());
/// ```
#[derive(Debug, Clone)]
pub struct WaypointFollower<W> {
    waypoints: W,
    current: usize,
    acceptance_radius_squared: f32,
    route_end: RouteEnd,
    /// Unit direction of the leg into the active waypoint, from the previous one; `None` for
    /// the first waypoint flown or a zero-length leg. Set on each advance, so the per-tick
    /// path does no sqrt.
    leg_direction: Option<Vec2>,
    /// Velocity emitted with the active waypoint when it has none of its own.
    leg_velocity_mps: Option<Vec2>,
    /// The last waypoint has been reached under [`RouteEnd::Hold`].
    complete: bool,
}

impl<W: AsRef<[WaypointCommand]>> WaypointFollower<W> {
    /// Starts at the first waypoint with [`DEFAULT_ACCEPTANCE_RADIUS_M`].
    ///
    /// # Errors
    ///
    /// Returns [`EmptyWaypoints`] if `waypoints` is empty.
    pub fn new(waypoints: W) -> Result<Self, EmptyWaypoints> {
        if waypoints.as_ref().is_empty() {
            return Err(EmptyWaypoints);
        }
        Ok(WaypointFollower {
            waypoints,
            current: 0,
            acceptance_radius_squared: DEFAULT_ACCEPTANCE_RADIUS_M * DEFAULT_ACCEPTANCE_RADIUS_M,
            route_end: RouteEnd::default(),
            leg_direction: None,
            leg_velocity_mps: None,
            complete: false,
        })
    }

    /// Sets what happens after the last waypoint (default [`RouteEnd::Loop`]).
    pub fn with_route_end(mut self, route_end: RouteEnd) -> Self {
        self.route_end = route_end;
        self
    }

    /// Overrides the horizontal acceptance radius (meters). `radius_m` must be non-negative.
    pub fn with_acceptance_radius(mut self, radius_m: f32) -> Self {
        debug_assert!(
            radius_m >= 0.0,
            "acceptance radius must be non-negative, got {radius_m}"
        );
        self.acceptance_radius_squared = radius_m * radius_m;
        self
    }

    /// The waypoints being followed.
    pub fn waypoints(&self) -> &[WaypointCommand] {
        self.waypoints.as_ref()
    }

    /// Index of the active waypoint.
    pub fn current_index(&self) -> usize {
        self.current
    }

    /// True once the last waypoint has been reached under [`RouteEnd::Hold`]; the follower
    /// keeps holding it. Always false under [`RouteEnd::Loop`].
    pub fn is_complete(&self) -> bool {
        self.complete
    }

    /// Makes `next` the active waypoint, arriving from `previous`.
    fn advance(&mut self, previous: usize, next: usize) {
        self.current = next;
        let waypoints = self.waypoints.as_ref();
        let to = &waypoints[next];
        // A zero-length or non-finite leg has no direction.
        self.leg_direction = (to.position_m - waypoints[previous].position_m)
            .try_normalize(0.0)
            .filter(all_finite);
        let holds_here = self.route_end == RouteEnd::Hold && next == waypoints.len() - 1;
        // `>` is false for NaN: no positive cruise speed, no leg velocity.
        self.leg_velocity_mps = if !holds_here && to.cruise_speed_mps > 0.0 {
            self.leg_direction
                .map(|direction| direction * to.cruise_speed_mps)
                .filter(all_finite)
        } else {
            None
        };
    }
}

impl<W: AsRef<[WaypointCommand]>> Commander for WaypointFollower<W> {
    fn next_command(&mut self, _dt: f32, current_state: &VehicleState) -> Command {
        let waypoints = self.waypoints.as_ref();

        // Compare squared horizontal distance to avoid a per-tick sqrt.
        let target = &waypoints[self.current];
        let dx = target.position_m.x - current_state.position_m.x;
        let dy = target.position_m.y - current_state.position_m.y;
        let within_radius = dx * dx + dy * dy <= self.acceptance_radius_squared;
        // Past the waypoint along its leg: the vehicle missed it wide. Without this, the leg
        // velocity fed forward would park it past the waypoint, outside the radius, where the
        // position loop's pull back balances it.
        let passed = self
            .leg_direction
            .is_some_and(|direction| direction.x * dx + direction.y * dy <= 0.0);
        if !self.complete && (within_radius || passed) {
            let previous = self.current;
            let last = waypoints.len() - 1;
            match self.route_end {
                RouteEnd::Hold if previous == last => self.complete = true,
                _ => self.advance(previous, (previous + 1) % waypoints.len()),
            }
        }

        let mut target = self.waypoints.as_ref()[self.current];
        if target.velocity_mps.is_none() {
            target.velocity_mps = self.leg_velocity_mps;
        }
        Command::armed(ControlInput::Waypoint(target))
    }
}
