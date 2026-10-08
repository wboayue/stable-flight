//! Follows and loops a fixed list of waypoints.

use core::fmt;

use crate::{Command, Commander, VehicleState, WaypointCommand};

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

/// A [`Commander`] that flies a looping sequence of waypoints.
///
/// Each tick it emits the active waypoint as a [`Command::Waypoint`]. When the vehicle is
/// within the acceptance radius of it (horizontal distance only), it advances to the next,
/// wrapping to the first after the last. It knows nothing about where the waypoints came
/// from or what shape they trace.
///
/// Generic over the storage: anything that is `AsRef<[WaypointCommand]>`. Firmware can use a
/// `[WaypointCommand; N]` or `&'static [WaypointCommand]`; a host can use a `Vec`. It never
/// allocates.
///
/// # Examples
///
/// ```
/// use stable_flight::Vec2;
/// use stable_flight::commanders::WaypointFollower;
/// use stable_flight::{Command, Commander, VehicleState, WaypointCommand};
///
/// let square = [
///     WaypointCommand { position_m: Vec2::zeros(), altitude_m: 10.0, ..Default::default() },
///     WaypointCommand { position_m: Vec2::new(20.0, 0.0), altitude_m: 10.0, ..Default::default() },
///     WaypointCommand { position_m: Vec2::new(20.0, 20.0), altitude_m: 10.0, ..Default::default() },
///     WaypointCommand { position_m: Vec2::new(0.0, 20.0), altitude_m: 10.0, ..Default::default() },
/// ];
/// let mut follower = WaypointFollower::new(square)
///     .expect("non-empty")
///     .with_acceptance_radius(2.0);
///
/// // Starting on the first waypoint, the follower advances to the second.
/// let Command::Waypoint(target) = follower.next_command(0.004, &VehicleState::default()) else {
///     unreachable!("WaypointFollower only emits waypoint commands");
/// };
/// assert_eq!(target.position_m.x, 20.0);
/// ```
#[derive(Debug, Clone)]
pub struct WaypointFollower<W> {
    waypoints: W,
    current: usize,
    acceptance_radius_squared: f32,
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
        })
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
}

impl<W: AsRef<[WaypointCommand]>> Commander for WaypointFollower<W> {
    fn next_command(&mut self, _dt: f32, current_state: &VehicleState) -> Command {
        let waypoints = self.waypoints.as_ref();

        // Compare squared horizontal distance to avoid a per-tick sqrt.
        let target = &waypoints[self.current];
        let dx = target.position_m.x - current_state.position_m.x;
        let dy = target.position_m.y - current_state.position_m.y;
        if dx * dx + dy * dy <= self.acceptance_radius_squared {
            self.current = (self.current + 1) % waypoints.len();
        }

        Command::Waypoint(waypoints[self.current])
    }
}
