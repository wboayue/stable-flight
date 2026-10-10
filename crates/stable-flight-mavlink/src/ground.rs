//! Home and the landed state together: see [`GroundTracker`].

use mavlink::common::MavLandedState;
use stable_flight::{ControlMode, Vec3};

use crate::{HomeTracker, LandDetector};

#[cfg(test)]
mod tests;

/// Tracks home ([`HomeTracker`]) and the landed state ([`LandDetector`], fed the height above
/// that home), and reports when either changes, so a caller can send `HOME_POSITION` and
/// `EXTENDED_SYS_STATE` at once instead of waiting for their next period. The landed state is
/// undefined until the first update.
///
/// # Examples
///
/// ```
/// use stable_flight::ControlMode::Attitude;
/// use stable_flight_mavlink::GroundTracker;
/// use stable_flight_mavlink::mavlink::common::MavLandedState;
///
/// let mut ground = GroundTracker::default();
/// let changes = ground.update(true, Attitude, [0.0, 0.0, 10.0].into()); // armed: home latched
/// assert!(changes.home && changes.landed);
/// assert_eq!(ground.landed_state(), MavLandedState::MAV_LANDED_STATE_ON_GROUND);
///
/// let changes = ground.update(true, Attitude, [0.0, 0.0, 12.0].into()); // took off
/// assert!(!changes.home && changes.landed);
/// assert_eq!(ground.landed_state(), MavLandedState::MAV_LANDED_STATE_IN_AIR);
/// ```
#[derive(Debug)]
pub struct GroundTracker {
    home: HomeTracker,
    land_detector: LandDetector,
    landed_state: MavLandedState,
}

/// What a [`GroundTracker::update`] changed.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct GroundChanges {
    /// Home was latched (set or moved).
    pub home: bool,
    /// The landed state differs from the previous update's.
    pub landed: bool,
}

impl Default for GroundTracker {
    fn default() -> Self {
        GroundTracker {
            home: HomeTracker::default(),
            land_detector: LandDetector::default(),
            landed_state: MavLandedState::MAV_LANDED_STATE_UNDEFINED,
        }
    }
}

impl GroundTracker {
    /// Advances by one update with the flight controller's arm state and `mode`, and local
    /// `position_m` (north/east/up).
    pub fn update(&mut self, armed: bool, mode: ControlMode, position_m: Vec3) -> GroundChanges {
        let home = self.home.update(armed, position_m);
        let height_m = self
            .home
            .home_m()
            .map_or(f32::NAN, |home_m| position_m.z - home_m.z);
        let landed_state = self.land_detector.update(armed, mode, height_m);
        let landed = landed_state != self.landed_state;
        self.landed_state = landed_state;
        GroundChanges { home, landed }
    }

    /// Home as local north/east/up, once a finite position has been seen.
    pub fn home_m(&self) -> Option<Vec3> {
        self.home.home_m()
    }

    /// The latest landed state.
    pub fn landed_state(&self) -> MavLandedState {
        self.landed_state
    }
}
