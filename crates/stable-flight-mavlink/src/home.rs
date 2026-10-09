//! Where home is: see [`HomeTracker`].

use stable_flight::Vec3;

#[cfg(test)]
mod tests;

/// Tracks home, the local position `HOME_POSITION` reports and relative altitude is measured
/// from: the position at the latest arming (the takeoff point), or before the first arm, the
/// first finite one. A position with a non-finite axis never becomes home: arming on one
/// latches the next finite position while still armed.
///
/// # Examples
///
/// ```
/// use stable_flight_mavlink::HomeTracker;
///
/// let mut home = HomeTracker::default();
/// assert!(home.update(false, [0.0, 0.0, 5.0].into())); // first finite position
/// assert!(!home.update(false, [1.0, 0.0, 5.0].into())); // moved while disarmed: home stays
/// assert!(home.update(true, [1.0, 0.0, 5.0].into())); // arming re-latches
/// assert_eq!(home.home_m(), Some([1.0, 0.0, 5.0].into()));
/// ```
#[derive(Debug, Default)]
pub struct HomeTracker {
    home_m: Option<Vec3>,
    /// Armed since the last finite position: latch home at the next one.
    latch_pending: bool,
    /// The previous update's arm state, to find the arming edge.
    was_armed: bool,
}

impl HomeTracker {
    /// Advances by one update with the arm state and local `position_m`. Returns whether home
    /// was latched (set or moved) by this update.
    pub fn update(&mut self, armed: bool, position_m: Vec3) -> bool {
        let arming = armed && !self.was_armed;
        self.was_armed = armed;
        self.latch_pending = arming || (self.latch_pending && armed);
        if !position_m.iter().all(|c| c.is_finite())
            || !(self.latch_pending || self.home_m.is_none())
        {
            return false;
        }
        self.home_m = Some(position_m);
        self.latch_pending = false;
        true
    }

    /// Home as local north/east/up, once a finite position has been seen.
    pub fn home_m(&self) -> Option<Vec3> {
        self.home_m
    }
}
