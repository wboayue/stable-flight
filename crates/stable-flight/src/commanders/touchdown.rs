//! Decides when a landing vehicle has touched down: see [`TouchdownDetector`].

#[cfg(test)]
mod tests;

/// Sinking faster than this, in m/s, shows the descent has begun.
pub const DESCENDING_MPS: f32 = 0.2;

/// Vertical speed below this, in m/s either way, counts as stopped.
pub const STOPPED_MPS: f32 = 0.1;

/// How long, in seconds, the vehicle must stay stopped after descending to count as down.
pub const TOUCHDOWN_HOLD_S: f32 = 1.0;

/// Detects touchdown during a land ([`ControlInput::Land`](crate::ControlInput::Land)) from
/// the vertical speed alone.
///
/// Landing guidance keeps lowering its altitude target, so a vehicle still in the air can't
/// hold a vertical speed near zero for long: its thrust falls until it sinks. Once the vehicle
/// has sunk faster than [`DESCENDING_MPS`], staying within ±[`STOPPED_MPS`] for
/// [`TOUCHDOWN_HOLD_S`] means the ground stopped it. Before the descent begins (the hover the
/// land started from), nothing counts. Touchdown latches until [`reset`](Self::reset).
///
/// It works only while a land is flown; hovering would read as touchdown. A non-finite speed
/// restarts the stopped timer without clearing the descent.
///
/// # Examples
///
/// ```
/// use stable_flight::commanders::TouchdownDetector;
///
/// let mut detector = TouchdownDetector::default();
/// assert!(!detector.update(0.5, 0.0)); // hovering before the descent: not down
/// assert!(!detector.update(0.5, -0.7)); // descending
/// assert!(!detector.update(0.5, 0.0)); // stopped for 0.5 s
/// assert!(detector.update(0.6, 0.0)); // stopped for 1.1 s: down
/// ```
#[derive(Debug, Default, Clone)]
pub struct TouchdownDetector {
    descended: bool,
    /// Seconds stopped since the descent; `None` while moving.
    stopped_s: Option<f32>,
    touched_down: bool,
}

impl TouchdownDetector {
    /// Advances by `dt` seconds with the vertical speed `vertical_speed_mps` (up positive) and
    /// returns whether the vehicle has touched down.
    pub fn update(&mut self, dt: f32, vertical_speed_mps: f32) -> bool {
        if self.touched_down {
            return true;
        }
        if vertical_speed_mps < -DESCENDING_MPS {
            self.descended = true;
        }
        self.stopped_s = if self.descended && vertical_speed_mps.abs() < STOPPED_MPS {
            Some(self.stopped_s.unwrap_or(0.0) + dt)
        } else {
            None
        };
        self.touched_down = self.stopped_s.is_some_and(|s| s >= TOUCHDOWN_HOLD_S);
        self.touched_down
    }

    /// Whether touchdown has been detected.
    pub fn touched_down(&self) -> bool {
        self.touched_down
    }

    /// Forgets the descent and any touchdown, for a new land.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}
