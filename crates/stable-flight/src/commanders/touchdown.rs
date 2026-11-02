//! Decides when a landing vehicle has touched down: see [`TouchdownDetector`].

use crate::controllers::{InvalidConfig, check, check_positive};

#[cfg(test)]
mod tests;

/// Sinking faster than this, in m/s, shows the descent has begun. A constant, not a
/// [`TouchdownConfig`] field: copter guidance's land speed
/// ([`land_speed_mps`](crate::guidance::copter::CopterGuidanceConfig::land_speed_mps)) is
/// validated above it, so every land is seen to descend. PX4 keeps the same coupling by capping
/// its threshold at the land speed (`MulticopterLandDetector`, `LNDMC_Z_VEL_MAX`).
pub const DESCENDING_MPS: f32 = 0.2;

/// Default [`TouchdownConfig::stopped_mps`]: tighter than PX4's multicopter `LNDMC_Z_VEL_MAX`
/// (0.25 m/s), whose land detector also checks thrust and motion, where this one has only the
/// vertical speed.
pub const DEFAULT_STOPPED_MPS: f32 = 0.1;

/// Default [`TouchdownConfig::hold_s`]: PX4's multicopter land detector and ArduCopter's
/// `LAND_DETECTOR_TRIGGER_SEC` both wait 1 s.
pub const DEFAULT_TOUCHDOWN_HOLD_S: f32 = 1.0;

/// What counts as stopped on the ground for a [`TouchdownDetector`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TouchdownConfig {
    /// Vertical speed below this, in m/s either way, counts as stopped. Wider: a landing gear
    /// that bounces, or a noisy vertical speed, still reads as stopped; too wide, and a slow
    /// final sink does too, disarming above the ground. Must be positive and below
    /// [`DESCENDING_MPS`]. Defaults to [`DEFAULT_STOPPED_MPS`].
    pub stopped_mps: f32,
    /// How long, in seconds, the vehicle must stay stopped after descending to count as down.
    /// Longer: a bounce or a pause in the sink is less likely to disarm in the air, but the
    /// motors run longer on the ground. Must be positive and finite. Defaults to
    /// [`DEFAULT_TOUCHDOWN_HOLD_S`].
    pub hold_s: f32,
}

impl Default for TouchdownConfig {
    fn default() -> Self {
        Self {
            stopped_mps: DEFAULT_STOPPED_MPS,
            hold_s: DEFAULT_TOUCHDOWN_HOLD_S,
        }
    }
}

impl TouchdownConfig {
    /// Checks that `stopped_mps` is in `(0, DESCENDING_MPS)` and `hold_s` is positive and
    /// finite.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::commanders::TouchdownConfig;
    ///
    /// let config = TouchdownConfig { hold_s: 0.0, ..TouchdownConfig::default() };
    /// assert_eq!(config.validate().unwrap_err().to_string(), "hold_s must be positive and finite");
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check(
            "stopped_mps",
            self.stopped_mps > 0.0 && self.stopped_mps < DESCENDING_MPS,
            "in (0, DESCENDING_MPS)",
        )?;
        check_positive("hold_s", self.hold_s)
    }
}

/// Detects touchdown during a land ([`ControlInput::Land`](crate::ControlInput::Land)) from
/// the vertical speed alone.
///
/// Landing guidance keeps lowering its altitude target, so a vehicle still in the air can't
/// hold a vertical speed near zero for long: its thrust falls until it sinks. Once the vehicle
/// has sunk faster than [`DESCENDING_MPS`], staying within
/// ±[`stopped_mps`](TouchdownConfig::stopped_mps) for [`hold_s`](TouchdownConfig::hold_s) means
/// the ground stopped it. Before the descent begins (the hover the
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
    config: TouchdownConfig,
    descended: bool,
    /// Seconds stopped since the descent; `None` while moving.
    stopped_s: Option<f32>,
    touched_down: bool,
}

impl TouchdownDetector {
    /// A detector that reads touchdown per `config`; [`Default`] uses
    /// [`TouchdownConfig::default`].
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] if `config` fails [`TouchdownConfig::validate`].
    pub fn new(config: TouchdownConfig) -> Result<Self, InvalidConfig> {
        config.validate()?;
        Ok(Self {
            config,
            ..Self::default()
        })
    }

    /// Advances by `dt` seconds with the vertical speed `vertical_speed_mps` (up positive) and
    /// returns whether the vehicle has touched down.
    pub fn update(&mut self, dt: f32, vertical_speed_mps: f32) -> bool {
        if self.touched_down {
            return true;
        }
        if vertical_speed_mps < -DESCENDING_MPS {
            self.descended = true;
        }
        self.stopped_s = if self.descended && vertical_speed_mps.abs() < self.config.stopped_mps {
            Some(self.stopped_s.unwrap_or(0.0) + dt)
        } else {
            None
        };
        self.touched_down = self.stopped_s.is_some_and(|s| s >= self.config.hold_s);
        self.touched_down
    }

    /// Whether touchdown has been detected.
    pub fn touched_down(&self) -> bool {
        self.touched_down
    }

    /// Forgets the descent and any touchdown, for a new land; keeps the config.
    pub fn reset(&mut self) {
        *self = Self {
            config: self.config,
            ..Self::default()
        };
    }
}
