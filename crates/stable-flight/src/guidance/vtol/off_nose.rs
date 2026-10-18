//! [`OffNoseSpeed`]: a quadplane's copter-flight speed cut while its nose is off the way to its
//! waypoint, so it doesn't fly fast sideways or tail first.

use crate::controllers::{InvalidConfig, check_limit};
use crate::math::{DEG, wrap_angle};
use crate::{Configurable, VehicleState, WaypointCommand};

mod params;
#[cfg(test)]
mod tests;

/// Default [`OffNoseSpeedConfig::speed_mps`], in m/s: its 5 m/s tail first is ArduPilot's
/// quadplane waypoint speed (`Q_WP_SPD`, `ArduPlane/quadplane.cpp:609`) and the speed the
/// RealFlight Titan Cobra flies legs behind it cleanly (#433). On the sim quadplanes it holds
/// an uncapped leg's turnaround within 1° of the tilt cap, where uncapped it overshoots by up
/// to 12° (#437).
pub const DEFAULT_OFF_NOSE_SPEED_MPS: f32 = 15.0;

/// Heading error, from the nose to the bearing to the waypoint, up to which the speed isn't
/// cut (ArduPilot, `ArduPlane/quadplane.cpp:3087-3105`).
const FREE_RAD: f32 = 20.0 * DEG;

/// Heading error at and past which the speed is cut the most (ArduPilot, as above).
const FULL_RAD: f32 = 160.0 * DEG;

/// What the speed is divided by at [`FULL_RAD`] (ArduPilot, as above).
const FULL_DIVISOR: f32 = 3.0;

/// [`OffNoseSpeed`] settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OffNoseSpeedConfig {
    /// Speed, in m/s, that the cut divides: the cap just past 20° off the nose, a third of it
    /// from 160°; `0` is off. Non-negative and finite. Defaults to
    /// [`DEFAULT_OFF_NOSE_SPEED_MPS`].
    pub speed_mps: f32,
}

impl OffNoseSpeedConfig {
    /// `Ok` when the speed is non-negative and finite.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming `speed_mps`.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_limit("speed_mps", self.speed_mps)
    }
}

impl Default for OffNoseSpeedConfig {
    /// [`DEFAULT_OFF_NOSE_SPEED_MPS`].
    fn default() -> Self {
        OffNoseSpeedConfig {
            speed_mps: DEFAULT_OFF_NOSE_SPEED_MPS,
        }
    }
}

/// Caps a quadplane's waypoint speed in copter flight by how far the nose is off the bearing to
/// the waypoint, as ArduPilot's quadplane does (`get_scaled_wp_speed`,
/// `ArduPlane/quadplane.cpp:3087-3105`): uncut within 20°, then
/// [`speed_mps`](OffNoseSpeedConfig::speed_mps) divided by a factor rising linearly from 1 at
/// 20° to 3 at 160° and past it. The waypoint's own cruise speed still applies when lower.
///
/// A quadplane's wing and tail push it around flying sideways or tail first: tail first at
/// 15 m/s the RealFlight Titan Cobra pitched past what its lift rotors hold and flipped (#437).
/// Turning the nose along the leg (copter guidance's
/// [`waypoint_yaw_radps`](crate::guidance::copter::CopterGuidanceConfig::waypoint_yaw_radps))
/// leaves the turnarounds, an overshoot flown back along the leg, and legs with the turn off.
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::vtol::{OffNoseSpeed, OffNoseSpeedConfig};
/// use stable_flight::{Vec2, VehicleState, WaypointCommand};
///
/// let off_nose = OffNoseSpeed::new(OffNoseSpeedConfig { speed_mps: 15.0 });
/// let behind = WaypointCommand { position_m: Vec2::new(-100.0, 0.0), ..Default::default() };
/// let capped = off_nose.cap(&behind, &VehicleState::default()); // heading north
/// assert_eq!(capped.cruise_speed_mps, 5.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OffNoseSpeed {
    config: OffNoseSpeedConfig,
}

impl OffNoseSpeed {
    /// The cut with `config`.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`OffNoseSpeedConfig::validate`]).
    pub fn new(config: OffNoseSpeedConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid OffNoseSpeedConfig: {error}");
        }
        OffNoseSpeed { config }
    }

    /// `waypoint` with its cruise speed lowered to the cap for the vehicle's heading and
    /// position; unchanged when off, within 20° of the nose, or below the cap already. A
    /// non-finite position or heading leaves it unchanged (input policy).
    pub fn cap(&self, waypoint: &WaypointCommand, state: &VehicleState) -> WaypointCommand {
        let to_m = waypoint.position_m - state.position_m.xy();
        let bearing_rad = libm::atan2f(to_m.y, to_m.x);
        let cap_mps = self.cap_mps(wrap_angle(bearing_rad - state.heading_rad).abs());
        // `>` is false for NaN: an unknown cap, or no cruise speed, takes the cap.
        let cruise_mps = waypoint.cruise_speed_mps;
        if !cap_mps.is_finite() || (cruise_mps > 0.0 && cruise_mps <= cap_mps) {
            return *waypoint;
        }
        WaypointCommand {
            cruise_speed_mps: cap_mps,
            ..*waypoint
        }
    }

    /// The speed cap, in m/s, `error_rad` off the nose: infinite when off, within 20°, or for
    /// a NaN error.
    fn cap_mps(&self, error_rad: f32) -> f32 {
        let speed_mps = self.config.speed_mps;
        // `>` is false for NaN.
        if !(speed_mps > 0.0 && error_rad > FREE_RAD) {
            return f32::INFINITY;
        }
        let share = ((error_rad - FREE_RAD) / (FULL_RAD - FREE_RAD)).min(1.0);
        speed_mps / (1.0 + (FULL_DIVISOR - 1.0) * share)
    }
}

impl Configurable for OffNoseSpeed {
    type Config = OffNoseSpeedConfig;

    fn config(&self) -> &OffNoseSpeedConfig {
        &self.config
    }

    /// Validates with [`OffNoseSpeedConfig::validate`]; applies from the next tick.
    fn set_config(&mut self, config: OffNoseSpeedConfig) -> Result<(), InvalidConfig> {
        config.validate()?;
        self.config = config;
        Ok(())
    }
}
