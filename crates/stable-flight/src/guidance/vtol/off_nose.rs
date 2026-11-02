//! [`OffNoseSpeed`]: a quadplane's copter-flight speed cut while its nose is off the way to its
//! waypoint, so it doesn't fly fast sideways or tail first.

use crate::controllers::{InvalidConfig, check, check_limit};
use crate::math::{DEG, wrap_angle};
use crate::{Configurable, VehicleState, WaypointCommand};
use core::f32::consts::PI;

mod params;
#[cfg(test)]
mod tests;

/// Default [`OffNoseSpeedConfig::speed_mps`], in m/s: its 5 m/s tail first is ArduPilot's
/// quadplane waypoint speed (`Q_WP_SPD`, in `ArduPlane/quadplane.cpp`'s `defaults_table`) and
/// the speed the RealFlight Titan Cobra flies legs behind it cleanly (#433). On the sim
/// quadplanes it holds an uncapped leg's turnaround within 1° of the tilt cap, where uncapped
/// it overshoots by up to 12° (#437).
pub const DEFAULT_OFF_NOSE_SPEED_MPS: f32 = 15.0;

/// Default [`OffNoseSpeedConfig::free_rad`], in rad: ArduPilot's
/// `QuadPlane::get_scaled_wp_speed`.
pub const DEFAULT_OFF_NOSE_FREE_RAD: f32 = 20.0 * DEG;

/// Default [`OffNoseSpeedConfig::full_rad`], in rad (ArduPilot, as above).
pub const DEFAULT_OFF_NOSE_FULL_RAD: f32 = 160.0 * DEG;

/// Default [`OffNoseSpeedConfig::tail_first_divisor`] (ArduPilot, as above): the Cobra's 5 m/s
/// tail first at the default speed (#437).
pub const DEFAULT_OFF_NOSE_TAIL_FIRST_DIVISOR: f32 = 3.0;

/// [`OffNoseSpeed`] settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OffNoseSpeedConfig {
    /// Speed, in m/s, that the cut divides: the cap just past `free_rad` off the nose,
    /// divided by `tail_first_divisor` from `full_rad`; `0` is off. Non-negative and finite.
    /// Defaults to [`DEFAULT_OFF_NOSE_SPEED_MPS`].
    pub speed_mps: f32,
    /// Heading error, from the nose to the bearing to the waypoint, up to which the speed isn't
    /// cut, in rad, in `[0, π)`. Higher: flies faster further off the nose.
    pub free_rad: f32,
    /// Heading error at and past which the speed is cut the most, in rad, in `(0, π]`; at or
    /// below `free_rad`, the whole cut applies just past `free_rad`. Higher: eases into the
    /// cut over a wider turn.
    pub full_rad: f32,
    /// What `speed_mps` is divided by from `full_rad` (tail first), at least 1; the divisor
    /// rises linearly from 1 at `free_rad`. Higher: slower tail first.
    pub tail_first_divisor: f32,
}

impl OffNoseSpeedConfig {
    /// `Ok` when the speed is non-negative and finite, the free angle in `[0, π)`, the full
    /// angle in `(0, π]`, and the divisor at least 1 and finite.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_limit("speed_mps", self.speed_mps)?;
        check(
            "free_rad",
            (0.0..PI).contains(&self.free_rad),
            "in [0, π) rad",
        )?;
        check(
            "full_rad",
            self.full_rad > 0.0 && self.full_rad <= PI,
            "in (0, π] rad",
        )?;
        check(
            "tail_first_divisor",
            self.tail_first_divisor >= 1.0 && self.tail_first_divisor.is_finite(),
            "at least 1, finite",
        )
    }
}

impl Default for OffNoseSpeedConfig {
    /// [`DEFAULT_OFF_NOSE_SPEED_MPS`], uncut within 20°, a third from 160°: ArduPilot's
    /// quadplane's.
    fn default() -> Self {
        OffNoseSpeedConfig {
            speed_mps: DEFAULT_OFF_NOSE_SPEED_MPS,
            free_rad: DEFAULT_OFF_NOSE_FREE_RAD,
            full_rad: DEFAULT_OFF_NOSE_FULL_RAD,
            tail_first_divisor: DEFAULT_OFF_NOSE_TAIL_FIRST_DIVISOR,
        }
    }
}

/// Caps a quadplane's waypoint speed in copter flight by how far the nose is off the bearing to
/// the waypoint, as ArduPilot's quadplane does (`QuadPlane::get_scaled_wp_speed`): uncut
/// within [`free_rad`](OffNoseSpeedConfig::free_rad) (20°), then
/// [`speed_mps`](OffNoseSpeedConfig::speed_mps) divided by a factor rising linearly from 1
/// there to [`tail_first_divisor`](OffNoseSpeedConfig::tail_first_divisor) (3) at
/// [`full_rad`](OffNoseSpeedConfig::full_rad) (160°) and past it. The waypoint's own cruise speed still
/// applies when lower.
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
/// let off_nose = OffNoseSpeed::new(OffNoseSpeedConfig {
///     speed_mps: 15.0,
///     ..OffNoseSpeedConfig::default()
/// });
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
    /// position; unchanged when off, within `free_rad` of the nose, or below the cap already. A
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

    /// The speed cap, in m/s, `error_rad` off the nose: infinite when off, within `free_rad`,
    /// or for a NaN error.
    fn cap_mps(&self, error_rad: f32) -> f32 {
        let OffNoseSpeedConfig {
            speed_mps,
            free_rad,
            full_rad,
            tail_first_divisor,
        } = self.config;
        // `>` is false for NaN.
        if !(speed_mps > 0.0 && error_rad > free_rad) {
            return f32::INFINITY;
        }
        // A full angle at or below the free one is a step to the whole cut.
        let span_rad = full_rad - free_rad;
        let share = if span_rad > 0.0 {
            ((error_rad - free_rad) / span_rad).min(1.0)
        } else {
            1.0
        };
        speed_mps / (1.0 + (tail_first_divisor - 1.0) * share)
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
