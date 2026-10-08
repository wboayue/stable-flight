//! Guidance for multirotors such as the QuadCopter X: waypoint navigation and pilot-stick
//! interpretation, feeding [`CopterStabilizer`](crate::stabilizers::copter::CopterStabilizer).
//!
//! | Command | Setpoint |
//! |---|---|
//! | [`Command::Rate`] | [`Setpoint::Rate`], sticks passed through |
//! | [`Command::Attitude`] | [`Setpoint::Attitude`]: roll/pitch sticks scaled to [`max_stick_angles_rad`](CopterGuidanceConfig::max_stick_angles_rad); yaw stick slews a heading-hold target |
//! | [`Command::Waypoint`] | [`Setpoint::Attitude`]: lateral position loop → acceleration → roll/pitch (every [`navigation_period_s`](CopterGuidanceConfig::navigation_period_s)), altitude loop → thrust (every tick), heading north |

use core::f32::consts::FRAC_PI_4;

use crate::{GRAVITY, Vec2};

use crate::controllers::*;
use crate::controllers::{check_limit, check_tilt};
use crate::{
    AttitudeCommand, AttitudeSetpoint, Command, Configurable, Guidance, RateSetpoint, Setpoint,
    VehicleState, WaypointCommand,
};

use super::rate_divider::RateDivider;
use crate::math::{assert_dt, cap_magnitude, saturate_unit, wrap_angle};

#[cfg(test)]
mod tests;

/// Heading-hold target slew rate at full yaw stick, in rad/s (~57°/s).
const HEADING_SLEW_RATE: f32 = 1.0;

/// Default [`CopterGuidanceConfig::max_tilt_rad`]: 27°, leaving thrust headroom to climb while
/// translating (60% of the default 45° stabilizer limit).
pub const DEFAULT_MAX_TILT_RAD: f32 = 27.0_f32.to_radians();

/// Default [`CopterGuidanceConfig::max_stick_angles_rad`] per axis: 45°, the default stabilizer
/// limit, so full stick is full tilt.
pub const DEFAULT_MAX_STICK_ANGLE_RAD: f32 = FRAC_PI_4;

/// Default [`CopterGuidanceConfig::navigation_period_s`]: 50 Hz, a fifth of the usual 250 Hz
/// inner loop.
pub const DEFAULT_NAVIGATION_PERIOD_S: f32 = 0.02;

/// Configuration for [CopterGuidance]: the altitude and lateral position loops and the
/// waypoint-flight limits.
///
/// Plain data: [`CopterGuidance::new`] validates it ([`validate`](Self::validate)), and the
/// guidance passes the loop configs to its controllers every tick.
///
/// # Examples
///
/// ```
/// use stable_flight::controllers::Gains;
/// use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
///
/// let mut config = CopterGuidanceConfig::default();
/// config.altitude.gains = Gains { p: 2.0, i: 0.2, d: 0.1 };
/// config.altitude.hover_thrust = 0.45; // airframe-specific
/// let guidance = CopterGuidance::new(config);
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CopterGuidanceConfig {
    /// Altitude loop: altitude error (m) → vertical acceleration (m/s²), fed to the thrust law
    /// with gravity and tilt compensation. Its `hover_thrust` defaults to
    /// [`DEFAULT_HOVER_THRUST`].
    pub altitude: AltitudeControllerConfig,
    /// Lateral position loop: horizontal position error (m) → acceleration (m/s²), flown as a
    /// tilt of `atan(|a| / g)` toward it.
    pub lateral: LateralControllerConfig,
    /// Seconds between runs of the lateral position loop; its roll/pitch output is held in
    /// between. `0.0` runs it every tick. Rounded to the nearest tick (at a 4 ms loop, 20 ms
    /// is every 5th tick); each run steps over the actual elapsed time. The altitude loop runs
    /// every tick, since its tilt compensation needs the current attitude. Must be
    /// non-negative and finite. Defaults to [`DEFAULT_NAVIGATION_PERIOD_S`].
    pub navigation_period_s: f32,
    /// Largest waypoint-flight tilt in radians, in `(0, π/2)`. Caps the magnitude of the
    /// combined roll/pitch request, so a far waypoint can't ask for full tilt while the
    /// altitude loop also wants full climb. It limits the request, not the achieved tilt: the
    /// attitude loop can overshoot it briefly, and the stabilizer's own `max_angles_rad` still
    /// applies. Piloted attitude flight is not capped. Defaults to [`DEFAULT_MAX_TILT_RAD`].
    pub max_tilt_rad: f32,
    /// Roll and pitch in radians that full stick (±1) commands in piloted attitude flight;
    /// the stick scales linearly. The stabilizer saturates to its own `max_angles_rad`, so a
    /// larger value only reaches that limit sooner. Must be non-negative and finite. Defaults
    /// to [`DEFAULT_MAX_STICK_ANGLE_RAD`] on both axes.
    pub max_stick_angles_rad: Vec2,
}

impl Default for CopterGuidanceConfig {
    /// The controller defaults (flight-checked on RealFlight's QuadCopterX), a 50 Hz lateral
    /// loop, a 27° waypoint tilt cap, and 45° at full stick.
    fn default() -> Self {
        CopterGuidanceConfig {
            altitude: AltitudeControllerConfig::default(),
            lateral: LateralControllerConfig::default(),
            navigation_period_s: DEFAULT_NAVIGATION_PERIOD_S,
            max_tilt_rad: DEFAULT_MAX_TILT_RAD,
            max_stick_angles_rad: Vec2::new(
                DEFAULT_MAX_STICK_ANGLE_RAD,
                DEFAULT_MAX_STICK_ANGLE_RAD,
            ),
        }
    }
}

impl CopterGuidanceConfig {
    /// Checks both loop configs ([`AltitudeControllerConfig::validate`],
    /// [`LateralControllerConfig::validate`]; errors carry the loop as their `section`), that
    /// `navigation_period_s` and `max_stick_angles_rad` are non-negative and finite, and that
    /// `max_tilt_rad` is in `(0, π/2)`.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::guidance::copter::CopterGuidanceConfig;
    ///
    /// let mut config = CopterGuidanceConfig::default();
    /// assert!(config.validate().is_ok());
    /// config.altitude.hover_thrust = 1.5;
    /// assert_eq!(config.validate().unwrap_err().to_string(), "altitude.hover_thrust must be in (0, 1]");
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        self.altitude
            .validate()
            .map_err(|e| e.in_section("altitude"))?;
        self.lateral
            .validate()
            .map_err(|e| e.in_section("lateral"))?;
        check_limit("navigation_period_s", self.navigation_period_s)?;
        check_limit("max_stick_angles_rad.x", self.max_stick_angles_rad.x)?;
        check_limit("max_stick_angles_rad.y", self.max_stick_angles_rad.y)?;
        check_tilt("max_tilt_rad", self.max_tilt_rad)
    }
}

/// Copter guidance: altitude and lateral position loops for waypoints, and heading hold for
/// piloted attitude flight.
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
/// use stable_flight::{AttitudeCommand, Command, Guidance, Setpoint, VehicleState};
///
/// let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
/// let state = VehicleState { heading_rad: 1.0, ..Default::default() };
/// guidance.prime(&state);
///
/// let command = Command::Attitude(AttitudeCommand { throttle: 0.5, ..Default::default() });
/// let Setpoint::Attitude(setpoint) = guidance.guide(0.004, &command, &state) else {
///     unreachable!("attitude commands yield attitude setpoints");
/// };
/// assert!((setpoint.heading_rad - 1.0).abs() < 1e-6); // holds the primed heading
/// assert_eq!(setpoint.thrust, 0.5);
/// ```
pub struct CopterGuidance {
    /// Loop configs and limits, passed into the controllers every tick.
    config: CopterGuidanceConfig,

    altitude: AltitudeController,
    lateral: LateralController,

    /// Heading-hold target for attitude commands.
    heading: f32,

    /// Paces the lateral loop.
    lateral_rate: RateDivider,
    /// The lateral loop's last roll/pitch in radians, held between its runs.
    tilt_rad: Vec2,
}

impl CopterGuidance {
    /// Creates a new [CopterGuidance] with the specified configuration.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`CopterGuidanceConfig::validate`]).
    pub fn new(config: CopterGuidanceConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid CopterGuidanceConfig: {error}");
        }
        CopterGuidance {
            config,
            altitude: AltitudeController::default(),
            lateral: LateralController::default(),

            heading: 0.0,

            lateral_rate: RateDivider::default(),
            tilt_rad: Vec2::zeros(),
        }
    }
}

impl Configurable for CopterGuidance {
    type Config = CopterGuidanceConfig;

    fn config(&self) -> &CopterGuidanceConfig {
        &self.config
    }

    /// Validates with [`CopterGuidanceConfig::validate`]. The heading-hold target and the
    /// lateral loop's timing are kept; a new `navigation_period_s` applies from the next tick,
    /// and the held waypoint tilt is re-capped to a lowered `max_tilt_rad` at once.
    fn set_config(&mut self, config: CopterGuidanceConfig) -> Result<(), InvalidConfig> {
        config.validate()?;
        self.config = config;
        self.tilt_rad = cap_magnitude(self.tilt_rad, config.max_tilt_rad);
        Ok(())
    }
}

impl Guidance for CopterGuidance {
    fn guide(&mut self, dt: f32, command: &Command, estimated_state: &VehicleState) -> Setpoint {
        assert_dt(dt);

        if !matches!(command, Command::Waypoint(_)) {
            // Re-entering waypoint mode runs the lateral loop at once instead of holding a
            // stale tilt.
            self.lateral_rate.reset();
        }

        match command {
            Command::Attitude(attitude_command) => {
                Setpoint::Attitude(self.attitude_setpoint(dt, attitude_command, estimated_state))
            }
            Command::Rate(rate_command) => Setpoint::Rate(RateSetpoint {
                roll_rate: rate_command.roll_rate,
                pitch_rate: rate_command.pitch_rate,
                yaw_rate: rate_command.yaw_rate,
                thrust: rate_command.throttle,
            }),
            Command::Waypoint(waypoint_command) => {
                Setpoint::Attitude(self.navigate(dt, waypoint_command, estimated_state))
            }
        }
    }

    /// Latches the current heading as the yaw-hold target, and drops any held waypoint tilt.
    fn prime(&mut self, state: &VehicleState) {
        self.heading = state.heading_rad;
        self.lateral_rate.reset();
    }
}

impl CopterGuidance {
    /// Scales the roll/pitch sticks to radians, passes throttle through, and slews the
    /// heading-hold target with the yaw stick.
    fn attitude_setpoint(
        &mut self,
        dt: f32,
        attitude_command: &AttitudeCommand,
        estimated_state: &VehicleState,
    ) -> AttitudeSetpoint {
        // Commands from input devices can be out of range or NaN; saturate rather than panic
        // (throttle saturates in the stabilizer; see `crate::math`).
        let yaw = saturate_unit(attitude_command.yaw);
        let max_stick = self.config.max_stick_angles_rad;

        // Wrapped so sustained yaw input can't grow the target without bound; the yaw
        // controller works on the wrapped error either way.
        let target = wrap_angle(self.heading + yaw * HEADING_SLEW_RATE * dt);
        if target.is_finite() {
            self.heading = target;
        } else if estimated_state.heading_rad.is_finite() {
            // A NaN target (e.g. primed from a NaN estimate) would otherwise disable heading
            // hold for good; re-latch from the current estimate once it's valid.
            self.heading = wrap_angle(estimated_state.heading_rad);
        }

        AttitudeSetpoint {
            roll_rad: saturate_unit(attitude_command.roll) * max_stick.x,
            pitch_rad: saturate_unit(attitude_command.pitch) * max_stick.y,
            heading_rad: self.heading,
            thrust: attitude_command.throttle,
        }
    }

    /// Flies toward the waypoint, holding heading north: the lateral position loop gives
    /// roll/pitch (once per navigation period, held in between) and the altitude loop gives
    /// thrust (every tick).
    fn navigate(
        &mut self,
        dt: f32,
        waypoint_command: &WaypointCommand,
        estimated_state: &VehicleState,
    ) -> AttitudeSetpoint {
        let tilt_rad = self.tilt_toward(dt, waypoint_command, estimated_state);

        let linear_thrust = self.altitude.control(
            &self.config.altitude,
            dt,
            AltitudeTarget {
                altitude_m: waypoint_command.altitude_m,
                velocity_mps: 0.0,
            },
            AltitudeEstimate {
                altitude_m: estimated_state.position_m.z,
                velocity_mps: estimated_state.velocity_mps.z,
                attitude: estimated_state.attitude,
            },
        );

        AttitudeSetpoint {
            roll_rad: tilt_rad.x,
            pitch_rad: tilt_rad.y,
            heading_rad: 0.0,
            thrust: linear_thrust,
        }
    }

    /// Roll/pitch in radians toward the waypoint from the lateral loop, run when
    /// [`lateral_rate`](Self::lateral_rate) is due and held otherwise.
    fn tilt_toward(
        &mut self,
        dt: f32,
        waypoint_command: &WaypointCommand,
        estimated_state: &VehicleState,
    ) -> Vec2 {
        let Some(step) = self.lateral_rate.due(dt, self.config.navigation_period_s) else {
            return self.tilt_rad;
        };

        let desired_position =
            Vec2::new(waypoint_command.position_m.x, waypoint_command.position_m.y);
        let current_position =
            Vec2::new(estimated_state.position_m.x, estimated_state.position_m.y);
        let velocity = Vec2::new(
            estimated_state.velocity_mps.x,
            estimated_state.velocity_mps.y,
        );

        // A non-finite target or estimate gets level for this tick only (input policy). The
        // loop doesn't run, so the neutral output isn't held for a period and the next tick
        // retries over the combined time.
        let error = desired_position - current_position;
        if ![error.x, error.y, velocity.x, velocity.y]
            .iter()
            .all(|v| v.is_finite())
        {
            return Vec2::zeros();
        }

        let accel_mps2 = self.lateral.control(
            &self.config.lateral,
            step,
            desired_position,
            current_position,
            velocity,
        );
        self.tilt_rad = cap_magnitude(tilt_for_acceleration(accel_mps2), self.config.max_tilt_rad);
        self.lateral_rate.ran();
        self.tilt_rad
    }
}

/// Roll/pitch in radians (heading north) that points thrust to give horizontal acceleration
/// `accel_mps2` (north, east) at constant altitude: a tilt of `atan(|a| / g)` toward it. North
/// is nose down (negative pitch), east is right roll. The tilt is a vector of that length,
/// split between the axes, exact for single-axis requests and close for diagonal ones at
/// waypoint tilts. A non-finite acceleration gives level.
fn tilt_for_acceleration(accel_mps2: Vec2) -> Vec2 {
    let accel_norm = accel_mps2.norm();
    if !accel_norm.is_finite() || accel_norm == 0.0 {
        return Vec2::zeros();
    }
    let tilt_rad = libm::atanf(accel_norm / GRAVITY);
    Vec2::new(accel_mps2.y, -accel_mps2.x) * (tilt_rad / accel_norm)
}
