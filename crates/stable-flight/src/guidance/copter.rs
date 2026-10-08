//! Guidance for multirotors such as the QuadCopter X: waypoint navigation and pilot-stick
//! interpretation, feeding [`CopterStabilizer`](crate::stabilizers::copter::CopterStabilizer).
//!
//! | Control input | Setpoint |
//! |---|---|
//! | [`ControlInput::Rate`] | [`Setpoint::Rate`], sticks passed through; the heading-hold target follows the vehicle |
//! | [`ControlInput::Attitude`] | [`Setpoint::Attitude`]: roll/pitch sticks scaled to [`max_stick_angles_rad`](CopterGuidanceConfig::max_stick_angles_rad); yaw stick slews a heading-hold target; throttle boosted by 1/cos(tilt) so tilting doesn't sink (none when inverted) |
//! | [`ControlInput::Waypoint`] | [`Setpoint::Attitude`]: lateral position → velocity → acceleration → roll/pitch (every [`navigation_period_s`](CopterGuidanceConfig::navigation_period_s)), altitude loop → thrust (every tick), heading held |
//!
//! Switching modes doesn't jump the heading: attitude and waypoint flight share one
//! heading-hold target, and rate flight keeps it on the vehicle's heading. The altitude and
//! lateral loops start fresh each time waypoint flight is entered.

use core::f32::consts::FRAC_PI_4;

use crate::{GRAVITY, Vec2};

use crate::controllers::*;
use crate::controllers::{check_limit, check_tilt};
use crate::{
    AttitudeCommand, AttitudeSetpoint, Configurable, ControlInput, Guidance, RateSetpoint,
    Saturation, Setpoint, VehicleState, WaypointCommand,
};

use super::rate_divider::RateDivider;
use crate::math::{
    all_finite, assert_dt, cap_magnitude, saturate_throttle, saturate_unit, tilt_boost, wrap_angle,
};

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
    /// Lateral loop: horizontal position error (m) → velocity (m/s, capped at the waypoint's
    /// `cruise_speed_mps`) → acceleration (m/s²), flown as a tilt of `atan(|a| / g)` toward it.
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
/// use stable_flight::{AttitudeCommand, ControlInput, Guidance, Setpoint, VehicleState};
///
/// let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
/// let state = VehicleState { heading_rad: 1.0, ..Default::default() };
/// guidance.prime(&state);
///
/// let input = ControlInput::Attitude(AttitudeCommand { throttle: 0.5, ..Default::default() });
/// let Setpoint::Attitude(setpoint) = guidance.guide(0.004, &input, &state) else {
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

    /// Heading-hold target for attitude and waypoint commands.
    heading: f32,

    /// Paces the lateral loop.
    lateral_rate: RateDivider,
    /// The lateral loop's last roll/pitch in radians, held between its runs.
    tilt_rad: Vec2,
    /// The held tilt was capped at `max_tilt_rad` when the lateral loop last ran.
    tilt_capped: bool,

    /// What the last `guide` call clipped.
    saturation: Saturation,
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
            tilt_capped: false,
            saturation: Saturation::default(),
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
    fn guide(&mut self, dt: f32, input: &ControlInput, estimated_state: &VehicleState) -> Setpoint {
        assert_dt(dt);
        self.saturation = Saturation::default();

        if !matches!(input, ControlInput::Waypoint(_)) {
            // The position loops are idle: clear them so re-entering waypoint mode starts
            // fresh, runs the lateral loop at once instead of holding a stale tilt, and
            // carries no integral from an earlier leg.
            self.altitude = AltitudeController::default();
            self.lateral = LateralController::default();
            self.lateral_rate.reset();
        }

        match input {
            ControlInput::Attitude(attitude_command) => {
                Setpoint::Attitude(self.attitude_setpoint(dt, attitude_command, estimated_state))
            }
            ControlInput::Rate(rate_command) => {
                // Track the vehicle so attitude or waypoint flight holds the heading it has
                // when rate flight ends, not the one it had when rate flight began.
                if estimated_state.heading_rad.is_finite() {
                    self.heading = wrap_angle(estimated_state.heading_rad);
                }
                Setpoint::Rate(RateSetpoint {
                    roll_rate: rate_command.roll_rate,
                    pitch_rate: rate_command.pitch_rate,
                    yaw_rate: rate_command.yaw_rate,
                    thrust: rate_command.throttle,
                })
            }
            ControlInput::Waypoint(waypoint_command) => {
                Setpoint::Attitude(self.navigate(dt, waypoint_command, estimated_state))
            }
        }
    }

    /// Clears the altitude and lateral controller state and any held waypoint tilt, and
    /// latches the current heading as the yaw-hold target.
    fn prime(&mut self, state: &VehicleState) {
        self.altitude = AltitudeController::default();
        self.lateral = LateralController::default();
        self.heading = state.heading_rad;
        self.lateral_rate.reset();
        self.tilt_rad = Vec2::zeros();
        self.tilt_capped = false;
        self.saturation = Saturation::default();
    }

    /// `tilt` when the waypoint tilt request was capped at
    /// [`max_tilt_rad`](CopterGuidanceConfig::max_tilt_rad) (held with the tilt between
    /// lateral-loop runs), and `thrust` when the altitude loop's thrust, or attitude mode's
    /// boosted throttle, was clipped to `[0, 1]`.
    fn saturation(&self) -> Saturation {
        self.saturation
    }
}

impl CopterGuidance {
    /// Scales the roll/pitch sticks to radians, boosts throttle for the measured tilt, and
    /// slews the heading-hold target with the yaw stick.
    fn attitude_setpoint(
        &mut self,
        dt: f32,
        attitude_command: &AttitudeCommand,
        estimated_state: &VehicleState,
    ) -> AttitudeSetpoint {
        // Commands from input devices can be out of range or NaN; saturate rather than panic
        // (see `crate::math`).
        let yaw = saturate_unit(attitude_command.yaw);
        let max_stick = self.config.max_stick_angles_rad;
        self.slew_heading(yaw * HEADING_SLEW_RATE * dt, estimated_state);

        AttitudeSetpoint {
            roll_rad: saturate_unit(attitude_command.roll) * max_stick.x,
            pitch_rad: saturate_unit(attitude_command.pitch) * max_stick.y,
            heading_rad: self.heading,
            // Angle boost: hold the vertical thrust the stick asks for while tilted, as the
            // altitude loop does (at most 2× past 60°, none when inverted). Saturated to
            // [0, 1]; NaN means none.
            thrust: self
                .clip_thrust(attitude_command.throttle * tilt_boost(&estimated_state.attitude)),
        }
    }

    /// Moves the heading-hold target by `delta_rad`.
    fn slew_heading(&mut self, delta_rad: f32, estimated_state: &VehicleState) {
        // Wrapped so sustained yaw input can't grow the target without bound; the yaw
        // controller works on the wrapped error either way.
        let target = wrap_angle(self.heading + delta_rad);
        if target.is_finite() {
            self.heading = target;
        } else if estimated_state.heading_rad.is_finite() {
            // A NaN target (e.g. primed from a NaN estimate) would otherwise disable heading
            // hold for good; re-latch from the current estimate once it's valid.
            self.heading = wrap_angle(estimated_state.heading_rad);
        }
    }

    /// Flies toward the waypoint, holding the heading-hold target: the lateral loop gives
    /// roll/pitch (once per navigation period, held in between) and the altitude loop gives
    /// thrust (every tick).
    fn navigate(
        &mut self,
        dt: f32,
        waypoint_command: &WaypointCommand,
        estimated_state: &VehicleState,
    ) -> AttitudeSetpoint {
        self.slew_heading(0.0, estimated_state);
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

        // The altitude loop clamps its thrust to [0, 1]; landing on a bound means it clipped.
        self.saturation.thrust = linear_thrust <= 0.0 || linear_thrust >= 1.0;
        self.saturation.tilt = self.tilt_capped;

        AttitudeSetpoint {
            roll_rad: tilt_rad.x,
            pitch_rad: tilt_rad.y,
            heading_rad: self.heading,
            thrust: linear_thrust,
        }
    }

    /// Saturates a thrust request to `[0, 1]` (NaN means none), flagging `thrust` when it
    /// was outside.
    fn clip_thrust(&mut self, thrust: f32) -> f32 {
        self.saturation.thrust = !(0.0..=1.0).contains(&thrust) && !thrust.is_nan();
        saturate_throttle(thrust)
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

        let target = LateralTarget {
            position_m: waypoint_command.position_m,
            velocity_mps: waypoint_command.velocity_mps.unwrap_or_else(Vec2::zeros),
            max_speed_mps: waypoint_command.cruise_speed_mps,
        };
        let estimate = LateralEstimate {
            position_m: estimated_state.position_m.xy(),
            velocity_mps: estimated_state.velocity_mps.xy(),
        };
        let heading_rad = estimated_state.heading_rad;

        // A non-finite target or estimate gets level for this tick only (input policy). The
        // loop doesn't run, so the neutral output isn't held for a period and the next tick
        // retries over the combined time.
        let error = target.position_m - estimate.position_m;
        if !(all_finite(&error)
            && all_finite(&target.velocity_mps)
            && all_finite(&estimate.velocity_mps)
            && heading_rad.is_finite())
        {
            self.tilt_capped = false;
            return Vec2::zeros();
        }

        let accel_mps2 = self
            .lateral
            .control(&self.config.lateral, step, target, estimate);
        let requested = tilt_for_acceleration(accel_mps2, heading_rad);
        self.tilt_capped = requested.norm() > self.config.max_tilt_rad;
        self.tilt_rad = cap_magnitude(requested, self.config.max_tilt_rad);
        self.lateral_rate.ran();
        self.tilt_rad
    }
}

/// Roll/pitch in radians, at `heading_rad`, that points thrust to give horizontal
/// acceleration `accel_mps2` (north, east) at constant altitude: a tilt of `atan(|a| / g)`
/// toward it. Ahead is nose down (negative pitch), to the right is right roll. The tilt is a
/// vector of that length, split between the axes, exact for single-axis requests and close for
/// diagonal ones at waypoint tilts. A non-finite acceleration or heading gives level.
fn tilt_for_acceleration(accel_mps2: Vec2, heading_rad: f32) -> Vec2 {
    let accel_norm = accel_mps2.norm();
    if !accel_norm.is_finite() || accel_norm == 0.0 || !heading_rad.is_finite() {
        return Vec2::zeros();
    }
    let tilt_rad = libm::atanf(accel_norm / GRAVITY);
    // Rotate (north, east) into (ahead, right) of the heading.
    let (sin, cos) = (libm::sinf(heading_rad), libm::cosf(heading_rad));
    let ahead = accel_mps2.x * cos + accel_mps2.y * sin;
    let right = -accel_mps2.x * sin + accel_mps2.y * cos;
    Vec2::new(right, -ahead) * (tilt_rad / accel_norm)
}
