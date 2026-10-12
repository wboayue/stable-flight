//! Guidance for multirotors such as the QuadCopter X: waypoint navigation and pilot-stick
//! interpretation, feeding [`CopterStabilizer`](crate::stabilizers::copter::CopterStabilizer).
//!
//! | Control input | Setpoint |
//! |---|---|
//! | [`ControlInput::Rate`] | [`Setpoint::Rate`], sticks passed through; the heading-hold target follows the vehicle |
//! | [`ControlInput::Attitude`] | [`Setpoint::Attitude`]: roll/pitch sticks scaled to [`max_stick_angles_rad`](CopterGuidanceConfig::max_stick_angles_rad); yaw stick slews a heading-hold target; throttle boosted by 1/cos(tilt) so tilting doesn't sink (none when inverted) |
//! | [`ControlInput::AbsoluteAttitude`] | [`Setpoint::Attitude`]: roll/pitch passed through (the stabilizer saturates them); the heading replaces the heading-hold target (NaN holds it); thrust saturated to `[0, 1]`, not boosted |
//! | [`ControlInput::Waypoint`] | [`Setpoint::Attitude`]: lateral position → velocity → acceleration → roll/pitch (every [`navigation_period_s`](CopterGuidanceConfig::navigation_period_s)), altitude loop → thrust (every tick), heading held; descents slewed at [`max_descent_mps`](CopterGuidanceConfig::max_descent_mps) |
//! | [`ControlInput::Land`] | [`Setpoint::Attitude`]: as a waypoint at the land position, with an altitude target that keeps descending at [`land_speed_mps`](CopterGuidanceConfig::land_speed_mps), at most [`LAND_LEASH_M`] below the vehicle |
//!
//! [`ThrustLimits`] bound the thrust while flying (all off by default): a floor, a sink guard
//! for piloted flight, and a ceiling on the tilt boost. A tailsitter's control surfaces need
//! the propwash, a sink turns the airflow against it, and differential thrust needs headroom.
//!
//! Switching modes doesn't jump the heading: attitude, absolute-attitude, waypoint, and land
//! flight share one heading-hold target, and rate flight keeps it on the vehicle's heading. The
//! altitude and lateral loops start fresh each time waypoint or land flight is entered, with
//! the altitude target starting from the vehicle's altitude.

use core::f32::consts::FRAC_PI_4;

use crate::nalgebra::UnitQuaternion;
use crate::{GRAVITY, Vec2};

use crate::commanders::touchdown::DESCENDING_MPS;
use crate::controllers::*;
use crate::controllers::{check, check_limit, check_positive, check_tilt, check_unit_interval};
use crate::{
    AbsoluteAttitudeCommand, AttitudeCommand, AttitudeSetpoint, Configurable, ControlInput,
    FlightRegime, Guidance, LandCommand, RateSetpoint, Saturation, Setpoint, VehicleState,
    WaypointCommand,
};

use super::rate_divider::RateDivider;
use crate::math::{
    all_finite, assert_dt, cap_magnitude, saturate_throttle, saturate_unit, tilt_boost, wrap_angle,
};

mod params;
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

/// Default [`CopterGuidanceConfig::max_descent_mps`]: fast enough for the figure-eight's
/// descending half (up to about 3 m/s at 7 m/s cruise).
pub const DEFAULT_MAX_DESCENT_MPS: f32 = 3.0;

/// Default [`CopterGuidanceConfig::land_speed_mps`], as PX4's `MPC_LAND_SPEED`.
pub const DEFAULT_LAND_SPEED_MPS: f32 = 0.7;

/// A piloted throttle (stick, rate, or absolute-attitude thrust) at or below this is idle:
/// [`ThrustLimits::min_thrust`] and the sink guard don't raise it, so the motors can idle on the ground
/// and the pilot can still cut them.
pub const IDLE_THROTTLE: f32 = 0.05;

/// How far, in meters, a land's altitude target may run below the vehicle. Once the ground
/// stops the vehicle, the altitude loop then settles on a thrust below hover instead of winding
/// down toward none.
pub const LAND_LEASH_M: f32 = 1.0;

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
    /// Fastest descent waypoint flight asks for, in m/s: a target below the vehicle is
    /// approached by an altitude target lowered at this rate, so a low waypoint descends
    /// instead of cutting thrust. Climbs aren't limited. It limits the target, not the achieved
    /// rate: the vehicle can exceed it briefly while catching up. Must be positive and finite.
    /// Defaults to [`DEFAULT_MAX_DESCENT_MPS`].
    pub max_descent_mps: f32,
    /// Descent rate of a land ([`ControlInput::Land`]), in m/s. Must be finite and above the
    /// touchdown detector's [`DESCENDING_MPS`] (0.2 m/s), so the detector sees the descent and
    /// the land ends in a disarm. Defaults to [`DEFAULT_LAND_SPEED_MPS`].
    pub land_speed_mps: f32,
    /// Thrust floor, sink guard, and tilt-boost ceiling; all off by default.
    pub thrust: ThrustLimits,
}

/// Bounds on [`CopterGuidance`]'s thrust while flying, for an airframe whose control comes
/// from the propwash (a tailsitter). The defaults bound nothing.
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::copter::{CopterGuidanceConfig, ThrustLimits};
///
/// let config = CopterGuidanceConfig {
///     thrust: ThrustLimits { min_thrust: 0.65, sink_limit_mps: 1.0, sink_gain: 0.15, max_boost_thrust: 0.9 },
///     ..Default::default()
/// };
/// assert!(config.validate().is_ok());
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrustLimits {
    /// Least linear thrust while flying, in `[0, 1)`; `0` (the default) is none. It keeps air
    /// over a tailsitter's control surfaces, whose authority comes from the propwash: at low
    /// throttle the elevons can't hold the nose up. ArduPilot's tailsitters floor the motors
    /// the same way (`Q_TAILSIT_MIN_VO`, `ArduPlane/tailsitter.cpp`, applied by
    /// `AP_MotorsTailsitter` only once spooled up). Every setpoint's thrust is raised to it,
    /// after the tilt boost (so even inverted), except a piloted throttle at or below
    /// [`IDLE_THROTTLE`], which passes through. Waypoint and land flight always get it. Set it
    /// below the altitude loop's `hover_thrust`, or waypoint flight can't descend and a land
    /// never touches down.
    pub min_thrust: f32,
    /// Sink rate, in m/s, beyond which piloted flight (attitude, rate, or absolute attitude,
    /// above [`IDLE_THROTTLE`]) gets more thrust: [`sink_gain`](Self::sink_gain) per m/s of
    /// sink past it, on top of the stick (and floor). Direct throttle can't hold a descent
    /// rate, and a tailsitter sinking a few m/s has air flowing up against its propwash, which
    /// takes the elevons' authority and drops the nose. Waypoint and land flight limit
    /// their own descents ([`CopterGuidanceConfig::max_descent_mps`]). Must be non-negative
    /// and finite; `0` (the default) disables it.
    pub sink_limit_mps: f32,
    /// Thrust added per m/s of sink past [`sink_limit_mps`](Self::sink_limit_mps), in s/m.
    /// Must be non-negative and finite. Defaults to 0.
    pub sink_gain: f32,
    /// Most thrust the attitude-mode tilt boost may raise the stick to, in `(0, 1]`. A stick
    /// above it passes through; the boost doesn't push past it, so a tilted vehicle keeps
    /// motor headroom for the moments (a tailsitter rolling on differential thrust).
    /// Defaults to 1 (no ceiling).
    pub max_boost_thrust: f32,
}

impl Default for ThrustLimits {
    /// No floor, no sink guard, and no boost ceiling.
    fn default() -> Self {
        ThrustLimits {
            min_thrust: 0.0,
            sink_limit_mps: 0.0,
            sink_gain: 0.0,
            max_boost_thrust: 1.0,
        }
    }
}

impl ThrustLimits {
    /// Checks that `min_thrust` is in `[0, 1)`, that `sink_limit_mps` and `sink_gain` are
    /// non-negative and finite, and that `max_boost_thrust` is in `(0, 1]`.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check(
            "min_thrust",
            (0.0..1.0).contains(&self.min_thrust),
            "in [0, 1)",
        )?;
        check_limit("sink_limit_mps", self.sink_limit_mps)?;
        check_limit("sink_gain", self.sink_gain)?;
        check_unit_interval("max_boost_thrust", self.max_boost_thrust)
    }
}

impl Default for CopterGuidanceConfig {
    /// The controller defaults (flight-checked on RealFlight's QuadCopterX), a 50 Hz lateral
    /// loop, a 27° waypoint tilt cap, 45° at full stick, a 3 m/s descent limit, a 0.7 m/s
    /// land, no thrust floor or sink guard, and no boost ceiling.
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
            max_descent_mps: DEFAULT_MAX_DESCENT_MPS,
            land_speed_mps: DEFAULT_LAND_SPEED_MPS,
            thrust: ThrustLimits::default(),
        }
    }
}

impl CopterGuidanceConfig {
    /// Checks both loop configs ([`AltitudeControllerConfig::validate`],
    /// [`LateralControllerConfig::validate`]; errors carry the loop as their `section`), the
    /// thrust limits ([`ThrustLimits::validate`], section `thrust`), that
    /// `navigation_period_s` and `max_stick_angles_rad` are non-negative and finite, that
    /// `max_tilt_rad` is in `(0, π/2)`, that `max_descent_mps` is positive and finite,
    /// and that `land_speed_mps` is finite and above [`DESCENDING_MPS`].
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
        check_tilt("max_tilt_rad", self.max_tilt_rad)?;
        check_positive("max_descent_mps", self.max_descent_mps)?;
        check(
            "land_speed_mps",
            self.land_speed_mps > DESCENDING_MPS && self.land_speed_mps.is_finite(),
            "above the touchdown detector's DESCENDING_MPS and finite",
        )?;
        self.thrust.validate().map_err(|e| e.in_section("thrust"))
    }
}

/// Copter guidance: altitude and lateral position loops for waypoints, and heading hold for
/// piloted attitude flight.
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
/// use stable_flight::{
///     AttitudeCommand, ControlInput, FlightRegime, Guidance, Setpoint, VehicleState,
/// };
///
/// let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
/// let state = VehicleState { heading_rad: 1.0, ..Default::default() };
/// guidance.prime(&state);
///
/// let input = ControlInput::Attitude(AttitudeCommand { throttle: 0.5, ..Default::default() });
/// let Setpoint::Attitude(setpoint) = guidance.guide(0.004, FlightRegime::Copter, &input, &state) else {
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
    /// Where the altitude loop is steered, in meters: the waypoint's altitude, or on the way
    /// down to it, slewed at the descent limit. `None` until waypoint or land flight latches
    /// it from the vehicle's altitude.
    altitude_target_m: Option<f32>,

    /// Heading-hold target for attitude, absolute-attitude, waypoint, and land commands.
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
            altitude_target_m: None,

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
        let old = &self.config;
        self.altitude
            .rescale_integral(&old.altitude, &config.altitude);
        self.lateral.rescale_integral(&old.lateral, &config.lateral);
        self.config = config;
        if self.tilt_rad.norm() > config.max_tilt_rad {
            self.tilt_capped = true;
        }
        self.tilt_rad = cap_magnitude(self.tilt_rad, config.max_tilt_rad);
        Ok(())
    }
}

impl Guidance for CopterGuidance {
    fn guide(
        &mut self,
        dt: f32,
        _regime: FlightRegime,
        input: &ControlInput,
        estimated_state: &VehicleState,
    ) -> Setpoint {
        assert_dt(dt);
        self.saturation = Saturation::default();

        if !matches!(input, ControlInput::Waypoint(_) | ControlInput::Land(_)) {
            // The position loops are idle: clear them so re-entering waypoint mode starts
            // fresh, runs the lateral loop at once instead of holding a stale tilt, carries no
            // integral from an earlier leg, and starts its altitude target from the vehicle.
            self.altitude = AltitudeController::default();
            self.lateral = LateralController::default();
            self.altitude_target_m = None;
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
                let mut setpoint: RateSetpoint = rate_command.into();
                setpoint.thrust = self.hold_piloted(
                    rate_command.throttle,
                    setpoint.thrust,
                    estimated_state.velocity_mps.z,
                );
                Setpoint::Rate(setpoint)
            }
            ControlInput::Waypoint(waypoint_command) => {
                Setpoint::Attitude(self.navigate(dt, waypoint_command, estimated_state))
            }
            ControlInput::AbsoluteAttitude(command) => {
                Setpoint::Attitude(self.absolute_attitude_setpoint(command, estimated_state))
            }
            ControlInput::Land(command) => {
                Setpoint::Attitude(self.land(dt, command, estimated_state))
            }
        }
    }

    /// Clears the altitude and lateral controller state, the altitude target, and any held
    /// waypoint tilt, and latches the current heading as the yaw-hold target.
    fn prime(&mut self, state: &VehicleState) {
        self.altitude = AltitudeController::default();
        self.lateral = LateralController::default();
        self.altitude_target_m = None;
        self.heading = state.heading_rad;
        self.lateral_rate.reset();
        self.tilt_rad = Vec2::zeros();
        self.tilt_capped = false;
        self.saturation = Saturation::default();
    }

    /// `tilt` when the waypoint tilt request was capped at
    /// [`max_tilt_rad`](CopterGuidanceConfig::max_tilt_rad) (held with the tilt between
    /// lateral-loop runs), and `thrust` when the altitude loop's thrust, attitude mode's
    /// boosted throttle, or an absolute attitude's thrust was clipped to `[0, 1]`, held at
    /// [`max_boost_thrust`](ThrustLimits::max_boost_thrust), raised to
    /// [`min_thrust`](ThrustLimits::min_thrust), or raised by the sink guard. A tilt
    /// beyond the stabilizer's limit in an absolute attitude is the stabilizer's to flag.
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
        self.set_heading_target(self.heading + yaw * HEADING_SLEW_RATE * dt, estimated_state);

        AttitudeSetpoint {
            roll_rad: saturate_unit(attitude_command.roll) * max_stick.x,
            pitch_rad: saturate_unit(attitude_command.pitch) * max_stick.y,
            heading_rad: self.heading,
            // Angle boost: hold the vertical thrust the stick asks for while tilted, as the
            // altitude loop does (at most 2× past 60°, none when inverted), up to
            // `max_boost_thrust`. Saturated to [0, 1]; NaN means none.
            thrust: {
                let throttle = attitude_command.throttle;
                let boosted = self.boost(throttle, &estimated_state.attitude);
                let boosted = self.clip_thrust(boosted);
                self.hold_piloted(throttle, boosted, estimated_state.velocity_mps.z)
            },
        }
    }

    /// Flies an absolute attitude: the heading becomes the heading-hold target, so stick
    /// flight afterward keeps it.
    fn absolute_attitude_setpoint(
        &mut self,
        command: &AbsoluteAttitudeCommand,
        estimated_state: &VehicleState,
    ) -> AttitudeSetpoint {
        // A non-finite heading holds the current target.
        let target_rad = if command.heading_rad.is_finite() {
            command.heading_rad
        } else {
            self.heading
        };
        self.set_heading_target(target_rad, estimated_state);
        AttitudeSetpoint {
            roll_rad: command.roll_rad,
            pitch_rad: command.pitch_rad,
            heading_rad: self.heading,
            thrust: {
                let thrust = self.clip_thrust(command.thrust);
                self.hold_piloted(command.thrust, thrust, estimated_state.velocity_mps.z)
            },
        }
    }

    /// Sets the heading-hold target to `target_rad`, wrapped.
    fn set_heading_target(&mut self, target_rad: f32, estimated_state: &VehicleState) {
        // Wrapped so sustained yaw input can't grow the target without bound; the yaw
        // controller works on the wrapped error either way.
        let target = wrap_angle(target_rad);
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
    /// thrust (every tick), toward an altitude target that descends no faster than
    /// `max_descent_mps`.
    fn navigate(
        &mut self,
        dt: f32,
        waypoint_command: &WaypointCommand,
        estimated_state: &VehicleState,
    ) -> AttitudeSetpoint {
        let target = self.descend_toward(
            dt,
            waypoint_command.altitude_m,
            estimated_state.position_m.z,
        );
        self.fly_to(dt, waypoint_command, target, estimated_state)
    }

    /// Lands at the command's position: flies there as to a waypoint, with an altitude target
    /// that keeps descending at `land_speed_mps`, leashed [`LAND_LEASH_M`] below the vehicle.
    fn land(
        &mut self,
        dt: f32,
        land_command: &LandCommand,
        estimated_state: &VehicleState,
    ) -> AttitudeSetpoint {
        let altitude_m = estimated_state.position_m.z;
        let speed_mps = self.config.land_speed_mps;
        // `f32::max` ignores a NaN side: a NaN altitude keeps lowering a latched target (the
        // altitude loop holds hover meanwhile). With nothing latched, the NaN target holds hover.
        let target_m = self
            .lowered_altitude_target_m(dt, speed_mps, altitude_m)
            .map_or(f32::NAN, |t| t.max(altitude_m - LAND_LEASH_M));
        self.altitude_target_m = Some(target_m).filter(|t| t.is_finite());
        let waypoint = WaypointCommand {
            position_m: land_command.position_m,
            ..Default::default()
        };
        let target = AltitudeTarget {
            altitude_m: target_m,
            velocity_mps: -speed_mps,
        };
        self.fly_to(dt, &waypoint, target, estimated_state)
    }

    /// The altitude loop's target on the way to `waypoint_m`: the waypoint itself when it is
    /// above the latched target or within one step of it, otherwise the latched target lowered
    /// by `max_descent_mps · dt`, with that descent rate fed forward.
    fn descend_toward(&mut self, dt: f32, waypoint_m: f32, altitude_m: f32) -> AltitudeTarget {
        let max_descent_mps = self.config.max_descent_mps;
        let level = AltitudeTarget {
            altitude_m: waypoint_m,
            velocity_mps: 0.0,
        };
        let Some(step_m) = self.lowered_altitude_target_m(dt, max_descent_mps, altitude_m) else {
            return level;
        };
        // `<` is false for NaN: a non-finite waypoint passes through, and the altitude loop
        // holds hover on it.
        let target = if waypoint_m < step_m {
            AltitudeTarget {
                altitude_m: step_m,
                velocity_mps: -max_descent_mps,
            }
        } else {
            level
        };
        self.altitude_target_m = Some(target.altitude_m).filter(|t| t.is_finite());
        target
    }

    /// The latched altitude target, or with none latched the vehicle's `altitude_m`, lowered
    /// by `rate_mps · dt`. `None` until there is a finite altitude to descend from; the
    /// altitude loop holds hover on a non-finite estimate meanwhile (input policy).
    fn lowered_altitude_target_m(&self, dt: f32, rate_mps: f32, altitude_m: f32) -> Option<f32> {
        self.altitude_target_m
            .or(Some(altitude_m).filter(|a| a.is_finite()))
            .map(|start_m| start_m - rate_mps * dt)
    }

    /// Flies toward the waypoint's position and the altitude `target`, holding the heading-hold
    /// target.
    fn fly_to(
        &mut self,
        dt: f32,
        waypoint_command: &WaypointCommand,
        target: AltitudeTarget,
        estimated_state: &VehicleState,
    ) -> AttitudeSetpoint {
        self.set_heading_target(self.heading, estimated_state);
        let tilt_rad = self.tilt_toward(dt, waypoint_command, estimated_state);

        let linear_thrust = self.altitude.control(
            &self.config.altitude,
            dt,
            target,
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
            thrust: self.floor_thrust(linear_thrust),
        }
    }

    /// Saturates a thrust request to `[0, 1]` (NaN means none), flagging `thrust` when it
    /// was outside.
    fn clip_thrust(&mut self, thrust: f32) -> f32 {
        // `|=`: the boost may have flagged it already this tick.
        self.saturation.thrust |= !(0.0..=1.0).contains(&thrust) && !thrust.is_nan();
        saturate_throttle(thrust)
    }

    /// The stick `throttle` boosted for the measured tilt, held at or below
    /// `max_boost_thrust` unless the stick itself is above it; flags `thrust` when held.
    fn boost(&mut self, throttle: f32, attitude: &UnitQuaternion<f32>) -> f32 {
        let boosted = throttle * tilt_boost(attitude);
        // `max` ignores a NaN throttle, and `>` is false for a NaN boost: NaN passes through.
        let ceiling = self.config.thrust.max_boost_thrust.max(throttle);
        if boosted > ceiling {
            self.saturation.thrust = true;
            ceiling
        } else {
            boosted
        }
    }

    /// Raises a piloted `thrust` to `min_thrust`, then adds the sink guard for the vertical
    /// speed `climb_mps` (up positive), unless the pilot's `throttle` is idle (at or below
    /// [`IDLE_THROTTLE`], or NaN).
    fn hold_piloted(&mut self, throttle: f32, thrust: f32, climb_mps: f32) -> f32 {
        if throttle > IDLE_THROTTLE {
            let floored = self.floor_thrust(thrust);
            self.guard_sink(floored, climb_mps)
        } else {
            thrust
        }
    }

    /// Adds `sink_gain` per m/s of sink past `sink_limit_mps`, saturated to `[0, 1]` and
    /// flagging `thrust`; a disabled guard (limit 0) or a NaN `climb_mps` adds nothing.
    fn guard_sink(&mut self, thrust: f32, climb_mps: f32) -> f32 {
        let limit_mps = self.config.thrust.sink_limit_mps;
        let excess_mps = -limit_mps - climb_mps;
        if limit_mps > 0.0 && excess_mps > 0.0 {
            self.saturation.thrust = true;
            saturate_throttle(thrust + self.config.thrust.sink_gain * excess_mps)
        } else {
            thrust
        }
    }

    /// Raises `thrust` to `min_thrust`, flagging `thrust` when it does.
    fn floor_thrust(&mut self, thrust: f32) -> f32 {
        let min_thrust = self.config.thrust.min_thrust;
        if thrust < min_thrust {
            self.saturation.thrust = true;
            min_thrust
        } else {
            thrust
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
