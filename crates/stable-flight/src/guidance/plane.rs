//! Fixed-wing guidance: [`PlaneGuidance`] turns pilot sticks and offboard commands into
//! [`Setpoint::Plane`] and [`Setpoint::PlaneRate`] for a
//! [`PlaneStabilizer`](crate::stabilizers::plane::PlaneStabilizer).
//!
//! | Control input | Setpoint |
//! |---|---|
//! | [`ControlInput::Attitude`] | [`Setpoint::Plane`]: roll stick × [`max_bank_rad`](PlaneGuidanceConfig::max_bank_rad), pitch stick × [`max_pitch_up_rad`](PlaneGuidanceConfig::max_pitch_up_rad) or [`max_pitch_down_rad`](PlaneGuidanceConfig::max_pitch_down_rad), throttle as thrust (ArduPlane's FBWA); turns coordinated, yaw stick unused |
//! | [`ControlInput::Rate`] | [`Setpoint::PlaneRate`]: sticks × [`max_rate_radps`](PlaneGuidanceConfig::max_rate_radps) (acro: rolls, loops), throttle as thrust |
//! | [`ControlInput::AbsoluteAttitude`] | [`Setpoint::Plane`]: roll, pitch, and thrust as given (heading unused) |
//! | [`ControlInput::AbsoluteRate`] | [`Setpoint::PlaneRate`]: rates and thrust as given |
//! | [`ControlInput::Waypoint`] | [`Setpoint::Plane`]: with a leg velocity, L1 along the line through the waypoint in its direction; without one (a route's final hold, an offboard point), a clockwise loiter of [`loiter_radius_m`](PlaneGuidanceConfig::loiter_radius_m) around it; bank `atan(a/g)` capped at [`max_bank_rad`](PlaneGuidanceConfig::max_bank_rad). Pitch and thrust hold the waypoint's altitude and its `cruise_speed_mps` as airspeed by total energy control ([`TecsConfig`]: thrust for the total energy, pitch for its balance); without an airspeed or a speed target, pitch holds the altitude (a PI loop) and thrust is [`cruise_thrust`](PlaneGuidanceConfig::cruise_thrust) plus a climb's share |
//! | [`ControlInput::Land`] | Neutral (level, zero thrust): a plane doesn't land here; a VTOL lands as a copter |
//!
//! Sticks and angles saturate (NaN as centered or level), throttle to `[0, 1]` (NaN as
//! none). It flies only [`FlightRegime::Plane`], and reports a `Copter` request as
//! [`ModeRefusal::Unsupported`](crate::ModeRefusal::Unsupported)
//! ([`FlightModeReport::plane_only`]).

use crate::controllers::{InvalidConfig, check, check_limit, check_positive};
use crate::math::{DEG, all_finite, assert_dt, clamp_symmetric, saturate_throttle, saturate_unit};
use crate::{
    BodyRateSetpoint, Configurable, ControlInput, FlightModeReport, FlightRegime, Guidance,
    PlaneSetpoint, Setpoint, Vec3, VehicleState, WaypointCommand,
};

mod navigation;
mod params;
mod tecs;
#[cfg(test)]
mod tests;

use navigation::{PiLoop, bank_for_accel_rad, loiter_accel_mps2, track_accel_mps2};
pub use tecs::TecsConfig;
use tecs::{Limits, Tecs};

/// [`PlaneGuidance`] settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlaneGuidanceConfig {
    /// Bank at full roll stick, in radians.
    pub max_bank_rad: f32,
    /// Pitch at full back stick, in radians.
    pub max_pitch_up_rad: f32,
    /// Pitch (down) at full forward stick, in radians, positive.
    pub max_pitch_down_rad: f32,
    /// Body rates at full stick in rate (acro) flight, roll/pitch/yaw, in rad/s.
    pub max_rate_radps: Vec3,
    /// Pitch for wings-level cruise, in radians.
    pub trim_pitch_rad: f32,
    /// Throttle for wings-level cruise at the trim airspeed, `[0, 1]`: total energy control's
    /// thrust at a steady energy.
    pub cruise_thrust: f32,
    /// L1 period, in seconds: how quickly the path is captured (shorter is tighter).
    pub l1_period_s: f32,
    /// L1 damping ratio.
    pub l1_damping: f32,
    /// Radius of the loiter around a waypoint with no leg, in meters.
    pub loiter_radius_m: f32,
    /// Without an airspeed: pitch per meter of altitude error, rad/m.
    pub altitude_p: f32,
    /// Without an airspeed: pitch per meter-second of accumulated altitude error.
    pub altitude_i: f32,
    /// Without an airspeed: the altitude loop's integral limit, in m·s.
    pub altitude_max_integral: f32,
    /// Total energy control of altitude and airspeed, with an airspeed and a speed target.
    pub tecs: TecsConfig,
    /// Least thrust waypoint flight commands, `[0, 1]`.
    pub min_thrust: f32,
    /// Most thrust waypoint flight commands, `[0, 1]`.
    pub max_thrust: f32,
    /// Without an airspeed: thrust added per radian of pitch up (a climb needs more).
    pub climb_thrust_per_rad: f32,
}

impl Default for PlaneGuidanceConfig {
    /// ArduPlane's FBWA limits (45° bank, +20°/−25° pitch, `mode_fbwa.cpp`), its acro rates
    /// (180°/s roll and pitch, yaw stick unused), level trim, and half throttle to cruise;
    /// for waypoints, ArduPilot's L1 damping (0.75) with a shorter period than its 17 s (12 s,
    /// for a small airframe), a 40 m loiter, ArduPilot's total energy control defaults
    /// ([`TecsConfig::default`]), and starting gains for the altitude loop without an airspeed
    /// (an airframe tunes its own).
    fn default() -> Self {
        PlaneGuidanceConfig {
            max_bank_rad: 45.0 * DEG,
            max_pitch_up_rad: 20.0 * DEG,
            max_pitch_down_rad: 25.0 * DEG,
            max_rate_radps: Vec3::new(180.0 * DEG, 180.0 * DEG, 0.0),
            trim_pitch_rad: 0.0,
            cruise_thrust: 0.5,
            l1_period_s: 12.0,
            l1_damping: 0.75,
            loiter_radius_m: 40.0,
            altitude_p: 0.05,
            altitude_i: 0.01,
            altitude_max_integral: 20.0,
            tecs: TecsConfig::default(),
            min_thrust: 0.2,
            max_thrust: 1.0,
            climb_thrust_per_rad: 0.5,
        }
    }
}

impl PlaneGuidanceConfig {
    /// `Ok` when the angle limits are in `[0, π/2]`, the rates non-negative, the trim pitch in
    /// `[−π/2, π/2]`, the cruise thrust in `[0, 1]`, and [`TecsConfig::validate`] passes.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        let quarter_turn = |angle: f32| (0.0..=core::f32::consts::FRAC_PI_2).contains(&angle);
        let in_range = "in [0, π/2] rad";
        check("max_bank_rad", quarter_turn(self.max_bank_rad), in_range)?;
        check(
            "max_pitch_up_rad",
            quarter_turn(self.max_pitch_up_rad),
            in_range,
        )?;
        check(
            "max_pitch_down_rad",
            quarter_turn(self.max_pitch_down_rad),
            in_range,
        )?;
        check_limit("max_rate_radps.x", self.max_rate_radps.x)?;
        check_limit("max_rate_radps.y", self.max_rate_radps.y)?;
        check_limit("max_rate_radps.z", self.max_rate_radps.z)?;
        let half = core::f32::consts::FRAC_PI_2;
        check(
            "trim_pitch_rad",
            (-half..=half).contains(&self.trim_pitch_rad),
            "in [−π/2, π/2] rad",
        )?;
        let unit = |value: f32| (0.0..=1.0).contains(&value);
        check("cruise_thrust", unit(self.cruise_thrust), "in [0, 1]")?;
        check_positive("l1_period_s", self.l1_period_s)?;
        check_positive("l1_damping", self.l1_damping)?;
        check_positive("loiter_radius_m", self.loiter_radius_m)?;
        check_limit("altitude_p", self.altitude_p)?;
        check_limit("altitude_i", self.altitude_i)?;
        check_limit("altitude_max_integral", self.altitude_max_integral)?;
        self.tecs.validate().map_err(|e| e.in_section("tecs"))?;
        check("min_thrust", unit(self.min_thrust), "in [0, 1]")?;
        check("max_thrust", unit(self.max_thrust), "in [0, 1]")?;
        check_limit("climb_thrust_per_rad", self.climb_thrust_per_rad)
    }
}

/// Fixed-wing guidance for piloted and offboard flight. See the [module docs](self).
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::plane::{PlaneGuidance, PlaneGuidanceConfig};
/// use stable_flight::{AttitudeCommand, ControlInput, FlightRegime, Guidance, Setpoint, VehicleState};
///
/// let mut guidance = PlaneGuidance::new(PlaneGuidanceConfig::default());
/// let full_right = ControlInput::Attitude(AttitudeCommand {
///     throttle: 0.7,
///     roll: 1.0,
///     ..Default::default()
/// });
/// let state = VehicleState::default();
/// let Setpoint::Plane(setpoint) = guidance.guide(0.01, FlightRegime::Plane, &full_right, &state)
/// else {
///     unreachable!("sticks give a plane attitude");
/// };
/// assert_eq!(setpoint.roll_rad, 45f32.to_radians());
/// assert_eq!(setpoint.thrust, 0.7);
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct PlaneGuidance {
    config: PlaneGuidanceConfig,
    requested: FlightRegime,
    /// Waypoint flight without an airspeed: altitude (→ pitch).
    altitude: PiLoop,
    /// Waypoint flight with an airspeed: total energy control.
    tecs: Tecs,
}

impl PlaneGuidance {
    /// Guidance with `config`.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`PlaneGuidanceConfig::validate`]).
    pub fn new(config: PlaneGuidanceConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid PlaneGuidanceConfig: {error}");
        }
        PlaneGuidance {
            config,
            requested: FlightRegime::Plane,
            altitude: PiLoop::default(),
            tecs: Tecs::default(),
        }
    }

    /// [`Guidance::guide`], with waypoint flight's pitch held within `pitch_cap_rad` either
    /// way when given: the forward transition's limits, so the energy control's pitch integral
    /// stops where the transition stops the pitch (`SLT_Transition::set_FW_roll_pitch`).
    pub(crate) fn guide_capped(
        &mut self,
        dt: f32,
        regime: FlightRegime,
        input: &ControlInput,
        state: &VehicleState,
        pitch_cap_rad: Option<f32>,
    ) -> Setpoint {
        assert_dt(dt);
        self.requested = regime;
        if !matches!(input, ControlInput::Waypoint(_)) {
            // The navigation loops are idle: clear them so waypoint flight starts fresh.
            self.altitude = PiLoop::default();
            self.tecs.reset();
        }
        let config = &self.config;
        match input {
            ControlInput::Attitude(sticks) => {
                Setpoint::Plane(self.sticks(sticks.roll, sticks.pitch, sticks.throttle))
            }
            ControlInput::Rate(sticks) => Setpoint::PlaneRate(BodyRateSetpoint {
                roll_radps: saturate_unit(sticks.roll_rate) * config.max_rate_radps.x,
                pitch_radps: saturate_unit(sticks.pitch_rate) * config.max_rate_radps.y,
                yaw_radps: saturate_unit(sticks.yaw_rate) * config.max_rate_radps.z,
                thrust: saturate_throttle(sticks.throttle),
                idle: false,
            }),
            ControlInput::AbsoluteAttitude(command) => Setpoint::Plane(PlaneSetpoint {
                roll_rad: command.roll_rad,
                pitch_rad: command.pitch_rad,
                thrust: saturate_throttle(command.thrust),
                transition: false,
            }),
            ControlInput::AbsoluteRate(command) => {
                let mut rates = BodyRateSetpoint::from(command);
                rates.thrust = saturate_throttle(command.thrust);
                Setpoint::PlaneRate(rates)
            }
            ControlInput::Waypoint(waypoint) => {
                Setpoint::Plane(self.navigate(dt, waypoint, state, pitch_cap_rad))
            }
            ControlInput::Land(_) => Setpoint::Plane(PlaneSetpoint::default()),
        }
    }

    /// Waypoint flight: bank from L1 along the leg (or a loiter without one); pitch and thrust
    /// from total energy control, or without an airspeed or a speed target, pitch from the
    /// altitude loop and thrust from cruise and the pitch.
    fn navigate(
        &mut self,
        dt: f32,
        waypoint: &WaypointCommand,
        state: &VehicleState,
        pitch_cap_rad: Option<f32>,
    ) -> PlaneSetpoint {
        let config = &self.config;
        let cap = |limit_rad: f32| pitch_cap_rad.map_or(limit_rad, |cap| limit_rad.min(cap));
        let (pitch_up_rad, pitch_down_rad) =
            (cap(config.max_pitch_up_rad), cap(config.max_pitch_down_rad));
        let (position_m, velocity_mps) = (state.position_m.xy(), state.velocity_mps.xy());
        let (period_s, damping) = (config.l1_period_s, config.l1_damping);
        let leg = waypoint.velocity_mps.filter(all_finite);
        let accel_mps2 = match leg {
            Some(leg) => track_accel_mps2(
                position_m,
                velocity_mps,
                waypoint.position_m,
                leg,
                period_s,
                damping,
            ),
            None => loiter_accel_mps2(
                position_m,
                velocity_mps,
                waypoint.position_m,
                config.loiter_radius_m,
                period_s,
                damping,
            ),
        };
        let roll_rad = clamp_symmetric(bank_for_accel_rad(accel_mps2), config.max_bank_rad);

        let (low, high) = (config.min_thrust, config.max_thrust.max(config.min_thrust));
        let trim = config.trim_pitch_rad;
        let target_mps = Some(waypoint.cruise_speed_mps).filter(|speed| *speed > 0.0);
        let (pitch_rad, thrust) = match (target_mps, state.valid_airspeed_mps()) {
            (Some(target_mps), Some(airspeed_mps)) => {
                self.altitude = PiLoop::default();
                let limits = Limits {
                    pitch_up_rad,
                    pitch_down_rad,
                    trim_pitch_rad: trim,
                    min_thrust: low,
                    max_thrust: high,
                    cruise_thrust: config.cruise_thrust,
                };
                self.tecs.step(
                    dt,
                    &config.tecs,
                    &limits,
                    (waypoint.altitude_m, target_mps),
                    airspeed_mps,
                    state,
                )
            }
            _ => {
                self.tecs.reset();
                // Pitch around trim, within the pitch limits; cruise thrust plus what a climb
                // needs.
                let pitch_rad = trim
                    + self.altitude.step(
                        dt,
                        waypoint.altitude_m - state.position_m.z,
                        config.altitude_p,
                        config.altitude_i,
                        config.altitude_max_integral,
                        (-pitch_down_rad - trim, pitch_up_rad - trim),
                        0.0,
                    );
                let thrust = (config.cruise_thrust
                    + config.climb_thrust_per_rad * (pitch_rad - trim))
                    .clamp(low, high);
                (pitch_rad, thrust)
            }
        };
        PlaneSetpoint {
            roll_rad,
            pitch_rad,
            thrust,
            transition: false,
        }
    }

    /// FBWA: sticks to bank and pitch, pitch asymmetric.
    fn sticks(&self, roll: f32, pitch: f32, throttle: f32) -> PlaneSetpoint {
        let config = &self.config;
        let pitch = saturate_unit(pitch);
        let pitch_limit = if pitch >= 0.0 {
            config.max_pitch_up_rad
        } else {
            config.max_pitch_down_rad
        };
        PlaneSetpoint {
            roll_rad: saturate_unit(roll) * config.max_bank_rad,
            pitch_rad: pitch * pitch_limit,
            thrust: saturate_throttle(throttle),
            transition: false,
        }
    }
}

impl Configurable for PlaneGuidance {
    type Config = PlaneGuidanceConfig;

    fn config(&self) -> &PlaneGuidanceConfig {
        &self.config
    }

    /// Validates with [`PlaneGuidanceConfig::validate`]; applies from the next tick.
    fn set_config(&mut self, config: PlaneGuidanceConfig) -> Result<(), InvalidConfig> {
        config.validate()?;
        self.config = config;
        Ok(())
    }
}

impl Guidance for PlaneGuidance {
    fn guide(
        &mut self,
        dt: f32,
        regime: FlightRegime,
        input: &ControlInput,
        state: &VehicleState,
    ) -> Setpoint {
        self.guide_capped(dt, regime, input, state, None)
    }

    /// Clears the navigation loops: integrals, demands, and underspeed.
    fn prime(&mut self, _state: &VehicleState) {
        self.altitude = PiLoop::default();
        self.tecs.reset();
    }

    /// Always plane; a `Copter` request is refused as unsupported.
    fn flight_mode(&self) -> Option<FlightModeReport> {
        Some(FlightModeReport::plane_only(self.requested))
    }
}
