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
//! | [`ControlInput::Waypoint`] | [`Setpoint::Plane`]: wings level at [`trim_pitch_rad`](PlaneGuidanceConfig::trim_pitch_rad) and [`cruise_thrust`](PlaneGuidanceConfig::cruise_thrust); navigation: TODO(#124) |
//! | [`ControlInput::Land`] | Neutral (level, zero thrust): a plane doesn't land here; a VTOL lands as a copter |
//!
//! Sticks and angles saturate (NaN as centered or level), throttle to `[0, 1]` (NaN as
//! none). It flies only [`FlightRegime::Plane`], and reports a `Copter` request as
//! [`ModeRefusal::Unsupported`](crate::ModeRefusal::Unsupported)
//! ([`FlightModeReport::plane_only`]).

use crate::controllers::{InvalidConfig, check, check_limit};
use crate::math::{saturate_throttle, saturate_unit};
use crate::{
    BodyRateSetpoint, Configurable, ControlInput, FlightModeReport, FlightRegime, Guidance,
    PlaneSetpoint, Setpoint, Vec3, VehicleState,
};

#[cfg(test)]
mod tests;

const DEG: f32 = core::f32::consts::PI / 180.0;

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
    /// Throttle for wings-level cruise, `[0, 1]`.
    pub cruise_thrust: f32,
}

impl Default for PlaneGuidanceConfig {
    /// ArduPlane's FBWA limits (45° bank, +20°/−25° pitch, `mode_fbwa.cpp`), its acro rates
    /// (180°/s roll and pitch, yaw stick unused), level trim, and half throttle to cruise.
    fn default() -> Self {
        PlaneGuidanceConfig {
            max_bank_rad: 45.0 * DEG,
            max_pitch_up_rad: 20.0 * DEG,
            max_pitch_down_rad: 25.0 * DEG,
            max_rate_radps: Vec3::new(180.0 * DEG, 180.0 * DEG, 0.0),
            trim_pitch_rad: 0.0,
            cruise_thrust: 0.5,
        }
    }
}

impl PlaneGuidanceConfig {
    /// `Ok` when the angle limits are in `[0, π/2]`, the rates non-negative, the trim pitch
    /// within the pitch limits, and the cruise thrust in `[0, 1]`.
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
        check(
            "trim_pitch_rad",
            (-self.max_pitch_down_rad..=self.max_pitch_up_rad).contains(&self.trim_pitch_rad),
            "within the pitch limits",
        )?;
        check(
            "cruise_thrust",
            (0.0..=1.0).contains(&self.cruise_thrust),
            "in [0, 1]",
        )
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
        _dt: f32,
        regime: FlightRegime,
        input: &ControlInput,
        _state: &VehicleState,
    ) -> Setpoint {
        self.requested = regime;
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
            }),
            ControlInput::AbsoluteAttitude(command) => Setpoint::Plane(PlaneSetpoint {
                roll_rad: command.roll_rad,
                pitch_rad: command.pitch_rad,
                thrust: saturate_throttle(command.thrust),
            }),
            ControlInput::AbsoluteRate(command) => {
                let mut rates = BodyRateSetpoint::from(command);
                rates.thrust = saturate_throttle(command.thrust);
                Setpoint::PlaneRate(rates)
            }
            ControlInput::Waypoint(_) => Setpoint::Plane(PlaneSetpoint {
                roll_rad: 0.0,
                pitch_rad: config.trim_pitch_rad,
                thrust: config.cruise_thrust,
            }),
            ControlInput::Land(_) => Setpoint::Plane(PlaneSetpoint::default()),
        }
    }

    /// Always plane; a `Copter` request is refused as unsupported.
    fn flight_mode(&self) -> Option<FlightModeReport> {
        Some(FlightModeReport::plane_only(self.requested))
    }
}
