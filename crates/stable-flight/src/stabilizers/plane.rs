//! Fixed-wing stabilization: [`PlaneStabilizer`] tracks a [`Setpoint::Plane`] attitude or a
//! [`Setpoint::PlaneRate`] body rate with control surfaces, scaled for airspeed.
//!
//! | Setpoint | Behavior |
//! |---|---|
//! | [`Setpoint::Plane`] | Bank and pitch (saturated to [`max_bank_rad`](PlaneStabilizerConfig::max_bank_rad) / [`max_pitch_rad`](PlaneStabilizerConfig::max_pitch_rad), pitch to ±π/2 for a [`transition`](PlaneSetpoint::transition) ramp) → rate targets = error / time constant, plus the coordinated-turn rates for the current bank and airspeed → rate loop |
//! | [`Setpoint::PlaneRate`] | Absolute body rates (saturated to [`max_body_rate_radps`](PlaneStabilizerConfig::max_body_rate_radps)) → rate loop, at any attitude: acro, rolls, loops |
//! | others | Neutral (zero thrust and moments) |
//!
//! The structure is PX4's fixed-wing attitude and rate control
//! (`FixedwingAttitudeControl.cpp`, `FixedwingRateControl.cpp`), described, not copied:
//!
//! - The attitude error is the yaw-free part of the reduced attitude error the copter uses:
//!   the rotation that turns the body `z` axis onto the setpoint's, in body axes, so it's
//!   defined at any attitude, inverted included.
//! - A level turn at bank φ and speed V turns at ψ̇ = g·tan φ / V, which in body axes is a pitch
//!   rate ψ̇·sin φ·cos θ and a yaw rate ψ̇·cos φ·cos θ; both are fed forward so the wing turns
//!   without slipping or losing its nose, faded out between 70° and 75° of bank (PX4's
//!   turn-coordination fade).
//! - The rate loop ([`PlaneRateController`]) scales by airspeed and freezes its integral
//!   below [`min_airspeed_mps`](PlaneStabilizerConfig::min_airspeed_mps). Without an airspeed
//!   it assumes trim airspeed (PX4).
//!
//! Unlike the copter's, the integrals aren't cleared at zero thrust: gliding is normal flight.

use crate::controllers::{
    InvalidConfig, PlaneRateController, PlaneRateControllerConfig, check, check_limit,
    check_positive,
};
use crate::math::{
    DEG, assert_dt, attitude_error, bank_rad, clamp_symmetric, pitch_limit_rad, saturate_rates,
    saturate_symmetric, thrust_curve,
};
use crate::{
    BodyRateSetpoint, Configurable, ControlOutput, GRAVITY, PlaneSetpoint, Saturation, Setpoint,
    Stabilizer, Vec2, Vec3, VehicleState,
};

mod params;
#[cfg(test)]
mod tests;

/// Bank where turn coordination starts to fade, and where it's gone (PX4's 70°–75°).
const COORDINATION_FADE_RAD: (f32, f32) = (70.0 * DEG, 75.0 * DEG);

/// [`PlaneStabilizer`] settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlaneStabilizerConfig {
    /// The rate loop's gains and integral limit.
    pub rate: PlaneRateControllerConfig,
    /// How fast the attitude loops close their error, roll (`x`) and pitch (`y`), in s: the
    /// rate target is the error divided by it.
    pub time_constant_s: Vec2,
    /// Largest roll (`x`) and pitch (`y`) rate the attitude loops ask for, in rad/s.
    pub max_attitude_rate_radps: Vec2,
    /// Largest body rate an absolute-rate setpoint gets, per axis, in rad/s.
    pub max_body_rate_radps: Vec3,
    /// Largest bank an attitude setpoint gets, in radians.
    pub max_bank_rad: f32,
    /// Largest pitch, up or down, an attitude setpoint gets, in radians; a
    /// [`transition`](PlaneSetpoint::transition) ramp's isn't held to it.
    pub max_pitch_rad: f32,
    /// The airspeed the gains are tuned at (scale 1), in m/s; assumed without an airspeed.
    pub trim_airspeed_mps: f32,
    /// Below this airspeed the scale stops growing and the rate integrals hold, in m/s.
    pub min_airspeed_mps: f32,
    /// Above this airspeed the scale stops shrinking, in m/s.
    pub max_airspeed_mps: f32,
    /// Linear throttle → motor command exponent (1 passes throttle through). A tailsitter uses
    /// its hover stabilizer's, so a handoff keeps motor commands continuous.
    pub thrust_curve_exponent: f32,
}

impl Default for PlaneStabilizerConfig {
    /// PX4's fixed-wing defaults (time constants 0.4 s, attitude rate limits 70°/s roll and
    /// 60°/s pitch, `fw_att_control_params.yaml`), acro up to 180°/s roll and pitch and 60°/s
    /// yaw, 50° bank and 30° pitch, airspeeds 10/15/25 m/s, and throttle passed through.
    /// Placeholders until an airframe is tuned.
    fn default() -> Self {
        PlaneStabilizerConfig {
            rate: PlaneRateControllerConfig::default(),
            time_constant_s: Vec2::new(0.4, 0.4),
            max_attitude_rate_radps: Vec2::new(70.0 * DEG, 60.0 * DEG),
            max_body_rate_radps: Vec3::new(180.0 * DEG, 180.0 * DEG, 60.0 * DEG),
            max_bank_rad: 50.0 * DEG,
            max_pitch_rad: 30.0 * DEG,
            trim_airspeed_mps: 15.0,
            min_airspeed_mps: 10.0,
            max_airspeed_mps: 25.0,
            thrust_curve_exponent: 1.0,
        }
    }
}

impl PlaneStabilizerConfig {
    /// `Ok` when every field is in range: positive time constants, non-negative rate limits,
    /// bank and pitch limits in `[0, π]`, positive airspeeds, and a positive thrust exponent.
    /// Each is checked on its own; the scale clamps the airspeed to `[min, max]` with a `max`
    /// below `min` read as `min`, so out-of-order airspeeds degrade, not fail.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        self.rate.validate().map_err(|e| e.in_section("rate"))?;
        check_positive("time_constant_s.x", self.time_constant_s.x)?;
        check_positive("time_constant_s.y", self.time_constant_s.y)?;
        check_limit("max_attitude_rate_radps.x", self.max_attitude_rate_radps.x)?;
        check_limit("max_attitude_rate_radps.y", self.max_attitude_rate_radps.y)?;
        check_limit("max_body_rate_radps.x", self.max_body_rate_radps.x)?;
        check_limit("max_body_rate_radps.y", self.max_body_rate_radps.y)?;
        check_limit("max_body_rate_radps.z", self.max_body_rate_radps.z)?;
        let half_turn = |angle: f32| (0.0..=core::f32::consts::PI).contains(&angle);
        check(
            "max_bank_rad",
            half_turn(self.max_bank_rad),
            "in [0, π] rad",
        )?;
        check(
            "max_pitch_rad",
            half_turn(self.max_pitch_rad),
            "in [0, π] rad",
        )?;
        check_positive("trim_airspeed_mps", self.trim_airspeed_mps)?;
        check_positive("min_airspeed_mps", self.min_airspeed_mps)?;
        check_positive("max_airspeed_mps", self.max_airspeed_mps)?;
        check_positive("thrust_curve_exponent", self.thrust_curve_exponent)
    }
}

/// A fixed-wing stabilizer: attitude and rate loops on the control surfaces, with turn
/// coordination and airspeed scaling. See the [module docs](self).
///
/// # Examples
///
/// ```
/// use stable_flight::stabilizers::plane::{PlaneStabilizer, PlaneStabilizerConfig};
/// use stable_flight::{PlaneSetpoint, Setpoint, Stabilizer, VehicleState};
///
/// let mut stabilizer = PlaneStabilizer::new(PlaneStabilizerConfig::default());
/// let bank_right = Setpoint::Plane(PlaneSetpoint {
///     roll_rad: 0.3,
///     thrust: 0.6,
///     ..Default::default()
/// });
/// let output = stabilizer.stabilize(0.004, &bank_right, &VehicleState::default());
/// assert!(output.moments.x > 0.0); // roll right
/// assert_eq!(output.thrust, 0.6);
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct PlaneStabilizer {
    config: PlaneStabilizerConfig,
    rate: PlaneRateController,
    saturation: Saturation,
}

impl PlaneStabilizer {
    /// A stabilizer with `config`.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`PlaneStabilizerConfig::validate`]).
    pub fn new(config: PlaneStabilizerConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid PlaneStabilizerConfig: {error}");
        }
        PlaneStabilizer {
            config,
            rate: PlaneRateController::default(),
            saturation: Saturation::default(),
        }
    }

    /// Rate targets for an attitude setpoint: the error over the time constant, clamped, plus
    /// the coordinated-turn rates.
    fn attitude_rates(&mut self, setpoint: &PlaneSetpoint, state: &VehicleState) -> Vec3 {
        let config = &self.config;
        let max_pitch_rad = pitch_limit_rad(config.max_pitch_rad, setpoint.transition);
        // `>` is false for NaN, which saturates to level.
        self.saturation.tilt = setpoint.roll_rad.abs() > config.max_bank_rad
            || setpoint.pitch_rad.abs() > max_pitch_rad;
        let roll_rad = saturate_symmetric(setpoint.roll_rad, config.max_bank_rad);
        let pitch_rad = saturate_symmetric(setpoint.pitch_rad, max_pitch_rad);
        let error = attitude_error(&state.attitude, roll_rad, pitch_rad, f32::NAN);
        let correction = Vec2::new(
            clamp_symmetric(
                error.x / config.time_constant_s.x,
                config.max_attitude_rate_radps.x,
            ),
            clamp_symmetric(
                error.y / config.time_constant_s.y,
                config.max_attitude_rate_radps.y,
            ),
        );
        coordinated_turn_radps(state, self.airspeed_mps(state))
            + Vec3::new(correction.x, correction.y, 0.0)
    }

    /// Rate targets for an absolute-rate setpoint, saturated (NaN as zero).
    fn body_rates(&self, setpoint: &BodyRateSetpoint) -> Vec3 {
        saturate_rates(
            Vec3::new(
                setpoint.roll_radps,
                setpoint.pitch_radps,
                setpoint.yaw_radps,
            ),
            self.config.max_body_rate_radps,
        )
    }

    /// The airspeed the loops work with: the state's when valid, clamped to the configured
    /// range, else trim.
    fn airspeed_mps(&self, state: &VehicleState) -> f32 {
        let config = &self.config;
        state
            .valid_airspeed_mps()
            .map_or(config.trim_airspeed_mps, |airspeed_mps| {
                let max_mps = config.max_airspeed_mps.max(config.min_airspeed_mps);
                airspeed_mps.clamp(config.min_airspeed_mps, max_mps)
            })
    }

    /// The rate loop at `desired_radps`, scaled for airspeed, and the thrust.
    fn track(
        &mut self,
        dt: f32,
        desired_radps: Vec3,
        thrust: f32,
        state: &VehicleState,
    ) -> ControlOutput {
        let config = &self.config;
        let scale = config.trim_airspeed_mps / self.airspeed_mps(state);
        // Without an airspeed there's nothing to hold the integral for.
        let integrate = state
            .valid_airspeed_mps()
            .is_none_or(|airspeed_mps| airspeed_mps >= config.min_airspeed_mps);
        let moments = self.rate.control(
            &config.rate,
            dt,
            desired_radps,
            state.angular_velocity_radps,
            scale,
            integrate,
        );
        self.saturation.thrust = !(0.0..=1.0).contains(&thrust) && !thrust.is_nan();
        ControlOutput {
            thrust: thrust_curve(thrust, config.thrust_curve_exponent),
            moments,
            cruise: None,
        }
    }
}

/// The body rates of a level turn at the vehicle's bank and `airspeed_mps`: ψ̇ = g·tan φ / V,
/// as pitch rate ψ̇·sin φ·cos θ and yaw rate ψ̇·cos φ·cos θ, read from the rotation matrix's
/// bottom row (`cos θ sin φ`, `cos θ cos φ`) without Euler angles. Faded out between 70° and
/// 75° of bank; zero inverted or for a non-finite attitude.
fn coordinated_turn_radps(state: &VehicleState, airspeed_mps: f32) -> Vec3 {
    let m = state.attitude.to_rotation_matrix();
    let (sin_roll_cos_pitch, cos_roll_cos_pitch) = (m[(2, 1)], m[(2, 2)]);
    let bank_rad = bank_rad(&state.attitude).abs();
    let (start, end) = COORDINATION_FADE_RAD;
    let fade = ((end - bank_rad) / (end - start)).clamp(0.0, 1.0);
    // A NaN bank (non-finite attitude) fades to NaN: no feedforward either.
    if fade.is_nan() || fade <= 0.0 || !airspeed_mps.is_finite() {
        return Vec3::zeros();
    }
    // ψ̇·cos φ·cos θ = g·sin φ·cos θ / V, and ψ̇·sin φ·cos θ = that · tan φ.
    let yaw_radps = GRAVITY * sin_roll_cos_pitch / airspeed_mps;
    let pitch_radps = yaw_radps * sin_roll_cos_pitch / cos_roll_cos_pitch;
    Vec3::new(0.0, pitch_radps, yaw_radps) * fade
}

impl Configurable for PlaneStabilizer {
    type Config = PlaneStabilizerConfig;

    fn config(&self) -> &PlaneStabilizerConfig {
        &self.config
    }

    /// Validates with [`PlaneStabilizerConfig::validate`]; the rate integrals are rescaled to
    /// the new integral gains.
    fn set_config(&mut self, config: PlaneStabilizerConfig) -> Result<(), InvalidConfig> {
        config.validate()?;
        self.rate.rescale_integral(&self.config.rate, &config.rate);
        self.config = config;
        Ok(())
    }
}

impl Stabilizer for PlaneStabilizer {
    /// Flies a [`Setpoint::Plane`] or [`Setpoint::PlaneRate`]; neutral for any other.
    fn stabilize(&mut self, dt: f32, setpoint: &Setpoint, state: &VehicleState) -> ControlOutput {
        assert_dt(dt);
        self.saturation = Saturation::default();
        let output = match setpoint {
            Setpoint::Plane(plane) => {
                let rates = self.attitude_rates(plane, state);
                self.track(dt, rates, plane.thrust, state)
            }
            Setpoint::PlaneRate(rates) => {
                let rates_radps = self.body_rates(rates);
                self.track(dt, rates_radps, rates.thrust, state)
            }
            _ => return ControlOutput::default(),
        };
        self.saturation.moments = output.moments.map(|m| m.abs() >= 1.0).into();
        output
    }

    /// Clears the rate integrals and saturation.
    fn prime(&mut self, _state: &VehicleState) {
        self.rate.clear_integral();
        self.saturation = Saturation::default();
    }

    /// `tilt` when an attitude setpoint's bank or pitch was beyond its limit, `thrust` when
    /// the setpoint's thrust was outside `[0, 1]`, and `moments` per axis at ±1.
    fn saturation(&self) -> Saturation {
        self.saturation
    }
}
