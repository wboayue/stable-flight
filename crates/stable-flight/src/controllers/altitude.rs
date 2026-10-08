use nalgebra::UnitQuaternion;

use crate::GRAVITY;
use crate::controllers::{Gains, InvalidConfig, check_limit, check_unit_interval};
use crate::math::clamp_symmetric;

/// Default [`AltitudeControllerConfig::hover_thrust`].
pub const DEFAULT_HOVER_THRUST: f32 = 0.5;

/// The integral only accumulates within this many meters of the target. With gravity fed
/// forward it just trims a `hover_thrust` mismatch; winding it up over a long climb made the
/// vehicle overshoot and settle slowly.
const INTEGRATION_BAND: f32 = 1.0;

/// Floor on the body-z alignment (cos of the tilt) used for tilt compensation: past 60° the
/// boost stops growing, so a near-inverted estimate can't demand unbounded thrust.
const MIN_BODY_Z_ALIGNMENT: f32 = 0.5;

/// Configures the [`AltitudeController`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AltitudeControllerConfig {
    /// PID gains from altitude error (m) to vertical acceleration (m/s², up).
    pub gains: Gains,
    /// Maximum absolute value of the integrated altitude error, in m·s.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,

    /// Linear thrust (before the stabilizer's thrust curve) that holds a level hover. The
    /// controller feeds it forward, so the integral only trims the residual. Airframe-specific.
    pub hover_thrust: f32,
}

impl Default for AltitudeControllerConfig {
    fn default() -> Self {
        Self {
            gains: Gains {
                p: 1.0,
                i: 0.1,
                d: 0.05,
            },
            // With feedforward the integral only trims hover; 5 m·s at I 0.2 is ±1 m/s².
            max_integral: 5.0,
            hover_thrust: DEFAULT_HOVER_THRUST,
        }
    }
}

impl AltitudeControllerConfig {
    /// Checks the config: `max_integral` must be non-negative and finite, and `hover_thrust`
    /// in `(0, 1]`.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::AltitudeControllerConfig;
    ///
    /// assert!(AltitudeControllerConfig::default().validate().is_ok());
    /// let bad = AltitudeControllerConfig { hover_thrust: 1.5, ..Default::default() };
    /// assert!(bad.validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_limit("max_integral", self.max_integral)?;
        check_unit_interval("hover_thrust", self.hover_thrust)
    }
}

/// Altitude PID loop with gravity feedforward: turns an altitude target into a linear thrust
/// command.
///
/// It holds only state (the integral); gains and limits come from the
/// [`AltitudeControllerConfig`] passed to each [`control`](Self::control) call, so they can
/// change between ticks without resetting the state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct AltitudeController {
    integrated_error: f32,
}

impl AltitudeController {
    /// Controls the altitude of the vehicle by computing a normalized thrust command using a PID controller.
    ///
    /// The PID gives a vertical acceleration `a` (m/s², up); the thrust is
    /// `hover_thrust · (g + a) / (g · cos tilt)`, saturated to `[0, 1]`. Zero error is hover,
    /// and a tilted vehicle gets more thrust so its vertical component still balances gravity.
    ///
    /// # Arguments
    ///
    /// * `config` - Gains, limits, and hover thrust; assumed valid
    ///   ([`AltitudeControllerConfig::validate`]). An invalid config does not panic: a negative
    ///   or NaN `max_integral` acts as 0.
    /// * `dt` - Time step since the last update (must be positive).
    /// * `desired_altitude_m` - Desired altitude in meters.
    /// * `desired_velocity_mps` - Desired vertical velocity in m/s (up); the D term damps toward it.
    /// * `estimated_altitude_m` - Current measured altitude in meters.
    /// * `estimated_velocity_mps` - Current vertical velocity in m/s.
    /// * `estimated_attitude` - Current attitude as a quaternion.
    ///
    /// # Returns
    ///
    /// A normalized thrust command (0.0 to 1.0).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::{
    ///     AltitudeController, AltitudeControllerConfig, DEFAULT_HOVER_THRUST,
    /// };
    /// use stable_flight::nalgebra::UnitQuaternion;
    ///
    /// let config = AltitudeControllerConfig::default();
    /// let mut controller = AltitudeController::default();
    /// let dt = 0.004;
    /// let level = UnitQuaternion::identity();
    ///
    /// // On target, still, and level: the feedforward hover thrust.
    /// let thrust = controller.control(&config, dt, 10.0, 0.0, 10.0, 0.0, level);
    /// assert!((thrust - DEFAULT_HOVER_THRUST).abs() < 1e-6);
    ///
    /// // Below target: more than hover.
    /// let thrust = controller.control(&config, dt, 10.0, 0.0, 9.5, -0.1, level);
    /// assert!(thrust > DEFAULT_HOVER_THRUST && thrust <= 1.0);
    /// ```
    // Config, dt, and five per-tick inputs: each is a distinct physical quantity, so a struct
    // would only rename them.
    #[allow(clippy::too_many_arguments)]
    pub fn control(
        &mut self,
        config: &AltitudeControllerConfig,
        dt: f32,
        desired_altitude_m: f32,
        desired_velocity_mps: f32,
        estimated_altitude_m: f32,
        estimated_velocity_mps: f32,
        estimated_attitude: UnitQuaternion<f32>,
    ) -> f32 {
        let gains = &config.gains;
        let hover_thrust = config.hover_thrust;

        let altitude_error = desired_altitude_m - estimated_altitude_m;
        let velocity_error = desired_velocity_mps - estimated_velocity_mps;

        // A non-finite error (NaN/∞ altitude or velocity) must never enter the integral, where
        // it would stick (NaN survives clamping) and hold thrust at NaN → 0 for good. Neutral
        // for altitude is *hover*, not zero thrust, so return the hover setpoint instead.
        if !(altitude_error.is_finite() && velocity_error.is_finite()) {
            return hover_thrust;
        }

        // Accumulate only near the target, but clamp every tick so a shrunk `max_integral`
        // applies on the next tick even outside the band.
        if altitude_error.abs() < INTEGRATION_BAND {
            self.integrated_error += altitude_error * dt;
        }
        self.integrated_error = clamp_symmetric(self.integrated_error, config.max_integral);

        let acceleration =
            gains.p * altitude_error + gains.d * velocity_error + gains.i * self.integrated_error;

        // Cos of the tilt, cos(roll)·cos(pitch); yaw doesn't change it. A non-finite attitude
        // counts as level (no boost).
        let body_z_alignment = estimated_attitude.to_rotation_matrix()[(2, 2)];
        let body_z_alignment = if body_z_alignment.is_finite() {
            body_z_alignment.max(MIN_BODY_Z_ALIGNMENT)
        } else {
            1.0
        };

        let linear_thrust = hover_thrust * (GRAVITY + acceleration) / (GRAVITY * body_z_alignment);
        linear_thrust.clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn level() -> UnitQuaternion<f32> {
        UnitQuaternion::identity()
    }

    fn nan_attitude() -> UnitQuaternion<f32> {
        let nan = f32::NAN;
        UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(nan, nan, nan, nan))
    }

    /// Thrust is `hover · (g + a) / (g · cos tilt)` with the PID's acceleration `a`: zero error
    /// is hover, a bank adds thrust (yaw doesn't), the boost stops at 60°, and the result
    /// saturates to [0, 1].
    #[test]
    fn thrust_feeds_gravity_forward_and_boosts_for_tilt() {
        let deg = f32::to_radians;
        let attitude = |roll: f32, pitch: f32, yaw: f32| {
            UnitQuaternion::from_euler_angles(deg(roll), deg(pitch), deg(yaw))
        };
        let hover = 0.4;
        let cos = |d: f32| d.to_radians().cos();
        // (altitude error m, climb rate m/s, attitude, expected thrust); P 1, D 0.5, no I
        let cases = [
            (0.0, 0.0, level(), hover),
            (0.0, 0.0, attitude(0.0, 0.0, 120.0), hover),
            (0.0, 0.0, attitude(30.0, 0.0, 0.0), hover / cos(30.0)),
            (
                0.0,
                0.0,
                attitude(-20.0, 20.0, 45.0),
                hover / (cos(20.0) * cos(20.0)),
            ),
            (0.0, 0.0, attitude(75.0, 0.0, 0.0), hover / 0.5), // floored at 60°
            (0.0, 0.0, attitude(170.0, 0.0, 0.0), hover / 0.5), // inverted: same floor
            (1.0, 0.0, level(), hover * (GRAVITY + 1.0) / GRAVITY),
            (-1.0, 0.0, level(), hover * (GRAVITY - 1.0) / GRAVITY),
            (0.0, 2.0, level(), hover * (GRAVITY - 1.0) / GRAVITY), // climbing: damped
            (100.0, 0.0, level(), 1.0),                             // saturates high
            (-100.0, 0.0, level(), 0.0),                            // and low
            (0.0, 0.0, nan_attitude(), hover),                      // non-finite: level
        ];
        for (error, climb, attitude, expected) in cases {
            let config = AltitudeControllerConfig {
                gains: Gains {
                    p: 1.0,
                    i: 0.0,
                    d: 0.5,
                },
                hover_thrust: hover,
                ..Default::default()
            };
            let thrust = AltitudeController::default().control(
                &config,
                0.004,
                10.0 + error,
                0.0,
                10.0,
                climb,
                attitude,
            );
            approx::assert_relative_eq!(thrust, expected, epsilon = 1e-5);
        }
    }

    /// The integral accumulates `error · dt` only within `INTEGRATION_BAND` of the target, stops
    /// at `max_integral`, and feeds into thrust as `I · ∫e`.
    #[test]
    fn integral_runs_near_target_only_and_is_clamped() {
        let config = AltitudeControllerConfig {
            gains: Gains {
                p: 0.0,
                i: 1.0,
                d: 0.0,
            },
            max_integral: 2.0,
            hover_thrust: 0.5,
        };
        // (altitude error m, expected integral after 1000 ticks of 0.01 s)
        let cases = [
            (10.0, 0.0),
            (-5.0, 0.0),
            (0.5, 2.0),
            (-0.5, -2.0),
            (0.001, 0.01),
        ];
        for (error, expected) in cases {
            let mut controller = AltitudeController::default();
            for _ in 0..1_000 {
                controller.control(&config, 0.01, 10.0 + error, 0.0, 10.0, 0.0, level());
            }
            approx::assert_relative_eq!(controller.integrated_error, expected, epsilon = 1e-4);
        }

        let mut controller = AltitudeController {
            integrated_error: 2.0,
        };
        let thrust = controller.control(&config, 0.01, 10.0, 0.0, 10.0, 0.0, level());
        approx::assert_relative_eq!(thrust, 0.5 * (GRAVITY + 2.0) / GRAVITY, epsilon = 1e-5);
    }

    #[test]
    fn non_finite_estimate_holds_hover_and_does_not_poison_later_ticks() {
        let nan = f32::NAN;
        // (estimated altitude, estimated velocity)
        let cases = [(nan, 0.0), (10.0, nan), (f32::INFINITY, 0.0), (nan, nan)];
        let config = AltitudeControllerConfig::default();
        for (altitude, velocity) in cases {
            let mut controller = AltitudeController::default();
            let during = controller.control(&config, 0.004, 20.0, 0.0, altitude, velocity, level());
            assert!(
                during == DEFAULT_HOVER_THRUST,
                "({altitude}, {velocity}) gave {during}"
            );

            let after = controller.control(&config, 0.004, 20.0, 0.0, 10.0, 0.0, level());
            let fresh = AltitudeController::default().control(
                &config,
                0.004,
                20.0,
                0.0,
                10.0,
                0.0,
                level(),
            );
            assert!(
                after == fresh,
                "after ({altitude}, {velocity}): {after} vs fresh {fresh}"
            );
        }
    }

    #[test]
    fn validate_rejects_invalid_config() {
        // (max_integral, hover_thrust, offending field)
        let cases = [
            (-1.0, 0.5, "max_integral"),
            (f32::NAN, 0.5, "max_integral"),
            (f32::INFINITY, 0.5, "max_integral"),
            (5.0, 0.0, "hover_thrust"),
            (5.0, 1.5, "hover_thrust"),
            (5.0, f32::NAN, "hover_thrust"),
        ];
        for (max_integral, hover_thrust, field) in cases {
            let config = AltitudeControllerConfig {
                max_integral,
                hover_thrust,
                ..Default::default()
            };
            assert_eq!(
                config.validate().map_err(|e| e.field),
                Err(field),
                "{config:?}"
            );
        }
    }

    /// An invalid `max_integral` fails `validate` but `control` stays panic-free: it acts as 0.
    #[test]
    fn invalid_max_integral_does_not_panic_the_tick() {
        for max_integral in [-1.0, f32::NAN] {
            let config = AltitudeControllerConfig {
                max_integral,
                ..Default::default()
            };
            let mut controller = AltitudeController::default();
            controller.control(&config, 0.01, 10.5, 0.0, 10.0, 0.0, level());
            assert_eq!(
                controller.integrated_error, 0.0,
                "max_integral {max_integral}"
            );
        }
    }

    /// A shrunk `max_integral` clamps the stored integral on the next tick, even when the error
    /// is outside the integration band (where nothing accumulates).
    #[test]
    fn shrinking_max_integral_outside_the_band_clamps_the_integral() {
        // (max_integral, expected integral after one tick 2 m below target)
        for (max_integral, expected) in [(0.0, 0.0), (0.2, 0.2), (1.0, 0.5)] {
            let mut controller = AltitudeController {
                integrated_error: 0.5,
            };
            let config = AltitudeControllerConfig {
                max_integral,
                ..Default::default()
            };
            controller.control(&config, 0.01, 12.0, 0.0, 10.0, 0.0, level());
            assert_eq!(
                controller.integrated_error, expected,
                "max_integral {max_integral}"
            );
        }
    }

    /// A gain change between ticks applies on the next tick and keeps the integral.
    #[test]
    fn gain_change_between_ticks_keeps_state() {
        let config = AltitudeControllerConfig::default();
        let mut controller = AltitudeController::default();
        controller.control(&config, 1.0, 10.5, 0.0, 10.0, 0.0, level());
        let integral = controller.integrated_error;

        let retuned = AltitudeControllerConfig {
            gains: Gains {
                p: 0.0,
                i: 0.4,
                d: 0.0,
            },
            ..config
        };
        let thrust = controller.control(&retuned, 0.01, 10.0, 0.0, 10.0, 0.0, level());

        assert_eq!(controller.integrated_error, integral);
        let expected = config.hover_thrust * (GRAVITY + 0.4 * integral) / GRAVITY;
        approx::assert_relative_eq!(thrust, expected, epsilon = 1e-5);
    }
}
