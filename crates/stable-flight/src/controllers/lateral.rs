use crate::Vec2;

use crate::controllers::{Gains, InvalidConfig, check_limit};
use crate::math::{assert_dt, clamp_symmetric_each};

/// Configures the [`LateralController`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LateralControllerConfig {
    /// PID gains from position error (m) to horizontal acceleration (m/s²).
    pub gains: Gains,

    /// Maximum absolute value of the integrated position error, in m·s, per axis.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,
}

impl Default for LateralControllerConfig {
    fn default() -> Self {
        Self {
            // Position → acceleration is a double integrator, so D sets the damping: ζ ≈ 0.8.
            // The #108 flight-checked gains (0.8, 1.6) fed a·45°/g of tilt; these are scaled by
            // π/4 so the small-angle loop gain matches now that tilt is atan(a/g) (#71).
            gains: Gains {
                p: 0.63,
                i: 0.0,
                d: 1.26,
            },
            max_integral: 100.0, // Arbitrary limit
        }
    }
}

impl LateralControllerConfig {
    /// Checks the config: `max_integral` must be non-negative and finite.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::LateralControllerConfig;
    ///
    /// assert!(LateralControllerConfig::default().validate().is_ok());
    /// let bad = LateralControllerConfig { max_integral: f32::NAN, ..Default::default() };
    /// assert!(bad.validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_limit("max_integral", self.max_integral)
    }
}

/// Horizontal position PID loop: turns a position target into a horizontal acceleration demand.
///
/// Turning the acceleration into tilt is the guidance's job (it depends on the airframe and
/// its heading); [`CopterGuidance`](crate::guidance::copter::CopterGuidance) tilts by
/// `atan(|a| / g)` toward it.
///
/// It holds only state (the integral); gains and limits come from the
/// [`LateralControllerConfig`] passed to each [`control`](Self::control) call, so they can
/// change between ticks without resetting the state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LateralController {
    integrated_error: Vec2, // Accumulated error (x, y)
}

impl LateralController {
    /// Controls the lateral (X-Y) position of the vehicle using a PID controller.
    ///
    /// This outer loop maintains the desired horizontal position by calculating the horizontal
    /// acceleration that closes the position error.
    ///
    /// # Arguments
    ///
    /// * `config` - Gains and limits; assumed valid ([`LateralControllerConfig::validate`]).
    ///   An invalid limit does not panic: a negative or NaN `max_integral` acts as 0.
    /// * `dt` - Time step since the last update in seconds (must be positive and finite).
    /// * `desired_position_m` - Target position (x, y) in meters.
    /// * `estimated_position_m` - Current position (x, y) in meters.
    /// * `estimated_velocity_mps` - Current velocity (vx, vy) in m/s.
    ///
    /// # Returns
    ///
    /// Desired horizontal acceleration (x, y) in m/s², in the position frame. It is not
    /// saturated: the caller caps the tilt it turns into. A non-finite error or velocity gives
    /// zero (level). The integral term is clamped to `max_integral` to limit windup.
    ///
    /// # Panics
    ///
    /// If `dt` is not positive and finite.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::{LateralController, LateralControllerConfig};
    /// use stable_flight::Vec2;
    ///
    /// let config = LateralControllerConfig::default();
    /// let mut controller = LateralController::default();
    /// let dt = 0.004;
    /// let desired_position_m = Vec2::new(5.0, 2.0); // 5 m north, 2 m east
    /// let estimated_position_m = Vec2::new(4.0, 1.0);
    /// let estimated_velocity_mps = Vec2::new(0.1, 0.05);
    ///
    /// // Target is north-east: accelerate north (+x) and east (+y)
    /// let accel_mps2 = controller.control(&config, dt, desired_position_m, estimated_position_m, estimated_velocity_mps);
    /// assert!(accel_mps2.x > 0.0 && accel_mps2.y > 0.0);
    /// ```
    pub fn control(
        &mut self,
        config: &LateralControllerConfig,
        dt: f32,
        desired_position_m: Vec2,
        estimated_position_m: Vec2,
        estimated_velocity_mps: Vec2,
    ) -> Vec2 {
        assert_dt(dt);
        let gains = &config.gains;

        let error = desired_position_m - estimated_position_m;

        // A non-finite error (NaN/∞ target or position) yields no acceleration and never enters
        // the integral, where NaN would stick (it survives clamping).
        if !(error.x.is_finite() && error.y.is_finite()) {
            return Vec2::zeros();
        }

        self.integrated_error =
            clamp_symmetric_each(self.integrated_error + dt * error, config.max_integral);
        let accel_mps2 =
            gains.p * error + gains.i * self.integrated_error + gains.d * (-estimated_velocity_mps);

        // A non-finite velocity estimate yields no acceleration rather than NaN.
        if accel_mps2.x.is_finite() && accel_mps2.y.is_finite() {
            accel_mps2
        } else {
            Vec2::zeros()
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::Vec2;

    use super::*;

    #[test]
    fn accelerates_toward_the_target_against_velocity() {
        let config = LateralControllerConfig {
            gains: Gains {
                p: 2.0,
                i: 0.0,
                d: 0.5,
            },
            ..Default::default()
        };
        // (target position, velocity, expected acceleration) with the vehicle at the origin
        let cases = [
            (Vec2::zeros(), Vec2::zeros(), Vec2::zeros()),
            (Vec2::new(10.0, 0.0), Vec2::zeros(), Vec2::new(20.0, 0.0)), // not saturated
            (Vec2::new(0.0, -3.0), Vec2::zeros(), Vec2::new(0.0, -6.0)),
            (Vec2::zeros(), Vec2::new(4.0, -2.0), Vec2::new(-2.0, 1.0)), // D brakes
            (Vec2::new(0.0, f32::NAN), Vec2::zeros(), Vec2::zeros()),
            (Vec2::new(f32::INFINITY, 0.0), Vec2::zeros(), Vec2::zeros()), // non-finite: none
        ];
        for (target, velocity, expected) in cases {
            let mut controller = LateralController::default();
            let accel = controller.control(&config, 0.004, target, Vec2::zeros(), velocity);
            assert!(
                (accel - expected).amax() < 1e-5,
                "target {target:?}, velocity {velocity:?} gave {accel:?}, expected {expected:?}"
            );
        }
    }

    /// A NaN or infinite velocity estimate gives no acceleration, not NaN.
    #[test]
    fn non_finite_velocity_yields_no_acceleration() {
        let nan = f32::NAN;
        let inf = f32::INFINITY;
        let velocities = [
            Vec2::new(nan, 0.0),
            Vec2::new(0.0, inf),
            Vec2::new(-inf, nan),
        ];
        let config = LateralControllerConfig::default();
        for velocity in velocities {
            let mut controller = LateralController::default();
            let angles = controller.control(
                &config,
                0.004,
                Vec2::new(10.0, 5.0),
                Vec2::zeros(),
                velocity,
            );
            assert!(
                angles == Vec2::zeros(),
                "velocity {velocity:?} gave {angles:?}"
            );
        }
    }

    #[test]
    fn integral_term_is_clamped_to_max_integral() {
        let config = LateralControllerConfig {
            max_integral: 5.0,
            ..Default::default()
        };
        let mut controller = LateralController::default();

        // A large, persistent error for many ticks would wind the integral far past the limit.
        for _ in 0..10_000 {
            controller.control(
                &config,
                0.01,
                Vec2::new(100.0, -100.0),
                Vec2::zeros(),
                Vec2::zeros(),
            );
        }

        assert!(controller.integrated_error == Vec2::new(5.0, -5.0));
    }

    #[test]
    fn recovers_after_a_nan_tick() {
        let config = LateralControllerConfig::default();
        let mut controller = LateralController::default();
        let at_rest = Vec2::zeros();

        // One tick with a NaN target must not poison later ticks.
        let during = controller.control(&config, 0.004, Vec2::new(f32::NAN, 0.0), at_rest, at_rest);
        let after = controller.control(&config, 0.004, Vec2::new(10.0, 0.0), at_rest, at_rest);

        assert!(during == Vec2::zeros(), "NaN tick gave {during:?}");
        let fresh = LateralController::default().control(
            &config,
            0.004,
            Vec2::new(10.0, 0.0),
            at_rest,
            at_rest,
        );
        assert!(after == fresh, "valid target after NaN gave {after:?}");
    }

    /// An invalid `max_integral` fails `validate` but `control` stays panic-free: it acts as 0.
    #[test]
    fn invalid_max_integral_is_rejected_and_does_not_panic_the_tick() {
        let invalid = LateralControllerConfig {
            max_integral: -1.0,
            ..Default::default()
        };
        assert_eq!(invalid.validate().map_err(|e| e.field), Err("max_integral"));
        for max_integral in [-1.0, f32::NAN] {
            let config = LateralControllerConfig {
                max_integral,
                ..Default::default()
            };
            let mut controller = LateralController::default();
            controller.control(
                &config,
                0.01,
                Vec2::new(1.0, 1.0),
                Vec2::zeros(),
                Vec2::zeros(),
            );
            assert!(
                controller.integrated_error == Vec2::zeros(),
                "max_integral {max_integral}"
            );
        }
    }

    /// A gain change between ticks applies on the next tick and keeps the integral.
    #[test]
    fn gain_change_between_ticks_keeps_state() {
        let config = LateralControllerConfig::default();
        let mut controller = LateralController::default();
        controller.control(
            &config,
            1.0,
            Vec2::new(0.0, 1.0),
            Vec2::zeros(),
            Vec2::zeros(),
        );
        let integral = controller.integrated_error;

        let retuned = LateralControllerConfig {
            gains: Gains {
                p: 0.0,
                i: 2.0,
                d: 0.0,
            },
            ..config
        };
        let accel = controller.control(&retuned, 0.01, Vec2::zeros(), Vec2::zeros(), Vec2::zeros());

        assert!(controller.integrated_error == integral);
        approx::assert_relative_eq!(accel.y, 2.0 * integral.y, epsilon = 1e-6);
    }

    #[test]
    #[should_panic(expected = "dt must be positive and finite")]
    fn zero_dt_panics() {
        LateralController::default().control(
            &LateralControllerConfig::default(),
            0.0,
            Vec2::zeros(),
            Vec2::zeros(),
            Vec2::zeros(),
        );
    }
}
