use crate::Vec2;

use super::derivative::DerivativeState;
use crate::controllers::{Gains, InvalidConfig, check_limit};
use crate::math::{all_finite, assert_dt, cap_magnitude, clamp_symmetric_each};

/// Configures the [`LateralController`]: a position loop feeding a velocity loop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LateralControllerConfig {
    /// Proportional gain from position error (m) to desired velocity (m/s), in 1/s. Sets how
    /// fast the vehicle closes on a target before the speed limit applies. Must be
    /// non-negative and finite.
    pub position_p: f32,

    /// PID gains from velocity error (m/s) to horizontal acceleration (m/s²).
    pub velocity: Gains,

    /// Maximum absolute value of the integrated velocity error, in m, per axis.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,
}

impl Default for LateralControllerConfig {
    fn default() -> Self {
        Self {
            // Unsaturated, `velocity.p · (position_p · e − v)` is the flight-checked position
            // PD (#108, rescaled for the atan tilt in #71): P 0.63, D 1.26, ζ ≈ 0.8.
            position_p: 0.5,
            velocity: Gains {
                p: 1.26,
                i: 0.0,
                d: 0.0,
            },
            max_integral: 100.0, // Arbitrary limit
        }
    }
}

impl LateralControllerConfig {
    /// Checks the config: `position_p` and `max_integral` must be non-negative and finite.
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
        check_limit("position_p", self.position_p)?;
        check_limit("max_integral", self.max_integral)
    }
}

/// What the [`LateralController`] tracks: a horizontal position, a velocity fed forward
/// toward it, and a speed limit. Horizontal components are (north, east), as in the
/// [`VehicleState`](crate::VehicleState) frame contract.
///
/// # Examples
///
/// ```
/// use stable_flight::Vec2;
/// use stable_flight::controllers::LateralTarget;
///
/// // Fly to 20 m north at up to 5 m/s, stopping there.
/// let target = LateralTarget {
///     position_m: Vec2::new(20.0, 0.0),
///     velocity_mps: Vec2::zeros(),
///     max_speed_mps: 5.0,
/// };
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LateralTarget {
    /// Desired position in meters.
    pub position_m: Vec2,
    /// Velocity in m/s added to the position loop's demand: zero stops at the target, a
    /// non-zero value flies through it.
    pub velocity_mps: Vec2,
    /// Cap on the desired speed in m/s. Zero, negative, or NaN means no cap (the guidance's
    /// tilt limit still applies).
    pub max_speed_mps: f32,
}

/// The vehicle's current horizontal state as the [`LateralController`] consumes it, (north,
/// east) per the [`VehicleState`](crate::VehicleState) frame contract.
///
/// # Examples
///
/// ```
/// use stable_flight::Vec2;
/// use stable_flight::controllers::LateralEstimate;
///
/// // 4 m north, 1 m east, drifting north at 0.1 m/s.
/// let estimate = LateralEstimate {
///     position_m: Vec2::new(4.0, 1.0),
///     velocity_mps: Vec2::new(0.1, 0.0),
/// };
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LateralEstimate {
    /// Estimated position in meters.
    pub position_m: Vec2,
    /// Estimated velocity in m/s.
    pub velocity_mps: Vec2,
}

/// Horizontal position → velocity → acceleration cascade: turns a position target into a
/// horizontal acceleration demand.
///
/// The position loop asks for `position_p · error + target velocity`, capped in magnitude at
/// the target's `max_speed_mps` (direction kept), and the velocity PID turns the velocity
/// error into acceleration. Turning the acceleration into tilt is the guidance's job (it
/// depends on the airframe and its heading);
/// [`CopterGuidance`](crate::guidance::copter::CopterGuidance) tilts by `atan(|a| / g)` toward
/// it.
///
/// It holds only state (the velocity integral and derivative history); gains and limits come
/// from the [`LateralControllerConfig`] passed to each [`control`](Self::control) call, so
/// they can change between ticks without resetting the state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LateralController {
    integrated_error: Vec2, // Accumulated velocity error (x, y)
    /// Previous velocity error for the derivative term; empty on the first tick and after a gap.
    derivative: DerivativeState<Vec2>,
}

impl LateralController {
    /// Desired horizontal acceleration toward `target`.
    ///
    /// # Arguments
    ///
    /// * `config` - Gains and limits; assumed valid ([`LateralControllerConfig::validate`]).
    ///   An invalid limit does not panic: a negative or NaN `max_integral` acts as 0.
    /// * `dt` - Time step since the last update in seconds (must be positive and finite).
    /// * `target` - Desired position, feedforward velocity, and speed limit.
    /// * `estimate` - Current position and velocity.
    ///
    /// # Returns
    ///
    /// Desired horizontal acceleration (x, y) in m/s², in the position frame. It is not
    /// saturated: the caller caps the tilt it turns into. A non-finite target or estimate gives
    /// zero (level) and leaves the integral untouched. The integral is clamped to
    /// `max_integral` to limit windup.
    ///
    /// # Panics
    ///
    /// If `dt` is not positive and finite.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::{
    ///     LateralController, LateralControllerConfig, LateralEstimate, LateralTarget,
    /// };
    /// use stable_flight::Vec2;
    ///
    /// let config = LateralControllerConfig::default();
    /// let mut controller = LateralController::default();
    /// let target = LateralTarget {
    ///     position_m: Vec2::new(5.0, 2.0), // 5 m north, 2 m east
    ///     velocity_mps: Vec2::zeros(),
    ///     max_speed_mps: 3.0,
    /// };
    /// let estimate = LateralEstimate {
    ///     position_m: Vec2::new(4.0, 1.0),
    ///     velocity_mps: Vec2::new(0.1, 0.05),
    /// };
    ///
    /// // Target is north-east: accelerate north (+x) and east (+y)
    /// let accel_mps2 = controller.control(&config, 0.004, target, estimate);
    /// assert!(accel_mps2.x > 0.0 && accel_mps2.y > 0.0);
    /// ```
    pub fn control(
        &mut self,
        config: &LateralControllerConfig,
        dt: f32,
        target: LateralTarget,
        estimate: LateralEstimate,
    ) -> Vec2 {
        assert_dt(dt);
        let gains = &config.velocity;

        // `>` is false for NaN: zero, negative, and NaN limits all mean no cap.
        let max_speed_mps = if target.max_speed_mps > 0.0 {
            target.max_speed_mps
        } else {
            f32::INFINITY
        };
        let desired_velocity =
            config.position_p * (target.position_m - estimate.position_m) + target.velocity_mps;
        let velocity_error = cap_magnitude(desired_velocity, max_speed_mps) - estimate.velocity_mps;

        // A non-finite target or estimate yields no acceleration and never enters the integral
        // or derivative history, where NaN would stick (it survives clamping). `cap_magnitude`
        // zeroes a non-finite demand, so check its input too.
        if !(all_finite(&desired_velocity) && all_finite(&velocity_error)) {
            self.derivative.forget();
            return Vec2::zeros();
        }
        let error_derivative = self.derivative.step(velocity_error, dt);

        self.integrated_error = clamp_symmetric_each(
            self.integrated_error + dt * velocity_error,
            config.max_integral,
        );
        gains.p * velocity_error + gains.i * self.integrated_error + gains.d * error_derivative
    }
}

#[cfg(test)]
mod tests {
    use crate::Vec2;

    use super::*;

    /// A target at `position_m`, stopping there, with no speed cap.
    fn at(position_m: Vec2) -> LateralTarget {
        LateralTarget {
            position_m,
            velocity_mps: Vec2::zeros(),
            max_speed_mps: 0.0,
        }
    }

    /// The vehicle at the origin moving at `velocity_mps`.
    fn moving(velocity_mps: Vec2) -> LateralEstimate {
        LateralEstimate {
            position_m: Vec2::zeros(),
            velocity_mps,
        }
    }

    fn p_only() -> LateralControllerConfig {
        LateralControllerConfig {
            position_p: 0.5,
            velocity: Gains {
                p: 2.0,
                i: 0.0,
                d: 0.0,
            },
            ..Default::default()
        }
    }

    #[test]
    fn accelerates_toward_the_target_against_velocity() {
        // (target, velocity, expected acceleration) with the vehicle at the origin
        let cases = [
            (at(Vec2::zeros()), Vec2::zeros(), Vec2::zeros()),
            (
                at(Vec2::new(10.0, 0.0)),
                Vec2::zeros(),
                Vec2::new(10.0, 0.0),
            ), // no cap
            (
                at(Vec2::new(0.0, -3.0)),
                Vec2::zeros(),
                Vec2::new(0.0, -3.0),
            ),
            (
                at(Vec2::zeros()),
                Vec2::new(4.0, -2.0),
                Vec2::new(-8.0, 4.0),
            ), // brakes
            (at(Vec2::new(0.0, f32::NAN)), Vec2::zeros(), Vec2::zeros()),
            (
                at(Vec2::new(f32::INFINITY, 0.0)),
                Vec2::zeros(),
                Vec2::zeros(),
            ), // non-finite
        ];
        for (target, velocity, expected) in cases {
            let mut controller = LateralController::default();
            let accel = controller.control(&p_only(), 0.004, target, moving(velocity));
            assert!(
                (accel - expected).amax() < 1e-5,
                "target {target:?}, velocity {velocity:?} gave {accel:?}, expected {expected:?}"
            );
        }
    }

    /// The desired velocity is capped at `max_speed_mps`, keeping its direction; zero,
    /// negative, and NaN limits leave it uncapped.
    #[test]
    fn caps_the_desired_speed() {
        let far = Vec2::new(30.0, 40.0); // 50 m away: desired 25 m/s uncapped
        // (max_speed_mps, velocity feedforward, expected acceleration from rest)
        let cases = [
            (5.0, Vec2::zeros(), Vec2::new(6.0, 8.0)), // 5 m/s toward (3, 4)/5, × P 2
            (100.0, Vec2::zeros(), Vec2::new(30.0, 40.0)),
            (0.0, Vec2::zeros(), Vec2::new(30.0, 40.0)),
            (-1.0, Vec2::zeros(), Vec2::new(30.0, 40.0)),
            (f32::NAN, Vec2::zeros(), Vec2::new(30.0, 40.0)),
            // Feedforward adds before the cap.
            (5.0, Vec2::new(15.0, 20.0), Vec2::new(6.0, 8.0)),
        ];
        for (max_speed_mps, velocity_mps, expected) in cases {
            let target = LateralTarget {
                position_m: far,
                velocity_mps,
                max_speed_mps,
            };
            let accel = LateralController::default().control(
                &p_only(),
                0.004,
                target,
                moving(Vec2::zeros()),
            );
            assert!(
                (accel - expected).amax() < 1e-4,
                "max {max_speed_mps}, ff {velocity_mps:?} gave {accel:?}, expected {expected:?}"
            );
        }
    }

    /// On the target, the feedforward velocity is the whole demand: flying at it needs no
    /// acceleration, and at rest the loop accelerates toward it.
    #[test]
    fn feedforward_flies_through_the_target() {
        let target = LateralTarget {
            position_m: Vec2::zeros(),
            velocity_mps: Vec2::new(7.0, 0.0),
            max_speed_mps: 7.0,
        };
        let config = p_only();
        let cruising = LateralController::default().control(
            &config,
            0.004,
            target,
            moving(Vec2::new(7.0, 0.0)),
        );
        let at_rest =
            LateralController::default().control(&config, 0.004, target, moving(Vec2::zeros()));
        assert!(cruising == Vec2::zeros());
        assert!((at_rest - Vec2::new(14.0, 0.0)).amax() < 1e-5);
    }

    /// Below the cap, the default cascade is the flight-checked position PD it replaced:
    /// `0.63 · e − 1.26 · v`.
    #[test]
    fn default_gains_match_the_position_pd() {
        let config = LateralControllerConfig::default();
        // (position error, velocity)
        let cases = [
            (Vec2::new(2.0, -1.0), Vec2::zeros()),
            (Vec2::zeros(), Vec2::new(1.0, 0.5)),
            (Vec2::new(-3.0, 4.0), Vec2::new(0.4, -0.2)),
        ];
        for (error, velocity) in cases {
            let accel =
                LateralController::default().control(&config, 0.004, at(error), moving(velocity));
            let expected = 0.63 * error - 1.26 * velocity;
            assert!(
                (accel - expected).amax() < 1e-5,
                "error {error:?}, velocity {velocity:?} gave {accel:?}"
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
            let accel =
                controller.control(&config, 0.004, at(Vec2::new(10.0, 5.0)), moving(velocity));
            assert!(
                accel == Vec2::zeros(),
                "velocity {velocity:?} gave {accel:?}"
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
                at(Vec2::new(100.0, -100.0)),
                moving(Vec2::zeros()),
            );
        }

        assert!(controller.integrated_error == Vec2::new(5.0, -5.0));
    }

    #[test]
    fn recovers_after_a_nan_tick() {
        let config = LateralControllerConfig {
            velocity: Gains {
                p: 1.0,
                i: 0.5,
                d: 0.1,
            },
            ..Default::default()
        };
        let mut controller = LateralController::default();
        let at_rest = moving(Vec2::zeros());

        // One tick with a NaN target must not poison later ticks.
        let during = controller.control(&config, 0.004, at(Vec2::new(f32::NAN, 0.0)), at_rest);
        let after = controller.control(&config, 0.004, at(Vec2::new(10.0, 0.0)), at_rest);

        assert!(during == Vec2::zeros(), "NaN tick gave {during:?}");
        let fresh =
            LateralController::default().control(&config, 0.004, at(Vec2::new(10.0, 0.0)), at_rest);
        assert!(after == fresh, "valid target after NaN gave {after:?}");
    }

    /// The D term acts on the change in velocity error, with no kick on the first tick.
    #[test]
    fn derivative_acts_on_the_velocity_error_change() {
        let config = LateralControllerConfig {
            position_p: 0.0,
            velocity: Gains {
                p: 0.0,
                i: 0.0,
                d: 0.5,
            },
            ..Default::default()
        };
        let mut controller = LateralController::default();
        let target = at(Vec2::zeros());
        let first = controller.control(&config, 0.1, target, moving(Vec2::new(1.0, 0.0)));
        let second = controller.control(&config, 0.1, target, moving(Vec2::new(2.0, 0.0)));
        assert!(first == Vec2::zeros());
        assert!((second - Vec2::new(-5.0, 0.0)).amax() < 1e-4);
    }

    /// An invalid limit fails `validate` but `control` stays panic-free: it acts as 0.
    #[test]
    fn invalid_config_is_rejected_and_does_not_panic_the_tick() {
        let negative_integral = LateralControllerConfig {
            max_integral: -1.0,
            ..Default::default()
        };
        let nan_position_p = LateralControllerConfig {
            position_p: f32::NAN,
            ..Default::default()
        };
        assert_eq!(
            negative_integral.validate().map_err(|e| e.field),
            Err("max_integral")
        );
        assert_eq!(
            nan_position_p.validate().map_err(|e| e.field),
            Err("position_p")
        );
        for max_integral in [-1.0, f32::NAN] {
            let config = LateralControllerConfig {
                max_integral,
                ..Default::default()
            };
            let mut controller = LateralController::default();
            controller.control(
                &config,
                0.01,
                at(Vec2::new(1.0, 1.0)),
                moving(Vec2::zeros()),
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
        controller.control(&config, 1.0, at(Vec2::new(0.0, 1.0)), moving(Vec2::zeros()));
        let integral = controller.integrated_error;

        let retuned = LateralControllerConfig {
            position_p: 0.0,
            velocity: Gains {
                p: 0.0,
                i: 2.0,
                d: 0.0,
            },
            ..config
        };
        let accel = controller.control(&retuned, 0.01, at(Vec2::zeros()), moving(Vec2::zeros()));

        assert!(controller.integrated_error == integral);
        approx::assert_relative_eq!(accel.y, 2.0 * integral.y, epsilon = 1e-6);
    }

    #[test]
    #[should_panic(expected = "dt must be positive and finite")]
    fn zero_dt_panics() {
        LateralController::default().control(
            &LateralControllerConfig::default(),
            0.0,
            at(Vec2::zeros()),
            moving(Vec2::zeros()),
        );
    }
}
