use crate::Vec3;

use super::{Gains, InvalidConfig, check_limit};
use crate::math::{all_finite, clamp_symmetric_each};

/// Default proportional gain, in normalized moment per rad/s of rate error.
const DEFAULT_P: f32 = 0.15;
/// Default derivative gain, in normalized moment per rad/s².
const DEFAULT_D: f32 = 0.003;
/// Default yaw proportional gain. Yaw authority comes only from differential prop drag, so
/// yaw needs more gain than roll and pitch.
const DEFAULT_YAW_P: f32 = 0.3;
/// Default yaw integral gain, in normalized moment per rad of accumulated error.
const DEFAULT_YAW_I: f32 = 0.1;
/// Default integral limit, in rad (accumulated rate error).
const DEFAULT_MAX_INTEGRAL: f32 = 1.0;

/// Configures the [BodyRateController] with PID gains and operational limits.
///
/// Gains map rate error (rad/s) straight to a normalized moment in `[-1, 1]`: `p` is moment per
/// rad/s, `i` per rad of accumulated error, and `d` per rad/s².
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyRateControllerConfig {
    /// PID gains for the roll and pitch rates.
    pub gains: Gains,

    /// PID gains for the yaw rate.
    pub yaw_gains: Gains,

    /// Maximum absolute value of the integrated rate error, in rad, per axis.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,
}

impl Default for BodyRateControllerConfig {
    fn default() -> Self {
        Self {
            gains: Gains {
                p: DEFAULT_P,
                i: 0.0,
                d: DEFAULT_D,
            },
            yaw_gains: Gains {
                p: DEFAULT_YAW_P,
                i: DEFAULT_YAW_I,
                d: 0.0,
            },
            max_integral: DEFAULT_MAX_INTEGRAL,
        }
    }
}

impl BodyRateControllerConfig {
    /// Checks the config: `max_integral` must be non-negative and finite.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::BodyRateControllerConfig;
    ///
    /// assert!(BodyRateControllerConfig::default().validate().is_ok());
    /// let bad = BodyRateControllerConfig { max_integral: -1.0, ..Default::default() };
    /// assert!(bad.validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_limit("max_integral", self.max_integral)
    }
}

/// A PID-based controller for managing angular rates (roll, pitch, yaw) of a vehicle.
///
/// This controller uses a PID algorithm to compute corrective moments based on desired and estimated
/// angular rates. It is designed as the innermost control loop, directly affecting motor commands.
///
/// It holds only state (integral and derivative history); gains and limits come from the
/// [`BodyRateControllerConfig`] passed to each [`control`](Self::control) call, so they can
/// change between ticks without resetting the state.
///
/// # Units
/// - Time (`dt`) is in seconds.
/// - Desired and estimated rates are expected in rad/s.
/// - Output moments are `P·e + I·∫e + D·ė`, saturated to `[-1, 1]` per axis; the mixer scales
///   them to motor commands.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BodyRateController {
    /// Integrated error term for the PID controller.
    integral_term: Vec3,

    /// Last error value for computing the derivative term.
    last_error: Vec3,
    /// Set after a non-finite tick: the next finite tick skips the derivative term, since
    /// `last_error` predates the gap.
    resync_derivative: bool,
}

impl BodyRateController {
    /// Controls the angular velocity (body rates) of the vehicle using a PID controller.
    ///
    /// Computes the error between desired and estimated rates (both rad/s) and applies PID
    /// control to it, per axis: `config.gains` for roll and pitch, `config.yaw_gains` for yaw.
    ///
    /// # Arguments
    ///
    /// * `config` - Gains and limits; assumed valid ([`BodyRateControllerConfig::validate`]).
    ///   An invalid limit does not panic: a negative or NaN `max_integral` acts as 0.
    /// * `dt` - Time step since the last update in seconds (must be positive and finite).
    /// * `desired_rates_radps` - Desired angular rates (roll, pitch, yaw) in rad/s.
    /// * `estimated_rates_radps` - Current measured angular rates (roll, pitch, yaw) in rad/s.
    ///
    /// # Returns
    ///
    /// Moments (roll, pitch, yaw), each saturated to `[-1, 1]`.
    ///
    /// # Panics
    ///
    /// Panics if:
    /// - `dt` is not positive or finite.
    ///
    /// A non-finite component in `desired_rates_radps` or `estimated_rates_radps` returns zero moments and
    /// leaves the controller state untouched (the input policy in `architecture.md`).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::{BodyRateController, BodyRateControllerConfig};
    /// use stable_flight::Vec3;
    ///
    /// let config = BodyRateControllerConfig::default();
    /// let mut controller = BodyRateController::default();
    /// let dt = 0.004; // seconds; 250 Hz control loop
    /// let desired_rates_radps = Vec3::new(0.1, 0.0, 0.05); // roll, pitch, yaw rates (rad/s)
    /// let estimated_rates_radps = Vec3::zeros();
    ///
    /// let moments = controller.control(&config, dt, desired_rates_radps, estimated_rates_radps);
    /// assert!(moments.x > 0.0 && moments.z > 0.0);
    /// assert!(moments.iter().all(|m| (-1.0..=1.0).contains(m)));
    /// ```
    pub fn control(
        &mut self,
        config: &BodyRateControllerConfig,
        dt: f32,
        desired_rates_radps: Vec3,
        estimated_rates_radps: Vec3,
    ) -> Vec3 {
        // Input validation
        assert!(
            dt > 0.0 && dt.is_finite(),
            "Time step (dt) must be positive and finite"
        );
        if !(all_finite(&desired_rates_radps) && all_finite(&estimated_rates_radps)) {
            self.resync_derivative = true;
            return Vec3::zeros();
        }

        let error = desired_rates_radps - estimated_rates_radps;

        let error_derivative = if self.resync_derivative {
            self.resync_derivative = false;
            Vec3::zeros()
        } else {
            (error - self.last_error) / dt
        };
        self.last_error = error;

        self.integral_term += dt * error;
        self.integral_term = clamp_symmetric_each(self.integral_term, config.max_integral);

        let (g, y) = (config.gains, config.yaw_gains);
        let p = Vec3::new(g.p, g.p, y.p);
        let i = Vec3::new(g.i, g.i, y.i);
        let d = Vec3::new(g.d, g.d, y.d);
        let moment = p.component_mul(&error)
            + i.component_mul(&self.integral_term)
            + d.component_mul(&error_derivative);

        clamp_symmetric_each(moment, 1.0)
    }

    /// Resets the controller state (integral and last error terms) to zero.
    pub fn reset(&mut self) {
        self.integral_term = Vec3::zeros();
        self.last_error = Vec3::zeros();
        self.resync_derivative = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use assert2::assert;

    const DT: f32 = 0.004;

    /// The same gains on every axis.
    fn config(p: f32, i: f32, d: f32) -> BodyRateControllerConfig {
        BodyRateControllerConfig {
            gains: Gains { p, i, d },
            yaw_gains: Gains { p, i, d },
            max_integral: 1.0,
        }
    }

    /// One tick from a fresh controller is `P·e + I·e·dt` (no derivative history yet would
    /// give `D·e/dt`, so `d` is zero here), saturated to `[-1, 1]` per axis.
    #[test]
    fn first_tick_output_is_saturated_p_plus_i() {
        // (p, i, desired, estimated, expected)
        let cases = [
            (0.15, 0.0, Vec3::zeros(), Vec3::zeros(), Vec3::zeros()),
            (
                0.15,
                0.0,
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::zeros(),
                Vec3::new(0.15, 0.0, 0.0),
            ),
            (
                0.15,
                0.0,
                Vec3::new(0.0, 2.0, -2.0),
                Vec3::zeros(),
                Vec3::new(0.0, 0.3, -0.3),
            ),
            (
                0.15,
                0.0,
                Vec3::new(1.0, 1.0, 1.0),
                Vec3::new(1.0, 2.0, 0.0),
                Vec3::new(0.0, -0.15, 0.15),
            ),
            (
                0.15,
                0.0,
                Vec3::new(10.0, -10.0, 0.0),
                Vec3::zeros(),
                Vec3::new(1.0, -1.0, 0.0),
            ),
            (
                0.0,
                10.0,
                Vec3::new(5.0, 0.0, 0.0),
                Vec3::zeros(),
                Vec3::new(10.0 * 5.0 * DT, 0.0, 0.0),
            ),
        ];
        for (p, i, desired, estimated, expected) in cases {
            let mut c = BodyRateController::default();
            let out = c.control(&config(p, i, 0.0), DT, desired, estimated);
            assert!(
                (out - expected).amax() < 1e-6,
                "p={p} i={i} {desired:?}/{estimated:?}: {out:?}"
            );
        }
    }

    /// The output is independent of `dt` for a pure P controller (no hidden normalization).
    #[test]
    fn proportional_output_does_not_depend_on_dt() {
        for dt in [0.0001, 0.004, 0.02, 0.1] {
            let mut c = BodyRateController::default();
            let out = c.control(
                &config(0.15, 0.0, 0.0),
                dt,
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::zeros(),
            );
            assert_abs_diff_eq!(out.x, 0.15, epsilon = 1e-6);
        }
    }

    /// The derivative term is `D·Δe/dt`; a falling error produces a negative contribution.
    #[test]
    fn derivative_term_opposes_a_falling_error() {
        let cfg = config(0.0, 0.0, 0.003);
        let mut c = BodyRateController::default();
        c.control(&cfg, DT, Vec3::new(1.0, 0.0, 0.0), Vec3::zeros());
        let steady = c.control(&cfg, DT, Vec3::new(1.0, 0.0, 0.0), Vec3::zeros());
        let falling = c.control(&cfg, DT, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.5, 0.0, 0.0));

        assert_abs_diff_eq!(steady.x, 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(falling.x, 0.003 * -0.5 / DT, epsilon = 1e-5);
    }

    /// The accumulated error grows by `e·dt` per tick and stops at `max_integral`.
    #[test]
    fn integral_accumulates_and_clamps() {
        let cfg = config(0.0, 0.1, 0.0);
        let mut c = BodyRateController::default();
        c.control(&cfg, 0.1, Vec3::new(2.0, -2.0, 0.0), Vec3::zeros());
        assert!((c.integral_term - Vec3::new(0.2, -0.2, 0.0)).amax() < 1e-6);

        for _ in 0..20 {
            c.control(&cfg, 0.1, Vec3::new(2.0, -2.0, 0.0), Vec3::zeros());
        }
        assert!((c.integral_term - Vec3::new(1.0, -1.0, 0.0)).amax() < 1e-6);
    }

    /// Yaw uses `yaw_gains`; roll and pitch use `gains`.
    #[test]
    fn yaw_uses_its_own_gains() {
        let cfg = BodyRateControllerConfig {
            gains: Gains {
                p: 0.15,
                i: 0.0,
                d: 0.0,
            },
            yaw_gains: Gains {
                p: 0.5,
                i: 0.0,
                d: 0.0,
            },
            max_integral: 1.0,
        };
        let out = BodyRateController::default().control(
            &cfg,
            DT,
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::zeros(),
        );

        assert!((out - Vec3::new(0.15, 0.15, 0.5)).amax() < 1e-6);
    }

    #[test]
    fn reset_clears_history() {
        let mut c = BodyRateController::default();
        c.control(
            &config(0.15, 0.1, 0.003),
            DT,
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::zeros(),
        );
        c.reset();

        assert!(c.integral_term == Vec3::zeros());
        assert!(c.last_error == Vec3::zeros());
    }

    /// A gain change between ticks takes effect on the next tick and keeps the integral.
    #[test]
    fn gain_change_between_ticks_keeps_state() {
        let mut c = BodyRateController::default();
        c.control(
            &config(0.0, 0.1, 0.0),
            0.1,
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::zeros(),
        );
        let integral = c.integral_term;

        let out = c.control(&config(0.0, 0.5, 0.0), 0.1, Vec3::zeros(), Vec3::zeros());

        assert!(c.integral_term == integral);
        assert_abs_diff_eq!(out.x, 0.5 * integral.x, epsilon = 1e-6);
    }

    /// A shrunk `max_integral` clamps the existing integral on the next tick.
    #[test]
    fn shrinking_max_integral_clamps_the_integral() {
        let mut c = BodyRateController::default();
        c.control(
            &config(0.0, 0.1, 0.0),
            1.0,
            Vec3::new(0.8, 0.0, 0.0),
            Vec3::zeros(),
        );

        let shrunk = BodyRateControllerConfig {
            max_integral: 0.25,
            ..config(0.0, 0.1, 0.0)
        };
        c.control(&shrunk, DT, Vec3::zeros(), Vec3::zeros());

        assert_abs_diff_eq!(c.integral_term.x, 0.25, epsilon = 1e-6);
    }

    /// An invalid limit fails `validate` but does not panic `control`: it acts as 0.
    #[test]
    fn invalid_max_integral_is_rejected_and_does_not_panic_the_tick() {
        for max_integral in [-1.0, f32::NAN, f32::INFINITY] {
            let cfg = BodyRateControllerConfig {
                max_integral,
                ..config(0.0, 0.1, 0.0)
            };
            assert!(cfg.validate().is_err(), "max_integral {max_integral}");
        }
        for max_integral in [-1.0, f32::NAN] {
            let cfg = BodyRateControllerConfig {
                max_integral,
                ..config(0.0, 0.1, 0.0)
            };
            let mut c = BodyRateController::default();
            let out = c.control(&cfg, DT, Vec3::new(1.0, 0.0, 0.0), Vec3::zeros());
            assert!(
                c.integral_term == Vec3::zeros(),
                "max_integral {max_integral}"
            );
            assert!(out == Vec3::zeros(), "max_integral {max_integral}");
        }
    }

    #[test]
    #[should_panic(expected = "Time step (dt) must be positive and finite")]
    fn zero_dt_panics() {
        BodyRateController::default().control(
            &config(0.15, 0.0, 0.0),
            0.0,
            Vec3::zeros(),
            Vec3::zeros(),
        );
    }

    /// Non-finite desired or estimated rates yield zero moments without panicking, and the
    /// first finite tick after them has no derivative kick.
    #[test]
    fn non_finite_rates_hold_and_do_not_poison_later_ticks() {
        let nan = f32::NAN;
        let inf = f32::INFINITY;
        // (desired, estimated)
        let cases = [
            (Vec3::new(inf, 0.0, 0.0), Vec3::zeros()),
            (Vec3::zeros(), Vec3::new(0.0, nan, 0.0)),
            (Vec3::new(nan, nan, nan), Vec3::zeros()),
        ];
        for (desired, estimated) in cases {
            let cfg = config(0.15, 0.1, 0.003);
            let mut c = BodyRateController::default();
            // A valid tick first, so there is derivative history to go stale.
            c.control(&cfg, DT, Vec3::new(0.5, 0.0, 0.0), Vec3::zeros());
            let integral_before = c.integral_term;

            let during = c.control(&cfg, DT, desired, estimated);
            assert!(
                during == Vec3::zeros(),
                "{desired:?}/{estimated:?} gave {during:?}"
            );
            assert!(c.integral_term == integral_before);

            // After the gap: P + I only, no derivative from the pre-gap error.
            let after = c.control(&cfg, DT, Vec3::new(0.2, 0.0, 0.0), Vec3::zeros());
            let expected = 0.15 * 0.2 + 0.1 * c.integral_term.x;
            assert_abs_diff_eq!(after.x, expected, epsilon = 1e-6);
        }
    }
}
