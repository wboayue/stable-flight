use crate::Vec3;

use super::derivative::DerivativeState;
use super::{Gains, InvalidConfig, check_limit, rescale_integral, seed_integral};
use crate::math::{assert_dt, clamp_symmetric};

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
    /// PID gains for the roll rate.
    pub roll_gains: Gains,

    /// PID gains for the pitch rate.
    pub pitch_gains: Gains,

    /// PID gains for the yaw rate.
    pub yaw_gains: Gains,

    /// Maximum absolute value of the integrated rate error, in rad, per axis.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,
}

impl Default for BodyRateControllerConfig {
    fn default() -> Self {
        let roll_pitch_gains = Gains {
            p: DEFAULT_P,
            i: 0.0,
            d: DEFAULT_D,
        };
        Self {
            roll_gains: roll_pitch_gains,
            pitch_gains: roll_pitch_gains,
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

    /// Gains per axis: roll, pitch, yaw.
    fn axis_gains(&self) -> [Gains; 3] {
        [self.roll_gains, self.pitch_gains, self.yaw_gains]
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
    /// Roll, pitch, and yaw, each with its own PID state.
    axes: [RateAxis; 3],
}

/// One axis of the rate loop: its integral and derivative history.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct RateAxis {
    /// Integrated rate error, in rad.
    integrated_error: f32,

    /// Previous error for the derivative term; empty on the first tick and after a gap.
    derivative: DerivativeState<f32>,
}

impl RateAxis {
    /// `P·e + I·∫e + D·ė` for one axis, saturated to `[-1, 1]`. A non-finite `error_radps`
    /// gives 0 and stays out of the integral and derivative state.
    fn step(&mut self, gains: Gains, max_integral: f32, dt: f32, error_radps: f32) -> f32 {
        if !error_radps.is_finite() {
            self.derivative.forget();
            return 0.0;
        }

        let error_derivative = self.derivative.step(error_radps, dt);
        self.integrated_error =
            clamp_symmetric(self.integrated_error + dt * error_radps, max_integral);

        let moment =
            gains.p * error_radps + gains.i * self.integrated_error + gains.d * error_derivative;
        clamp_symmetric(moment, 1.0)
    }
}

impl BodyRateController {
    /// Controls the angular velocity (body rates) of the vehicle using a PID controller.
    ///
    /// Computes the error between desired and estimated rates (both rad/s) and applies PID
    /// control to it, per axis with `config.roll_gains`, `pitch_gains`, and `yaw_gains`.
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
    /// If `dt` is not positive and finite.
    ///
    /// A non-finite component in `desired_rates_radps` or `estimated_rates_radps` gives a zero
    /// moment on that axis alone and leaves that axis's state untouched, so a bad gyro axis
    /// doesn't take damping off the others (the input policy in `architecture.md`).
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
        assert_dt(dt);
        let error = desired_rates_radps - estimated_rates_radps;
        let gains = config.axis_gains();
        Vec3::from_fn(|axis, _| {
            self.axes[axis].step(gains[axis], config.max_integral, dt, error[axis])
        })
    }

    /// Rescales the integral for a change of integral gain from `old` to `new`, so the next
    /// tick's I term picks up where the last left off (`rescale_integral`).
    pub(crate) fn rescale_integral(
        &mut self,
        old: &BodyRateControllerConfig,
        new: &BodyRateControllerConfig,
    ) {
        let gains = old.axis_gains().into_iter().zip(new.axis_gains());
        for (axis, (old, new)) in self.axes.iter_mut().zip(gains) {
            axis.integrated_error = rescale_integral(axis.integrated_error, old.i, new.i);
        }
    }

    /// Zeroes every axis's integral, keeping the derivative history.
    pub(crate) fn clear_integral(&mut self) {
        for axis in &mut self.axes {
            axis.integrated_error = 0.0;
        }
    }

    /// The I term (`i · integral`) of `axis` (0 roll, 1 pitch, 2 yaw): the moment the loop
    /// holds as trim.
    pub(crate) fn integral_term(&self, config: &BodyRateControllerConfig, axis: usize) -> f32 {
        config.axis_gains()[axis].i * self.axes[axis].integrated_error
    }

    /// Restarts `axis` from a trim: its I term becomes `i_term` (`seed_integral`) and its
    /// derivative history is cleared. The other axes are untouched.
    pub(crate) fn hand_back(
        &mut self,
        config: &BodyRateControllerConfig,
        axis: usize,
        i_term: f32,
    ) {
        let i = config.axis_gains()[axis].i;
        self.axes[axis] = RateAxis {
            integrated_error: seed_integral(i_term, i, config.max_integral),
            derivative: DerivativeState::default(),
        };
    }

    /// The integrated rate error per axis, in rad.
    #[cfg(test)]
    fn integrated_error(&self) -> Vec3 {
        Vec3::from_fn(|axis, _| self.axes[axis].integrated_error)
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
            roll_gains: Gains { p, i, d },
            pitch_gains: Gains { p, i, d },
            yaw_gains: Gains { p, i, d },
            max_integral: 1.0,
        }
    }

    /// One tick from a fresh controller is `P·e + I·e·dt` (no derivative history yet; see
    /// `first_tick_has_no_derivative_kick`), saturated to `[-1, 1]` per axis.
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

    /// Closed loop against a constant yaw disturbance torque (`testing::settled_yaw_rate`),
    /// commanding zero rate: the integral cancels it up to its authority
    /// `I · max_integral`; past it, the rate settles at `−(D − I · max_integral) / P` (#192).
    #[test]
    fn integral_cancels_a_constant_yaw_torque_up_to_its_authority() {
        let cfg = BodyRateControllerConfig {
            yaw_gains: Gains {
                p: 0.6,
                i: 0.2,
                d: 0.0,
            },
            max_integral: 1.0,
            ..config(0.0, 0.0, 0.0)
        };
        // (disturbance torque, expected settled yaw rate in rad/s)
        let cases = [
            (0.0, 0.0),
            (0.15, 0.0),
            (-0.15, 0.0),
            (0.4, -0.333_333), // #192: CCW drift
            (-0.4, 0.333_333),
        ];
        for (disturbance, expected) in cases {
            let rate = crate::testing::settled_yaw_rate(&cfg, disturbance);
            assert!(
                (rate - expected).abs() < 1e-3,
                "disturbance {disturbance}: settled at {rate} rad/s, expected {expected}"
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

    /// A fresh controller has no previous error, so D alone gives nothing on the first tick.
    #[test]
    fn first_tick_has_no_derivative_kick() {
        let mut c = BodyRateController::default();
        let moments = c.control(
            &config(0.0, 0.0, 0.01),
            DT,
            Vec3::new(1.0, -2.0, 0.5),
            Vec3::zeros(),
        );
        assert!(moments == Vec3::zeros());
    }

    /// The accumulated error grows by `e·dt` per tick and stops at `max_integral`.
    #[test]
    fn integral_accumulates_and_clamps() {
        let cfg = config(0.0, 0.1, 0.0);
        let mut c = BodyRateController::default();
        c.control(&cfg, 0.1, Vec3::new(2.0, -2.0, 0.0), Vec3::zeros());
        assert!((c.integrated_error() - Vec3::new(0.2, -0.2, 0.0)).amax() < 1e-6);

        for _ in 0..20 {
            c.control(&cfg, 0.1, Vec3::new(2.0, -2.0, 0.0), Vec3::zeros());
        }
        assert!((c.integrated_error() - Vec3::new(1.0, -1.0, 0.0)).amax() < 1e-6);
    }

    /// Each axis uses its own gains: roll `roll_gains`, pitch `pitch_gains`, yaw `yaw_gains`.
    #[test]
    fn each_axis_uses_its_own_gains() {
        let p_only = |p| Gains { p, i: 0.0, d: 0.0 };
        let cfg = BodyRateControllerConfig {
            roll_gains: p_only(0.15),
            pitch_gains: p_only(0.25),
            yaw_gains: p_only(0.5),
            max_integral: 1.0,
        };
        let out = BodyRateController::default().control(
            &cfg,
            DT,
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::zeros(),
        );

        assert!((out - Vec3::new(0.15, 0.25, 0.5)).amax() < 1e-6);
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
        let integral = c.integrated_error();

        let out = c.control(&config(0.0, 0.5, 0.0), 0.1, Vec3::zeros(), Vec3::zeros());

        assert!(c.integrated_error() == integral);
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

        assert_abs_diff_eq!(c.integrated_error().x, 0.25, epsilon = 1e-6);
    }

    /// An invalid limit fails `validate` but does not panic `control`: it acts as 0.
    #[test]
    fn invalid_max_integral_is_rejected_and_does_not_panic_the_tick() {
        let invalid = BodyRateControllerConfig {
            max_integral: -1.0,
            ..config(0.0, 0.1, 0.0)
        };
        assert_eq!(invalid.validate().map_err(|e| e.field), Err("max_integral"));
        for max_integral in [-1.0, f32::NAN] {
            let cfg = BodyRateControllerConfig {
                max_integral,
                ..config(0.0, 0.1, 0.0)
            };
            let mut c = BodyRateController::default();
            let out = c.control(&cfg, DT, Vec3::new(1.0, 0.0, 0.0), Vec3::zeros());
            assert!(
                c.integrated_error() == Vec3::zeros(),
                "max_integral {max_integral}"
            );
            assert!(out == Vec3::zeros(), "max_integral {max_integral}");
        }
    }

    #[test]
    #[should_panic(expected = "dt must be positive and finite")]
    fn zero_dt_panics() {
        BodyRateController::default().control(
            &config(0.15, 0.0, 0.0),
            0.0,
            Vec3::zeros(),
            Vec3::zeros(),
        );
    }

    /// A non-finite desired or estimated rate zeroes that axis alone (#67): the other axes
    /// match a controller that never saw it, the bad axis's integral is untouched, and its
    /// first finite tick afterwards has no derivative kick.
    #[test]
    fn non_finite_rate_zeroes_only_its_axis() {
        let nan = f32::NAN;
        let inf = f32::INFINITY;
        // (desired, estimated)
        let cases = [
            (Vec3::new(inf, 0.3, -0.2), Vec3::zeros()),
            (Vec3::new(0.4, 0.3, -0.2), Vec3::new(0.0, nan, 0.0)),
            (Vec3::new(0.4, 0.3, nan), Vec3::zeros()),
            (Vec3::new(0.4, inf, 0.0), Vec3::new(0.0, inf, 0.0)), // ∞ − ∞
            (Vec3::new(nan, nan, nan), Vec3::zeros()),
        ];
        let cfg = config(0.15, 0.1, 0.003);
        let warm_up = Vec3::new(0.5, -0.4, 0.3);
        let after_gap = Vec3::new(0.2, 0.2, 0.2);
        for (desired, estimated) in cases {
            let bad = (desired - estimated).map(|e| !e.is_finite());
            let finite = |v: Vec3| v.map(|c| if c.is_finite() { c } else { 0.0 });
            let mut c = BodyRateController::default();
            let mut twin = BodyRateController::default();
            // A valid tick first, so there is derivative history to go stale.
            c.control(&cfg, DT, warm_up, Vec3::zeros());
            twin.control(&cfg, DT, warm_up, Vec3::zeros());
            let integral_before = c.integrated_error();

            let during = c.control(&cfg, DT, desired, estimated);
            let twin_during = twin.control(&cfg, DT, finite(desired), finite(estimated));
            let after = c.control(&cfg, DT, after_gap, Vec3::zeros());
            let twin_after = twin.control(&cfg, DT, after_gap, Vec3::zeros());

            for axis in 0..3 {
                let row = format!("{desired:?}/{estimated:?} axis {axis}");
                if bad[axis] {
                    assert!(during[axis] == 0.0, "{row}: {during:?}");
                    // After the gap: P + I only, no derivative from the pre-gap error.
                    let integral = integral_before[axis] + DT * after_gap[axis];
                    let expected = 0.15 * after_gap[axis] + 0.1 * integral;
                    assert!((after[axis] - expected).abs() < 1e-6, "{row}: {after:?}");
                } else {
                    assert!(during[axis] == twin_during[axis], "{row}: {during:?}");
                    assert!(after[axis] == twin_after[axis], "{row}: {after:?}");
                }
            }
        }
    }
}
