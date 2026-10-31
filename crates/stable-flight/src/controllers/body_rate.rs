use crate::Vec3;

use super::derivative::DerivativeState;
use super::{Gains, InvalidConfig, check};
use crate::math::{assert_dt, clamp_symmetric};

/// Default proportional gain, in normalized moment per rad/s of rate error.
const DEFAULT_P: f32 = 0.15;
/// Default derivative gain, in normalized moment per rad/s².
const DEFAULT_D: f32 = 0.003;
/// Roll and pitch rate I as a share of rate P, in 1/s: the autotune's rule for every gain it sets
/// (`tuning::autotune`). The I term holds a steady trim moment (a CG offset, a weak motor)
/// so the vehicle doesn't tilt while the attitude loop's slow I winds. Higher: trim held
/// sooner; the sim autotune passed every clean and delayed robustness row at 0.1–0.3 and
/// failed most at 1.0, ArduPilot's final ratio (`AUTOTUNE_PI_RATIO_FINAL`) (#374).
pub const RATE_I_PER_P: f32 = 0.2;
/// Yaw rate I as a share of yaw rate P, in 1/s: the autotune's rule for the yaw I it sets. A
/// multirotor's yaw trim is large (0.40 of the moment on QuadCopterX, #192), and after liftoff
/// the I has to rebuild it. Higher: trim back sooner; on the sim's QuadCopterX yaw the
/// liftoff heading is back within 2° in 4.9 s against 7.3 s at 0.2, with the same tuned P.
/// Past 0.3 the tuned steps overshoot more: 0.05 at 0.4, past the step spec's 0.05 at 0.5
/// (#558).
pub const YAW_RATE_I_PER_P: f32 = 0.3;
/// Default yaw proportional gain. Yaw authority comes only from differential prop drag, so
/// yaw needs more gain than roll and pitch.
const DEFAULT_YAW_P: f32 = 0.3;
/// Default yaw integral gain, in normalized moment per rad of accumulated error.
const DEFAULT_YAW_I: f32 = 0.1;
/// Default roll and pitch I term limit, in normalized moment: half of full scale, ArduPilot's
/// default rate IMAX (`AC_ATC_MULTI_RATE_RP_IMAX`). It holds a CG offset's trim whatever the
/// I gain: limited to `i` × 1 rad of error, core's default gains held 0.03, and the sim
/// autotune couldn't start under a 1 cm CG offset (#374). PX4's 0.3 (`MC_RR_INT_LIM`) passes
/// the same rows.
const DEFAULT_MAX_INTEGRAL: f32 = 0.5;
/// Default yaw I term limit, in normalized moment: ArduPilot's default yaw IMAX
/// (`AC_ATC_MULTI_RATE_YAW_IMAX`). A multirotor's yaw trim can be large: QuadCopterX hovers on
/// 0.40 of yaw moment in RealFlight (#192), past PX4's 0.3 (`MC_YR_INT_LIM`).
const DEFAULT_YAW_MAX_INTEGRAL: f32 = 0.5;

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

    /// Most moment the roll and pitch I terms hold, each, in normalized moment: the trim they
    /// can carry, and their windup limit. Like ArduPilot's rate IMAX and PX4's
    /// `MC_RR_INT_LIM`, it doesn't depend on the I gain.
    pub max_integral: f32,

    /// Most moment the yaw I term holds, in normalized moment.
    pub yaw_max_integral: f32,
}

impl Default for BodyRateControllerConfig {
    fn default() -> Self {
        let roll_pitch_gains = Gains {
            p: DEFAULT_P,
            i: RATE_I_PER_P * DEFAULT_P,
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
            yaw_max_integral: DEFAULT_YAW_MAX_INTEGRAL,
        }
    }
}

impl BodyRateControllerConfig {
    /// Checks the config: `max_integral` and `yaw_max_integral` must be in `[0, 1]`. The output
    /// clips at ±1, so a larger limit would only leave the I term free to wind up; rejecting it
    /// also catches a value saved under the old unit, rad of error (1 and 5 by default).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::BodyRateControllerConfig;
    ///
    /// assert!(BodyRateControllerConfig::default().validate().is_ok());
    /// let bad = BodyRateControllerConfig { max_integral: -1.0, ..Default::default() };
    /// assert!(bad.validate().is_err());
    /// let rad = BodyRateControllerConfig { yaw_max_integral: 5.0, ..Default::default() };
    /// assert!(rad.validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_moment_limit("max_integral", self.max_integral)?;
        check_moment_limit("yaw_max_integral", self.yaw_max_integral)
    }

    /// Gains per axis: roll, pitch, yaw.
    fn axis_gains(&self) -> [Gains; 3] {
        [self.roll_gains, self.pitch_gains, self.yaw_gains]
    }

    /// I term limits per axis, in normalized moment: roll, pitch, yaw.
    fn axis_max_integrals(&self) -> [f32; 3] {
        [self.max_integral, self.max_integral, self.yaw_max_integral]
    }
}

/// `Ok` when an I term limit is in `[0, 1]` of normalized moment.
fn check_moment_limit(field: &'static str, limit: f32) -> Result<(), InvalidConfig> {
    check(field, (0.0..=1.0).contains(&limit), "in [0, 1]")
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
/// - Output moments are `P·e + ∫I·e − D·ω̇`, saturated to `[-1, 1]` per axis; the mixer scales
///   them to motor commands.
///
/// # The I term
///
/// The integral accumulates `I·e·dt`, the moment itself, as ArduPilot's `AC_PID` and PX4's
/// `RateControl` do, and is clamped to the config's limit in moment. A change of I gain
/// changes how fast it grows, not what it holds, so a retune doesn't bump the output; setting
/// I to 0 drops it.
///
/// # D on the measured rate
///
/// D damps the measured rate's change (`ω̇`, the angular acceleration), not the error's, so a
/// setpoint step adds no derivative kick: on the error, a step of `A` rad/s would kick the
/// moment by `D·A/dt` for one tick (35·A with a designed `d` of 0.139 at 250 Hz), clipping it
/// and showing as overshoot. Inside the loop the two are the same, so stability and the
/// response to a disturbance don't change; only a setpoint change is followed by P and I alone.
/// PX4's `RateControl` takes D on the measured rate too; ArduPilot's `AC_PID` takes it on the
/// error and low-passes it (`ATC_RAT_RLL_FLTD`).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BodyRateController {
    /// Roll, pitch, and yaw, each with its own PID state.
    axes: [RateAxis; 3],
}

/// One axis of the rate loop: its I term and derivative history.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct RateAxis {
    /// The I term, `∫I·e dt`, in normalized moment.
    integral: f32,

    /// Previous measured rate for the derivative term; empty on the first tick and after a gap.
    derivative: DerivativeState<f32>,
}

impl RateAxis {
    /// `P·e + ∫I·e − D·ω̇` for one axis, with `e = desired − measured` and `ω̇` the measured
    /// rate's change, saturated to `[-1, 1]`. A non-finite error gives 0 and stays out of the
    /// integral and derivative state. A zero I gain zeroes the integral.
    ///
    /// While the moment is clipped, the integral may shrink but not grow: growing it can't
    /// add moment, and it would wind up and overshoot once the clip ends (anti-windup). The
    /// clip is this loop's own ±1; a mixer limit below it isn't seen, where ArduPilot's
    /// `AC_PID::update_i` holds the integral on the mixer's limit flags.
    fn step(
        &mut self,
        gains: Gains,
        max_integral: f32,
        dt: f32,
        desired_radps: f32,
        measured_radps: f32,
    ) -> f32 {
        let error_radps = desired_radps - measured_radps;
        if !error_radps.is_finite() {
            self.derivative.forget();
            return 0.0;
        }

        let accel_radps2 = self.derivative.step(measured_radps, dt);
        let pd = gains.p * error_radps - gains.d * accel_radps2;
        if gains.i == 0.0 {
            self.integral = 0.0;
        }
        let integral = clamp_symmetric(self.integral + gains.i * error_radps * dt, max_integral);
        let clipped = (pd + integral).abs() > 1.0;
        if !clipped || integral.abs() < self.integral.abs() {
            self.integral = integral;
        }

        clamp_symmetric(pd + self.integral, 1.0)
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
    ///   An invalid limit does not panic: a negative or NaN integral limit acts as 0.
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
        let gains = config.axis_gains();
        let max_integrals = config.axis_max_integrals();
        Vec3::from_fn(|axis, _| {
            self.axes[axis].step(
                gains[axis],
                max_integrals[axis],
                dt,
                desired_rates_radps[axis],
                estimated_rates_radps[axis],
            )
        })
    }

    /// Zeroes every axis's integral, keeping the derivative history.
    pub(crate) fn clear_integral(&mut self) {
        for axis in &mut self.axes {
            axis.integral = 0.0;
        }
    }

    /// The I term of `axis` (0 roll, 1 pitch, 2 yaw): the moment the loop holds as trim.
    #[cfg(test)]
    pub(crate) fn integral_term(&self, axis: usize) -> f32 {
        self.axes[axis].integral
    }

    /// Restarts `axis` with no derivative history, keeping its integral. The other axes are
    /// untouched.
    pub(crate) fn hand_back(&mut self, axis: usize) {
        self.axes[axis].derivative = DerivativeState::default();
    }

    /// The I term per axis, in normalized moment.
    #[cfg(test)]
    fn integral(&self) -> Vec3 {
        Vec3::from_fn(|axis, _| self.axes[axis].integral)
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
            yaw_max_integral: 1.0,
        }
    }

    /// Handing back one axis clears its derivative history and keeps its integral; the other
    /// axes keep both.
    #[test]
    fn hand_back_restarts_one_axis_only() {
        let config = config(1.0, 2.0, 0.5);
        let mut controller = BodyRateController::default();
        controller.control(&config, DT, Vec3::new(0.5, 0.5, 0.5), Vec3::zeros());
        let before = controller.axes;

        controller.hand_back(0);

        assert!(controller.axes[0].integral == before[0].integral);
        assert!(controller.axes[0].derivative == DerivativeState::default());
        assert!(controller.axes[1..] == before[1..]);
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
    /// commanding zero rate: the integral cancels it up to `yaw_max_integral`; past it, the
    /// rate settles at `−(D − yaw_max_integral) / P` (#192).
    #[test]
    fn integral_cancels_a_constant_yaw_torque_up_to_its_authority() {
        let cfg = BodyRateControllerConfig {
            yaw_gains: Gains {
                p: 0.6,
                i: 0.2,
                d: 0.0,
            },
            yaw_max_integral: 0.2,
            ..config(0.0, 0.0, 0.0)
        };
        // (disturbance torque, expected settled yaw rate in rad/s)
        let cases = [
            (0.0, 0.0),
            (0.15, 0.0),
            (-0.15, 0.0),
            (0.4, -0.333_333), // past the authority: CCW drift (#192)
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

    /// The derivative term is `−D·Δω/dt` on the measured rate: a rising rate is damped.
    #[test]
    fn derivative_term_opposes_a_rising_rate() {
        let cfg = config(0.0, 0.0, 0.003);
        let mut c = BodyRateController::default();
        c.control(&cfg, DT, Vec3::new(1.0, 0.0, 0.0), Vec3::zeros());
        let steady = c.control(&cfg, DT, Vec3::new(1.0, 0.0, 0.0), Vec3::zeros());
        let rising = c.control(&cfg, DT, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.5, 0.0, 0.0));

        assert_abs_diff_eq!(steady.x, 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(rising.x, 0.003 * -0.5 / DT, epsilon = 1e-5);
    }

    /// A setpoint step on a held rate moves the moment by `P·A` alone: no `D·A/dt` kick, for
    /// any step size or D (#373).
    #[test]
    fn a_setpoint_step_has_no_derivative_kick() {
        // (step in rad/s, d): the d's span the hand tune (0.003) to a designed quad_x (0.139).
        let rows = [
            (1.0, 0.003),
            (-1.0, 0.003),
            (0.2, 0.139),
            (1.0, 0.139),
            (-3.0, 0.05),
        ];
        let p = 0.15;
        for (step_radps, d) in rows {
            let cfg = config(p, 0.0, d);
            let mut c = BodyRateController::default();
            let measured = Vec3::new(0.3, 0.3, 0.3);
            c.control(&cfg, DT, measured, measured);
            let stepped = measured + Vec3::repeat(step_radps);
            let moments = c.control(&cfg, DT, stepped, measured);
            let expected = clamp_symmetric(p * step_radps, 1.0);
            assert!(
                (moments - Vec3::repeat(expected)).amax() < 1e-6,
                "step {step_radps} rad/s, d {d}: {moments:?}"
            );
        }
    }

    /// A fresh controller has no previous rate, so D alone gives nothing on the first tick.
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

    /// The I term grows by `I·e·dt` per tick and stops at the axis's limit, in moment:
    /// `max_integral` for roll and pitch, `yaw_max_integral` for yaw.
    #[test]
    fn integral_accumulates_and_clamps_per_axis() {
        let cfg = BodyRateControllerConfig {
            max_integral: 0.1,
            yaw_max_integral: 0.15,
            ..config(0.0, 0.1, 0.0)
        };
        let error = Vec3::new(2.0, -2.0, 2.0);
        let mut c = BodyRateController::default();
        c.control(&cfg, 0.1, error, Vec3::zeros());
        assert!((c.integral() - Vec3::new(0.02, -0.02, 0.02)).amax() < 1e-6);

        for _ in 0..20 {
            c.control(&cfg, 0.1, error, Vec3::zeros());
        }
        assert!((c.integral() - Vec3::new(0.1, -0.1, 0.15)).amax() < 1e-6);
    }

    /// A limit set to the I gain × the old limit in rad of error flies as the old law did,
    /// windup and clipping included: the conversion the airframes took (#374).
    #[test]
    fn a_limit_of_i_times_the_rad_limit_flies_as_the_rad_limit_did() {
        /// The rate loop before #374, one axis: the error integrated in rad and clamped there.
        fn rad_law(
            integrated_error: &mut f32,
            gains: Gains,
            limit_rad: f32,
            dt: f32,
            e: f32,
        ) -> f32 {
            let pd = gains.p * e;
            let integral = clamp_symmetric(*integrated_error + dt * e, limit_rad);
            let clipped = (pd + gains.i * integral).abs() > 1.0;
            if !clipped || integral.abs() < integrated_error.abs() {
                *integrated_error = integral;
            }
            clamp_symmetric(pd + gains.i * *integrated_error, 1.0)
        }
        // (airframe, roll I, the old limit in rad): the converted RealFlight and X-Plane sets.
        let rows = [
            (
                "eXplora pitch",
                Gains {
                    p: 1.4217,
                    i: 0.2844,
                    d: 0.0,
                },
                1.0,
            ),
            (
                "Cobra roll",
                Gains {
                    p: 0.4847,
                    i: 0.0969,
                    d: 0.0,
                },
                3.0,
            ),
            (
                "Alia",
                Gains {
                    p: 1.52,
                    i: 0.2,
                    d: 0.0,
                },
                1.0,
            ),
        ];
        // A trim held long enough to reach the limit, a clipping push, then release.
        let errors = (0..2000).map(|tick| match tick {
            0..1000 => 0.8,
            1000..1100 => 3.0,
            1100..1500 => -0.5,
            _ => 0.05,
        });
        for (name, gains, limit_rad) in rows {
            let cfg = BodyRateControllerConfig {
                roll_gains: gains,
                max_integral: gains.i * limit_rad,
                ..config(0.0, 0.0, 0.0)
            };
            let mut c = BodyRateController::default();
            let mut integrated_error = 0.0;
            for (tick, e) in errors.clone().enumerate() {
                let new = c.control(&cfg, DT, Vec3::new(e, 0.0, 0.0), Vec3::zeros()).x;
                let old = rad_law(&mut integrated_error, gains, limit_rad, DT, e);
                assert!(
                    (new - old).abs() < 1e-5,
                    "{name} tick {tick}: {new} vs {old}"
                );
            }
        }
    }

    /// The limit is in moment whatever the I gain: a small I holds as much trim as a large
    /// one, given time (#374).
    #[test]
    fn integral_limit_does_not_scale_with_the_i_gain() {
        for i in [0.03, 0.3, 3.0] {
            let cfg = config(0.0, i, 0.0);
            let mut c = BodyRateController::default();
            for _ in 0..10_000 {
                c.control(&cfg, DT, Vec3::new(1.0, 0.0, 0.0), Vec3::zeros());
            }
            assert_abs_diff_eq!(c.integral().x, cfg.max_integral, epsilon = 1e-6);
        }
    }

    /// While the moment is clipped, the integral shrinks but doesn't grow; unclipped, it
    /// grows as usual.
    #[test]
    fn clipped_moment_stops_the_integral_growing() {
        let start = 0.5;
        // (case, p, error, expected integral after one 0.1 s tick at I 0.1)
        let rows = [
            ("unclipped grows", 0.1, 1.0, start + 0.01),
            ("clipped toward the error holds", 2.0, 1.0, start),
            (
                "clipped against the integral shrinks",
                2.0,
                -1.0,
                start - 0.01,
            ),
            (
                "clipped, error crossing zero shrinks",
                2.0,
                -0.6,
                start - 0.006,
            ),
        ];
        for (case, p, error, expected) in rows {
            let cfg = config(p, 0.1, 0.0);
            let mut c = BodyRateController::default();
            c.axes[0].integral = start;
            let out = c.control(&cfg, 0.1, Vec3::new(error, 0.0, 0.0), Vec3::zeros());
            let integral = c.integral().x;
            assert!((integral - expected).abs() < 1e-6, "{case}: {integral}");
            assert!(out.x.abs() <= 1.0, "{case}");
        }
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
            yaw_max_integral: 1.0,
        };
        let out = BodyRateController::default().control(
            &cfg,
            DT,
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::zeros(),
        );

        assert!((out - Vec3::new(0.15, 0.25, 0.5)).amax() < 1e-6);
    }

    /// An I gain change between ticks keeps the I term, so the output doesn't bump; a zero I
    /// gain drops it, and it restarts from zero.
    #[test]
    fn i_gain_change_keeps_the_i_term_or_drops_it_at_zero() {
        // (case, new I, expected I term after a tick at zero error)
        let rows = [
            ("raised", 0.5, 0.02),
            ("lowered", 0.01, 0.02),
            ("zeroed", 0.0, 0.0),
        ];
        for (case, new_i, expected) in rows {
            let mut c = BodyRateController::default();
            c.control(
                &config(0.0, 0.1, 0.0),
                0.1,
                Vec3::new(2.0, 0.0, 0.0),
                Vec3::zeros(),
            );

            let out = c.control(&config(0.0, new_i, 0.0), 0.1, Vec3::zeros(), Vec3::zeros());

            assert!((c.integral().x - expected).abs() < 1e-6, "{case}");
            assert!((out.x - expected).abs() < 1e-6, "{case}");
        }
    }

    /// A shrunk `max_integral` clamps the existing integral on the next tick.
    #[test]
    fn shrinking_max_integral_clamps_the_integral() {
        let mut c = BodyRateController::default();
        c.control(
            &config(0.0, 0.1, 0.0),
            1.0,
            Vec3::new(8.0, 0.0, 0.0),
            Vec3::zeros(),
        );

        let shrunk = BodyRateControllerConfig {
            max_integral: 0.25,
            ..config(0.0, 0.1, 0.0)
        };
        c.control(&shrunk, DT, Vec3::zeros(), Vec3::zeros());

        assert_abs_diff_eq!(c.integral().x, 0.25, epsilon = 1e-6);
    }

    /// An invalid limit fails `validate` but does not panic `control`: a negative or NaN one
    /// acts as 0 on its axes. Past 1 (a value in the old rad unit) fails too.
    #[test]
    fn invalid_max_integral_is_rejected_and_does_not_panic_the_tick() {
        type SetLimit = fn(&mut BodyRateControllerConfig, f32);
        // (field, how to set it, the error axis it limits)
        let rows: [(&str, SetLimit, Vec3); 2] = [
            ("max_integral", |c, v| c.max_integral = v, Vec3::x()),
            ("yaw_max_integral", |c, v| c.yaw_max_integral = v, Vec3::z()),
        ];
        for (field, set, error) in rows {
            for limit in [5.0, 1.01] {
                let mut cfg = config(0.0, 0.1, 0.0);
                set(&mut cfg, limit);
                assert_eq!(cfg.validate().map_err(|e| e.field), Err(field));
            }
            for limit in [-1.0, f32::NAN] {
                let mut cfg = config(0.0, 0.1, 0.0);
                set(&mut cfg, limit);
                assert_eq!(cfg.validate().map_err(|e| e.field), Err(field));
                let mut c = BodyRateController::default();
                let out = c.control(&cfg, DT, error, Vec3::zeros());
                assert!(c.integral() == Vec3::zeros(), "{field} {limit}");
                assert!(out == Vec3::zeros(), "{field} {limit}");
            }
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

    /// A non-finite desired or estimated rate zeroes that axis alone: the other axes
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
            let integral_before = c.integral();

            let during = c.control(&cfg, DT, desired, estimated);
            let twin_during = twin.control(&cfg, DT, finite(desired), finite(estimated));
            let after = c.control(&cfg, DT, after_gap, Vec3::zeros());
            let twin_after = twin.control(&cfg, DT, after_gap, Vec3::zeros());

            for axis in 0..3 {
                let row = format!("{desired:?}/{estimated:?} axis {axis}");
                if bad[axis] {
                    assert!(during[axis] == 0.0, "{row}: {during:?}");
                    // After the gap: P + I only, no derivative from the pre-gap error.
                    let integral = integral_before[axis] + 0.1 * after_gap[axis] * DT;
                    let expected = 0.15 * after_gap[axis] + integral;
                    assert!((after[axis] - expected).abs() < 1e-6, "{row}: {after:?}");
                } else {
                    assert!(during[axis] == twin_during[axis], "{row}: {during:?}");
                    assert!(after[axis] == twin_after[axis], "{row}: {after:?}");
                }
            }
        }
    }
}
