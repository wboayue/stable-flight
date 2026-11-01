use crate::Vec2;

use super::derivative::DerivativeState;
use crate::controllers::{Gains, InvalidConfig, check_limit, rescale_integral};
use crate::math::{
    all_finite, assert_dt, cap_magnitude, clamp_symmetric_each, shaped_closing_speed,
};

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
    /// Bounds the I term's authority at `velocity.i · max_integral` m/s².
    pub max_integral: f32,

    /// Most the velocity target changes per second, in m/s²; 0 is off. When set, the target
    /// starts at the measured velocity and moves toward the position loop's demand at this
    /// rate, its change fed forward as acceleration (PX4 `MPC_ACC_HOR`,
    /// `FlightTaskAuto::activate`; ArduPilot `AC_PosControl::NE_init_controller`).
    /// A new leg then asks for at most `atan(max_accel_mps2 / g)` of tilt at once, instead of
    /// the tilt for its whole speed. The position loop then slows toward the target along a
    /// braking curve: the speed that decelerating at [`BRAKING_SHARE`] of this rate still stops
    /// on the target, so stopping from `v` starts about `v² / (2 · BRAKING_SHARE ·
    /// max_accel_mps2)` out. Must be non-negative and finite.
    pub max_accel_mps2: f32,

    /// Deceleration the position loop brakes at with shaping off, in m/s²; 0 is off. When set,
    /// the speed toward the target follows the braking curve at this rate, instead of
    /// `position_p · error`, which starts braking only `2 · v` meters out whatever the
    /// vehicle's speed. Higher: brakes later, overshoots once past what the vehicle can
    /// shed. Ignored with shaping on, which brakes at [`BRAKING_SHARE`] of
    /// [`max_accel_mps2`](Self::max_accel_mps2). Must be non-negative and finite.
    pub brake_mps2: f32,
}

impl Default for LateralControllerConfig {
    fn default() -> Self {
        Self {
            // Unsaturated, `velocity.p · (position_p · e − v)` is a flight-checked position PD
            // for the atan tilt: P 0.63, D 1.26, ζ ≈ 0.8 (#108).
            position_p: 0.5,
            // I trims a steady wind; at 0.4 its closed-loop pole (≈ 0.26 /s) leaves the step
            // response unchanged (#357).
            velocity: Gains {
                p: 1.26,
                i: 0.4,
                d: 0.0,
            },
            // 12.5 m at I 0.4 is ±5 m/s², the default 27° tilt cap (g · tan 27°).
            max_integral: 12.5,
            max_accel_mps2: 0.0,
            brake_mps2: 0.0,
        }
    }
}

impl LateralControllerConfig {
    /// Checks the config: `position_p`, `max_integral`, `max_accel_mps2`, and `brake_mps2`
    /// must be non-negative and finite.
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
        check_limit("max_integral", self.max_integral)?;
        check_limit("max_accel_mps2", self.max_accel_mps2)?;
        check_limit("brake_mps2", self.brake_mps2)
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
/// error into acceleration. With [`max_accel_mps2`](LateralControllerConfig::max_accel_mps2)
/// set, the position loop slows toward the target along a braking curve instead of in
/// proportion to the distance, and the PID tracks a shaped velocity target: it starts at the
/// measured velocity, moves toward the demand at that rate, and its change is fed forward.
/// With shaping off, [`brake_mps2`](LateralControllerConfig::brake_mps2) sets the braking
/// curve alone, so a fast leg starts slowing in time to stop.
/// Turning the acceleration into tilt is the guidance's job (it depends on the airframe and its
/// heading); [`CopterGuidance`](crate::guidance::copter::CopterGuidance) tilts by `atan(|a| /
/// g)` toward it.
///
/// It holds only state (the velocity integral, derivative history, and shaped velocity
/// target); gains and limits come from the [`LateralControllerConfig`] passed to each
/// [`control`](Self::control) call, so they can change between ticks without resetting the
/// state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LateralController {
    integrated_error: Vec2, // Accumulated velocity error (x, y)
    /// Previous velocity error for the derivative term; empty on the first tick and after a gap.
    derivative: DerivativeState<Vec2>,
    /// The shaped velocity target, in m/s; empty until the first run with shaping on.
    shaped_velocity_mps: Option<Vec2>,
    /// The velocity the last finite run asked the velocity PID for, in m/s.
    commanded_velocity_mps: Vec2,
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
    /// * `limit_mps2` - The acceleration the caller last had to cap (e.g. at its tilt limit), or
    ///   zero. The integral doesn't grow in that direction, so it can't wind up against a
    ///   limit; it may still shrink or turn.
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
    /// let accel_mps2 = controller.control(&config, 0.004, target, estimate, Vec2::zeros());
    /// assert!(accel_mps2.x > 0.0 && accel_mps2.y > 0.0);
    /// ```
    pub fn control(
        &mut self,
        config: &LateralControllerConfig,
        dt: f32,
        target: LateralTarget,
        estimate: LateralEstimate,
        limit_mps2: Vec2,
    ) -> Vec2 {
        assert_dt(dt);
        let gains = &config.velocity;

        // `>` is false for NaN: zero, negative, and NaN limits all mean no cap.
        let max_speed_mps = if target.max_speed_mps > 0.0 {
            target.max_speed_mps
        } else {
            f32::INFINITY
        };
        let desired_velocity = closing_velocity(
            target.position_m - estimate.position_m,
            config.position_p,
            braking_decel_mps2(config),
        ) + target.velocity_mps;
        let (commanded_velocity, feedforward_mps2) = self.shape(
            config.max_accel_mps2,
            dt,
            cap_magnitude(desired_velocity, max_speed_mps),
            estimate.velocity_mps,
        );
        let velocity_error = commanded_velocity - estimate.velocity_mps;

        // A non-finite target or estimate yields no acceleration and never enters the integral
        // or derivative history, where NaN would stick (it survives clamping). `cap_magnitude`
        // zeroes a non-finite demand, so check its input too.
        if !(all_finite(&desired_velocity) && all_finite(&velocity_error)) {
            self.derivative.forget();
            return Vec2::zeros();
        }
        let error_derivative = self.derivative.step(velocity_error, dt);
        self.commanded_velocity_mps = commanded_velocity;

        self.integrated_error = clamp_symmetric_each(
            hold_against_limit(
                self.integrated_error,
                dt * velocity_error,
                gains.i,
                limit_mps2,
            ),
            config.max_integral,
        );
        feedforward_mps2
            + gains.p * velocity_error
            + gains.i * self.integrated_error
            + gains.d * error_derivative
    }

    /// The velocity target to track and the acceleration to feed forward: `demand_mps` and
    /// zero with shaping off (`max_accel_mps2` 0), which forgets the shaped target so turning
    /// it on starts again from the measured velocity; else the shaped target moved toward the
    /// demand by at most `max_accel_mps2 · dt`, starting from `measured_mps`, and its change
    /// per second. A non-finite demand or measurement passes through unshaped (the caller
    /// neutralizes it) and leaves the shaped target as it was.
    fn shape(
        &mut self,
        max_accel_mps2: f32,
        dt: f32,
        demand_mps: Vec2,
        measured_mps: Vec2,
    ) -> (Vec2, Vec2) {
        // `>` is false for NaN: an invalid limit acts as off.
        let shaping = max_accel_mps2 > 0.0;
        if !shaping {
            self.shaped_velocity_mps = None;
            return (demand_mps, Vec2::zeros());
        }
        if !(all_finite(&demand_mps) && all_finite(&measured_mps)) {
            return (demand_mps, Vec2::zeros());
        }
        let from = self.shaped_velocity_mps.unwrap_or(measured_mps);
        let to = from + cap_magnitude(demand_mps - from, max_accel_mps2 * dt);
        self.shaped_velocity_mps = Some(to);
        (to, (to - from) / dt)
    }

    /// The velocity, in m/s (north, east), the last finite [`control`](Self::control) asked the
    /// velocity PID to track: the position loop's demand, shaped when shaping is on. Zero
    /// before the first.
    pub(crate) fn commanded_velocity_mps(&self) -> Vec2 {
        self.commanded_velocity_mps
    }

    /// The accumulated error, for tests of the composites' integrator policy.
    #[cfg(test)]
    pub(crate) fn integrated_error(&self) -> Vec2 {
        self.integrated_error
    }

    /// Rescales the integral for a change of integral gain from `old` to `new`, so the next
    /// tick's I term picks up where the last left off (`rescale_integral`).
    pub(crate) fn rescale_integral(
        &mut self,
        old: &LateralControllerConfig,
        new: &LateralControllerConfig,
    ) {
        let (old_i, new_i) = (old.velocity.i, new.velocity.i);
        self.integrated_error = self
            .integrated_error
            .map(|e| rescale_integral(e, old_i, new_i));
    }
}

/// Share of [`max_accel_mps2`](LateralControllerConfig::max_accel_mps2) the position loop's
/// braking curve slows at. The curve is read at the vehicle's distance, and the vehicle trails
/// its shaped velocity target, so it runs ahead of the curve: there the curve's speed drops
/// faster than its deceleration. Braking below the shaping rate leaves the rate-limited target
/// room to follow. Higher: arrives sooner. On a 260 m
/// quadplane leg from 0.5 to 4 m/s², 0.7 overshoots at most 0.7 m, 0.9 up to 3.5 m, and 1.0
/// up to 6.4 m (#466).
pub const BRAKING_SHARE: f32 = 0.7;

/// The deceleration the position loop's braking curve slows at, in m/s²: [`BRAKING_SHARE`] of
/// `max_accel_mps2` with shaping on, else `brake_mps2`; 0 when neither is set, for the linear
/// loop. An invalid value counts as unset.
fn braking_decel_mps2(config: &LateralControllerConfig) -> f32 {
    // `>` is false for NaN: an invalid limit acts as off.
    if config.max_accel_mps2 > 0.0 {
        BRAKING_SHARE * config.max_accel_mps2
    } else if config.brake_mps2 > 0.0 {
        config.brake_mps2
    } else {
        0.0
    }
}

/// The position loop's velocity toward a target `error_m` away (north, east): `position_p ·
/// error` with no braking deceleration (`decel_mps2` 0), else the braking curve's speed at
/// `decel_mps2` ([`shaped_closing_speed`]), pointed at the target. A non-finite error passes
/// through, so the caller's check sees it.
fn closing_velocity(error_m: Vec2, position_p: f32, decel_mps2: f32) -> Vec2 {
    if decel_mps2 <= 0.0 {
        return position_p * error_m;
    }
    let distance_m = error_m.norm();
    if !(distance_m > 0.0 && distance_m.is_finite()) {
        return position_p * error_m;
    }
    let speed_mps = shaped_closing_speed(distance_m, position_p, decel_mps2);
    error_m * (speed_mps / distance_m)
}

/// `integral + step`, except that a step whose I term pushes along `limit_mps2` may not
/// lengthen the integral (it can still turn it): conditional integration against a capped
/// output. A zero or non-finite `limit_mps2` never holds.
fn hold_against_limit(integral: Vec2, step: Vec2, i: f32, limit_mps2: Vec2) -> Vec2 {
    let next = integral + step;
    // `>` is false for NaN, so a non-finite limit integrates freely.
    if i * step.dot(&limit_mps2) > 0.0 {
        cap_magnitude(next, integral.norm())
    } else {
        next
    }
}

#[cfg(test)]
mod tests {
    use assert2::check;

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
            let accel =
                controller.control(&p_only(), 0.004, target, moving(velocity), Vec2::zeros());
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
                Vec2::zeros(),
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
            Vec2::zeros(),
        );
        let at_rest = LateralController::default().control(
            &config,
            0.004,
            target,
            moving(Vec2::zeros()),
            Vec2::zeros(),
        );
        assert!(cruising == Vec2::zeros());
        assert!((at_rest - Vec2::new(14.0, 0.0)).amax() < 1e-5);
    }

    /// Below the cap, the default cascade is the flight-checked position PD it replaced,
    /// `0.63 · e − 1.26 · v`, plus the velocity I, here one tick of it.
    #[test]
    fn default_gains_are_the_position_pd_plus_velocity_i() {
        let config = LateralControllerConfig::default();
        const DT: f32 = 0.004;
        // (position error, velocity)
        let cases = [
            (Vec2::new(2.0, -1.0), Vec2::zeros()),
            (Vec2::zeros(), Vec2::new(1.0, 0.5)),
            (Vec2::new(-3.0, 4.0), Vec2::new(0.4, -0.2)),
        ];
        for (error, velocity) in cases {
            let accel = LateralController::default().control(
                &config,
                DT,
                at(error),
                moving(velocity),
                Vec2::zeros(),
            );
            let velocity_error = 0.5 * error - velocity;
            let expected = 0.63 * error - 1.26 * velocity + 0.4 * DT * velocity_error;
            assert!(
                (accel - expected).amax() < 1e-5,
                "error {error:?}, velocity {velocity:?} gave {accel:?}"
            );
        }
    }

    /// Against a constant disturbance (a steady wind's drag), the default gains settle on the
    /// target rather than offset from it: drag over the position-to-tilt gain without I.
    #[test]
    fn default_gains_trim_a_constant_disturbance() {
        const DT: f32 = 0.02;
        const DRAG_MPS2: f32 = 1.5;
        let config = LateralControllerConfig::default();
        let mut controller = LateralController::default();
        let (mut position_m, mut velocity_mps) = (Vec2::zeros(), Vec2::zeros());
        let drag = Vec2::new(DRAG_MPS2, 0.0);
        for _ in 0..(60.0 / DT) as usize {
            let estimate = LateralEstimate {
                position_m,
                velocity_mps,
            };
            let accel = controller.control(&config, DT, at(Vec2::zeros()), estimate, Vec2::zeros());
            velocity_mps += (accel + drag) * DT;
            position_m += velocity_mps * DT;
        }
        assert!(position_m.norm() < 0.01, "settled at {position_m:?}");
    }

    /// While `limit` points along the I term's push, the integral can't lengthen; it still
    /// shrinks or turns, and a zero, opposed, or non-finite limit doesn't hold.
    #[test]
    fn integral_is_held_against_the_limit() {
        let x = Vec2::new(1.0, 0.0);
        let y = Vec2::new(0.0, 1.0);
        // (integral, step, i, limit, expected)
        let cases = [
            (x, x, 0.4, Vec2::zeros(), 2.0 * x),            // no limit
            (x, x, 0.4, x, x),                              // held at its length
            (Vec2::zeros(), x, 0.4, x, Vec2::zeros()),      // can't start along the limit
            (x, -x, 0.4, x, Vec2::zeros()),                 // shrinks
            (x, 0.5 * x, 0.4, -x, 1.5 * x),                 // limit the other way
            (x, y, 0.4, y, (x + y) / 2.0_f32.sqrt()),       // turns, keeps length
            (x, x, 0.0, x, 2.0 * x),                        // no I term: nothing to hold
            (x, x, 0.4, Vec2::new(f32::NAN, 0.0), 2.0 * x), // non-finite limit
        ];
        for (integral, step, i, limit, expected) in cases {
            let held = hold_against_limit(integral, step, i, limit);
            assert!(
                (held - expected).amax() < 1e-6,
                "{integral:?} + {step:?} against {limit:?} gave {held:?}, expected {expected:?}"
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
            let accel = controller.control(
                &config,
                0.004,
                at(Vec2::new(10.0, 5.0)),
                moving(velocity),
                Vec2::zeros(),
            );
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
                Vec2::zeros(),
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
        let during = controller.control(
            &config,
            0.004,
            at(Vec2::new(f32::NAN, 0.0)),
            at_rest,
            Vec2::zeros(),
        );
        let after = controller.control(
            &config,
            0.004,
            at(Vec2::new(10.0, 0.0)),
            at_rest,
            Vec2::zeros(),
        );

        assert!(during == Vec2::zeros(), "NaN tick gave {during:?}");
        let fresh = LateralController::default().control(
            &config,
            0.004,
            at(Vec2::new(10.0, 0.0)),
            at_rest,
            Vec2::zeros(),
        );
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
        let first = controller.control(
            &config,
            0.1,
            target,
            moving(Vec2::new(1.0, 0.0)),
            Vec2::zeros(),
        );
        let second = controller.control(
            &config,
            0.1,
            target,
            moving(Vec2::new(2.0, 0.0)),
            Vec2::zeros(),
        );
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
                Vec2::zeros(),
            );
            assert!(
                controller.integrated_error == Vec2::zeros(),
                "max_integral {max_integral}"
            );
        }
    }

    /// `p_only` with the velocity target shaped at `max_accel_mps2`.
    fn shaped(max_accel_mps2: f32) -> LateralControllerConfig {
        LateralControllerConfig {
            max_accel_mps2,
            ..p_only()
        }
    }

    /// 30 m north at up to 3 m/s: a 3 m/s demand north.
    fn leg() -> LateralTarget {
        LateralTarget {
            max_speed_mps: 3.0,
            ..at(Vec2::new(30.0, 0.0))
        }
    }

    /// The first run's shaped target starts at the measured velocity and moves at most
    /// `max_accel_mps2 · dt` toward the demand, that change fed forward; 0 is off.
    #[test]
    fn shaped_target_starts_at_the_measured_velocity() {
        let dt = 0.1;
        // Sideways at 2 m/s east, toward a 3 m/s demand north: a 0.1 m/s step along (3, −2).
        let turn = Vec2::new(3.0, -2.0).normalize();
        // (max_accel_mps2, measured velocity, expected acceleration: feedforward + P 2 · error)
        let cases = [
            (0.0, Vec2::zeros(), Vec2::new(6.0, 0.0)), // off: the whole 3 m/s error
            (1.0, Vec2::zeros(), Vec2::new(1.0 + 2.0 * 0.1, 0.0)),
            (1.0, Vec2::new(3.0, 0.0), Vec2::zeros()), // already at the demand
            (1.0, Vec2::new(0.0, 2.0), turn + 2.0 * 0.1 * turn),
            // Within one step: the demand at once, its 3 m/s in 0.1 s fed forward.
            (100.0, Vec2::zeros(), Vec2::new(30.0 + 2.0 * 3.0, 0.0)),
        ];
        for (max_accel_mps2, velocity, expected) in cases {
            let accel = LateralController::default().control(
                &shaped(max_accel_mps2),
                dt,
                leg(),
                moving(velocity),
                Vec2::zeros(),
            );
            check!(
                (accel - expected).amax() < 1e-4,
                "limit {max_accel_mps2}, velocity {velocity:?}: {accel:?}, expected {expected:?}"
            );
        }
    }

    /// From rest with the vehicle held still, the shaped target ramps to the demand at the
    /// limit, then stops ramping: its feedforward ends and the PID sees the whole error.
    #[test]
    fn shaped_target_ramps_to_the_demand() {
        let (dt, config) = (0.1, shaped(1.0));
        let mut controller = LateralController::default();
        let mut accels = Vec::new();
        for _ in 0..32 {
            accels.push(controller.control(
                &config,
                dt,
                leg(),
                moving(Vec2::zeros()),
                Vec2::zeros(),
            ));
        }
        // (tick, expected north acceleration: feedforward + P 2 · shaped speed)
        let cases = [
            (0, 1.0 + 2.0 * 0.1),
            (14, 1.0 + 2.0 * 1.5),
            (29, 1.0 + 2.0 * 3.0), // the last step reaches 3 m/s
            (30, 2.0 * 3.0),
            (31, 2.0 * 3.0),
        ];
        for (tick, expected) in cases {
            let accel = accels[tick];
            check!(
                (accel - Vec2::new(expected, 0.0)).amax() < 1e-3,
                "tick {tick}: {accel:?}, expected {expected}"
            );
        }
    }

    /// The braking curve slows at `BRAKING_SHARE` of the shaping limit with shaping on, whatever
    /// `brake_mps2`; at `brake_mps2` with it off; and not at all (linear) with neither, an
    /// invalid value acting as unset.
    #[test]
    fn braking_decel_comes_from_shaping_then_brake() {
        // (max_accel_mps2, brake_mps2, expected decel in m/s²)
        let cases = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.7),
            (1.0, 1.5, 0.7), // shaping wins
            (0.0, 1.5, 1.5),
            (f32::NAN, 1.5, 1.5),
            (-1.0, 1.5, 1.5),
            (0.0, f32::NAN, 0.0),
            (0.0, -1.0, 0.0),
        ];
        for (max_accel_mps2, brake_mps2, expected) in cases {
            let config = LateralControllerConfig {
                max_accel_mps2,
                brake_mps2,
                ..LateralControllerConfig::default()
            };
            let decel = braking_decel_mps2(&config);
            check!(
                (decel - expected).abs() < 1e-6,
                "shaping {max_accel_mps2}, brake {brake_mps2}: {decel}, expected {expected}"
            );
        }
    }

    /// The position loop's speed toward the target follows the braking curve at its
    /// deceleration, pointed at the target: linear within `a / p²` (2.8 m at 0.7 m/s² and P
    /// 0.5), the square root beyond; with none, `p · error`.
    #[test]
    fn closing_velocity_follows_the_braking_curve() {
        let p = 0.5;
        // (error, decel_mps2, expected velocity)
        let cases = [
            (Vec2::new(30.0, 0.0), 0.0, Vec2::new(15.0, 0.0)), // none: linear
            (Vec2::new(2.0, 0.0), 0.7, Vec2::new(1.0, 0.0)),   // inside the linear part
            // √(2 · 0.7 · (30 − 1.4)) beyond it
            (Vec2::new(30.0, 0.0), 0.7, Vec2::new(6.3277, 0.0)),
            (Vec2::new(0.0, -30.0), 0.7, Vec2::new(0.0, -6.3277)),
            (Vec2::new(18.0, 24.0), 0.7, Vec2::new(0.6, 0.8) * 6.3277),
            (Vec2::zeros(), 0.7, Vec2::zeros()),
        ];
        for (error_m, decel_mps2, expected) in cases {
            let velocity = closing_velocity(error_m, p, decel_mps2);
            check!(
                (velocity - expected).amax() < 1e-3,
                "error {error_m:?}, decel {decel_mps2}: {velocity:?}, expected {expected:?}"
            );
        }
        check!(
            closing_velocity(Vec2::new(f32::NAN, 0.0), p, 0.7)
                .x
                .is_nan()
        );
    }

    /// 10 m out with shaping on, a vehicle closing at the braking curve's 3.47 m/s
    /// (`√(2 · 0.7 · (10 − 1.4))`) holds that speed; the linear loop's 5 m/s would speed it up.
    #[test]
    fn closing_at_the_braking_curve_holds_its_speed() {
        let closing_mps = (2.0_f32 * 0.7 * (10.0 - 1.4)).sqrt();
        let accel = LateralController::default().control(
            &shaped(1.0),
            0.1,
            at(Vec2::new(10.0, 0.0)),
            moving(Vec2::new(closing_mps, 0.0)),
            Vec2::zeros(),
        );
        check!(accel.amax() < 1e-4, "{accel:?}");
    }

    /// 30 m out with shaping off, `brake_mps2` 1.5 puts the braking curve at 9 m/s
    /// (`√(2 · 1.5 · (30 − 3))`): a vehicle closing at it holds its speed, a faster one brakes,
    /// and without it the linear loop's 15 m/s speeds the vehicle up (#488).
    #[test]
    fn unshaped_braking_slows_along_the_curve() {
        // (brake_mps2, closing speed in m/s, expected acceleration in m/s² at velocity P 2)
        let cases = [(1.5, 9.0, 0.0), (1.5, 12.0, -6.0), (0.0, 9.0, 12.0)];
        for (brake_mps2, closing_mps, expected) in cases {
            let config = LateralControllerConfig {
                brake_mps2,
                ..p_only()
            };
            let accel = LateralController::default().control(
                &config,
                0.1,
                at(Vec2::new(30.0, 0.0)),
                moving(Vec2::new(closing_mps, 0.0)),
                Vec2::zeros(),
            );
            check!(
                (accel - Vec2::new(expected, 0.0)).amax() < 1e-3,
                "brake {brake_mps2} at {closing_mps} m/s: {accel:?}, expected {expected}"
            );
        }
    }

    /// A negative or NaN braking deceleration fails validation, and in the tick brakes linearly.
    #[test]
    fn invalid_brake_is_rejected_and_brakes_linearly() {
        for brake_mps2 in [-1.0, f32::NAN] {
            let config = LateralControllerConfig {
                brake_mps2,
                ..p_only()
            };
            check!(config.validate().map_err(|e| e.field) == Err("brake_mps2"));
            let accel = LateralController::default().control(
                &config,
                0.1,
                at(Vec2::new(30.0, 0.0)),
                moving(Vec2::zeros()),
                Vec2::zeros(),
            );
            check!(
                (accel - Vec2::new(30.0, 0.0)).amax() < 1e-3,
                "brake {brake_mps2}: {accel:?}"
            );
        }
    }

    /// A non-finite velocity gives no acceleration and leaves the shaped target where it was:
    /// the next finite tick moves on from it.
    #[test]
    fn non_finite_velocity_keeps_the_shaped_target() {
        let (dt, config) = (0.1, shaped(1.0));
        let mut controller = LateralController::default();
        let mut run = |velocity: Vec2| {
            controller.control(&config, dt, leg(), moving(velocity), Vec2::zeros())
        };
        run(Vec2::zeros()); // shaped 0.1 m/s
        check!(run(Vec2::new(f32::NAN, 0.0)) == Vec2::zeros());
        let accel = run(Vec2::zeros()); // shaped 0.2 m/s
        check!((accel - Vec2::new(1.0 + 2.0 * 0.2, 0.0)).amax() < 1e-4);
    }

    /// Turning shaping off between runs forgets the shaped target: turned back on, it starts
    /// again from the measured velocity, not from where it was.
    #[test]
    fn shaping_off_forgets_the_shaped_target() {
        let dt = 0.1;
        let mut controller = LateralController::default();
        for _ in 0..10 {
            controller.control(
                &shaped(1.0),
                dt,
                leg(),
                moving(Vec2::zeros()),
                Vec2::zeros(),
            );
        }
        controller.control(
            &shaped(0.0),
            dt,
            leg(),
            moving(Vec2::zeros()),
            Vec2::zeros(),
        );
        let accel = controller.control(
            &shaped(1.0),
            dt,
            leg(),
            moving(Vec2::zeros()),
            Vec2::zeros(),
        );
        // From rest again: 0.1 m/s, not 1.1.
        check!(
            (accel - Vec2::new(1.0 + 2.0 * 0.1, 0.0)).amax() < 1e-4,
            "{accel:?}"
        );
    }

    /// A negative or NaN acceleration limit fails validation, and in the tick shapes nothing.
    #[test]
    fn invalid_accel_limit_is_rejected_and_shapes_nothing() {
        for max_accel_mps2 in [-1.0, f32::NAN] {
            let config = shaped(max_accel_mps2);
            check!(config.validate().map_err(|e| e.field) == Err("max_accel_mps2"));
            let accel = LateralController::default().control(
                &config,
                0.1,
                leg(),
                moving(Vec2::zeros()),
                Vec2::zeros(),
            );
            check!(
                (accel - Vec2::new(6.0, 0.0)).amax() < 1e-4,
                "limit {max_accel_mps2}: {accel:?}"
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
            at(Vec2::new(0.0, 1.0)),
            moving(Vec2::zeros()),
            Vec2::zeros(),
        );
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
        let accel = controller.control(
            &retuned,
            0.01,
            at(Vec2::zeros()),
            moving(Vec2::zeros()),
            Vec2::zeros(),
        );

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
            Vec2::zeros(),
        );
    }
}
