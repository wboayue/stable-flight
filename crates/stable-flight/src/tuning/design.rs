//! Gains from an identified [`RateModel`] by loop shaping: see [`design_rate`] and
//! [`design_angle_p`].
//!
//! Both read the model's frequency response and set a gain for a target
//! [phase margin](https://github.com/wboayue/stable-flight/blob/main/docs/glossary.md): how far the loop is from oscillating where its
//! open-loop gain crosses 1. The rule and its alternatives (PX4's pole placement, a step search
//! on the model) are in `architecture.md#autotune`; the comparison behind them is the
//! `autotune_methods` sweep in `stable-flight-sim` (#312).

use core::f32::consts::PI;

use nalgebra::Complex;
use thiserror::Error;

use super::RateModel;
use super::identify::unit_delay;
use crate::controllers::low_pass::LowPass;
use crate::controllers::{Gains, InvalidConfig, check, check_positive};

#[cfg(test)]
mod tests;

/// Lowest frequency of the design grid, in rad/s: below any copter's crossover.
const LOWEST_RADPS: f32 = 0.1;

/// Points on the log-spaced grid from [`LOWEST_RADPS`] to Nyquist: about 2% apart.
const GRID_POINTS: usize = 400;

/// Bisections between two grid points to place a crossing, and on angle P.
const BISECTIONS: usize = 16;

/// How the design sets the gains.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LoopShapingConfig {
    /// The rate loop's phase margin, in degrees, in `(0, 90)`: more is a slower rate loop with
    /// less overshoot. 65° met the step spec on every clean and delayed sim row; 60° and 55°
    /// missed on some (#312).
    pub rate_margin_deg: f32,
    /// The angle loop's phase margin around the designed rate loop, in degrees, in `(0, 90)`.
    pub angle_margin_deg: f32,
    /// Least angle P, in rad/s of rate per rad of angle: the bottom of the angle search's range.
    pub angle_p_min: f32,
    /// Most angle P, in rad/s per rad: the top of the angle search's range.
    pub angle_p_max: f32,
    /// Share of the largest angle P that meets the margin kept, in `(0, 1]`, as the search
    /// backs off its result.
    pub angle_back_off: f32,
}

impl LoopShapingConfig {
    /// Checks every field's stated requirement, and that the angle range is ordered.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::autotune::COPTER_LOOP_SHAPING;
    /// use stable_flight::tuning::LoopShapingConfig;
    ///
    /// assert!(COPTER_LOOP_SHAPING.validate().is_ok());
    /// let bad = LoopShapingConfig { rate_margin_deg: 95.0, ..COPTER_LOOP_SHAPING };
    /// assert!(bad.validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        let margin = |deg: f32| deg > 0.0 && deg < 90.0;
        check(
            "rate_margin_deg",
            margin(self.rate_margin_deg),
            "in (0, 90)",
        )?;
        check(
            "angle_margin_deg",
            margin(self.angle_margin_deg),
            "in (0, 90)",
        )?;
        check_positive("angle_p_min", self.angle_p_min)?;
        check(
            "angle_p_max",
            self.angle_p_max.is_finite() && self.angle_p_max >= self.angle_p_min,
            "finite and at least angle_p_min",
        )?;
        check(
            "angle_back_off",
            self.angle_back_off > 0.0 && self.angle_back_off <= 1.0,
            "in (0, 1]",
        )
    }
}

/// Why a design found no gains.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
pub enum DesignError {
    /// The fitted pole isn't a lag ([`RateModel::lag_s`] is `None`): at or past 1, the fit
    /// grows rather than lags.
    #[error("the fitted model has no motor lag")]
    NotALag,
    /// The rate loop's phase never reaches the margin, or falls past it before any frequency,
    /// or the model's moment doesn't push the axis the way commanded: the model has no usable
    /// crossover.
    #[error("no frequency meets the rate margin")]
    NoCrossover,
    /// Even the least angle P misses the angle margin around the designed rate loop.
    #[error("the least angle P misses the angle margin")]
    AngleUnstable,
}

/// Rate gains for `model`: D cancels the motor lag, and P puts the crossover where the loop
/// keeps `config`'s rate margin; I stays `start`'s, since it holds trim rather than shaping the
/// response.
///
/// D/P is the fitted lag `τ`, so the PD's zero `1 + τ·s` cancels the lag's pole. P is
/// `1/|C·G·F|` at the first frequency where the phase of `C·G·F` falls to `−180° + margin`,
/// with `C = 1 + τ·(1 − z⁻¹)/dt` the PD at unit P, `G` the model's
/// [`response`](RateModel::response), and `F` the gyro low-pass at `gyro_cutoff_hz` (the
/// stabilizer's [`gyro_cutoff_hz`](crate::stabilizers::copter::CopterStabilizerConfig::gyro_cutoff_hz);
/// 0 for none) that the loop's P and D both act through. Its lag at the crossover is most of the
/// margin: on `quad_x` at 40 Hz, a design that leaves it out keeps about 2° of its 65° (#373).
/// The kept I isn't in that loop: its lag at the crossover costs the flown loop a little margin
/// (0.6° at I 0.2 over seven ticks of delay). A fitted integrator past 1 (`r₁ + r₂ > 1`, an
/// unstable pole from noise, whose phase meets no crossover) is held at 1, the lag kept.
///
/// # Errors
///
/// [`DesignError::NotALag`] without a lag, [`DesignError::NoCrossover`] when the moment
/// doesn't push the axis or no frequency on the grid meets the margin.
///
/// # Examples
///
/// ```
/// use stable_flight::controllers::Gains;
/// use stable_flight::tuning::autotune::COPTER_LOOP_SHAPING;
/// use stable_flight::tuning::{RateModel, design_rate};
///
/// // quad_x's fit on the sim: a 30 ms lag, one tick of delay.
/// let model = RateModel {
///     rate_weights: [1.87437, -0.87447],
///     moment_weights: [0.0136, 0.0, 0.0],
///     delay_ticks: 1,
///     dt_s: 0.004,
/// };
/// let start = Gains { p: 0.1, i: 0.0, d: 0.002 };
/// let gains = design_rate(&model, start, 40.0, &COPTER_LOOP_SHAPING).unwrap();
/// assert!((gains.d / gains.p - model.lag_s().unwrap()).abs() < 1e-6);
/// ```
pub fn design_rate(
    model: &RateModel,
    start: Gains,
    gyro_cutoff_hz: f32,
    config: &LoopShapingConfig,
) -> Result<Gains, DesignError> {
    let model = &integrating(model);
    let lag_s = model.lag_s().ok_or(DesignError::NotALag)?;
    // A reversed axis's phase starts half a turn from an integrator's; through the gyro
    // filter it then drifts across the margin, which would read as a crossover.
    if model.moment_effect() <= 0.0 {
        return Err(DesignError::NoCrossover);
    }
    let dt_s = model.dt_s;
    let unit_pd = Gains {
        p: 1.0,
        i: 0.0,
        d: lag_s,
    };
    let open_loop = |w_radps: f32| {
        let z1 = unit_delay(w_radps, dt_s);
        pid_response(unit_pd, z1, dt_s)
            * model.response(w_radps)
            * LowPass::response(gyro_cutoff_hz, dt_s, z1)
    };
    let floor_deg = config.rate_margin_deg - 180.0;
    let crossover = first_crossing(dt_s, open_loop, |point| point.phase_deg > floor_deg)
        .ok_or(DesignError::NoCrossover)?;
    let p = 1.0 / crossover.gain;
    Ok(Gains {
        p,
        i: start.i,
        d: lag_s * p,
    })
}

/// Angle P for `model` flown on the `rate` gains: the largest in `config`'s range whose angle
/// loop keeps the angle margin, times the back-off. `angle`'s I and D stay.
///
/// The angle loop's open loop is `C_angle · T · dt/(1 − z⁻¹)`: the angle PID, the closed rate
/// loop `T = P_rate·G/(1 + C_rate·G·F)`, and the integration from rate to angle. In `T`,
/// `C_rate` is the rate P and D (`BodyRateController`; its I is left out) acting through the
/// gyro low-pass `F` at `gyro_cutoff_hz`, and only P acts on the rate setpoint (D acts on the
/// measured rate). The angle loop reads the attitude, not the filtered rates, so `F` is in the
/// rate loop alone. Its margin is read where its gain falls to 1, and angle P is bisected on
/// it. A fitted integrator past 1 is held at 1, as in [`design_rate`].
///
/// # Errors
///
/// [`DesignError::AngleUnstable`] when even `angle_p_min` misses the margin.
pub fn design_angle_p(
    model: &RateModel,
    rate: Gains,
    angle: Gains,
    gyro_cutoff_hz: f32,
    config: &LoopShapingConfig,
) -> Result<f32, DesignError> {
    let model = &integrating(model);
    let dt_s = model.dt_s;
    let one = Complex::new(1.0, 0.0);
    // The rate loop's I acts far below the angle crossover (its zero at I/P), so it costs
    // the margin little, and its integrator around a fit with a pole just outside the unit
    // circle swings the margin from one near-identical fit to the next (#374): the model
    // leaves it out.
    let rate = Gains { i: 0.0, ..rate };
    let open_loop = |w_radps: f32, angle_p: f32| {
        let z1 = unit_delay(w_radps, dt_s);
        let rate_pid = pid_response(rate, z1, dt_s);
        let plant = model.response(w_radps);
        let gyro = LowPass::response(gyro_cutoff_hz, dt_s, z1);
        let closed_rate = plant * rate.p / (one + rate_pid * plant * gyro);
        let angle_pid = pid_response(
            Gains {
                p: angle_p,
                ..angle
            },
            z1,
            dt_s,
        );
        let to_angle = Complex::new(dt_s, 0.0) / (one - z1);
        angle_pid * closed_rate * to_angle
    };
    let floor_deg = config.angle_margin_deg - 180.0;
    let keeps_margin = |angle_p: f32| {
        let crossover = first_crossing(dt_s, |w| open_loop(w, angle_p), |point| point.gain > 1.0);
        crossover.is_some_and(|point| point.phase_deg > floor_deg)
    };
    let (low, high) = (config.angle_p_min, config.angle_p_max);
    if !keeps_margin(low) {
        return Err(DesignError::AngleUnstable);
    }
    if keeps_margin(high) {
        return Ok(high * config.angle_back_off);
    }
    Ok(bisect_geometric(low, high, keeps_margin) * config.angle_back_off)
}

/// `model` with its integrator held at 1 (`r₁ + r₂ ≤ 1`, the lag's pole `−r₂` kept): a body
/// integrates its moment exactly, less any rate damping, so a fit past 1 is an unstable pole
/// from noise, whose phase starts near −180° and meets no crossover (`r₁ + r₂ = 1.00012` on
/// `quad_x` from a start ringing with 25 ms of delay, #518). A fit at or below 1 stays.
fn integrating(model: &RateModel) -> RateModel {
    let [_, r2] = model.rate_weights;
    if model.rate_weights.iter().sum::<f32>() <= 1.0 {
        return *model;
    }
    RateModel {
        rate_weights: [1.0 - r2, r2],
        ..*model
    }
}

/// A PID's frequency response at `z1 = e^(−jω·dt)`: `P + I·dt/(1 − z⁻¹) + D·(1 − z⁻¹)/dt`,
/// core's law in its feedback path (`BodyRateController`, `AttitudeController`: a running
/// integral and a one-tick difference).
fn pid_response(gains: Gains, z1: Complex<f32>, dt_s: f32) -> Complex<f32> {
    let one = Complex::new(1.0, 0.0);
    let integral = Complex::new(dt_s, 0.0) / (one - z1);
    let difference = (one - z1) / dt_s;
    one * gains.p + integral * gains.i + difference * gains.d
}

/// One frequency of an open loop's response: its gain and its phase in degrees, unwrapped from
/// [`LOWEST_RADPS`] up, so a lag past −360° reads as such rather than wrapping back toward 0.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Point {
    w_radps: f32,
    gain: f32,
    phase_deg: f32,
}

/// The last point where `above` holds before it first turns false, walking `open_loop` up the
/// log grid from [`LOWEST_RADPS`] with its phase unwrapped, refined by bisection between that
/// grid point and the next; `None` if `above` never turns false or is false from the start.
fn first_crossing(
    dt_s: f32,
    open_loop: impl Fn(f32) -> Complex<f32>,
    above: impl Fn(&Point) -> bool,
) -> Option<Point> {
    // The phase on the branch nearest `near_deg`, in degrees.
    let point = |w_radps: f32, near_deg: f32| {
        let response = open_loop(w_radps);
        let raw_deg = libm::atan2f(response.im, response.re).to_degrees();
        let turns = libm::roundf((near_deg - raw_deg) / 360.0);
        Point {
            w_radps,
            gain: libm::hypotf(response.re, response.im),
            phase_deg: raw_deg + 360.0 * turns,
        }
    };
    let nyquist_radps = PI / dt_s;
    let step = libm::powf(nyquist_radps / LOWEST_RADPS, 1.0 / (GRID_POINTS - 1) as f32);
    // Open loops here start as an integrator: −90°.
    let mut last = point(LOWEST_RADPS, -90.0);
    if !above(&last) {
        return None;
    }
    for i in 1..GRID_POINTS {
        let next = point(LOWEST_RADPS * libm::powf(step, i as f32), last.phase_deg);
        if !above(&next) {
            let keeps = |w_radps: f32| above(&point(w_radps, last.phase_deg));
            let w_radps = bisect_geometric(last.w_radps, next.w_radps, keeps);
            return Some(point(w_radps, last.phase_deg));
        }
        last = next;
    }
    None
}

/// The largest value between `keeps`, where `holds` is true, and `misses`, where it's false,
/// after [`BISECTIONS`] geometric halvings: the steps are even in ratio, for values that span
/// orders of magnitude (frequency, angle P).
fn bisect_geometric(mut keeps: f32, mut misses: f32, holds: impl Fn(f32) -> bool) -> f32 {
    for _ in 0..BISECTIONS {
        let middle = libm::sqrtf(keeps * misses);
        if holds(middle) {
            keeps = middle;
        } else {
            misses = middle;
        }
    }
    keeps
}
