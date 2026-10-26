//! Gains from an identified [`RateModel`] by loop shaping: see [`design_rate`] and
//! [`design_angle_p`].
//!
//! Both read the model's frequency response and set a gain for a target
//! [phase margin](../../../docs/glossary.md): how far the loop is from oscillating where its
//! open-loop gain crosses 1. The rule and its alternatives (PX4's pole placement, a step search
//! on the model) are in `architecture.md#autotune`; the comparison behind them is the
//! `autotune_methods` sweep in `stable-flight-sim` (#312).

use core::f32::consts::PI;

use nalgebra::Complex;
use thiserror::Error;

use super::RateModel;
use super::identify::unit_delay;
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
    /// Least and most angle P (rate per angle), the angle search's range.
    pub angle_p_min: f32,
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
    /// The fitted pole isn't a lag ([`RateModel::lag_s`] is `None`): the fit saw overshoot or
    /// noise, not a motor.
    #[error("the fitted model has no motor lag")]
    NotALag,
    /// The rate loop's phase never reaches the margin, or falls past it before any frequency:
    /// the model has no usable crossover.
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
/// `1/|C·G|` at the first frequency where the phase of `C·G` falls to `−180° + margin`, with
/// `C = 1 + τ·(1 − z⁻¹)/dt` the PD at unit P and `G` the model's [`response`](RateModel::response).
///
/// # Errors
///
/// [`DesignError::NotALag`] without a lag, [`DesignError::NoCrossover`] when no frequency on
/// the grid meets the margin.
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
/// let gains = design_rate(&model, start, &COPTER_LOOP_SHAPING).unwrap();
/// assert!((gains.d / gains.p - model.lag_s().unwrap()).abs() < 1e-6);
/// ```
pub fn design_rate(
    model: &RateModel,
    start: Gains,
    config: &LoopShapingConfig,
) -> Result<Gains, DesignError> {
    let lag_s = model.lag_s().ok_or(DesignError::NotALag)?;
    let dt_s = model.dt_s;
    let open_loop = |w_radps: f32| {
        let z1 = unit_delay(w_radps, dt_s);
        let pd = Complex::new(1.0, 0.0) + (Complex::new(1.0, 0.0) - z1) * (lag_s / dt_s);
        pd * model.response(w_radps)
    };
    let floor_deg = config.rate_margin_deg - 180.0;
    let crossover_radps = first_crossing(dt_s, |w_radps| {
        let response = open_loop(w_radps);
        (response, phase_deg(response) > floor_deg)
    })
    .ok_or(DesignError::NoCrossover)?;
    let p = 1.0 / magnitude(open_loop(crossover_radps));
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
/// loop `T = C_rate·G/(1 + C_rate·G)` (core's PID on the error, `BodyRateController`), and the
/// integration from rate to angle. Its margin is read where its gain falls to 1, and angle P is
/// bisected on it.
///
/// # Errors
///
/// [`DesignError::AngleUnstable`] when even `angle_p_min` misses the margin.
pub fn design_angle_p(
    model: &RateModel,
    rate: Gains,
    angle: Gains,
    config: &LoopShapingConfig,
) -> Result<f32, DesignError> {
    let dt_s = model.dt_s;
    let one = Complex::new(1.0, 0.0);
    let open_loop = |w_radps: f32, angle_p: f32| {
        let z1 = unit_delay(w_radps, dt_s);
        let integrator = Complex::new(dt_s, 0.0) / (one - z1);
        let derivative = (one - z1) / dt_s;
        let plant = model.response(w_radps);
        let rate_pid = one * rate.p + integrator * rate.i + derivative * rate.d;
        let closed_rate = rate_pid * plant / (one + rate_pid * plant);
        let angle_pid = one * angle_p + integrator * angle.i + derivative * angle.d;
        angle_pid * closed_rate * integrator
    };
    let keeps_margin = |angle_p: f32| {
        let floor_deg = config.angle_margin_deg - 180.0;
        let crossing = first_crossing(dt_s, |w_radps| {
            let response = open_loop(w_radps, angle_p);
            (response, magnitude(response) > 1.0)
        });
        crossing.is_some_and(|w_radps| phase_deg(open_loop(w_radps, angle_p)) > floor_deg)
    };
    let (low, high) = (config.angle_p_min, config.angle_p_max);
    if !keeps_margin(low) {
        return Err(DesignError::AngleUnstable);
    }
    if keeps_margin(high) {
        return Ok(high * config.angle_back_off);
    }
    // Geometric bisection: angle P spans an order of magnitude.
    let (mut keeps, mut misses) = (low, high);
    for _ in 0..BISECTIONS {
        let middle = libm::sqrtf(keeps * misses);
        if keeps_margin(middle) {
            keeps = middle;
        } else {
            misses = middle;
        }
    }
    Ok(keeps * config.angle_back_off)
}

/// The first frequency on the log grid (refined by bisection) where `above` turns false, in
/// rad/s, walking up from [`LOWEST_RADPS`] with `response`'s phase unwrapped; `None` if it never
/// does or is false from the start.
///
/// `probe(w)` returns the response at `w` and whether it is still above the threshold; the
/// phase unwrap reads the response, so a threshold on phase sees it continuous.
fn first_crossing(dt_s: f32, probe: impl Fn(f32) -> (Complex<f32>, bool)) -> Option<f32> {
    let nyquist_radps = PI / dt_s;
    let step = libm::powf(nyquist_radps / LOWEST_RADPS, 1.0 / (GRID_POINTS - 1) as f32);
    let (_, above_at_start) = probe(LOWEST_RADPS);
    if !above_at_start {
        return None;
    }
    let mut below_radps = LOWEST_RADPS;
    for i in 1..GRID_POINTS {
        let w_radps = LOWEST_RADPS * libm::powf(step, i as f32);
        if !probe(w_radps).1 {
            // Bisect between the last point above and this one.
            let (mut above, mut below) = (below_radps, w_radps);
            for _ in 0..BISECTIONS {
                let middle = libm::sqrtf(above * below);
                if probe(middle).1 {
                    above = middle;
                } else {
                    below = middle;
                }
            }
            return Some(above);
        }
        below_radps = w_radps;
    }
    None
}

/// `|c|`.
fn magnitude(c: Complex<f32>) -> f32 {
    libm::hypotf(c.re, c.im)
}

/// The phase of `c`, in degrees, in `(−360, 0]`: open loops here lag (an integrator's −90°
/// and more), so a phase past −180° reads as below it rather than wrapping to +180°.
fn phase_deg(c: Complex<f32>) -> f32 {
    let deg = libm::atan2f(c.im, c.re).to_degrees();
    if deg > 0.0 { deg - 360.0 } else { deg }
}
