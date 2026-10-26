//! Closed-loop identification of a rate axis: see [`IdentifyProbe`].
//!
//! Follows the excitation and model PX4's autotune flies
//! (`src/modules/mc_autotune_attitude_control/mc_autotune_attitude_control.cpp:603-639`,
//! `src/lib/system_identification/arx_rls.hpp` at `ae4fd97`), described, not copied: a
//! washed-out square wave whose half-periods shrink, and a second-order model with a delay.
//! PX4 fits it by recursive least squares; this probe accumulates the least-squares sums
//! instead and solves once at the end, in a form that holds its precision in `f32` (#516).

use nalgebra::{SMatrix, SVector};

use crate::controllers::{InvalidConfig, check, check_positive};
use crate::math::assert_dt;

#[cfg(test)]
mod tests;

/// Longest delay the fit tries, in ticks: 40 ms at 250 Hz, past the sim's 25 ms actuator delay
/// (7 ticks with the hold).
pub const MAX_DELAY_TICKS: usize = 10;

/// Moments kept: the longest delay's oldest term, `u[k − d − 2]`.
const HISTORY: usize = MAX_DELAY_TICKS + 2;

/// Terms in the fit: two rates, three moments, a constant.
const TERMS: usize = 6;

/// The square wave's half-periods, in s: 1.0 down to 0.2 by 0.1, then 0.6 down to 0.2,
/// repeating (PX4's sweep, `mc_autotune_attitude_control.cpp:603-619`).
fn half_period_s(index: usize) -> f32 {
    const FIRST: usize = 9;
    match index {
        0..FIRST => 1.0 - 0.1 * index as f32,
        _ => 0.6 - 0.1 * ((index - FIRST) % 5) as f32,
    }
}

/// Configures an [`IdentifyProbe`].
///
/// The amplitude is in the probed loop's setpoint units (rad on an angle loop), so there is no
/// `Default`; [`COPTER_IDENTIFY`](crate::tuning::autotune::COPTER_IDENTIFY) is the copter's.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IdentifyConfig {
    /// Height of the square wave added to the setpoint, in setpoint units. Non-zero and
    /// finite: larger moves the vehicle further and lifts the response over noise.
    pub amplitude: f32,
    /// How long the axis is excited and recorded, in s. Positive and finite.
    pub duration_s: f32,
    /// Time constant of the washout taken off the square wave, in s (PX4: 0.2 s): each edge
    /// becomes a pulse that decays, so the setpoint offset averages near zero. Positive and
    /// finite.
    pub washout_tc_s: f32,
}

impl IdentifyConfig {
    /// Checks every field's stated requirement.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::autotune::COPTER_IDENTIFY;
    /// use stable_flight::tuning::IdentifyConfig;
    ///
    /// assert!(COPTER_IDENTIFY.validate().is_ok());
    /// let bad = IdentifyConfig { duration_s: 0.0, ..COPTER_IDENTIFY };
    /// assert!(bad.validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check(
            "amplitude",
            self.amplitude != 0.0 && self.amplitude.is_finite(),
            "non-zero and finite",
        )?;
        check_positive("duration_s", self.duration_s)?;
        check_positive("washout_tc_s", self.washout_tc_s)
    }
}

/// A rate axis's response to its moment command, fitted in flight: the body rate one tick on
/// from the last two rates and three moments, `delay_ticks` back.
///
/// `y[k] = r₁·y[k−1] + r₂·y[k−2] + m₀·u[k−d] + m₁·u[k−d−1] + m₂·u[k−d−2]`, with `y` the body
/// rate in rad/s, `u` the normalized moment, `r` the [`rate_weights`](Self::rate_weights), `m`
/// the [`moment_weights`](Self::moment_weights), and `d` the delay. A body that integrates its
/// moment through one motor lag fits it with `r₁ = 1 + p`, `r₂ = −p`, `p` the lag's pole.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RateModel {
    /// Weights of the last two body rates, `y[k−1]` and `y[k−2]`.
    pub rate_weights: [f32; 2],
    /// Weights of the moments `u[k−d]`, `u[k−d−1]`, and `u[k−d−2]`.
    pub moment_weights: [f32; 3],
    /// Ticks from a moment command to its first effect on the rate, at least 1.
    pub delay_ticks: usize,
    /// The tick the model steps at, in s: the mean `dt` it was fitted over.
    pub dt_s: f32,
}

impl RateModel {
    /// The motor lag's time constant, in s, from its pole `p = −r₂`; `None` unless the pole
    /// is real and in `(0, 1)`, a lag rather than an overshoot or a fit to noise.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::RateModel;
    ///
    /// // A 30 ms lag at 250 Hz: p = e^(−0.004/0.03).
    /// let p = (-0.004f32 / 0.03).exp();
    /// let model = RateModel {
    ///     rate_weights: [1.0 + p, -p],
    ///     moment_weights: [0.1, 0.0, 0.0],
    ///     delay_ticks: 1,
    ///     dt_s: 0.004,
    /// };
    /// assert!((model.lag_s().unwrap() - 0.03).abs() < 1e-5);
    /// ```
    pub fn lag_s(&self) -> Option<f32> {
        let pole = -self.rate_weights[1];
        (pole > 0.0 && pole < 1.0).then(|| -self.dt_s / libm::logf(pole))
    }

    /// The axis's angular acceleration per unit moment once the lag has settled, in rad/s²:
    /// the moments' total weight over `(1 − p)·dt`. `None` without a lag ([`lag_s`](Self::lag_s)).
    pub fn accel_radps2(&self) -> Option<f32> {
        self.lag_s()?;
        let pole = -self.rate_weights[1];
        let total: f32 = self.moment_weights.iter().sum();
        Some(total / ((1.0 - pole) * self.dt_s))
    }
}

/// Why an [`IdentifyProbe`] gave up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IdentifyAbort {
    /// No delay's fit could be solved: the axis didn't move enough to tell its terms apart.
    Unexcited,
}

/// Where an [`IdentifyProbe`] stands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum IdentifyStatus {
    /// Exciting and recording.
    Running {
        /// Time since the excitation began, in s.
        elapsed_s: f32,
    },
    /// The fitted model.
    Done(RateModel),
    /// Gave up.
    Aborted(IdentifyAbort),
}

/// The least-squares sums at one delay.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Sums {
    /// `Σ φ·φᵀ` over the regressors.
    normal: SMatrix<f32, TERMS, TERMS>,
    /// `Σ φ·t` against the targets.
    projected: SVector<f32, TERMS>,
    /// `Σ t²`.
    target_squared: f32,
}

impl Default for Sums {
    fn default() -> Self {
        Sums {
            normal: SMatrix::zeros(),
            projected: SVector::zeros(),
            target_squared: 0.0,
        }
    }
}

/// Excites one loop with a square wave on its setpoint while recording its axis's body rate and
/// moment command, then fits a [`RateModel`] to them.
///
/// Each tick the host hands it the axis's body rate and the moment it commanded on the last
/// tick; the probe returns the offset to add to the loop's setpoint. Every loop stays closed, so
/// the fit sees the airframe through its own controllers. It fits each delay from 1 to
/// [`MAX_DELAY_TICKS`] on the same ticks and keeps the one with the least residual.
///
/// The fit takes the rate's change `y[k] − y[k−1]` as its target and `y[k−1]`, `y[k−1] −
/// y[k−2]` as its rate terms: at 250 Hz the last two rates are nearly equal, and fitting them
/// side by side loses the lag in `f32` (#516). It keeps only the sums of each delay's least
/// squares (about 1.7 KB), not the record, and solves them when the excitation ends. A
/// constant term takes up the trim moment. A non-finite rate or moment drops that tick and the
/// [`MAX_DELAY_TICKS`]` + 2` after it, while the history refills.
///
/// # Examples
///
/// ```
/// use stable_flight::tuning::autotune::COPTER_IDENTIFY;
/// use stable_flight::tuning::{IdentifyProbe, IdentifyStatus};
///
/// let mut probe = IdentifyProbe::new(COPTER_IDENTIFY);
/// // The square wave starts high: the first tick's offset is the step less its washout.
/// let offset = probe.step(0.004, 0.0, 0.0);
/// assert!(offset > 0.0 && offset <= COPTER_IDENTIFY.amplitude);
/// assert!(matches!(probe.status(), IdentifyStatus::Running { .. }));
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct IdentifyProbe {
    config: IdentifyConfig,
    status: IdentifyStatus,
    /// The square wave: the time of its next edge, in s, that edge's index, and its level.
    next_edge_s: f32,
    edge: usize,
    high: bool,
    /// The square wave through a low-pass at the washout's time constant, in setpoint units.
    washout: f32,
    /// The last two body rates, `y[k−1]` then `y[k−2]`, in rad/s.
    rates_radps: [f32; 2],
    /// The moments, newest first: `moments[0]` is `u[k−1]`.
    moments: [f32; HISTORY],
    /// Consecutive finite ticks recorded, up to [`HISTORY`].
    filled: usize,
    /// Each delay's sums, delay 1 first.
    sums: [Sums; MAX_DELAY_TICKS],
    /// Ticks the sums hold, and their total `dt`, in s.
    rows: usize,
    rows_dt_s: f32,
}

impl IdentifyProbe {
    /// A probe ready to run.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`IdentifyConfig::validate`]).
    pub fn new(config: IdentifyConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid IdentifyConfig: {error}");
        }
        IdentifyProbe {
            config,
            status: IdentifyStatus::Running { elapsed_s: 0.0 },
            next_edge_s: half_period_s(0),
            edge: 0,
            high: true,
            washout: 0.0,
            rates_radps: [0.0; 2],
            moments: [0.0; HISTORY],
            filled: 0,
            sums: [Sums::default(); MAX_DELAY_TICKS],
            rows: 0,
            rows_dt_s: 0.0,
        }
    }

    /// Advances the probe by `dt` seconds with the axis's body rate this tick (rad/s) and its
    /// moment command on the last tick, and returns the offset to add to the loop's setpoint
    /// (0 once it has finished). Fits the model on the tick the excitation ends.
    ///
    /// # Panics
    ///
    /// If `dt` is not positive and finite.
    pub fn step(&mut self, dt: f32, rate_radps: f32, last_moment: f32) -> f32 {
        assert_dt(dt);
        let IdentifyStatus::Running { elapsed_s } = self.status else {
            return 0.0;
        };
        self.record(dt, rate_radps, last_moment);
        let elapsed_s = elapsed_s + dt;
        let offset = self.excitation(dt, elapsed_s);
        self.status = if elapsed_s >= self.config.duration_s {
            self.fit()
        } else {
            IdentifyStatus::Running { elapsed_s }
        };
        offset
    }

    /// Where the probe stands.
    pub fn status(&self) -> IdentifyStatus {
        self.status
    }

    /// Adds this tick to every delay's sums once the history is full, then shifts the history.
    fn record(&mut self, dt: f32, rate_radps: f32, last_moment: f32) {
        if !(rate_radps.is_finite() && last_moment.is_finite()) {
            self.filled = 0;
            return;
        }
        self.moments.rotate_right(1);
        self.moments[0] = last_moment;
        self.filled = (self.filled + 1).min(HISTORY);
        if self.filled == HISTORY {
            let [previous, before] = self.rates_radps;
            let target = rate_radps - previous;
            for (index, sums) in self.sums.iter_mut().enumerate() {
                // Delay d = index + 1: u[k − d − j] is moments[d − 1 + j].
                let regressors = SVector::<f32, TERMS>::from([
                    previous,
                    previous - before,
                    self.moments[index],
                    self.moments[index + 1],
                    self.moments[index + 2],
                    1.0,
                ]);
                sums.normal += regressors * regressors.transpose();
                sums.projected += regressors * target;
                sums.target_squared += target * target;
            }
            self.rows += 1;
            self.rows_dt_s += dt;
        }
        self.rates_radps = [rate_radps, self.rates_radps[0]];
    }

    /// The setpoint offset at `elapsed_s`: the square wave less its washout.
    fn excitation(&mut self, dt: f32, elapsed_s: f32) -> f32 {
        while elapsed_s >= self.next_edge_s {
            self.edge += 1;
            self.next_edge_s += half_period_s(self.edge);
            self.high = !self.high;
        }
        let square = if self.high {
            self.config.amplitude
        } else {
            0.0
        };
        let share = (dt / self.config.washout_tc_s).min(1.0);
        self.washout += (square - self.washout) * share;
        square - self.washout
    }

    /// Solves each delay's sums and keeps the least residual: `Done`, or `Unexcited` when none
    /// solves.
    fn fit(&self) -> IdentifyStatus {
        let dt_s = self.rows_dt_s / self.rows.max(1) as f32;
        let best = self
            .sums
            .iter()
            .enumerate()
            .filter_map(|(index, sums)| {
                let solution = sums.normal.cholesky()?.solve(&sums.projected);
                // At the least-squares solution the residual is Σt² − θ·Σφt.
                let residual = sums.target_squared - solution.dot(&sums.projected);
                let finite = solution.iter().all(|x| x.is_finite()) && residual.is_finite();
                finite.then_some((index + 1, solution, residual))
            })
            .min_by(|a, b| a.2.total_cmp(&b.2));
        let Some((delay_ticks, solution, _)) = best else {
            return IdentifyStatus::Aborted(IdentifyAbort::Unexcited);
        };
        // Δy[k] = c₀·y[k−1] + c₁·(y[k−1] − y[k−2]) + …, so y[k] = (1 + c₀ + c₁)·y[k−1] − c₁·y[k−2] + ….
        let (change, difference) = (solution[0], solution[1]);
        IdentifyStatus::Done(RateModel {
            rate_weights: [1.0 + change + difference, -difference],
            moment_weights: [solution[2], solution[3], solution[4]],
            delay_ticks,
            dt_s,
        })
    }
}
