//! Closed-loop identification of a rate axis: see [`IdentifyProbe`].
//!
//! Follows the excitation and model PX4's autotune flies (`mc_autotune_attitude_control`'s
//! `getIdentificationSignal`, added to the rate setpoint by `mc_att_control`; `ArxRls` in
//! `system_identification`), described, not copied: a washed-out square wave whose
//! half-periods shrink, and a second-order model with a delay.
//! PX4 fits it by recursive least squares; this probe accumulates the least-squares sums
//! instead and solves once at the end, in a form that holds its precision in `f32` (#516).

use nalgebra::{Complex, SMatrix, SVector};

use crate::controllers::{InvalidConfig, check, check_positive};
use crate::math::{assert_dt, lagged};

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
/// repeating (PX4's sweep, `getIdentificationSignal`).
fn half_period_s(index: usize) -> f32 {
    // The first sweep's nine halves, 1.0 down to 0.2 s; then sweeps of five, 0.6 down to 0.2.
    const FIRST_SWEEP_HALVES: usize = 9;
    match index {
        0..FIRST_SWEEP_HALVES => 1.0 - 0.1 * index as f32,
        _ => 0.6 - 0.1 * ((index - FIRST_SWEEP_HALVES) % 5) as f32,
    }
}

/// Residuals within this share of the least count as tied, and the shortest such delay wins:
/// `f32` cancellation in `Σt² − θ·Σφt` would otherwise pick between near-equal delays on
/// rounding (a 25 ms delay fit at 6 or 7 ticks, #516). Wider favors delays too short, which
/// design too hot a loop: on QuadCopterX in RealFlight, 5% took 1 tick in 6 of 67 fits
/// against 1 at 1%, and in run 35 a 1-tick pick at 2.2% over the least designed roll rate
/// P 1.79, which rang and failed its stage (#529).
const TIE_SHARE: f32 = 0.01;

/// Configures an [`IdentifyProbe`].
///
/// The amplitude is in the probed loop's setpoint units (rad/s on a rate loop), so there is no
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

    /// The model's frequency response at `w_radps`: body rate per unit moment, delay included,
    /// `(m₀ + m₁·z⁻¹ + m₂·z⁻²)·z⁻ᵈ / (1 − r₁·z⁻¹ − r₂·z⁻²)` at `z⁻¹ = e^(−jω·dt)`. Its phase
    /// starts near −90° (the body integrates its moment) and falls with the lag and the delay.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::RateModel;
    ///
    /// // An integrator alone, one tick of delay: about 1/(jω) per unit gain.
    /// let model = RateModel {
    ///     rate_weights: [1.0, 0.0],
    ///     moment_weights: [0.004, 0.0, 0.0],
    ///     delay_ticks: 1,
    ///     dt_s: 0.004,
    /// };
    /// let response = model.response(1.0);
    /// assert!((response.norm_sqr() - 1.0).abs() < 0.02);
    /// ```
    pub fn response(&self, w_radps: f32) -> Complex<f32> {
        let z1 = unit_delay(w_radps, self.dt_s);
        let [m0, m1, m2] = self.moment_weights;
        let [r1, r2] = self.rate_weights;
        let one = Complex::new(1.0, 0.0);
        let numerator = (one * m0 + z1 * m1 + z1 * z1 * m2) * z1.powu(self.delay_ticks as u32);
        numerator / (one - z1 * r1 - z1 * z1 * r2)
    }

    /// The axis's angular acceleration per unit moment once the lag has settled, in rad/s²:
    /// the moments' total weight over `(1 − p)·dt`. `None` without a lag ([`lag_s`](Self::lag_s)).
    pub fn accel_radps2(&self) -> Option<f32> {
        self.lag_s()?;
        let pole = -self.rate_weights[1];
        Some(self.moment_effect() / ((1.0 - pole) * self.dt_s))
    }

    /// The moments' total weight: its sign is the sign of the axis's response to a held moment
    /// (`accel_radps2` divides it by a positive `(1 − p)·dt`, and the design holds a pole past
    /// 1 at 1). Negative, the model pushes the axis against the moment commanded.
    pub(crate) fn moment_effect(&self) -> f32 {
        self.moment_weights.iter().sum()
    }
}

/// `z⁻¹ = e^(−jω·dt)`: one tick's delay at `w_radps`.
pub(crate) fn unit_delay(w_radps: f32, dt_s: f32) -> Complex<f32> {
    let angle = -w_radps * dt_s;
    Complex::new(libm::cosf(angle), libm::sinf(angle))
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

/// Excites one loop with a square wave on its setpoint while recording its axis's body rate and
/// moment command, then fits a [`RateModel`] to them.
///
/// Each tick the host hands it the axis's body rate and the moment it commanded on the last
/// tick; the probe returns the offset to add to the loop's setpoint. Every loop stays closed, so
/// the fit sees the airframe through its own controllers. It fits each delay from 1 to
/// [`MAX_DELAY_TICKS`] on the same ticks and keeps the one with the least residual (the
/// shortest of a near-tie) among those whose moment pushes the axis the way it was pushed (among
/// all when none does): a fit with a negative moment effect can't be designed from.
///
/// The fit takes the rate's change `y[k] − y[k−1]` as its target and `y[k−1]`, `y[k−1] −
/// y[k−2]` as its rate terms: at 250 Hz the last two rates are nearly equal, and fitting them
/// side by side loses the lag in `f32` (#516). It keeps only the sums of each delay's least
/// squares (about 1.7 KB), not the record, and solves them when the excitation ends. A
/// constant term takes up the trim moment. A non-finite rate or moment drops that tick and the
/// next `MAX_DELAY_TICKS + 2` ticks while the history refills.
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
    duration_s: f32,
    status: IdentifyStatus,
    sweep: SquareSweep,
    fits: DelayFits,
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
            duration_s: config.duration_s,
            status: IdentifyStatus::Running { elapsed_s: 0.0 },
            sweep: SquareSweep::new(config.amplitude, config.washout_tc_s),
            fits: DelayFits::new(),
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
        self.fits.record(dt, rate_radps, last_moment);
        let elapsed_s = elapsed_s + dt;
        let offset = self.sweep.next(dt, elapsed_s);
        self.status = if elapsed_s < self.duration_s {
            IdentifyStatus::Running { elapsed_s }
        } else {
            match self.fits.solve() {
                Some(model) => IdentifyStatus::Done(model),
                None => IdentifyStatus::Aborted(IdentifyAbort::Unexcited),
            }
        };
        offset
    }

    /// Where the probe stands.
    pub fn status(&self) -> IdentifyStatus {
        self.status
    }
}

/// PX4's shrinking square wave ([`half_period_s`]), less its washout: each edge a pulse that
/// decays over the washout's time constant.
#[derive(Clone, Copy, Debug, PartialEq)]
struct SquareSweep {
    amplitude: f32,
    washout_tc_s: f32,
    /// When the next edge falls, in s since the start, and its index.
    next_edge_s: f32,
    edge: usize,
    /// Whether the wave is at its amplitude (else 0).
    high: bool,
    /// The wave through a low-pass at the washout's time constant, in setpoint units.
    washout: f32,
}

impl SquareSweep {
    fn new(amplitude: f32, washout_tc_s: f32) -> Self {
        SquareSweep {
            amplitude,
            washout_tc_s,
            next_edge_s: half_period_s(0),
            edge: 0,
            high: true,
            washout: 0.0,
        }
    }

    /// The offset at `elapsed_s`, `dt` after the last call.
    fn next(&mut self, dt: f32, elapsed_s: f32) -> f32 {
        while elapsed_s >= self.next_edge_s {
            self.edge += 1;
            self.next_edge_s += half_period_s(self.edge);
            self.high = !self.high;
        }
        let square = if self.high { self.amplitude } else { 0.0 };
        self.washout = lagged(self.washout, square, dt, self.washout_tc_s);
        square - self.washout
    }
}

/// Each delay's least-squares sums, fed one tick at a time, and their solution.
#[derive(Clone, Debug, PartialEq)]
struct DelayFits {
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

impl DelayFits {
    fn new() -> Self {
        DelayFits {
            rates_radps: [0.0; 2],
            moments: [0.0; HISTORY],
            filled: 0,
            sums: [Sums::default(); MAX_DELAY_TICKS],
            rows: 0,
            rows_dt_s: 0.0,
        }
    }

    /// Adds this tick to every delay's sums once the history is full, then shifts the history;
    /// a non-finite tick empties it instead.
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
                sums.add(&regressors, target);
            }
            self.rows += 1;
            self.rows_dt_s += dt;
        }
        self.rates_radps = [rate_radps, self.rates_radps[0]];
    }

    /// The model at the delay [`delay_to_fit`] picks; `None` when no delay's sums solve.
    fn solve(&self) -> Option<RateModel> {
        let dt_s = self.rows_dt_s / self.rows.max(1) as f32;
        let fits: [Option<(RateModel, f32)>; MAX_DELAY_TICKS] = core::array::from_fn(|index| {
            let (solution, residual) = self.sums[index].solve()?;
            Some((model_at(index + 1, &solution, dt_s), residual))
        });
        let effects =
            fits.map(|fit| fit.map(|(model, residual)| (model.moment_effect(), residual)));
        let delay_ticks = delay_to_fit(&effects)?;
        fits[delay_ticks - 1].map(|(model, _)| model)
    }
}

/// The model a delay's least-squares `solution` gives, at `delay_ticks` and ticks of `dt_s`.
fn model_at(delay_ticks: usize, solution: &SVector<f32, TERMS>, dt_s: f32) -> RateModel {
    // Δy[k] = c₀·y[k−1] + c₁·(y[k−1] − y[k−2]) + …, so y[k] = (1 + c₀ + c₁)·y[k−1] − c₁·y[k−2] + ….
    let (change, difference) = (solution[0], solution[1]);
    RateModel {
        rate_weights: [1.0 + change + difference, -difference],
        moment_weights: [solution[2], solution[3], solution[4]],
        delay_ticks,
        dt_s,
    }
}

/// The least-squares sums at one delay. `normal` keeps the full symmetric matrix (21 entries
/// would do) for a plain `φ·φᵀ` and nalgebra's solve; halving it saves about 0.6 KB.
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

impl Sums {
    fn add(&mut self, regressors: &SVector<f32, TERMS>, target: f32) {
        self.normal += regressors * regressors.transpose();
        self.projected += regressors * target;
        self.target_squared += target * target;
    }

    /// The least-squares solution and its residual `Σt² − θ·Σφt` (at least 0); `None` when
    /// the sums don't factor or solve to a non-finite value.
    fn solve(&self) -> Option<(SVector<f32, TERMS>, f32)> {
        let solution = self.normal.cholesky()?.solve(&self.projected);
        let residual = (self.target_squared - solution.dot(&self.projected)).max(0.0);
        let finite = solution.iter().all(|x| x.is_finite()) && residual.is_finite();
        finite.then_some((solution, residual))
    }
}

/// The delay, in ticks (index + 1), to fit from each delay's moment effect (the sum of its
/// moment weights) and residual: [`shortest_tied`] among those with a positive moment effect,
/// or among all when none has one (a reversed axis, which the design rejects as such). A
/// negative effect pushes the axis against the moment commanded, which no design can use, and
/// in flight it comes from noise favoring a wrong delay (#529). A pushing delay wins however
/// far its residual is from a wrong-way least: on QuadCopterX in RealFlight the 7 fits it
/// decided sat 1–9% above it, 5 read 37–63 rad/s² (the axis's healthy fits read 30–70), and
/// the design rejects the other 2 (0 and 8) as it did before. `None` when no delay solved.
fn delay_to_fit(fits: &[Option<(f32, f32)>]) -> Option<usize> {
    let fit_at = |index: usize| fits.get(index).copied().flatten();
    let pushing: [Option<f32>; MAX_DELAY_TICKS] = core::array::from_fn(|index| {
        fit_at(index)
            .filter(|(effect, _)| *effect > 0.0)
            .map(|(_, residual)| residual)
    });
    let all: [Option<f32>; MAX_DELAY_TICKS] =
        core::array::from_fn(|index| fit_at(index).map(|(_, residual)| residual));
    shortest_tied(&pushing).or_else(|| shortest_tied(&all))
}

/// The shortest delay, in ticks (index + 1), whose residual is within [`TIE_SHARE`] of the
/// least; `None` when no delay solved.
fn shortest_tied(residuals: &[Option<f32>]) -> Option<usize> {
    let least = residuals.iter().flatten().copied().reduce(f32::min)?;
    let tied = least + TIE_SHARE * least;
    residuals
        .iter()
        .position(|residual| residual.is_some_and(|r| r <= tied))
        .map(|index| index + 1)
}
