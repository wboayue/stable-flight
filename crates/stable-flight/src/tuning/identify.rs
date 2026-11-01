//! Closed-loop identification of a rate axis: see [`IdentifyProbe`].
//!
//! Flies the excitation PX4's autotune flies (`mc_autotune_attitude_control`'s
//! `getIdentificationSignal`, added to the rate setpoint by `mc_att_control`), described, not
//! copied: a washed-out square wave whose half-periods shrink. PX4 fits a second-order model by
//! recursive least squares on the measured rate (`ArxRls` in `system_identification`); in
//! closed loop the controller feeds gyro noise into the moment, and that fit reads every lag as
//! 0 under the sim's noise. This probe keeps a least-squares fit on the recorded moments for a
//! quiet window and fits a noisy one in two stages instead (`architecture.md#autotune`, #551).

use nalgebra::{Complex, SMatrix, SVector};

use crate::controllers::{InvalidConfig, check, check_positive};
use crate::math::{assert_dt, lagged};

#[cfg(test)]
mod tests;

/// Longest delay the fit tries, in ticks: 40 ms at 250 Hz, past the sim's 25 ms actuator delay
/// (7 ticks with the hold).
pub const MAX_DELAY_TICKS: usize = 10;

/// Share of the window's moments at the rate loop's limit past which [`fit_window`] fits the
/// recorded moments rather than the explained ones: the copter's ×3 starts ring at ±1 for
/// 510–629 of 750 ticks with no noise, and the excitation then explains little of the moment,
/// so [`fit_explained`] misread their lags (racer 7.6 ms against 15) where [`fit_recorded`] is
/// exact; the low starts and RealFlight's QuadCopterX don't saturate at all (#551).
const SATURATED_SHARE: f32 = 0.2;

/// A moment at or past this magnitude counts as at the rate loop's ±1 limit.
const SATURATED_MOMENT: f32 = 0.999;

/// The least-squares fit's residual, RMS per tick in rad/s, past which [`fit_window`] fits the
/// explained moments instead: the least squares then doesn't explain the window, which is
/// gyro noise read through the closed loop. On a quiet window the moment's fast part is the
/// controller answering the airframe, which is what tells a delay from a lag, and the
/// projection drops it: on RealFlight's eXplora roll every delay from 1 to 10½ ticks fitted
/// within 1.8% of the least, and the two-stage fit designed rate P 3.92 against the least
/// squares' (and RealFlight's) 0.94. RealFlight's windows leave 0.002–0.012 (eXplora and
/// QuadCopterX, 8 windows), the sim eXplora with 0.01 rad/s of gyro noise 0.012, and the sim's
/// noise rows 0.024–0.100. A clean sim window leaves under 0.001 whatever its gains, a loop
/// that rings included (#551).
const NOISY_RESIDUAL_RADPS: f32 = 0.017;

/// Ticks of rate, moment, and excitation the probe records: 4 s at 250 Hz, past the copter's
/// 3 s window. A longer window fits its first `WINDOW_CAPACITY` ticks.
const WINDOW_CAPACITY: usize = 1000;

/// Past excitation ticks the moment is projected on ([`project`]): 40 ms at 250 Hz, the
/// moment's first answer to an edge. More terms fit more of the noise in the window by chance,
/// and that part moved the airframe too, so it reads as a lag of 0: at 40 taps `racer`'s fits
/// under gyro noise at its 75 Hz low-pass read 0 on 8 of 10 roll and pitch fits, at 10 on none
/// (#551).
const PROJECTION_TAPS: usize = 10;

/// Time constants, in s, of the slow terms the projection adds past the taps: the excitation
/// through first-order lags, for the rest of the moment's answer to an edge (an angle loop
/// around the rate loop answers over about a second). With the taps alone a 20 ms lag read
/// 43 ms with the excitation on an angle loop (#551).
const SLOW_TCS_S: [f32; 6] = [0.02, 0.05, 0.1, 0.2, 0.5, 1.0];

/// Terms of the projection: the taps, the slow terms, and a constant.
const PROJECTION_TERMS: usize = PROJECTION_TAPS + SLOW_TCS_S.len() + 1;

/// The lag grid [`fit_explained`] searches, in s: 0, then `MIN_LAG_S` up to `MAX_LAG_S` in
/// steps of `LAG_RATIO`, then [`refine`]s between the best point's neighbors. The top is past
/// the Alia 250's 0.17 s motors on the sim.
const MIN_LAG_S: f32 = 0.001;
const MAX_LAG_S: f32 = 0.5;
const LAG_RATIO: f32 = 1.05;

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

/// Residuals within this share of the least count as tied, and the shortest such delay wins,
/// so rounding doesn't pick between near-equal delays (a 25 ms delay fit at 6 or 7 ticks, #516). Wider favors delays too short, which
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
/// moment through one motor lag fits it with `r₁ = 1 + p`, `r₂ = −p`, `p` the lag's pole: the
/// form [`IdentifyProbe`] fits, its moment on one weight or, half a tick later, two equal ones.
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
    /// The motor lag's time constant, in s, from its pole `p = −r₂`: `−dt / ln p` for a pole
    /// in `(0, 1)`. A pole at or below 0 is a lag under a tick, which the delay absorbs, so 0
    /// (#563). `None` for a pole at or past 1, which grows rather than lags.
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
        if pole >= 1.0 || pole.is_nan() {
            return None;
        }
        Some(if pole > 0.0 {
            -self.dt_s / libm::logf(pole)
        } else {
            0.0
        })
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
    /// the moments' total weight over `(1 − p)·dt`, a pole at or below 0 taken as 0, as
    /// [`lag_s`](Self::lag_s) takes it. `None` without a lag.
    pub fn accel_radps2(&self) -> Option<f32> {
        self.lag_s()?;
        let pole = (-self.rate_weights[1]).max(0.0);
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
#[non_exhaustive]
pub enum IdentifyAbort {
    /// The fit couldn't be solved: the window was too short, or the excitation didn't move the
    /// moment enough to tell its terms apart.
    Unexcited,
    /// Most of the window's moments sat at the rate loop's limit, and the least squares left
    /// it noisy: the excitation explains too little of a clipped moment for the two-stage fit,
    /// and the least squares reads the noise as the airframe. Lower gains unclip it.
    Saturated,
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
/// the fit sees the airframe through its own controllers.
///
/// The fit asks how the moment commanded, through a motor lag and a delay, accelerates the
/// axis. It first fits the rate's change per tick by least squares to the last two rates and
/// the recorded moments at each delay from 1 to [`MAX_DELAY_TICKS`], keeping the delay with
/// the least residual (the shortest of a near-tie) among those whose moment pushes the axis
/// the way it was pushed (among all when none does). That is exact without noise.
///
/// In closed loop, though, the moment also carries the controller's answer to gyro noise,
/// which that fit reads as part of the airframe. When it leaves more than 0.017 rad/s of the
/// rate's change per tick unexplained (RMS), the probe fits in two stages instead: it keeps only the part of
/// the moment the excitation explains (the excitation is the probe's own, so no noise is in
/// it), filters that through each lag on a grid and each delay, half ticks included, and
/// fits the rate's change per tick to it (a gain and a constant for the trim), the measured
/// rate only the target. On a quiet window the least squares stays: there the moment's fast
/// part is the controller answering the airframe, which is what tells a delay from a lag, and
/// the two-stage fit drops it. So does a window whose moments mostly sit at the rate loop's
/// limit (a start that rings), whose moment the excitation explains too little of.
///
/// The probe records the window (13 KB) and fits it on the tick the excitation ends. The
/// least squares costs about 0.06 ms there on a desktop, the two-stage fit about 2 ms (128
/// lags × 20 delays over the window), so a noisy window's last tick runs long. A non-finite
/// rate or moment is recorded as the last finite one for the two-stage fit; the least squares
/// skips that tick and the next `MAX_DELAY_TICKS + 2` while its history refills.
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
    window: Window,
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
            window: Window::new(),
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
        let elapsed_s = elapsed_s + dt;
        let offset = self.sweep.next(dt, elapsed_s);
        self.window.record(dt, rate_radps, last_moment, offset);
        self.status = if elapsed_s < self.duration_s {
            IdentifyStatus::Running { elapsed_s }
        } else {
            match fit_window(&self.window) {
                Ok(model) => IdentifyStatus::Done(model),
                Err(abort) => IdentifyStatus::Aborted(abort),
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
pub(super) struct SquareSweep {
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
    pub(super) fn new(amplitude: f32, washout_tc_s: f32) -> Self {
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
    pub(super) fn next(&mut self, dt: f32, elapsed_s: f32) -> f32 {
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

/// The recorded window: each tick's body rate, the moment commanded on the tick before, and
/// the excitation offset returned for the tick.
#[derive(Clone, Debug, PartialEq)]
struct Window {
    rates_radps: [f32; WINDOW_CAPACITY],
    moments: [f32; WINDOW_CAPACITY],
    offsets: [f32; WINDOW_CAPACITY],
    /// Ticks whose rate or moment wasn't finite, recorded as the last finite one.
    held: [bool; WINDOW_CAPACITY],
    len: usize,
    /// Sum of the recorded ticks' `dt`, in s.
    duration_s: f32,
}

impl Window {
    fn new() -> Self {
        Window {
            rates_radps: [0.0; WINDOW_CAPACITY],
            moments: [0.0; WINDOW_CAPACITY],
            offsets: [0.0; WINDOW_CAPACITY],
            held: [false; WINDOW_CAPACITY],
            len: 0,
            duration_s: 0.0,
        }
    }

    /// Records a tick, a non-finite rate or moment as the last finite one (0 before any);
    /// past [`WINDOW_CAPACITY`] ticks, records nothing.
    fn record(&mut self, dt: f32, rate_radps: f32, last_moment: f32, offset: f32) {
        let index = self.len;
        if index == WINDOW_CAPACITY {
            return;
        }
        let held = |value: f32, values: &[f32; WINDOW_CAPACITY]| {
            if value.is_finite() {
                value
            } else {
                index.checked_sub(1).map_or(0.0, |last| values[last])
            }
        };
        self.rates_radps[index] = held(rate_radps, &self.rates_radps);
        self.moments[index] = held(last_moment, &self.moments);
        self.offsets[index] = offset;
        self.held[index] = !(rate_radps.is_finite() && last_moment.is_finite());
        self.len += 1;
        self.duration_s += dt;
    }
}

/// The [`RateModel`] fitted to `window`: by least squares on the recorded moments
/// ([`fit_recorded`]), unless that fit leaves more than [`NOISY_RESIDUAL_RADPS`] unexplained
/// and no more than [`SATURATED_SHARE`] of the moments sit at the rate loop's limit, when the
/// explained moments are fitted instead ([`fit_explained`]). `None` when the fit doesn't
/// solve.
fn fit_window(window: &Window) -> Result<RateModel, IdentifyAbort> {
    let recorded = fit_recorded(window).ok_or(IdentifyAbort::Unexcited)?;
    let moments = &window.moments[..window.len];
    let saturated = moments
        .iter()
        .filter(|moment| moment.abs() >= SATURATED_MOMENT)
        .count();
    let saturates = saturated as f32 > SATURATED_SHARE * window.len as f32;
    let noisy = recorded.residual_radps > NOISY_RESIDUAL_RADPS;
    match (noisy, saturates) {
        (true, true) => Err(IdentifyAbort::Saturated),
        (true, false) => fit_explained(window).ok_or(IdentifyAbort::Unexcited),
        (false, _) => Ok(recorded.model),
    }
}

/// The [`RateModel`] fitted to the part of the moment the excitation explains
/// ([`IdentifyProbe`] says how), at the delay [`delay_to_fit`] picks; `None` when the
/// projection doesn't solve or the window is too short to fit any delay.
fn fit_explained(window: &Window) -> Option<RateModel> {
    let len = window.len;
    let dt_s = window.duration_s / len.max(1) as f32;
    let explained = project(window)?;
    // The first tick every delay's two regressors and the projection reach.
    let first = PROJECTION_TAPS + MAX_DELAY_TICKS + 1;
    if len <= first + 1 {
        return None;
    }
    let mut best: [Option<LagFit>; DELAY_STEPS] = [None; DELAY_STEPS];
    let mut accel = [0.0f32; WINDOW_CAPACITY];
    let mut previous_pole = 0.0;
    let mut poles = lag_poles(dt_s).peekable();
    while let Some(pole) = poles.next() {
        lag_through(&explained, pole, len, &mut accel);
        let next_pole = poles.peek().copied().unwrap_or(pole);
        for (step, best) in best.iter_mut().enumerate() {
            let fit = fit_gain(window, &accel, Delay::at(step), first, len);
            if fit.is_some_and(|fit| best.is_none_or(|kept| fit.residual < kept.residual)) {
                *best = fit.map(|fit| LagFit {
                    pole,
                    bracket: [previous_pole, next_pole],
                    ..fit
                });
            }
        }
        previous_pole = pole;
    }
    let effects = best.map(|fit| fit.map(|fit| (fit.gain, fit.residual)));
    let step = delay_to_fit(&effects)? - 1;
    let delay = Delay::at(step);
    let LagFit { pole, gain, .. } = refine(window, &explained, delay, best[step]?, first);
    let [now, next] = delay.split().map(|share| share * gain * (1.0 - pole));
    Some(RateModel {
        rate_weights: [1.0 + pole, -pole],
        moment_weights: [now, next, 0.0],
        delay_ticks: delay.ticks,
        dt_s,
    })
}

/// `explained` through a lag of pole `pole` into `accel`, over the first `len` ticks:
/// `a[k] = p·a[k−1] + (1 − p)·u[k]`.
fn lag_through(
    explained: &[f32; WINDOW_CAPACITY],
    pole: f32,
    len: usize,
    accel: &mut [f32; WINDOW_CAPACITY],
) {
    let mut lagged = 0.0;
    for (accel, moment) in accel.iter_mut().zip(explained).take(len) {
        lagged = pole * lagged + (1.0 - pole) * moment;
        *accel = lagged;
    }
}

/// Golden-section steps that refine the grid's best lag between its neighbors: the bracket
/// shrinks to 0.618¹² ≈ 0.3% of a 10% step, and a grid point alone sits up to 2.5% off, past
/// the lag a test can hold (a 31 ms lag read 31.9 between grid points 30.4 and 31.9, #551).
const REFINE_STEPS: usize = 12;

/// `grid_fit` at `delay`, its pole refined by golden section within its bracket on the
/// residual; the grid's own fit if no refined pole beats it.
fn refine(
    window: &Window,
    explained: &[f32; WINDOW_CAPACITY],
    delay: Delay,
    grid_fit: LagFit,
    first: usize,
) -> LagFit {
    const GOLDEN: f32 = 0.618_034;
    let len = window.len;
    let mut accel = [0.0f32; WINDOW_CAPACITY];
    let mut fit_at = |pole: f32| {
        lag_through(explained, pole, len, &mut accel);
        fit_gain(window, &accel, delay, first, len).map(|fit| LagFit { pole, ..fit })
    };
    let [mut low, mut high] = grid_fit.bracket;
    let mut best = grid_fit;
    for _ in 0..REFINE_STEPS {
        let left = high - GOLDEN * (high - low);
        let right = low + GOLDEN * (high - low);
        let (Some(left_fit), Some(right_fit)) = (fit_at(left), fit_at(right)) else {
            break;
        };
        let kept = if left_fit.residual < right_fit.residual {
            high = right;
            left_fit
        } else {
            low = left;
            right_fit
        };
        if kept.residual < best.residual {
            best = kept;
        }
    }
    best
}

/// The part of each recorded moment the excitation explains: the moment regressed by least
/// squares on the last [`PROJECTION_TAPS`] offsets, the offset through lags of
/// [`SLOW_TCS_S`], and a constant, then predicted from them (0 before the taps fill). Gyro noise drives the rest of the moment through the controller;
/// the offsets are the probe's own, so their prediction carries none of it. `None` when the
/// regression doesn't solve.
fn project(window: &Window) -> Option<[f32; WINDOW_CAPACITY]> {
    let dt_s = window.duration_s / window.len.max(1) as f32;
    let mut normal = SMatrix::<f32, PROJECTION_TERMS, PROJECTION_TERMS>::zeros();
    let mut projected = SVector::<f32, PROJECTION_TERMS>::zeros();
    let mut terms = ProjectionTerms::new(dt_s);
    for tick in 0..window.len {
        if let Some(terms) = terms.next(window, tick) {
            normal += terms * terms.transpose();
            projected += terms * window.moments[tick];
        }
    }
    let weights = normal.cholesky()?.solve(&projected);
    if !weights.iter().all(|weight| weight.is_finite()) {
        return None;
    }
    let mut explained = [0.0; WINDOW_CAPACITY];
    let mut terms = ProjectionTerms::new(dt_s);
    for (tick, moment) in explained.iter_mut().enumerate().take(window.len) {
        if let Some(terms) = terms.next(window, tick) {
            *moment = weights.dot(&terms);
        }
    }
    Some(explained)
}

/// The projection's terms tick by tick: the moment recorded on tick k was commanded on tick
/// k − 1, after that tick's offset, so its terms are the offsets from tick k − 1 back, the slow
/// lags of the offset up to tick k − 1, and 1.
struct ProjectionTerms {
    dt_s: f32,
    slow: [f32; SLOW_TCS_S.len()],
}

impl ProjectionTerms {
    fn new(dt_s: f32) -> Self {
        ProjectionTerms {
            dt_s,
            slow: [0.0; SLOW_TCS_S.len()],
        }
    }

    /// Tick `tick`'s terms, `None` until the taps fill; call once per tick, in order.
    fn next(&mut self, window: &Window, tick: usize) -> Option<SVector<f32, PROJECTION_TERMS>> {
        let last_offset = window.offsets[tick.checked_sub(1)?];
        for (slow, tc_s) in self.slow.iter_mut().zip(SLOW_TCS_S) {
            *slow = lagged(*slow, last_offset, self.dt_s, tc_s);
        }
        if tick < PROJECTION_TAPS {
            return None;
        }
        let mut terms = SVector::<f32, PROJECTION_TERMS>::zeros();
        for (tap, term) in terms.iter_mut().take(PROJECTION_TAPS).enumerate() {
            *term = window.offsets[tick - 1 - tap];
        }
        for (index, slow) in self.slow.iter().enumerate() {
            terms[PROJECTION_TAPS + index] = *slow;
        }
        terms[PROJECTION_TERMS - 1] = 1.0;
        Some(terms)
    }
}

/// The lag grid's poles at ticks of `dt_s`: a lag of 0 (pole 0), then `e^(−dt/τ)` for each
/// lag `τ` from [`MIN_LAG_S`] to [`MAX_LAG_S`] by [`LAG_RATIO`].
fn lag_poles(dt_s: f32) -> impl Iterator<Item = f32> {
    let lags_s = core::iter::successors(Some(MIN_LAG_S), |lag_s| Some(lag_s * LAG_RATIO))
        .take_while(|lag_s| *lag_s <= MAX_LAG_S);
    core::iter::once(0.0).chain(lags_s.map(move |lag_s| libm::expf(-dt_s / lag_s)))
}

/// Rate change per tick fitted to the last two rates and three recorded moments
/// (`y[k] − y[k−1]` on `y[k−1]`, `y[k−1] − y[k−2]`, `u[k−d]`, `u[k−d−1]`, `u[k−d−2]`, and a
/// constant) at each delay, by least squares, at the delay [`delay_to_fit`] picks; `None` when
/// no delay's sums solve. Exact on a clean window, saturated or not; under gyro noise it reads
/// every lag as 0, as the measured rate is among its terms (#551). Its rate terms are the
/// change and the difference rather than the two rates: at 250 Hz the last two rates are
/// nearly equal, and fitting them side by side loses the lag in `f32` (#516).
fn fit_recorded(window: &Window) -> Option<RecordedFit> {
    let dt_s = window.duration_s / window.len.max(1) as f32;
    let mut fits = DelayFits::new();
    for tick in 0..window.len {
        // A held tick goes in as not finite, which restarts the history past it.
        let rate_radps = if window.held[tick] {
            f32::NAN
        } else {
            window.rates_radps[tick]
        };
        fits.record(dt_s, rate_radps, window.moments[tick]);
    }
    fits.solve()
}

/// [`fit_recorded`]'s model, and how much of the rate's change per tick it leaves unexplained.
#[derive(Clone, Copy, Debug, PartialEq)]
struct RecordedFit {
    model: RateModel,
    /// RMS of the residual per tick, in rad/s.
    residual_radps: f32,
}

/// Moments [`fit_recorded`] keeps: the longest delay's oldest term, `u[k − d − 2]`.
const HISTORY: usize = MAX_DELAY_TICKS + 2;

/// Terms in [`fit_recorded`]: two rates, three moments, a constant.
const TERMS: usize = 6;

/// Each delay's least-squares sums for [`fit_recorded`], fed one tick at a time, and their
/// solution.
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

    /// The fit at the delay [`delay_to_fit`] picks; `None` when no delay's sums solve.
    fn solve(&self) -> Option<RecordedFit> {
        let dt_s = self.rows_dt_s / self.rows.max(1) as f32;
        let fits: [Option<(RateModel, f32)>; MAX_DELAY_TICKS] = core::array::from_fn(|index| {
            let (solution, residual) = self.sums[index].solve()?;
            Some((model_at(index + 1, &solution, dt_s), residual))
        });
        let effects =
            fits.map(|fit| fit.map(|(model, residual)| (model.moment_effect(), residual)));
        let delay_ticks = delay_to_fit(&effects)?;
        let (model, residual) = fits[delay_ticks - 1]?;
        Some(RecordedFit {
            model,
            residual_radps: libm::sqrtf(residual / self.rows.max(1) as f32),
        })
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

/// Delays the fit tries: each whole tick from 1 to [`MAX_DELAY_TICKS`], and each half tick
/// past it.
const DELAY_STEPS: usize = 2 * MAX_DELAY_TICKS;

/// A delay in half ticks: `ticks`, plus half a tick when `half`, the moment then felt half on
/// each of two ticks. The zero-order hold leaves a moment's first tick partly felt, and with
/// whole ticks alone a 20 ms lag read 24 ms at 1 tick, or right at 2 (#551). A free weight on
/// each tick instead fitted noise, opposite in sign.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Delay {
    ticks: usize,
    half: bool,
}

impl Delay {
    /// The `step`-th delay, shortest first: 1, 1½, 2, 2½, … ticks.
    fn at(step: usize) -> Self {
        Delay {
            ticks: step / 2 + 1,
            half: step % 2 == 1,
        }
    }

    /// The share of the moment felt `ticks` back and one tick more.
    fn split(self) -> [f32; 2] {
        if self.half { [0.5, 0.5] } else { [1.0, 0.0] }
    }
}

/// One lag and delay's least squares: the gain from the lagged moment to the rate's change per
/// tick (positive pushes the axis the way the moment does), and the residual left. `pole` and
/// `bracket` are filled in by the caller.
#[derive(Clone, Copy, Debug, PartialEq)]
struct LagFit {
    pole: f32,
    /// The neighboring grid poles, which [`refine`] searches between.
    bracket: [f32; 2],
    gain: f32,
    residual: f32,
}

/// Fits `y[k] − y[k−1] ≈ gain·x[k] + constant` over ticks `first..len` by least squares, `x`
/// the lagged moment at `delay` (`a[k − d + 1]`, or its mean with `a[k − d]` at a half tick);
/// `None` when it doesn't vary. Sums in `f64`: the residual, `Σt² − gain·Σxt − constant·Σt`,
/// cancels to a small share of `Σt²`, and delays are told apart by 1% of it ([`TIE_SHARE`]).
fn fit_gain(
    window: &Window,
    accel: &[f32; WINDOW_CAPACITY],
    delay: Delay,
    first: usize,
    len: usize,
) -> Option<LagFit> {
    let [now, next] = delay.split().map(f64::from);
    let (mut sxx, mut sx, mut sxt, mut st, mut stt) = (0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for tick in first..len {
        let felt = tick + 1 - delay.ticks;
        let x = now * f64::from(accel[felt]) + next * f64::from(accel[felt - 1]);
        let target = f64::from(window.rates_radps[tick] - window.rates_radps[tick - 1]);
        sxx += x * x;
        sx += x;
        sxt += x * target;
        st += target;
        stt += target * target;
    }
    let count = (len - first) as f64;
    let det = sxx * count - sx * sx;
    // A lagged moment that doesn't vary (or a NaN) leaves the gain and the constant unsolved.
    let varies = det > f64::EPSILON * sxx * count;
    if !varies {
        return None;
    }
    let gain = (sxt * count - sx * st) / det;
    let constant = (sxx * st - sx * sxt) / det;
    let residual = (stt - gain * sxt - constant * st).max(0.0);
    Some(LagFit {
        pole: 0.0,
        bracket: [0.0; 2],
        gain: gain as f32,
        residual: residual as f32,
    })
}

/// The delay to fit, as its index in `fits` + 1 (shortest first, at most [`DELAY_STEPS`]), from
/// each delay's moment effect and residual: [`shortest_tied`] among those with a positive
/// moment effect,
/// or among all when none has one (a reversed axis, which the design rejects as such). A
/// negative effect pushes the axis against the moment commanded, which no design can use, and
/// in flight it comes from noise favoring a wrong delay (#529). A pushing delay wins however
/// far its residual is from a wrong-way least: on QuadCopterX in RealFlight the 7 fits it
/// decided sat 1–9% above it, 5 read 37–63 rad/s² (the axis's healthy fits read 30–70), and
/// the design rejects the other 2 (0 and 8) as it did before. `None` when no delay solved.
fn delay_to_fit(fits: &[Option<(f32, f32)>]) -> Option<usize> {
    let fit_at = |index: usize| fits.get(index).copied().flatten();
    let pushing: [Option<f32>; DELAY_STEPS] = core::array::from_fn(|index| {
        fit_at(index)
            .filter(|(effect, _)| *effect > 0.0)
            .map(|(_, residual)| residual)
    });
    let all: [Option<f32>; DELAY_STEPS] =
        core::array::from_fn(|index| fit_at(index).map(|(_, residual)| residual));
    shortest_tied(&pushing).or_else(|| shortest_tied(&all))
}

/// The shortest delay, as its index + 1, whose residual is within [`TIE_SHARE`] of the least;
/// `None` when no delay solved.
fn shortest_tied(residuals: &[Option<f32>]) -> Option<usize> {
    let least = residuals.iter().flatten().copied().reduce(f32::min)?;
    let tied = least + TIE_SHARE * least;
    residuals
        .iter()
        .position(|residual| residual.is_some_and(|r| r <= tied))
        .map(|index| index + 1)
}
