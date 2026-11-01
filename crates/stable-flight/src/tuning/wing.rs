//! Closed-loop identification of a wing's roll or pitch axis: see [`WingIdentifyProbe`].

use nalgebra::{SMatrix, SVector};

use super::identify::SquareSweep;
use super::{IdentifyAbort, IdentifyConfig};
use crate::math::assert_dt;

#[cfg(test)]
mod tests;

/// Ticks per recorded sample: every second tick, 8 ms at 250 Hz. A wing's loops answer over
/// tenths of a second, and the surfaces' lags on the sim's planes fit at 0–0.1 s (#533).
const DECIMATION: usize = 2;

/// Samples the probe records: 6 s at 250 Hz, the window the sweep fitted (#533). A longer
/// excitation fits its first `WINDOW_CAPACITY` samples.
const WINDOW_CAPACITY: usize = 750;

/// The surface lags the fit tries, in s, up to the Alia 250's 0.17 s motors with room.
const LAGS_S: [f32; 12] = [
    0.0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14, 0.17, 0.2, 0.25, 0.3,
];

/// Longest delay the fit tries, in samples: 40 ms, past the sim's 25 ms actuator delay.
const MAX_DELAY_SAMPLES: usize = 5;

/// Terms of the fit: angle of attack, rate, lagged moment, and a constant for the trim.
const TERMS: usize = 4;

/// An angle of attack that varies by less than this over the window, in rad, isn't fitted:
/// a roll axis, whose host reports none.
const MIN_ALPHA_SPREAD_RAD: f32 = 1e-4;

/// A wing axis's response to its moment command, fitted in flight: how the moment, through the
/// surfaces' lag and a delay, accelerates the axis against its damping and, in pitch, the
/// wing's stiffness in angle of attack.
///
/// `rate′ = −stiffness·α − damping·rate + control·surface`, with `rate` the body rate in rad/s,
/// `α` the angle of attack off its trim in rad, and `surface` the normalized moment command
/// through a first-order lag of `lag_s`, `delay_s` late. A stable wing has positive stiffness
/// and damping. In pitch the flight path turns with angle of attack, `γ′ = path_turn·α`, so a
/// held pitch settles back to the trim angle of attack over `1 / path_turn` seconds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WingModel {
    /// Pitch acceleration per rad of angle of attack, nose down for a positive one, in 1/s²; 0
    /// on a roll axis.
    pub stiffness_per_s2: f32,
    /// Rate acceleration per rad/s of body rate, against the rate, in 1/s.
    pub damping_per_s: f32,
    /// Rate acceleration per unit moment at full surface, in rad/s².
    pub control_radps2: f32,
    /// The surfaces' first-order lag, in s.
    pub lag_s: f32,
    /// Time from a moment command to its first effect, in s.
    pub delay_s: f32,
    /// The flight path's turn per rad of angle of attack (lift over mass and speed), in 1/s; 0
    /// on a roll axis.
    pub path_turn_per_s: f32,
}

impl WingModel {
    /// The moment per rad/s of body rate that holds a rate against the damping: the rate
    /// loop's feedforward, `damping / control`.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::WingModel;
    ///
    /// let model = WingModel {
    ///     stiffness_per_s2: 0.0,
    ///     damping_per_s: 7.0,
    ///     control_radps2: 14.0,
    ///     lag_s: 0.0,
    ///     delay_s: 0.0,
    ///     path_turn_per_s: 0.0,
    /// };
    /// assert_eq!(model.feedforward(), 0.5);
    /// ```
    pub fn feedforward(&self) -> f32 {
        self.damping_per_s / self.control_radps2
    }
}

/// Where a [`WingIdentifyProbe`] stands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WingStatus {
    /// Exciting and recording.
    Running {
        /// Time since the excitation began, in s.
        elapsed_s: f32,
    },
    /// The fitted model.
    Done(WingModel),
    /// Gave up.
    Aborted(IdentifyAbort),
}

/// Excites a wing's rate loop with the square wave an [`IdentifyProbe`](super::IdentifyProbe)
/// flies while recording its axis's body rate, moment command, and angle of attack, then fits
/// a [`WingModel`] to them.
///
/// Each tick the host hands it the axis's body rate, the moment it commanded on the last tick,
/// and the angle of attack (0 on a roll axis); the probe returns the offset to add to the rate
/// setpoint. Every loop stays closed.
///
/// The fit is least squares on every second tick: the rate's change against the angle of
/// attack, the rate, the moment through each surface lag in a grid from 0 to 0.3 s and each
/// delay up to 40 ms, and a constant for the trim, keeping the lag and delay that leave the
/// least residual. The flight path's turn is fitted apart: the climb angle (pitch less angle of
/// attack) changes at `rate − α′`, regressed on α. A wing's pitch damping and stiffness both
/// change the moment a held rate needs, so a fit on rate and moment alone misreads the
/// feedforward (0.53 against the Alia 250's 0.907 on the sim, #533).
///
/// The window holds 750 samples (9 KB) and is fitted on the tick the excitation ends: 12 lags
/// × 6 delays over it. A non-finite rate, moment, or angle of attack is recorded as the last
/// finite one.
///
/// # Examples
///
/// ```
/// use stable_flight::tuning::{IdentifyConfig, WingIdentifyProbe, WingStatus};
///
/// let config = IdentifyConfig { amplitude: 0.3, duration_s: 6.0, washout_tc_s: 0.2 };
/// let mut probe = WingIdentifyProbe::new(config);
/// let offset = probe.step(0.004, 0.0, 0.0, 0.0);
/// assert!(offset > 0.0 && offset <= config.amplitude);
/// assert!(matches!(probe.status(), WingStatus::Running { .. }));
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct WingIdentifyProbe {
    duration_s: f32,
    status: WingStatus,
    sweep: SquareSweep,
    window: Window,
}

impl WingIdentifyProbe {
    /// A probe ready to run.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`IdentifyConfig::validate`]).
    pub fn new(config: IdentifyConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid IdentifyConfig: {error}");
        }
        Self {
            duration_s: config.duration_s,
            status: WingStatus::Running { elapsed_s: 0.0 },
            sweep: SquareSweep::new(config.amplitude, config.washout_tc_s),
            window: Window::new(),
        }
    }

    /// Advances the probe by `dt` seconds with the axis's body rate this tick (rad/s), its
    /// moment command on the last tick, and the angle of attack (rad), and returns the offset
    /// to add to the rate setpoint (0 once it has finished). Fits the model on the tick the
    /// excitation ends.
    ///
    /// # Panics
    ///
    /// If `dt` is not positive and finite.
    pub fn step(&mut self, dt: f32, rate_radps: f32, last_moment: f32, alpha_rad: f32) -> f32 {
        assert_dt(dt);
        let WingStatus::Running { elapsed_s } = self.status else {
            return 0.0;
        };
        let elapsed_s = elapsed_s + dt;
        let offset = self.sweep.next(dt, elapsed_s);
        self.window.record(dt, rate_radps, last_moment, alpha_rad);
        self.status = if elapsed_s < self.duration_s {
            WingStatus::Running { elapsed_s }
        } else {
            match fit(&self.window) {
                Some(model) => WingStatus::Done(model),
                None => WingStatus::Aborted(IdentifyAbort::Unexcited),
            }
        };
        offset
    }

    /// Where the probe stands.
    pub fn status(&self) -> WingStatus {
        self.status
    }
}

/// The recorded window: every [`DECIMATION`]th tick's body rate and angle of attack, and the
/// mean of the moments that moved the rate there from the last sample (the last
/// [`DECIMATION`] ticks' last moments).
#[derive(Clone, Debug, PartialEq)]
struct Window {
    rates_radps: [f32; WINDOW_CAPACITY],
    moments: [f32; WINDOW_CAPACITY],
    alphas_rad: [f32; WINDOW_CAPACITY],
    len: usize,
    /// Ticks seen, recorded or not.
    ticks: usize,
    /// Sum of every tick's `dt` up to the last sample, in s.
    duration_s: f32,
    /// The last finite rate, moment, and angle of attack, which stand in for a non-finite one.
    last: [f32; 3],
    /// The moments since the last sample, summed.
    moment_sum: f32,
}

impl Window {
    fn new() -> Self {
        Self {
            rates_radps: [0.0; WINDOW_CAPACITY],
            moments: [0.0; WINDOW_CAPACITY],
            alphas_rad: [0.0; WINDOW_CAPACITY],
            len: 0,
            ticks: 0,
            duration_s: 0.0,
            last: [0.0; 3],
            moment_sum: 0.0,
        }
    }

    /// Notes a tick, recording every [`DECIMATION`]th; past [`WINDOW_CAPACITY`] samples,
    /// records nothing.
    fn record(&mut self, dt: f32, rate_radps: f32, last_moment: f32, alpha_rad: f32) {
        if self.len == WINDOW_CAPACITY {
            return;
        }
        for (last, value) in self
            .last
            .iter_mut()
            .zip([rate_radps, last_moment, alpha_rad])
        {
            if value.is_finite() {
                *last = value;
            }
        }
        self.duration_s += dt;
        self.ticks += 1;
        let [rate_radps, moment, alpha_rad] = self.last;
        self.moment_sum += moment;
        if !self.ticks.is_multiple_of(DECIMATION) {
            return;
        }
        self.rates_radps[self.len] = rate_radps;
        self.moments[self.len] = self.moment_sum / DECIMATION as f32;
        self.alphas_rad[self.len] = alpha_rad;
        self.moment_sum = 0.0;
        self.len += 1;
    }

    /// The time between samples, in s.
    fn sample_s(&self) -> f32 {
        self.duration_s / self.ticks.max(1) as f32 * DECIMATION as f32
    }

    /// Whether the angle of attack varied enough to fit the wing's stiffness on.
    fn has_alpha(&self) -> bool {
        let alphas = &self.alphas_rad[..self.len];
        let (low, high) = alphas
            .iter()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(low, high), &alpha| {
                (low.min(alpha), high.max(alpha))
            });
        high - low > MIN_ALPHA_SPREAD_RAD
    }
}

/// One lag and delay's least squares: the terms and the RMS residual of the rate's change, in
/// rad/s².
struct Fit {
    terms: SVector<f64, TERMS>,
    residual: f64,
}

/// The [`WingModel`] fitted to `window` (see [`WingIdentifyProbe`]); `None` when no lag and
/// delay solve.
fn fit(window: &Window) -> Option<WingModel> {
    let sample_s = window.sample_s();
    let with_alpha = window.has_alpha();
    let mut best: Option<(Fit, f32, usize)> = None;
    let mut surface = [0.0f32; WINDOW_CAPACITY];
    for lag_s in LAGS_S {
        lag_through(window, lag_s, sample_s, &mut surface);
        for delay in 0..=MAX_DELAY_SAMPLES {
            let Some(fit) = fit_at(window, &surface, delay, sample_s, with_alpha) else {
                continue;
            };
            if best
                .as_ref()
                .is_none_or(|(kept, ..)| fit.residual < kept.residual)
            {
                best = Some((fit, lag_s, delay));
            }
        }
    }
    let (fit, lag_s, delay) = best?;
    let path_turn_per_s = if with_alpha {
        fit_path_turn(window, sample_s)?
    } else {
        0.0
    };
    Some(WingModel {
        stiffness_per_s2: -fit.terms[0] as f32,
        damping_per_s: -fit.terms[1] as f32,
        control_radps2: fit.terms[2] as f32,
        lag_s,
        delay_s: delay as f32 * sample_s,
        path_turn_per_s,
    })
}

/// The window's moments through a first-order lag of `lag_s` into `surface`, starting at the
/// first moment.
fn lag_through(window: &Window, lag_s: f32, sample_s: f32, surface: &mut [f32; WINDOW_CAPACITY]) {
    let share = sample_s / (sample_s + lag_s);
    let mut lagged = window.moments[0];
    for (out, &moment) in surface.iter_mut().zip(&window.moments[..window.len]) {
        lagged += share * (moment - lagged);
        *out = lagged;
    }
}

/// The least squares of the rate's change per second from sample `k − 1` to `k` on the angle
/// of attack (when `with_alpha`) and the rate between them, `surface` `delay` samples before
/// `k` (the moments that moved the rate), and a constant. `None` when it doesn't solve or the
/// window is too short.
fn fit_at(
    window: &Window,
    surface: &[f32; WINDOW_CAPACITY],
    delay: usize,
    sample_s: f32,
    with_alpha: bool,
) -> Option<Fit> {
    let len = window.len;
    if len < delay + TERMS + 2 {
        return None;
    }
    let between = |values: &[f32; WINDOW_CAPACITY], k: usize| 0.5 * (values[k] + values[k - 1]);
    let regressors = |k: usize| {
        let alpha = if with_alpha {
            between(&window.alphas_rad, k)
        } else {
            0.0
        };
        SVector::<f64, TERMS>::new(
            f64::from(alpha),
            f64::from(between(&window.rates_radps, k)),
            f64::from(surface[k - delay]),
            1.0,
        )
    };
    let accel =
        |k: usize| f64::from((window.rates_radps[k] - window.rates_radps[k - 1]) / sample_s);
    let first = delay + 1;
    let mut normal = SMatrix::<f64, TERMS, TERMS>::zeros();
    let mut moment = SVector::<f64, TERMS>::zeros();
    for k in first..len {
        let x = regressors(k);
        normal += x * x.transpose();
        moment += x * accel(k);
    }
    if !with_alpha {
        // No stiffness term: its row and column are empty, so pin its weight to 0.
        normal[(0, 0)] = 1.0;
    }
    let terms = normal.lu().solve(&moment)?;
    let mut sum_squares = 0.0f64;
    for k in first..len {
        let error = accel(k) - terms.dot(&regressors(k));
        sum_squares += error * error;
    }
    let count = (len - first) as f64;
    terms.iter().all(|term| term.is_finite()).then(|| Fit {
        terms,
        residual: libm::sqrt(sum_squares / count),
    })
}

/// The flight path's turn per rad of angle of attack, in 1/s: the climb angle changes at
/// `rate − α′`, regressed on α with a constant, each between two samples. `None` when α
/// doesn't vary.
fn fit_path_turn(window: &Window, sample_s: f32) -> Option<f32> {
    let (mut sxx, mut sx, mut sxy, mut sy, mut n) = (0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for k in 1..window.len {
        let (alphas, rates) = (&window.alphas_rad, &window.rates_radps);
        let alpha = f64::from(0.5 * (alphas[k] + alphas[k - 1]));
        let alpha_rate = (alphas[k] - alphas[k - 1]) / sample_s;
        let path_rate = f64::from(0.5 * (rates[k] + rates[k - 1]) - alpha_rate);
        sxx += alpha * alpha;
        sx += alpha;
        sxy += alpha * path_rate;
        sy += path_rate;
        n += 1.0;
    }
    let spread = n * sxx - sx * sx;
    (spread > 0.0).then(|| ((n * sxy - sx * sy) / spread) as f32)
}
