//! Gains for a wing axis from its fitted model: see [`WingDesign`].

use super::WingModel;
use crate::controllers::{InvalidConfig, RateGains, check, check_positive};
use crate::stabilizers::plane::PlaneAxisGains;
use crate::tuning::{StepConfig, StepMetrics, StepProbe, StepStatus};

#[cfg(test)]
mod tests;

/// The tick the model steps at, in s: the flight loop's 250 Hz.
const MODEL_DT: f32 = 0.004;

/// The rate P values the design tries: 16 from 0.005 to 10, evenly spaced in ratio (×1.66),
/// spanning the sim's planes' 0.02–2.5 (#533).
const P_GRID: [f32; 16] = [
    0.005, 0.0083, 0.0138, 0.0229, 0.0381, 0.0632, 0.105, 0.175, 0.29, 0.482, 0.8, 1.33, 2.21,
    3.67, 6.09, 10.0,
];

/// The rate I values the design tries: none, then 11 from 0.01 to 10 (×2), spanning the sim's
/// planes' 0.04–6.5 (#533).
const I_GRID: [f32; 12] = [
    0.0, 0.01, 0.02, 0.04, 0.08, 0.16, 0.32, 0.63, 1.26, 2.51, 5.01, 10.0,
];

const CELLS: usize = P_GRID.len() * I_GRID.len();

/// The bisection stops once the passing time constant is within this ratio of the failing
/// one.
const TAU_RATIO: f32 = 1.05;

/// What the design holds a step on the model to, and the limits the model flies under.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WingDesignConfig {
    /// The attitude step, in rad, held for `step_duration_s`.
    pub amplitude_rad: f32,
    /// How long the step is held, in s.
    pub step_duration_s: f32,
    /// The attitude loop's rate limit, in rad/s.
    pub max_rate_radps: f32,
    /// The moment the rate loop may command, normalized: full deflection less the trim and a
    /// margin.
    pub max_moment: f32,
    /// The rate loop's integral limit, in rad of accumulated rate error.
    pub max_integral: f32,
    /// The trim the rate loop adds without its integral, normalized
    /// ([`PlaneRateControllerConfig::trim`](crate::controllers::PlaneRateControllerConfig)):
    /// the integral holds the rest of the model's [`trim_moment`](WingModel::trim_moment).
    pub static_trim: f32,
    /// Largest overshoot, as a fraction of the step.
    pub max_overshoot: f32,
    /// Longest the moment may clip, in s.
    pub max_saturated_s: f32,
    /// The time constants the design may pick, in s, from the least to the most.
    pub time_constant_range_s: [f32; 2],
    /// The 10–90% rise allowed, in time constants: a first-order response rises in 2.2.
    pub rise_per_time_constant: f32,
}

impl WingDesignConfig {
    /// Checks that every field is positive and finite (`max_overshoot` and `max_saturated_s`
    /// may be 0, `max_moment` at most 1) and the time constant range is in order.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::WingDesignConfig;
    ///
    /// let config = WingDesignConfig {
    ///     amplitude_rad: 0.17,
    ///     step_duration_s: 4.0,
    ///     max_rate_radps: 1.0,
    ///     max_moment: 0.8,
    ///     max_integral: 2.0,
    ///     static_trim: 0.0,
    ///     max_overshoot: 0.05,
    ///     max_saturated_s: 0.042,
    ///     time_constant_range_s: [0.2, 1.5],
    ///     rise_per_time_constant: 2.2,
    /// };
    /// assert!(config.validate().is_ok());
    /// assert!(WingDesignConfig { max_moment: 1.5, ..config }.validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_positive("amplitude_rad", self.amplitude_rad)?;
        check_positive("step_duration_s", self.step_duration_s)?;
        check_positive("max_rate_radps", self.max_rate_radps)?;
        check(
            "max_moment",
            self.max_moment > 0.0 && self.max_moment <= 1.0,
            "in (0, 1]",
        )?;
        check_positive("max_integral", self.max_integral)?;
        check("static_trim", self.static_trim.abs() <= 1.0, "in [-1, 1]")?;
        check(
            "max_overshoot",
            self.max_overshoot >= 0.0 && self.max_overshoot.is_finite(),
            "non-negative and finite",
        )?;
        check(
            "max_saturated_s",
            self.max_saturated_s >= 0.0 && self.max_saturated_s.is_finite(),
            "non-negative and finite",
        )?;
        let [least, most] = self.time_constant_range_s;
        check_positive("time_constant_range_s[0]", least)?;
        check(
            "time_constant_range_s[1]",
            most > least && most.is_finite(),
            "finite and above time_constant_range_s[0]",
        )?;
        check_positive("rise_per_time_constant", self.rise_per_time_constant)
    }
}

/// Where a [`WingDesign`] stands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WingDesignStatus {
    /// Still stepping candidates on the model.
    Running,
    /// The designed gains.
    Done(PlaneAxisGains),
    /// The model can't be designed for: its control or damping isn't positive, or no gains
    /// meet the targets at the slowest time constant.
    Failed,
}

/// Designs one wing axis's rate gains and attitude time constant on its fitted
/// [`WingModel`], by stepping the attitude loop on the model rather than in flight.
///
/// The feedforward is the model's: the moment that holds a rate against the damping. On a
/// stiff axis (stiffness above 0, a wing's pitch) a damping fitted at or below 0 is read as
/// none, so no feedforward: the angle of attack follows the rate, and the fit splits the
/// damping loosely between the two. For P, I,
/// and the time constant τ, each candidate flies an attitude step on the model through the
/// plane's loops (the rate target the error over τ within the rate limit, the moment its P, I,
/// and feedforward terms within the limit) and passes when the step overshoots by no more than
/// the allowance, settles, rises within `rise_per_time_constant` × τ, and clips for no longer
/// than allowed: the plane spec's targets. τ is bisected to the quickest at which some P and I
/// on a coarse grid pass (16 × 12), then raised by 15% for margin; at that τ the design takes
/// the passing P and I farthest from any failing one on the grid, the middle of what works,
/// so a model a little off still flies. An I too small to hold the model's trim (beyond the
/// static trim) within half the integral limit never passes: the integral would clip holding
/// it. The airplane's pitch designed at I 0.25 against a 2 rad limit sat 1.75° off its held
/// attitude, slowed 1.75 m/s, and its stage timed out settling (#534).
///
/// Each candidate is a step of up to `step_duration_s` on the model at 250 Hz, a few thousand
/// multiplies; a design takes up to about 1,350. [`advance`](Self::advance) runs a given number
/// of candidates, so a host spreads the design over its updates.
///
/// # Examples
///
/// ```
/// use stable_flight::tuning::{WingDesign, WingDesignConfig, WingDesignStatus, WingModel};
///
/// let model = WingModel {
///     stiffness_per_s2: 0.0,
///     damping_per_s: 8.0,
///     control_radps2: 40.0,
///     lag_s: 0.02,
///     delay_s: 0.0,
///     path_turn_per_s: 0.0,
///     trim_moment: 0.0,
/// };
/// let config = WingDesignConfig {
///     amplitude_rad: 0.5,
///     step_duration_s: 4.0,
///     max_rate_radps: 1.2,
///     max_moment: 0.8,
///     max_integral: 2.0,
///     static_trim: 0.0,
///     max_overshoot: 0.05,
///     max_saturated_s: 0.042,
///     time_constant_range_s: [0.2, 1.5],
///     rise_per_time_constant: 2.2,
/// };
/// let mut design = WingDesign::new(model, config);
/// while design.advance(100) == WingDesignStatus::Running {}
/// let WingDesignStatus::Done(gains) = design.advance(1) else { panic!("no design") };
/// assert_eq!(gains.rate.ff, model.feedforward());
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WingDesign {
    model: WingModel,
    config: WingDesignConfig,
    /// The least I that holds the model's trim within [`TRIM_INTEGRAL_SHARE`] of the limit.
    min_i: f32,
    phase: Phase,
}

/// The share of the integral limit the trim may take: the rest answers steps.
const TRIM_INTEGRAL_SHARE: f32 = 0.5;

#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    /// Searching the grid at `tau_s` for a passing cell, from `cell`; `passing_tau_s` is the
    /// quickest τ found to pass so far, `failing_tau_s` the slowest found to fail.
    Bisect {
        tau_s: f32,
        cell: usize,
        passing_tau_s: Option<f32>,
        failing_tau_s: f32,
    },
    /// Mapping which cells pass at the design's `tau_s`, up to `cell`.
    Map {
        tau_s: f32,
        cell: usize,
        passes: [bool; CELLS],
    },
    Done(WingDesignStatus),
}

impl WingDesign {
    /// A design of `model`'s axis to `config`'s targets, ready to [`advance`](Self::advance).
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`WingDesignConfig::validate`]).
    pub fn new(model: WingModel, config: WingDesignConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid WingDesignConfig: {error}");
        }
        // A stiff axis flies on its stiffness whatever the damping: the eXplora's pitch fits
        // 0.89/s of damping from its own gains and −0.72/s from three times them, its
        // stiffness 62/s² and 96/s² (#618).
        let stiff = model.stiffness_per_s2 > 0.0;
        let usable = model.control_radps2 > 0.0
            && (model.damping_per_s > 0.0 || stiff)
            && model.feedforward().is_finite();
        let model = WingModel {
            damping_per_s: model.damping_per_s.max(0.0),
            ..model
        };
        let [least_s, most_s] = config.time_constant_range_s;
        let phase = if usable {
            // The slowest first: if nothing passes there, nothing will.
            Phase::Bisect {
                tau_s: most_s,
                cell: 0,
                passing_tau_s: None,
                failing_tau_s: least_s / TAU_RATIO,
            }
        } else {
            Phase::Done(WingDesignStatus::Failed)
        };
        let integral_trim = (model.trim_moment - config.static_trim).abs();
        Self {
            model,
            config,
            min_i: integral_trim / (TRIM_INTEGRAL_SHARE * config.max_integral),
            phase,
        }
    }

    /// Steps up to `candidates` candidates on the model, and returns where the design stands.
    pub fn advance(&mut self, candidates: usize) -> WingDesignStatus {
        for _ in 0..candidates {
            if let Phase::Done(status) = self.phase {
                return status;
            }
            self.phase = self.next_phase();
        }
        match self.phase {
            Phase::Done(status) => status,
            _ => WingDesignStatus::Running,
        }
    }

    /// The phase after stepping one candidate.
    fn next_phase(&self) -> Phase {
        match self.phase {
            Phase::Bisect {
                tau_s,
                cell,
                passing_tau_s,
                failing_tau_s,
            } => {
                if self.passes(cell, tau_s) {
                    self.bisected(failing_tau_s, Some(tau_s))
                } else if cell + 1 < CELLS {
                    Phase::Bisect {
                        tau_s,
                        cell: cell + 1,
                        passing_tau_s,
                        failing_tau_s,
                    }
                } else if passing_tau_s.is_none() {
                    // Nothing passes at the slowest time constant.
                    Phase::Done(WingDesignStatus::Failed)
                } else {
                    self.bisected(tau_s, passing_tau_s)
                }
            }
            Phase::Map {
                tau_s,
                cell,
                mut passes,
            } => {
                passes[cell] = self.passes(cell, tau_s);
                if cell + 1 < CELLS {
                    Phase::Map {
                        tau_s,
                        cell: cell + 1,
                        passes,
                    }
                } else {
                    Phase::Done(self.pick(tau_s, &passes))
                }
            }
            Phase::Done(status) => Phase::Done(status),
        }
    }

    /// The next τ to search between `failing_tau_s` and `passing_tau_s` (geometric middle), or
    /// the map at the passing one with margin once they're close.
    fn bisected(&self, failing_tau_s: f32, passing_tau_s: Option<f32>) -> Phase {
        let passing_tau_s = passing_tau_s.unwrap_or_else(|| unreachable!("bisected after a pass"));
        if passing_tau_s <= failing_tau_s * TAU_RATIO {
            let most_s = self.config.time_constant_range_s[1];
            return Phase::Map {
                tau_s: (passing_tau_s * TAU_MARGIN).min(most_s),
                cell: 0,
                passes: [false; CELLS],
            };
        }
        Phase::Bisect {
            tau_s: libm::sqrtf(failing_tau_s * passing_tau_s),
            cell: 0,
            passing_tau_s: Some(passing_tau_s),
            failing_tau_s,
        }
    }

    /// The gains at the passing cell farthest, in grid steps along P and I, from a failing
    /// one or the grid's edge.
    fn pick(&self, tau_s: f32, passes: &[bool; CELLS]) -> WingDesignStatus {
        let passing = |p: isize, i: isize| {
            let inside =
                (0..P_GRID.len() as isize).contains(&p) && (0..I_GRID.len() as isize).contains(&i);
            inside && passes[p as usize * I_GRID.len() + i as usize]
        };
        let depth = |p: isize, i: isize| {
            (1..)
                .take_while(|&d| {
                    [(d, 0), (-d, 0), (0, d), (0, -d)]
                        .iter()
                        .all(|&(dp, di)| passing(p + dp, i + di))
                })
                .count()
        };
        let best = (0..CELLS)
            .filter(|&cell| passes[cell])
            .map(|cell| {
                let (p, i) = (
                    (cell / I_GRID.len()) as isize,
                    (cell % I_GRID.len()) as isize,
                );
                (cell, depth(p, i))
            })
            // The first deepest: `max_by_key` keeps the last of a tie.
            .fold(
                None,
                |best: Option<(usize, usize)>, (cell, depth)| match best {
                    Some((_, kept)) if kept >= depth => best,
                    _ => Some((cell, depth)),
                },
            );
        match best {
            Some((cell, _)) => WingDesignStatus::Done(self.gains(cell, tau_s)),
            None => WingDesignStatus::Failed,
        }
    }

    /// The axis gains at grid `cell` and `tau_s`.
    fn gains(&self, cell: usize, tau_s: f32) -> PlaneAxisGains {
        PlaneAxisGains {
            rate: RateGains {
                p: P_GRID[cell / I_GRID.len()],
                i: I_GRID[cell % I_GRID.len()],
                ff: self.model.feedforward(),
            },
            time_constant_s: tau_s,
        }
    }

    /// Whether the step on the model at grid `cell` and `tau_s` meets the targets.
    fn passes(&self, cell: usize, tau_s: f32) -> bool {
        let gains = self.gains(cell, tau_s);
        if gains.rate.i < self.min_i {
            return false;
        }
        model_step(&self.model, gains, &self.config)
            .is_some_and(|metrics| meets_targets(&metrics, tau_s, &self.config))
    }
}

/// The margin on the quickest passing time constant: a model a little off still settles.
const TAU_MARGIN: f32 = 1.15;

/// Whether `metrics` of a step at time constant `tau_s` meet `config`'s targets: overshoot
/// within the allowance, settled, risen within the allowed time constants, clipped no longer
/// than allowed.
pub(crate) fn meets_targets(metrics: &StepMetrics, tau_s: f32, config: &WingDesignConfig) -> bool {
    let max_rise_s = config.rise_per_time_constant * tau_s;
    metrics.overshoot <= config.max_overshoot
        && metrics.settling_time_s.is_some()
        && metrics.saturated_s <= config.max_saturated_s
        && metrics
            .rise_time_s
            .is_some_and(|rise_s| rise_s <= max_rise_s)
}

/// An attitude step of `config`'s amplitude on `model`, flown by the plane's loops at `gains`
/// from level and still, measured by a [`StepProbe`] as a flight step is; `None` when the step
/// aborts or overshoots past the allowance (no need to fly it out).
fn model_step(
    model: &WingModel,
    gains: PlaneAxisGains,
    config: &WingDesignConfig,
) -> Option<StepMetrics> {
    let RateGains { p, i, ff } = gains.rate;
    let surface_share = MODEL_DT / (MODEL_DT + model.lag_s);
    let delay_ticks = libm::roundf(model.delay_s / MODEL_DT) as usize;
    let mut probe = StepProbe::new(StepConfig {
        amplitude: config.amplitude_rad,
        duration_s: config.step_duration_s,
        settle_band: 0.1,
        filter_hz: 0.0,
    });
    // Moments on their way to the surfaces, oldest first, as a ring.
    let mut queue = [0.0f32; MAX_MODEL_DELAY_TICKS];
    let delay_ticks = delay_ticks.min(MAX_MODEL_DELAY_TICKS);
    let (mut angle_rad, mut rate_radps, mut alpha_rad) = (0.0f32, 0.0f32, 0.0f32);
    let (mut surface, mut integral, mut clipped) = (0.0f32, 0.0f32, false);
    let peak_limit = 1.0 + config.max_overshoot;
    let mut tick = 0usize;
    loop {
        let target_rad = probe.step(MODEL_DT, angle_rad, clipped);
        match probe.status() {
            StepStatus::Running { .. } => {}
            StepStatus::Done(metrics) => return Some(metrics),
            StepStatus::Aborted(_) => return None,
        }
        if angle_rad > peak_limit * config.amplitude_rad {
            return None;
        }
        let rate_target_radps = ((target_rad - angle_rad) / gains.time_constant_s)
            .clamp(-config.max_rate_radps, config.max_rate_radps);
        let error_radps = rate_target_radps - rate_radps;
        integral =
            (integral + error_radps * MODEL_DT).clamp(-config.max_integral, config.max_integral);
        let raw = p * error_radps + i * integral + ff * rate_target_radps;
        let moment = raw.clamp(-config.max_moment, config.max_moment);
        clipped = raw.abs() > config.max_moment;
        let felt = if delay_ticks == 0 {
            moment
        } else {
            let slot = tick % delay_ticks;
            core::mem::replace(&mut queue[slot], moment)
        };
        surface += surface_share * (felt - surface);
        let accel_radps2 = -model.stiffness_per_s2 * alpha_rad - model.damping_per_s * rate_radps
            + model.control_radps2 * surface;
        alpha_rad += (rate_radps - model.path_turn_per_s * alpha_rad) * MODEL_DT;
        angle_rad += rate_radps * MODEL_DT;
        rate_radps += accel_radps2 * MODEL_DT;
        tick += 1;
    }
}

/// The longest delay the model flies, in ticks: 100 ms, past the fit's 40 ms.
const MAX_MODEL_DELAY_TICKS: usize = 25;
