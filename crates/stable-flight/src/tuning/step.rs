//! Step-response measurement of a loop: see [`StepProbe`].

use core::f32::consts::PI;

use crate::controllers::{InvalidConfig, check, check_positive};
use crate::math::assert_dt;

#[cfg(test)]
mod tests;

/// A response this far the wrong way (as a fraction of the step), once the first tenth of the
/// window has passed, aborts: the loop pushes away from the setpoint.
const REVERSED: f32 = -0.5;
/// A response this large (as a multiple of the step) aborts: the loop is running away.
const RUNAWAY: f32 = 3.0;
/// The fraction of the window before [`REVERSED`] counts, so a D kick or noise at the start
/// doesn't abort.
const REVERSED_AFTER: f32 = 0.1;
/// Rise time runs between these fractions of the step.
const RISE_FROM: f32 = 0.1;
const RISE_TO: f32 = 0.9;

/// Configures a [`StepProbe`].
///
/// The amplitude is in the loop's setpoint units (rad/s for a rate loop, rad for an angle
/// loop), so there is no `Default`: the caller picks it per loop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepConfig {
    /// Step size and direction, in setpoint units: the setpoint is held at the measurement at
    /// the start plus this. Non-zero and finite.
    pub amplitude: f32,
    /// How long the step is held and measured, in seconds. Positive and finite: long enough to
    /// settle (a few rise times), short enough that a held rate step doesn't tilt too far.
    pub duration_s: f32,
    /// Half-width of the band around the step the response must stay in to count as settled,
    /// as a fraction of the step. In `(0, 1)`.
    pub settle_band: f32,
    /// Cutoff of the first-order low-pass on the measured response, in Hz; 0 turns it off.
    /// Non-negative and finite. It delays the response a little, the same for every step it
    /// measures.
    pub filter_hz: f32,
}

impl StepConfig {
    /// Checks every field's stated requirement.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::autotune::COPTER_RATE_STEP;
    /// use stable_flight::tuning::StepConfig;
    ///
    /// assert!(COPTER_RATE_STEP.validate().is_ok());
    /// let bad = StepConfig { settle_band: 1.5, ..COPTER_RATE_STEP };
    /// assert!(bad.validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check(
            "amplitude",
            self.amplitude != 0.0 && self.amplitude.is_finite(),
            "non-zero and finite",
        )?;
        check_positive("duration_s", self.duration_s)?;
        check(
            "settle_band",
            self.settle_band > 0.0 && self.settle_band < 1.0,
            "in (0, 1)",
        )?;
        check(
            "filter_hz",
            self.filter_hz >= 0.0 && self.filter_hz.is_finite(),
            "non-negative and finite",
        )
    }

    /// This step in the other direction.
    pub fn reversed(self) -> Self {
        Self {
            amplitude: -self.amplitude,
            ..self
        }
    }
}

/// Why a [`StepProbe`] gave up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepAbort {
    /// The response went half a step the wrong way.
    Reversed,
    /// The response passed three times the step.
    Runaway,
}

/// What a step response looked like, normalized to the step: 1 is the step, whatever its
/// size and direction.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StepMetrics {
    /// The step, in setpoint units (signed).
    pub amplitude: f32,
    /// The largest response.
    pub peak: f32,
    /// When it peaked, in seconds from the step.
    pub peak_time_s: f32,
    /// How far the peak went past the step: `peak − 1`, or 0.
    pub overshoot: f32,
    /// Time from 10% to 90% of the step, in seconds; `None` if it never reached 90%.
    pub rise_time_s: Option<f32>,
    /// When the response entered the settle band for good, in seconds from the step; `None`
    /// if it was outside the band at the end of the window.
    pub settling_time_s: Option<f32>,
    /// How far it fell back after the peak: `peak − lowest after it`.
    pub bounce_back: f32,
    /// The response at the end of the window.
    pub final_value: f32,
    /// How long the loop's output (or the loop under it) was clipped during the step, in
    /// seconds: a clipped step measures the limit, not the loop.
    pub saturated_s: f32,
}

/// Holds one loop's setpoint at a step and measures the response, inside the tick.
///
/// On its first tick the probe records the measurement `m₀` and from then on asks for the
/// setpoint `m₀ + amplitude`, for `duration_s`. The response `(m − m₀) / amplitude` (low-passed
/// at `filter_hz`) gives the [`StepMetrics`]. The window is fixed, so steps compare on equal
/// terms. Plain data, no allocation; NaN measurements are skipped.
///
/// # Examples
///
/// ```
/// use stable_flight::tuning::{StepConfig, StepProbe, StepStatus};
///
/// let config = StepConfig { amplitude: 1.0, duration_s: 0.1, settle_band: 0.1, filter_hz: 0.0 };
/// let mut probe = StepProbe::new(config);
/// // A loop that follows its setpoint after one tick.
/// let mut measured = 0.0;
/// while matches!(probe.status(), StepStatus::Running { .. }) {
///     measured = probe.step(0.01, measured, false);
/// }
/// let StepStatus::Done(metrics) = probe.status() else { panic!() };
/// assert_eq!(metrics.overshoot, 0.0);
/// assert_eq!(metrics.final_value, 1.0);
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct StepProbe {
    config: StepConfig,
    /// The filter's time constant, in s (0: no filter).
    filter_tau_s: f32,
    /// The measurement at the step and the setpoint held, once the first tick ran.
    start: Option<(f32, f32)>,
    elapsed_s: f32,
    filtered: f32,
    peak: f32,
    peak_time_s: f32,
    min_after_peak: f32,
    rise_from_s: Option<f32>,
    rise_to_s: Option<f32>,
    in_band_since_s: Option<f32>,
    saturated_s: f32,
    result: Option<Result<StepMetrics, StepAbort>>,
}

/// Where a [`StepProbe`] stands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StepStatus {
    /// Holding the step.
    Running {
        /// Time since the step, in seconds.
        elapsed_s: f32,
    },
    /// The window ended.
    Done(StepMetrics),
    /// The response ran the wrong way or away.
    Aborted(StepAbort),
}

impl StepProbe {
    /// A probe ready to step.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`StepConfig::validate`]).
    pub fn new(config: StepConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid StepConfig: {error}");
        }
        let filter_tau_s = if config.filter_hz > 0.0 {
            1.0 / (2.0 * PI * config.filter_hz)
        } else {
            0.0
        };
        Self {
            config,
            filter_tau_s,
            start: None,
            elapsed_s: 0.0,
            filtered: 0.0,
            peak: 0.0,
            peak_time_s: 0.0,
            min_after_peak: 0.0,
            rise_from_s: None,
            rise_to_s: None,
            in_band_since_s: None,
            saturated_s: 0.0,
            result: None,
        }
    }

    /// Advances the probe by `dt` seconds with the loop's `measurement` and whether its output
    /// was clipped, and returns the setpoint to hold. The first call records the starting
    /// measurement (a NaN one is taken as 0).
    ///
    /// # Panics
    ///
    /// If `dt` is not positive and finite.
    pub fn step(&mut self, dt: f32, measurement: f32, saturated: bool) -> f32 {
        assert_dt(dt);
        let (start, setpoint) = match self.start {
            Some(start) => {
                self.elapsed_s += dt;
                start
            }
            None => {
                let start = if measurement.is_finite() {
                    measurement
                } else {
                    0.0
                };
                let held = (start, start + self.config.amplitude);
                self.start = Some(held);
                held
            }
        };
        if self.result.is_some() {
            return setpoint;
        }
        if saturated {
            self.saturated_s += dt;
        }
        let response = (measurement - start) / self.config.amplitude;
        if response.is_finite() {
            self.track(dt, response);
        }
        if self.elapsed_s + 0.5 * dt >= self.config.duration_s && self.result.is_none() {
            self.result = Some(Ok(self.metrics()));
        }
        setpoint
    }

    /// Filters one response sample and updates the metrics; aborts on a reversed or runaway
    /// response.
    fn track(&mut self, dt: f32, response: f32) {
        // With no filter (τ 0) the gain is 1: the response itself.
        self.filtered += dt / (dt + self.filter_tau_s) * (response - self.filtered);
        let (y, t) = (self.filtered, self.elapsed_s);
        if y > self.peak {
            self.peak = y;
            self.peak_time_s = t;
            self.min_after_peak = y;
        } else if y < self.min_after_peak {
            self.min_after_peak = y;
        }
        if self.rise_from_s.is_none() && y >= RISE_FROM {
            self.rise_from_s = Some(t);
        }
        if self.rise_to_s.is_none() && y >= RISE_TO {
            self.rise_to_s = Some(t);
        }
        let in_band = (y - 1.0).abs() <= self.config.settle_band;
        self.in_band_since_s = match (in_band, self.in_band_since_s) {
            (true, Some(since)) => Some(since),
            (true, None) => Some(t),
            (false, _) => None,
        };
        if y.abs() > RUNAWAY {
            self.result = Some(Err(StepAbort::Runaway));
        } else if y < REVERSED && t > REVERSED_AFTER * self.config.duration_s {
            self.result = Some(Err(StepAbort::Reversed));
        }
    }

    fn metrics(&self) -> StepMetrics {
        let rise_time_s = match (self.rise_from_s, self.rise_to_s) {
            (Some(from), Some(to)) => Some(to - from),
            _ => None,
        };
        StepMetrics {
            amplitude: self.config.amplitude,
            peak: self.peak,
            peak_time_s: self.peak_time_s,
            overshoot: (self.peak - 1.0).max(0.0),
            rise_time_s,
            settling_time_s: self.in_band_since_s,
            bounce_back: self.peak - self.min_after_peak,
            final_value: self.filtered,
            saturated_s: self.saturated_s,
        }
    }

    /// Where the probe stands.
    pub fn status(&self) -> StepStatus {
        match self.result {
            None => StepStatus::Running {
                elapsed_s: self.elapsed_s,
            },
            Some(Ok(metrics)) => StepStatus::Done(metrics),
            Some(Err(abort)) => StepStatus::Aborted(abort),
        }
    }
}

/// Running statistics of one metric over several steps.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Stat {
    /// Samples taken.
    pub count: usize,
    sum: f32,
    /// The smallest sample (0 with none).
    pub min: f32,
    /// The largest sample (0 with none).
    pub max: f32,
}

impl Stat {
    /// Adds a sample.
    pub fn add(&mut self, value: f32) {
        if self.count == 0 {
            self.min = value;
            self.max = value;
        } else {
            self.min = self.min.min(value);
            self.max = self.max.max(value);
        }
        self.count += 1;
        self.sum += value;
    }

    /// The mean, or `None` with no samples.
    pub fn mean(&self) -> Option<f32> {
        (self.count > 0).then(|| self.sum / self.count as f32)
    }
}

/// Statistics over a series of [`StepMetrics`] (e.g. alternating steps of one loop): each
/// metric's mean and range, how many steps never rose or settled, and the overshoot per
/// direction, whose difference shows a trim or CG offset. Plain data, no allocation.
///
/// # Examples
///
/// ```
/// use stable_flight::tuning::{StepMetrics, StepSummary};
///
/// let mut summary = StepSummary::default();
/// summary.add(&StepMetrics { amplitude: 1.0, overshoot: 0.2, ..Default::default() });
/// summary.add(&StepMetrics { amplitude: -1.0, overshoot: 0.1, ..Default::default() });
/// assert_eq!(summary.overshoot.mean(), Some(0.15));
/// assert_eq!(summary.overshoot_positive.mean(), Some(0.2));
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StepSummary {
    /// [`StepMetrics::overshoot`] over every step.
    pub overshoot: Stat,
    /// Overshoot of the steps with a positive amplitude.
    pub overshoot_positive: Stat,
    /// Overshoot of the steps with a negative amplitude.
    pub overshoot_negative: Stat,
    /// [`StepMetrics::rise_time_s`] over the steps that rose.
    pub rise_time_s: Stat,
    /// [`StepMetrics::settling_time_s`] over the steps that settled.
    pub settling_time_s: Stat,
    /// [`StepMetrics::bounce_back`].
    pub bounce_back: Stat,
    /// [`StepMetrics::final_value`].
    pub final_value: Stat,
    /// Steps that never reached 90%.
    pub not_risen: usize,
    /// Steps that ended outside the settle band.
    pub not_settled: usize,
}

impl StepSummary {
    /// Adds one step's metrics.
    pub fn add(&mut self, metrics: &StepMetrics) {
        self.overshoot.add(metrics.overshoot);
        if metrics.amplitude > 0.0 {
            self.overshoot_positive.add(metrics.overshoot);
        } else {
            self.overshoot_negative.add(metrics.overshoot);
        }
        match metrics.rise_time_s {
            Some(rise) => self.rise_time_s.add(rise),
            None => self.not_risen += 1,
        }
        match metrics.settling_time_s {
            Some(settling) => self.settling_time_s.add(settling),
            None => self.not_settled += 1,
        }
        self.bounce_back.add(metrics.bounce_back);
        self.final_value.add(metrics.final_value);
    }

    /// Steps added.
    pub fn count(&self) -> usize {
        self.overshoot.count
    }
}
