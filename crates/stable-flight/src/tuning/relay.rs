//! Relay-feedback measurement of a loop's ultimate gain and period: see [`RelayTuner`].

use core::f32::consts::PI;

use crate::controllers::{InvalidConfig, check, check_limit, check_positive};
use crate::math::{assert_dt, clamp_symmetric};

#[cfg(test)]
mod tests;

/// The most cycles [`RelayConfig::cycles`] may ask for: the window is a fixed-size array.
pub const MAX_CYCLES: usize = 8;

/// Configures a [`RelayTuner`].
///
/// Relay and bias values are in the hosting loop's output units (a normalized moment for a
/// rate loop, rad/s for an attitude loop); hysteresis and cycle amplitudes in its error units
/// (rad/s for a rate loop, rad for an attitude loop). They depend on the loop, so there is no
/// `Default`: the caller picks them per stage.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RelayConfig {
    /// Relay amplitude `h` to start with, in output units. Positive and finite.
    pub initial_relay: f32,
    /// Largest relay amplitude the ramp may reach, in output units. At least `initial_relay`.
    pub max_relay: f32,
    /// Factor the relay amplitude grows by while the cycle is below `min_cycle_amplitude`.
    /// Greater than 1.
    pub relay_step: f32,
    /// Error band `ε` the relay must cross to switch, in error units, so noise doesn't chatter
    /// it. Non-negative and finite; set it above the sensor noise. The band delays each switch,
    /// which biases `Ku` low and `Tu` high by roughly `ε/a`: keep it near a twentieth of
    /// `min_cycle_amplitude` (about 5%) or less.
    pub hysteresis: f32,
    /// Error amplitude a cycle must reach to be measured, in error units; smaller cycles raise
    /// the relay. Greater than `hysteresis`.
    pub min_cycle_amplitude: f32,
    /// Error magnitude that aborts the run, in error units. Greater than
    /// `min_cycle_amplitude`; above the error the run starts from.
    pub max_cycle_amplitude: f32,
    /// Longest the relay may hold one side before it is raised, in seconds: a relay smaller
    /// than the loop's steady offset never switches back. Positive and finite; a few times the
    /// expected half-period.
    pub max_half_cycle_s: f32,
    /// Largest bias magnitude, in output units. Non-negative and finite.
    pub max_bias: f32,
    /// Consecutive consistent cycles needed for a result, in `2..=MAX_CYCLES`.
    pub cycles: usize,
    /// Largest relative deviation from the mean, for the period and the amplitude of the
    /// measured cycles, and of each cycle's time on the positive side from half its period.
    /// In `(0, 1]`.
    pub tolerance: f32,
    /// Time after which a run that hasn't converged aborts, in seconds. Positive and finite.
    pub timeout_s: f32,
}

impl RelayConfig {
    /// Checks every field's stated requirement.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::tuning::RelayConfig;
    ///
    /// let mut config = RelayConfig {
    ///     initial_relay: 0.05,
    ///     max_relay: 0.3,
    ///     relay_step: 1.5,
    ///     hysteresis: 0.002,
    ///     min_cycle_amplitude: 0.05,
    ///     max_cycle_amplitude: 2.0,
    ///     max_half_cycle_s: 1.0,
    ///     max_bias: 0.3,
    ///     cycles: 4,
    ///     tolerance: 0.1,
    ///     timeout_s: 20.0,
    /// };
    /// assert!(config.validate().is_ok());
    /// config.min_cycle_amplitude = 0.001; // inside the hysteresis band
    /// assert!(config.validate().is_err());
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_positive("initial_relay", self.initial_relay)?;
        check(
            "max_relay",
            self.max_relay >= self.initial_relay && self.max_relay.is_finite(),
            "finite and at least initial_relay",
        )?;
        check(
            "relay_step",
            self.relay_step > 1.0 && self.relay_step.is_finite(),
            "finite and greater than 1",
        )?;
        check_limit("hysteresis", self.hysteresis)?;
        check(
            "min_cycle_amplitude",
            self.min_cycle_amplitude > self.hysteresis && self.min_cycle_amplitude.is_finite(),
            "finite and greater than hysteresis",
        )?;
        check(
            "max_cycle_amplitude",
            self.max_cycle_amplitude > self.min_cycle_amplitude
                && self.max_cycle_amplitude.is_finite(),
            "finite and greater than min_cycle_amplitude",
        )?;
        check_positive("max_half_cycle_s", self.max_half_cycle_s)?;
        check_limit("max_bias", self.max_bias)?;
        check(
            "cycles",
            (2..=MAX_CYCLES).contains(&self.cycles),
            "in 2..=MAX_CYCLES",
        )?;
        check(
            "tolerance",
            self.tolerance > 0.0 && self.tolerance <= 1.0,
            "in (0, 1]",
        )?;
        check_positive("timeout_s", self.timeout_s)
    }
}

/// A loop's ultimate point, measured by a [`RelayTuner`]: the input to
/// [`gains_from_ultimate`](super::gains_from_ultimate), plus the run's figures for logging.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UltimatePoint {
    /// Ultimate gain `Ku`: the proportional gain at which the loop would oscillate, in output
    /// units per error unit.
    pub ku: f32,
    /// Ultimate period `Tu`: the oscillation period at `Ku`, in seconds.
    pub tu_s: f32,
    /// Relay amplitude `h` the result was measured at, in output units.
    pub relay: f32,
    /// Mean error amplitude `a` of the measured cycles, in error units.
    pub cycle_amplitude: f32,
    /// Bias at the end of the run, in output units: the steady output the loop needs (trim).
    pub bias: f32,
}

/// Why a [`RelayTuner`] gave up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RelayAbort {
    /// No consistent cycles within [`timeout_s`](RelayConfig::timeout_s).
    Timeout,
    /// The error passed [`max_cycle_amplitude`](RelayConfig::max_cycle_amplitude).
    CycleTooLarge,
}

/// Where a [`RelayTuner`] run stands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RelayStatus {
    /// Still cycling: the current relay amplitude and how many consistent cycles are in hand.
    Running {
        /// Relay amplitude `h`, in output units.
        relay: f32,
        /// Measured cycles in the window.
        cycles: usize,
    },
    /// Measured.
    Done(UltimatePoint),
    /// Gave up.
    Aborted(RelayAbort),
}

/// Which side of the relay is on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Side {
    Positive,
    Negative,
}

/// One measured cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Cycle {
    period_s: f32,
    amplitude: f32,
}

/// The cycle in progress, from one negative-to-positive switch to the next.
#[derive(Clone, Copy, Debug, PartialEq)]
struct OpenCycle {
    start_s: f32,
    positive_s: f32,
    min_error: f32,
    max_error: f32,
}

impl OpenCycle {
    fn new(start_s: f32, error: f32) -> Self {
        Self {
            start_s,
            positive_s: 0.0,
            min_error: error,
            max_error: error,
        }
    }
}

/// Measures a loop's ultimate gain and period by relay feedback (Åström–Hägglund).
///
/// Hosted in place of one loop's controller: each tick, [`step`](Self::step) takes the loop's
/// error and returns the output to apply instead of the controller's. The output is
/// `bias + h` while the error is above the hysteresis band and `bias − h` below it (direct
/// action, like the PID it replaces), so the plant settles into a limit cycle.
///
/// - **Ramp:** `h` starts at `initial_relay` and grows by `relay_step` (up to `max_relay`)
///   while the cycle's error amplitude is below `min_cycle_amplitude`, or when the relay holds
///   one side longer than `max_half_cycle_s` (too small to overcome the loop's offset).
/// - **Bias:** at each cycle's end the bias moves to that cycle's mean output (clamped to
///   `max_bias`), trimming a steady offset such as a CG offset or yaw drag so the two
///   half-cycles even out. Seed it with the loop's current trim (e.g. its I term).
/// - **Measure:** cycles with an even duty (within `tolerance`) fill a window of `cycles`; the
///   first cycle after the start or an `h` change is skipped as transient. When the window's
///   periods and amplitudes agree within `tolerance`, the run is [`Done`](RelayStatus::Done)
///   with `Ku = 4h / (π·√(a² − ε²))` and `Tu` from the window's means.
/// - **Abort:** after `timeout_s`, or when the error passes `max_cycle_amplitude`.
///
/// The describing function reads the error cycle's peak, which overstates its fundamental, so
/// `Ku` reads a few percent low and `Tu` a few percent high (3–12% on rate-axis-like plants at
/// 250 Hz): the conservative direction, giving softer gains.
///
/// A non-finite error holds the relay's side and stays out of the measurement. Once the run
/// is done or aborted, the output is the bias alone; the caller stops hosting the tuner.
///
/// # Examples
///
/// ```
/// use stable_flight::tuning::{RelayConfig, RelayStatus, RelayTuner};
///
/// let config = RelayConfig {
///     initial_relay: 0.05,
///     max_relay: 0.3,
///     relay_step: 1.5,
///     hysteresis: 0.002,
///     min_cycle_amplitude: 0.05,
///     max_cycle_amplitude: 2.0,
///     max_half_cycle_s: 1.0,
///     max_bias: 0.3,
///     cycles: 4,
///     tolerance: 0.1,
///     timeout_s: 20.0,
/// };
/// let mut tuner = RelayTuner::new(config, 0.02); // seeded with the loop's trim
///
/// assert_eq!(tuner.step(0.004, 0.5), 0.02 + 0.05); // error above the band: bias + h
/// assert_eq!(tuner.step(0.004, 0.001), 0.02 + 0.05); // inside the band: holds its side
/// assert_eq!(tuner.step(0.004, -0.5), 0.02 - 0.05); // below the band: bias − h
/// assert!(matches!(tuner.status(), RelayStatus::Running { .. }));
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct RelayTuner {
    config: RelayConfig,
    relay: f32,
    bias: f32,
    /// None until the first finite error.
    side: Option<Side>,
    elapsed_s: f32,
    /// When the relay last changed side or amplitude.
    side_since_s: f32,
    /// None until the first negative-to-positive switch.
    open: Option<OpenCycle>,
    /// The next closed cycle is transient (after the start or a relay change): skip it.
    skip_next: bool,
    window: [Cycle; MAX_CYCLES],
    measured: usize,
    result: Option<Result<UltimatePoint, RelayAbort>>,
}

impl RelayTuner {
    /// A tuner ready to run, with `initial_bias` (output units) as the starting trim, clamped
    /// to `max_bias`.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`RelayConfig::validate`]).
    pub fn new(config: RelayConfig, initial_bias: f32) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid RelayConfig: {error}");
        }
        let initial_bias = if initial_bias.is_finite() {
            initial_bias
        } else {
            0.0
        };
        Self {
            config,
            relay: config.initial_relay,
            bias: clamp_symmetric(initial_bias, config.max_bias),
            side: None,
            elapsed_s: 0.0,
            side_since_s: 0.0,
            open: None,
            skip_next: true,
            window: [Cycle::default(); MAX_CYCLES],
            measured: 0,
            result: None,
        }
    }

    /// Advances the run by `dt` seconds with the loop's `error` (setpoint − measurement) and
    /// returns the output to apply in place of the loop's controller.
    ///
    /// # Panics
    ///
    /// If `dt` is not positive and finite.
    pub fn step(&mut self, dt: f32, error: f32) -> f32 {
        assert_dt(dt);
        if self.result.is_some() {
            return self.bias;
        }
        self.elapsed_s += dt;

        if error.is_finite() {
            self.switch(error);
            if error.abs() > self.config.max_cycle_amplitude {
                self.result = Some(Err(RelayAbort::CycleTooLarge));
                return self.bias;
            }
        }
        if self.elapsed_s - self.side_since_s > self.config.max_half_cycle_s {
            // Stuck on one side: the relay can't overcome the loop's offset.
            self.raise_relay();
        }

        if let Some(open) = &mut self.open {
            if self.side == Some(Side::Positive) {
                open.positive_s += dt;
            }
            if error.is_finite() {
                open.min_error = open.min_error.min(error);
                open.max_error = open.max_error.max(error);
            }
        }

        if self.result.is_none() && self.elapsed_s > self.config.timeout_s {
            self.result = Some(Err(RelayAbort::Timeout));
        }
        if self.result.is_some() {
            return self.bias;
        }

        match self.side {
            Some(Side::Positive) => self.bias + self.relay,
            Some(Side::Negative) => self.bias - self.relay,
            None => self.bias,
        }
    }

    /// The run's status.
    pub fn status(&self) -> RelayStatus {
        match self.result {
            Some(Ok(point)) => RelayStatus::Done(point),
            Some(Err(abort)) => RelayStatus::Aborted(abort),
            None => RelayStatus::Running {
                relay: self.relay,
                cycles: self.measured,
            },
        }
    }

    /// Updates the relay's side for a finite `error`; a negative-to-positive switch closes the
    /// open cycle and starts the next.
    fn switch(&mut self, error: f32) {
        let band = self.config.hysteresis;
        let side = match self.side {
            None if error >= 0.0 => Side::Positive,
            None => Side::Negative,
            Some(Side::Negative) if error > band => Side::Positive,
            Some(Side::Positive) if error < -band => Side::Negative,
            Some(side) => side,
        };
        let rising = self.side == Some(Side::Negative) && side == Side::Positive;
        if self.side != Some(side) {
            self.side_since_s = self.elapsed_s;
        }
        self.side = Some(side);
        if !rising {
            return;
        }
        if let Some(open) = self.open {
            self.close(open);
        }
        self.open = Some(OpenCycle::new(self.elapsed_s, error));
    }

    /// Closes a cycle: updates the bias, then ramps the relay or measures the cycle.
    fn close(&mut self, open: OpenCycle) {
        let period_s = self.elapsed_s - open.start_s;
        let negative_s = period_s - open.positive_s;
        let amplitude = 0.5 * (open.max_error - open.min_error);

        // The cycle's mean output: the steady output the loop needs.
        let mean_offset = self.relay * (open.positive_s - negative_s) / period_s;
        self.bias = clamp_symmetric(self.bias + mean_offset, self.config.max_bias);

        if amplitude < self.config.min_cycle_amplitude && self.relay < self.config.max_relay {
            self.raise_relay();
            return;
        }
        if core::mem::take(&mut self.skip_next) {
            return;
        }
        let duty_error = (open.positive_s - 0.5 * period_s).abs() / (0.5 * period_s);
        if duty_error > self.config.tolerance {
            self.measured = 0;
            return;
        }
        self.push(Cycle {
            period_s,
            amplitude,
        });
    }

    /// Adds a measured cycle; finishes when the window is full and consistent.
    fn push(&mut self, cycle: Cycle) {
        let n = self.config.cycles;
        if self.measured == n {
            self.window.copy_within(1..n, 0);
            self.measured -= 1;
        }
        self.window[self.measured] = cycle;
        self.measured += 1;
        if self.measured < n {
            return;
        }

        let cycles = &self.window[..n];
        let mean_period_s = cycles.iter().map(|c| c.period_s).sum::<f32>() / n as f32;
        let mean_amplitude = cycles.iter().map(|c| c.amplitude).sum::<f32>() / n as f32;
        let tolerance = self.config.tolerance;
        let consistent = cycles.iter().all(|c| {
            (c.period_s - mean_period_s).abs() <= tolerance * mean_period_s
                && (c.amplitude - mean_amplitude).abs() <= tolerance * mean_amplitude
        });
        if !consistent {
            return;
        }

        let Some(ku) = ultimate_gain(self.relay, mean_amplitude, self.config.hysteresis) else {
            return;
        };
        self.result = Some(Ok(UltimatePoint {
            ku,
            tu_s: mean_period_s,
            relay: self.relay,
            cycle_amplitude: mean_amplitude,
            bias: self.bias,
        }));
    }

    /// Raises the relay by `relay_step` (up to `max_relay`), drops the measured cycles, and
    /// skips the next one as transient.
    fn raise_relay(&mut self) {
        self.relay = (self.relay * self.config.relay_step).min(self.config.max_relay);
        self.side_since_s = self.elapsed_s;
        self.measured = 0;
        self.skip_next = true;
    }
}

/// `Ku = 4h / (π·√(a² − ε²))`: the describing function of a relay of amplitude `relay` with
/// hysteresis `hysteresis`, for an error cycle of amplitude `amplitude`. None unless the cycle
/// clears the band (`a > ε`).
fn ultimate_gain(relay: f32, amplitude: f32, hysteresis: f32) -> Option<f32> {
    let root = libm::sqrtf(amplitude * amplitude - hysteresis * hysteresis);
    (root > 0.0).then(|| 4.0 * relay / (PI * root))
}
