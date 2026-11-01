//! The state two-pole filters share: see [`Biquad`].

/// A two-pole filter's weights, computed from its settings and tick; each filter keeps its own
/// arithmetic, so a shared state changes no filter's output.
pub(crate) trait Section: Copy {
    /// What the weights are computed from (a cutoff, a notch's center and width).
    type Settings: Copy + PartialEq;

    /// The weights for `settings` at ticks of `dt_s`; `None` when the filter passes the signal
    /// through.
    fn new(settings: &Self::Settings, dt_s: f32) -> Option<Self>;

    /// The output for input `value` after the last two `inputs` and `outputs`, newest first.
    fn step(&self, value: f32, inputs: [f32; 2], outputs: [f32; 2]) -> f32;
}

/// A two-pole filter's state, direct form I: its last two inputs and outputs, and its weights
/// cached for the settings and tick they were computed for. The first sample fills the
/// history, so the output starts at the input instead of ramping up from 0; a non-finite
/// sample passes through and leaves the history as it was; while the weights pass the signal
/// through, the history empties.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Biquad<S: Section> {
    weights: Option<(S, S::Settings, f32)>,
    history: Option<([f32; 2], [f32; 2])>,
}

impl<S: Section> Default for Biquad<S> {
    fn default() -> Self {
        Self {
            weights: None,
            history: None,
        }
    }
}

impl<S: Section> Biquad<S> {
    /// Filters `value`, sampled `dt` s after the last, with `settings`. The weights are
    /// recomputed only when `settings` or `dt` changes.
    pub(crate) fn apply(&mut self, value: f32, dt: f32, settings: &S::Settings) -> f32 {
        if !value.is_finite() {
            return value;
        }
        let Some(weights) = self.weights(dt, settings) else {
            self.history = None;
            return value;
        };
        let (inputs, outputs) = *self.history.get_or_insert(([value; 2], [value; 2]));
        let output = weights.step(value, inputs, outputs);
        self.history = Some(([value, inputs[0]], [output, outputs[0]]));
        output
    }

    fn weights(&mut self, dt: f32, settings: &S::Settings) -> Option<S> {
        match self.weights {
            Some((weights, cached, cached_dt)) if cached == *settings && cached_dt == dt => {
                Some(weights)
            }
            _ => {
                let weights = S::new(settings, dt)?;
                self.weights = Some((weights, *settings, dt));
                Some(weights)
            }
        }
    }
}
