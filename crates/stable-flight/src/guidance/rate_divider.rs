//! Paces a slow loop from a faster tick.

#[cfg(test)]
mod tests;

/// Decides on which ticks a slow loop runs and over how much time it steps.
///
/// The loop is due on its first tick, then on the tick nearest each `period` (half a tick of
/// slack, which also absorbs f32 accumulation: 8 × 2.5 ms sums to 19.999998 ms). A run steps
/// over the whole time since the last one, so integrals see real time even when the period
/// isn't a whole number of ticks. Allocation-free.
///
/// The period is passed to each [`due`](Self::due) call rather than stored, so the owner's
/// config stays the single source of truth and a new period applies on the next tick.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct RateDivider {
    /// Seconds since the loop last ran; `None` before its first run (or after a reset).
    elapsed: Option<f32>,
}

impl RateDivider {
    /// Advances by one tick of `dt` seconds for a loop that runs every `period` seconds
    /// (`0.0`: every tick; the caller validates it as non-negative and finite). Returns the
    /// time the loop should step over if it is due; call [`ran`](Self::ran) once it has.
    ///
    /// A due loop that doesn't run (e.g. its input was non-finite) stays due, so the next tick
    /// retries over the combined time.
    pub(crate) fn due(&mut self, dt: f32, period: f32) -> Option<f32> {
        let Some(elapsed) = self.elapsed.as_mut() else {
            return Some(dt);
        };
        *elapsed += dt;
        (*elapsed >= period - 0.5 * dt).then_some(*elapsed)
    }

    /// Records that the loop ran this tick.
    pub(crate) fn ran(&mut self) {
        self.elapsed = Some(0.0);
    }

    /// Forgets the last run, so the loop is due on the next tick (e.g. on entering a mode).
    pub(crate) fn reset(&mut self) {
        self.elapsed = None;
    }
}
