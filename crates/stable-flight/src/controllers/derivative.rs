//! Derivative-term history shared by the PID controllers.

use core::ops::{Div, Sub};

/// The previous tick's error, for a PID loop's `D·Δe/dt` term.
///
/// Starts empty, so the first tick after construction has no derivative kick (there is no
/// previous error to difference against). [`forget`](Self::forget) empties it again after a
/// non-finite tick, so the first finite tick after a gap has no kick from a stale error.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct DerivativeState<T> {
    previous_error: Option<T>,
}

// `T::default()` stands in for a zero derivative, so it must be the additive zero (true for
// `f32` and nalgebra vectors).
impl<T> DerivativeState<T>
where
    T: Copy + Default + Sub<Output = T> + Div<f32, Output = T>,
{
    /// `(error - previous) / dt`, or zero with no history; records `error` for the next tick.
    /// `error` must be finite: call [`forget`](Self::forget) instead on a non-finite tick.
    pub(crate) fn step(&mut self, error: T, dt: f32) -> T {
        let derivative = match self.previous_error {
            Some(previous) => (error - previous) / dt,
            None => T::default(),
        };
        self.previous_error = Some(error);
        derivative
    }

    /// Drops the history: the next [`step`](Self::step) has no derivative term.
    pub(crate) fn forget(&mut self) {
        self.previous_error = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vec2;

    #[test]
    fn differences_consecutive_errors_after_an_empty_first_tick() {
        let mut state = DerivativeState::default();
        // (error, expected derivative) at dt = 0.5
        let cases = [(2.0, 0.0), (3.0, 2.0), (3.0, 0.0), (1.0, -4.0)];
        for (error, expected) in cases {
            assert!(state.step(error, 0.5) == expected, "error {error}");
        }
    }

    #[test]
    fn forget_drops_the_history() {
        let mut state = DerivativeState::default();
        state.step(Vec2::new(1.0, -1.0), 0.1);
        state.forget();
        assert!(state.step(Vec2::new(5.0, 5.0), 0.1) == Vec2::zeros());
        assert!(state.step(Vec2::new(6.0, 4.0), 0.5) == Vec2::new(2.0, -2.0));
    }
}
