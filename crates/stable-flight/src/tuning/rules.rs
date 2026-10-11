//! Tuning rules: [`UltimatePoint`] → [`Gains`].

use crate::controllers::Gains;

use super::UltimatePoint;

#[cfg(test)]
mod tests;

/// A tuning rule: the proportional gain, integral time, and derivative time as fractions of the
/// ultimate gain `Ku` and period `Tu`.
///
/// Both rules are conservative variants of Ziegler–Nichols, which aims for quarter-amplitude
/// decay and overshoots too much for a multirotor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TuningRule {
    /// Tyreus–Luyben: PID `Kp = Ku/2.2`, `Ti = 2.2·Tu`, `Td = Tu/6.3`; PI `Kp = Ku/3.2`,
    /// `Ti = 2.2·Tu`. More damping and a slower integral than Ziegler–Nichols.
    TyreusLuyben,
    /// Ziegler–Nichols "no overshoot": `Kp = 0.2·Ku`, `Ti = Tu/2`, `Td = Tu/3` (PI and P keep
    /// `Kp`). The softest of the two.
    NoOvershootZn,
}

/// Which terms the tuned controller uses. [`gains_from_ultimate`] zeroes a dropped term; the
/// autotune supervisor leaves it at its current gain instead.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControllerForm {
    /// Proportional, integral, and derivative (rate loops).
    Pid,
    /// Proportional and integral: the rule's PI row.
    Pi,
    /// Proportional and derivative (attitude loops): the PID row without the integral.
    Pd,
    /// Proportional only: the PI row's `Kp`.
    P,
}

impl ControllerForm {
    /// Whether the form tunes the integral term.
    pub fn has_integral(self) -> bool {
        matches!(self, Self::Pid | Self::Pi)
    }

    /// Whether the form tunes the derivative term.
    pub fn has_derivative(self) -> bool {
        matches!(self, Self::Pid | Self::Pd)
    }
}

/// Gains for a loop from its measured [`UltimatePoint`], by `rule`, in the parallel form
/// [`Gains`] uses: `p = Kp`, `i = Kp/Ti`, `d = Kp·Td`.
///
/// # Examples
///
/// ```
/// use stable_flight::tuning::{ControllerForm, TuningRule, UltimatePoint, gains_from_ultimate};
///
/// let point = UltimatePoint { ku: 2.0, tu_s: 0.5 };
/// let gains = gains_from_ultimate(&point, TuningRule::NoOvershootZn, ControllerForm::Pd);
/// assert!((gains.p - 0.4).abs() < 1e-6); // 0.2·Ku
/// assert_eq!(gains.i, 0.0); // PD drops the integral
/// assert!((gains.d - 0.4 * 0.5 / 3.0).abs() < 1e-6); // Kp·Tu/3
/// ```
pub fn gains_from_ultimate(point: &UltimatePoint, rule: TuningRule, form: ControllerForm) -> Gains {
    let (ku, tu_s) = (point.ku, point.tu_s);
    // (Kp, Ti, Td) for the PID and PI rows.
    let (pid, pi) = match rule {
        TuningRule::TyreusLuyben => (
            (ku / 2.2, 2.2 * tu_s, tu_s / 6.3),
            (ku / 3.2, 2.2 * tu_s, 0.0),
        ),
        TuningRule::NoOvershootZn => {
            let kp = 0.2 * ku;
            ((kp, tu_s / 2.0, tu_s / 3.0), (kp, tu_s / 2.0, 0.0))
        }
    };
    // The PID row unless the form drops the derivative.
    let (kp, ti_s, td_s) = if form.has_derivative() { pid } else { pi };
    Gains {
        p: kp,
        i: if form.has_integral() { kp / ti_s } else { 0.0 },
        d: if form.has_derivative() {
            kp * td_s
        } else {
            0.0
        },
    }
}
