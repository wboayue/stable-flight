//! The air mass the vehicle flies through: see [`Wind`].

use core::f32::consts::TAU;

use stable_flight::Vec3;

#[cfg(test)]
mod tests;

/// Air-mass velocity: a steady wind plus an optional [`Gust`].
///
/// Velocities are in the [`VehicleState`](stable_flight::VehicleState) world axes (north,
/// east, up), in m/s, and give where the air moves *to*: a west wind (blowing east) is
/// `Vec3::new(0.0, 5.0, 0.0)`. It is a function of time only, so a run is repeatable.
///
/// # Examples
///
/// ```
/// use stable_flight::Vec3;
/// use stable_flight_sim::{Gust, Wind};
///
/// let wind = Wind {
///     steady_mps: Vec3::new(0.0, 3.0, 0.0),
///     gust: Some(Gust { start_s: 5.0, duration_s: 2.0, peak_mps: Vec3::new(4.0, 0.0, 0.0) }),
/// };
/// assert_eq!(wind.velocity_mps(0.0), Vec3::new(0.0, 3.0, 0.0));
/// assert_eq!(wind.velocity_mps(6.0), Vec3::new(4.0, 3.0, 0.0)); // gust peak
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Wind {
    /// Constant wind, in m/s (north, east, up).
    pub steady_mps: Vec3,
    /// A gust added on top of the steady wind.
    pub gust: Option<Gust>,
}

/// A discrete "1 − cos" gust: rises smoothly from zero to [`peak_mps`](Self::peak_mps) at the
/// middle of its window and back to zero at its end.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gust {
    /// When the gust begins, in seconds of simulated time.
    pub start_s: f32,
    /// How long it lasts, in s. A non-positive duration means no gust.
    pub duration_s: f32,
    /// Gust velocity at its peak, in m/s (north, east, up).
    pub peak_mps: Vec3,
}

impl Wind {
    /// Air-mass velocity at `time_s` seconds of simulated time, in m/s (north, east, up).
    pub fn velocity_mps(&self, time_s: f32) -> Vec3 {
        let gust = self
            .gust
            .map_or(Vec3::zeros(), |gust| gust.velocity_mps(time_s));
        self.steady_mps + gust
    }
}

impl Gust {
    /// Gust velocity at `time_s`: zero outside its window.
    pub fn velocity_mps(&self, time_s: f32) -> Vec3 {
        let elapsed_s = time_s - self.start_s;
        if !(self.duration_s > 0.0 && (0.0..=self.duration_s).contains(&elapsed_s)) {
            return Vec3::zeros();
        }
        let shape = 0.5 * (1.0 - libm::cosf(TAU * elapsed_s / self.duration_s));
        self.peak_mps * shape
    }
}
