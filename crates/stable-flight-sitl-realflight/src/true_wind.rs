//! RealFlight's own wind at the aircraft, shared out of the vehicle to log beside a wind
//! estimate.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use stable_flight::Vec2;

/// The wind RealFlight last reported (`SimulatorState::wind_mps`, the air's velocity), north
/// and east in m/s: a cheap, cloneable handle onto one value the vehicle updates on each
/// exchange ([`RealFlightVehicle::true_wind`](crate::RealFlightVehicle::true_wind)). Lock-free,
/// so a recorder on the flight loop's thread reads it without blocking.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl_realflight::TrueWind;
///
/// let wind = TrueWind::default();
/// assert_eq!(wind.get(), None); // nothing reported yet
/// ```
#[derive(Debug, Clone)]
pub struct TrueWind(Arc<AtomicU64>);

impl Default for TrueWind {
    fn default() -> Self {
        TrueWind(Arc::new(AtomicU64::new(pack(Vec2::repeat(f32::NAN)))))
    }
}

impl TrueWind {
    /// The last reported wind (north, east), in m/s; `None` before the first report.
    pub fn get(&self) -> Option<Vec2> {
        let wind = unpack(self.0.load(Ordering::Relaxed));
        wind.iter().all(|c| c.is_finite()).then_some(wind)
    }

    /// Stores RealFlight's wind, `x` east and `y` north (its position axes).
    pub(crate) fn set_from_realflight(&self, east_mps: f32, north_mps: f32) {
        self.0
            .store(pack(Vec2::new(north_mps, east_mps)), Ordering::Relaxed);
    }
}

fn pack(wind: Vec2) -> u64 {
    (u64::from(wind.x.to_bits()) << 32) | u64::from(wind.y.to_bits())
}

fn unpack(bits: u64) -> Vec2 {
    Vec2::new(
        f32::from_bits((bits >> 32) as u32),
        f32::from_bits(bits as u32),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// RealFlight's east/north order is swapped to north/east, and clones share the value.
    #[test]
    fn reports_north_east_to_every_clone() {
        let wind = TrueWind::default();
        let reader = wind.clone();
        check!(reader.get() == None);
        wind.set_from_realflight(1.5, -2.0);
        check!(reader.get() == Some(Vec2::new(-2.0, 1.5)));
    }

    use assert2::check;
}
