//! Placing local position on the earth: a [`GeodeticOrigin`] and the [`FlatEarth`] projection
//! around it, for the messages that report global position.

use thiserror::Error;

#[cfg(test)]
mod tests;

/// Earth's mean radius, for a spherical flat-earth projection: its error grows with distance
/// from the origin but stays small at SITL ranges (a few hundred meters).
pub(crate) const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// A default place for local origin: 34°53'03.8"N 120°24'12.6"W, at sea level.
pub const DEFAULT_ORIGIN: GeodeticOrigin = GeodeticOrigin {
    latitude_deg: 34.884389,
    longitude_deg: -120.4035,
    altitude_m: 0.0,
};

/// The geodetic point that local position (0, 0, 0) maps to, for ground stations that
/// plot global position. `VehicleState` is local, so this is the only source of latitude
/// and longitude.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeodeticOrigin {
    /// Latitude, north positive; strictly between -90 and 90 (the projection degenerates at
    /// the poles).
    pub latitude_deg: f64,
    /// Longitude, east positive; -180 to 180.
    pub longitude_deg: f64,
    /// Altitude above mean sea level of the backend's altitude datum (0 for a sea-level datum).
    pub altitude_m: f64,
}

impl GeodeticOrigin {
    fn is_valid(&self) -> bool {
        self.latitude_deg.abs() < 90.0
            && self.longitude_deg.abs() <= 180.0
            && self.altitude_m.is_finite()
    }
}

/// Returned by [`FlatEarth::new`]: a coordinate is out of range or not finite (see
/// [`GeodeticOrigin`]).
#[derive(Debug, Clone, Copy, PartialEq, Error)]
#[error("invalid geodetic origin {0:?}")]
pub struct InvalidOrigin(pub GeodeticOrigin);

/// Projects local north/east/up positions onto latitude, longitude, and altitude around a
/// [`GeodeticOrigin`], treating the earth as flat near it. Meters-per-degree are computed
/// once here so a projection costs no trig.
///
/// # Examples
///
/// ```
/// use stable_flight_mavlink::{DEFAULT_ORIGIN, FlatEarth, GeodeticOrigin};
///
/// let earth = FlatEarth::new(DEFAULT_ORIGIN)?;
/// assert!(FlatEarth::new(GeodeticOrigin { latitude_deg: 90.0, ..DEFAULT_ORIGIN }).is_err());
/// # Ok::<(), stable_flight_mavlink::InvalidOrigin>(())
/// ```
#[derive(Debug, Clone, Copy)]
pub struct FlatEarth {
    origin: GeodeticOrigin,
    m_per_deg_lat: f64,
    m_per_deg_lon: f64,
}

impl FlatEarth {
    /// The projection around `origin`.
    ///
    /// # Errors
    ///
    /// [`InvalidOrigin`] if a coordinate is out of range or not finite.
    pub fn new(origin: GeodeticOrigin) -> Result<Self, InvalidOrigin> {
        if !origin.is_valid() {
            return Err(InvalidOrigin(origin));
        }
        let m_per_deg_lat = EARTH_RADIUS_M.to_radians();
        Ok(FlatEarth {
            origin,
            m_per_deg_lat,
            m_per_deg_lon: m_per_deg_lat * origin.latitude_deg.to_radians().cos(),
        })
    }

    /// Latitude and longitude in degrees, and altitude in meters, of local `position_m`.
    pub(crate) fn project(&self, position_m: [f32; 3]) -> (f64, f64, f64) {
        let [north, east, up] = position_m.map(f64::from);
        (
            self.origin.latitude_deg + north / self.m_per_deg_lat,
            self.origin.longitude_deg + east / self.m_per_deg_lon,
            self.origin.altitude_m + up,
        )
    }
}
