//! The `--origin` argument: see [`parse_origin`].

use stable_flight_mavlink::{FlatEarth, GeodeticOrigin};

#[cfg(test)]
mod tests;

/// Parses `lat,lon,alt` (degrees, degrees, meters above mean sea level) into the projection
/// around that origin, rejecting a malformed or out-of-range one.
pub(crate) fn parse_origin(s: &str) -> Result<FlatEarth, String> {
    let parts: Vec<_> = s.split(',').map(str::trim).collect();
    let [lat, lon, alt] = parts.as_slice() else {
        return Err(format!("expected lat,lon,alt, got {s:?}"));
    };
    let number = |part: &str| {
        part.parse::<f64>()
            .map_err(|e| format!("{part:?} is not a number: {e}"))
    };
    let origin = GeodeticOrigin {
        latitude_deg: number(lat)?,
        longitude_deg: number(lon)?,
        altitude_m: number(alt)?,
    };
    FlatEarth::new(origin).map_err(|e| e.to_string())
}
