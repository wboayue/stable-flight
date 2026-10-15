//! [`LocalOrigin`]: where X-Plane's local frame sits, so position stays continuous when it
//! moves.

use stable_flight::Vec2;

use crate::XPlaneSample;
use crate::state::local_north_east;

#[cfg(test)]
mod tests;

/// Meters per degree of latitude (and of longitude at the equator), on a sphere of the earth's
/// mean radius (6371 km). Only for telling a relocation from a moved origin, never for position.
const METERS_PER_DEGREE: f64 = 111_195.0;

/// Distance beyond what the aircraft could fly between two samples that reads as a relocation:
/// above the lat/lon quantization (under 1 m) and any frame-timing error at flight speeds.
const RELOCATION_MARGIN_M: f32 = 100.0;

/// The geodetic point local position (0, 0) maps to: where the aircraft was when
/// [`LocalOrigin`] anchored. Up is height above mean sea level, so the bridge's
/// `--origin` is `latitude,longitude,0`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Anchor {
    /// Latitude, degrees north.
    pub latitude_deg: f64,
    /// Longitude, degrees east.
    pub longitude_deg: f64,
}

/// What [`LocalOrigin::update`] changed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OriginEvent {
    /// The first sample anchored position at the aircraft.
    Anchored(Anchor),
    /// X-Plane moved its local origin under the aircraft; the offset moved by `by_m` (north,
    /// east) so position is continuous.
    Shifted {
        /// Change of the offset, north and east, m.
        by_m: Vec2,
    },
    /// The aircraft jumped farther than it could fly (a new airport, a reset elsewhere):
    /// position is anchored at its new place.
    Relocated(Anchor),
}

/// Keeps the offset that places X-Plane's local position relative to a fixed [`Anchor`].
///
/// X-Plane's local frame has its origin on a whole degree (`lat_ref`, `lon_ref`) and moves
/// it when the aircraft relocates, and when it flies far from it; local position then jumps by
/// tens of kilometers (xplane-bridge `design.md`, Spike findings: Reloads). This type tells the
/// two apart by the aircraft's latitude and longitude:
///
/// - **Origin moved, aircraft continuous:** the offset absorbs the jump, from the last
///   position carried forward by the last velocity over the elapsed simulator time, so
///   position stays continuous to within a frame's motion.
/// - **Aircraft jumped** (farther than its speed allows, plus 100 m): re-anchors there, so
///   position restarts at (0, 0).
///
/// # Examples
///
/// ```
/// use stable_flight_sitl_xplane::{LocalOrigin, OriginEvent, XPlaneSample, vehicle_state};
///
/// let mut origin = LocalOrigin::default();
/// let sample = XPlaneSample {
///     quaternion: [1.0, 0.0, 0.0, 0.0],
///     local_position_m: [-237.0, 12.0, -33_867.0],
///     latitude_deg: 46.3,
///     longitude_deg: 11.0,
///     origin_deg: [46.0, 11.0],
///     ..Default::default()
/// };
/// assert!(matches!(origin.update(&sample), Some(OriginEvent::Anchored(_))));
///
/// // Position starts at the anchor.
/// let state = vehicle_state(&sample, origin.offset_m());
/// assert!(state.position_m.xy().norm() < 1e-3);
/// ```
#[derive(Debug, Clone, Default)]
pub struct LocalOrigin {
    anchor: Option<Anchor>,
    /// Added to local north/east to place it relative to the anchor.
    offset_m: Vec2,
    last: Option<Last>,
}

/// What [`LocalOrigin::update`] compares the next sample against.
#[derive(Debug, Clone, Copy)]
struct Last {
    sim_time_s: f32,
    origin_deg: [f32; 2],
    local_m: Vec2,
    velocity_mps: Vec2,
    latitude_deg: f32,
    longitude_deg: f32,
}

impl LocalOrigin {
    /// Takes the next sample; returns what changed, if anything.
    pub fn update(&mut self, sample: &XPlaneSample) -> Option<OriginEvent> {
        let local_m = local_north_east(sample.local_position_m);
        let event = match self.last {
            None => Some(OriginEvent::Anchored(self.anchor_at(sample, local_m))),
            Some(last) => self.follow(&last, sample, local_m),
        };
        self.last = Some(Last {
            sim_time_s: sample.sim_time_s,
            origin_deg: sample.origin_deg,
            local_m,
            velocity_mps: local_north_east(sample.local_velocity_mps),
            latitude_deg: sample.latitude_deg,
            longitude_deg: sample.longitude_deg,
        });
        event
    }

    /// Offset to add to local north/east, m.
    pub fn offset_m(&self) -> Vec2 {
        self.offset_m
    }

    /// Where position (0, 0) is; `None` before the first sample.
    pub fn anchor(&self) -> Option<Anchor> {
        self.anchor
    }

    fn follow(&mut self, last: &Last, sample: &XPlaneSample, local_m: Vec2) -> Option<OriginEvent> {
        // Simulator time, so a pause or a stall between samples doesn't count as flight.
        let dt_s = (sample.sim_time_s - last.sim_time_s).max(0.0);
        let reach_m = last.velocity_mps.norm() * dt_s + RELOCATION_MARGIN_M;
        if sample.origin_deg == last.origin_deg {
            if (local_m - last.local_m).norm() <= reach_m {
                return None;
            }
            return Some(OriginEvent::Relocated(self.anchor_at(sample, local_m)));
        }
        if moved_m(last, sample) > reach_m {
            return Some(OriginEvent::Relocated(self.anchor_at(sample, local_m)));
        }
        let expected_m = last.local_m + self.offset_m + last.velocity_mps * dt_s;
        let offset_m = expected_m - local_m;
        let by_m = offset_m - self.offset_m;
        self.offset_m = offset_m;
        Some(OriginEvent::Shifted { by_m })
    }

    /// Anchors at the aircraft: position (0, 0) here.
    fn anchor_at(&mut self, sample: &XPlaneSample, local_m: Vec2) -> Anchor {
        let anchor = Anchor {
            latitude_deg: f64::from(sample.latitude_deg),
            longitude_deg: f64::from(sample.longitude_deg),
        };
        self.anchor = Some(anchor);
        self.offset_m = -local_m;
        anchor
    }
}

/// How far the aircraft moved between `last` and `sample`, by latitude and longitude, m.
fn moved_m(last: &Last, sample: &XPlaneSample) -> f32 {
    let north = f64::from(sample.latitude_deg - last.latitude_deg) * METERS_PER_DEGREE;
    let east = f64::from(sample.longitude_deg - last.longitude_deg)
        * METERS_PER_DEGREE
        * f64::from(last.latitude_deg).to_radians().cos();
    north.hypot(east) as f32
}
