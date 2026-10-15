//! [`vehicle_state`]: one [`XPlaneSample`] in the `VehicleState` frame contract.

use stable_flight::nalgebra::{Quaternion, UnitQuaternion};
use stable_flight::{Vec2, Vec3, VehicleState, euler_angles};

use crate::XPlaneSample;

#[cfg(test)]
mod tests;

/// Converts one X-Plane sample to the `VehicleState` frame contract (north/east/up, FRD body,
/// radians).
///
/// - **Attitude:** X-Plane's quaternion `q` as is: it rotates north-east-down to the body
///   frame with the same yaw-pitch-roll as `psi`/`theta`/`phi`, which is the contract's
///   attitude (xplane-bridge `design.md`, Spike findings: Quaternion). The Euler fields come
///   from it, so they always agree.
/// - **Body rates:** `P`, `Q`, `R` are already FRD (roll right, nose up, nose right); deg/s to
///   rad/s.
/// - **Position:** north and east from the OpenGL local frame (x east, z south) plus
///   `offset_m` (north, east), which [`LocalOrigin`](crate::LocalOrigin) keeps so position
///   stays continuous when X-Plane moves its local origin. Up is the elevation above mean sea
///   level, not `local_y`: that is height above the plane tangent at the local origin, which
///   drifts from the ground by d²/2R (2 m at 5 km from it).
/// - **Velocity, acceleration:** the local frame's (east, up, south) as north/east/up.
/// - **Airspeed:** true airspeed.
///
/// # Examples
///
/// ```
/// use stable_flight::Vec2;
/// use stable_flight_sitl_xplane::{XPlaneSample, vehicle_state};
///
/// // 3 m east and 4 m south of the local origin, 120 m above sea level.
/// let sample = XPlaneSample {
///     quaternion: [1.0, 0.0, 0.0, 0.0],
///     local_position_m: [3.0, 50.0, 4.0],
///     elevation_m: 120.0,
///     ..Default::default()
/// };
/// let state = vehicle_state(&sample, Vec2::zeros());
/// assert_eq!(state.position_m.x, -4.0);
/// assert_eq!(state.position_m.y, 3.0);
/// assert_eq!(state.position_m.z, 120.0);
/// ```
pub fn vehicle_state(sample: &XPlaneSample, offset_m: Vec2) -> VehicleState {
    let [w, x, y, z] = sample.quaternion;
    let attitude = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
    let (roll_rad, pitch_rad, heading_rad) = euler_angles(&attitude);
    let north_east_m = local_north_east(sample.local_position_m) + offset_m;
    VehicleState {
        position_m: Vec3::new(north_east_m.x, north_east_m.y, sample.elevation_m),
        velocity_mps: north_east_up(sample.local_velocity_mps),
        acceleration_mps2: north_east_up(sample.local_acceleration_mps2),
        attitude,
        angular_velocity_radps: Vec3::from(sample.body_rates_degps.map(f32::to_radians)),
        roll_rad,
        pitch_rad,
        heading_rad,
        airspeed_mps: Some(sample.true_airspeed_mps),
    }
}

/// North and east of an OpenGL local vector (x east, y up, z south).
pub(crate) fn local_north_east([east, _, south]: [f32; 3]) -> Vec2 {
    Vec2::new(-south, east)
}

/// An OpenGL local vector (x east, y up, z south) as north/east/up.
fn north_east_up([east, up, south]: [f32; 3]) -> Vec3 {
    Vec3::new(-south, east, up)
}
