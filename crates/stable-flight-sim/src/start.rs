//! Where a simulated vehicle starts: see [`Start`].

use stable_flight::Vec3;
use stable_flight::nalgebra::UnitQuaternion;

/// Where and how a vehicle starts, in the [frame
/// contract](stable_flight::VehicleState#frame-contract)'s axes.
///
/// # Examples
///
/// ```
/// use stable_flight_sim::Start;
///
/// // 100 m up, flying east at 15 m/s.
/// let start = Start::level(100.0, 15.0, core::f32::consts::FRAC_PI_2);
/// assert!((start.velocity_mps.y - 15.0).abs() < 1e-5);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Start {
    /// Position (north, east, up), in m.
    pub position_m: Vec3,
    /// Velocity (north, east, up), in m/s.
    pub velocity_mps: Vec3,
    /// Attitude, as [`VehicleState::attitude`](stable_flight::VehicleState::attitude).
    pub attitude: UnitQuaternion<f32>,
    /// Body rates (FRD roll, pitch, yaw), in rad/s.
    pub angular_velocity_radps: Vec3,
}

impl Start {
    /// Wings level over the origin at `altitude_m`, nose and velocity along `heading_rad`
    /// (clockwise from north) at `speed_mps`, not turning.
    pub fn level(altitude_m: f32, speed_mps: f32, heading_rad: f32) -> Self {
        let (sin, cos) = (libm::sinf(heading_rad), libm::cosf(heading_rad));
        Start {
            position_m: Vec3::new(0.0, 0.0, altitude_m),
            velocity_mps: Vec3::new(cos, sin, 0.0) * speed_mps,
            attitude: UnitQuaternion::from_euler_angles(0.0, 0.0, heading_rad),
            angular_velocity_radps: Vec3::zeros(),
        }
    }
}
