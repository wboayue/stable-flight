/// Configures the attitude controller for a vehicle.
/// This includes the maximum roll and pitch angles that the vehicle can achieve.
/// The yaw angle is managed by the yaw controller.
pub struct AttitudeConfiguration {
    /// Maximum roll angle in radians.
    /// This limits how far the vehicle can tilt to the side.
    pub max_roll: f32,
    /// Maximum pitch angle in radians.
    /// This limits how far the vehicle can tilt forward or backward.
    pub max_pitch: f32,
}

pub struct AttitudeController {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}
