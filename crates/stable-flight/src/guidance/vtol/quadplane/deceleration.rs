//! [`BackDeceleration`]: the nose-up that slows a quadplane's back transition.

use crate::VehicleState;

#[cfg(test)]
mod tests;

/// The most nose-up the back transition flies, in radians: PX4's
/// `kDecelerationIntegratorLimit` (`FlightTaskTransition.hpp:65`, PX4 `c4e4ef98e9`), about 17°.
const PITCH_MAX_RAD: f32 = 0.3;

/// Slows a quadplane's back transition by pitching the nose up: the less it slows than the
/// target deceleration, the further the nose comes up, from level to at most
/// [`PITCH_MAX_RAD`]. With the pusher off only drag slows it, which a clean wing has too
/// little of: the Alia glided from 50 to 31 m/s in 28 s at pitch 0 (#496).
///
/// The pitch is an integral of the shortfall, as PX4's back transition flies it
/// (`FlightTaskTransition.cpp:112-153`):
///
/// `pitch += gain · (target − deceleration) · dt`, clamped to `[0, PITCH_MAX_RAD]`,
///
/// with the deceleration measured along the ground velocity (positive slowing). A NaN
/// velocity or acceleration holds the pitch. Allocation-free, one square root per tick.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(super) struct BackDeceleration {
    /// The nose-up flown, in radians, nose up positive.
    pitch_rad: f32,
}

impl BackDeceleration {
    /// Back to level: call as a back transition begins.
    pub(super) fn reset(&mut self) {
        self.pitch_rad = 0.0;
    }

    /// The pitch to fly this tick, in radians nose up: grown by `gain` (rad/s per m/s²) times
    /// how far the deceleration falls short of `target_mps2`.
    pub(super) fn pitch_rad(
        &mut self,
        dt: f32,
        state: &VehicleState,
        target_mps2: f32,
        gain: f32,
    ) -> f32 {
        let deceleration_mps2 = ground_deceleration_mps2(state);
        if deceleration_mps2.is_finite() {
            let shortfall_mps2 = target_mps2 - deceleration_mps2;
            self.pitch_rad =
                (self.pitch_rad + gain * shortfall_mps2 * dt).clamp(0.0, PITCH_MAX_RAD);
        }
        self.pitch_rad
    }
}

/// The horizontal acceleration against the ground velocity, in m/s², positive slowing: 0 when
/// standing still (no direction to slow along), NaN when the velocity or acceleration is.
fn ground_deceleration_mps2(state: &VehicleState) -> f32 {
    let velocity_mps = state.velocity_mps.xy();
    let speed_mps = velocity_mps.norm();
    if speed_mps == 0.0 {
        return 0.0;
    }
    -state.acceleration_mps2.xy().dot(&velocity_mps) / speed_mps
}
