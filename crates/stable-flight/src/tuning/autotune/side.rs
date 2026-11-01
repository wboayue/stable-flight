//! What the supervisor reads from the vehicle each update ([`Motion`]), and what a stage's
//! procedure decides on ([`Decide`]).

use super::AutotuneAbort;
use crate::math::euler_angles;
use crate::tuning::{Measurement, ProbeAbort};
use crate::{Setpoint, Vec2, Vec3, VehicleState};
use nalgebra::UnitQuaternion;

/// What the envelope and the settle check, read once per update from the state. The envelope
/// reads tilt from level and the settle from the attitude target, so a hover leaning to hold
/// its position settles where it leans. The settle's rates are filtered through the
/// stabilizer's gyro low-pass, the rates its loops fly on: read raw, a still vehicle's gyro
/// noise and vibration hold them over the settle (0.2 rad/s on the sim's noise rows, the
/// settle 0.1), so no stage starts (#373). The envelope reads the raw rate, so a breach shows
/// on its first tick. Angles and rates are in the probed loops' frame
/// ([`Tunable::probe_frame`]): view axes for a tailsitter's hover, where nose-up reads level.
pub(super) struct Motion {
    /// The larger of |roll| and |pitch|, from level, in rad.
    pub(super) tilt_rad: f32,
    /// The largest raw body rate magnitude, in rad/s.
    pub(super) rate_radps: f32,
    /// The |roll| and |pitch| errors from the attitude target, in rad.
    pub(super) axis_tilt_error_rad: [f32; 2],
    /// The horizontal ground speed, in m/s; 0 for a NaN velocity.
    pub(super) speed_mps: f32,
    /// The filtered |roll| and |pitch| rates, in rad/s.
    pub(super) axis_rate_radps: [f32; 2],
    /// The filtered |yaw| rate, in rad/s.
    pub(super) yaw_rate_radps: f32,
    pub(super) altitude_m: f32,
}

impl Motion {
    /// `state`'s motion in `frame` (the probed loops' frame, in the body frame), with
    /// `filtered_radps` its body rates through the gyro low-pass and `target_rad` the roll
    /// and pitch the loops hold ([`attitude_target`]), in that frame.
    pub(super) fn of(
        state: &VehicleState,
        frame: &UnitQuaternion<f32>,
        filtered_radps: Vec3,
        target_rad: Vec2,
    ) -> Self {
        // The view as `VehicleState::rotate_body` builds it, without cloning the state.
        let (roll, pitch, _) = euler_angles(&(state.attitude * frame));
        let raw_radps = frame.inverse_transform_vector(&state.angular_velocity_radps);
        let filtered_radps = frame.inverse_transform_vector(&filtered_radps);
        let roll_error_rad = (roll - target_rad.x).abs();
        let pitch_error_rad = (pitch - target_rad.y).abs();
        Self {
            tilt_rad: roll.abs().max(pitch.abs()),
            rate_radps: raw_radps.amax(),
            axis_tilt_error_rad: [roll_error_rad, pitch_error_rad],
            speed_mps: zero_if_nan(state.velocity_mps.xy().norm()),
            axis_rate_radps: [filtered_radps.x.abs(), filtered_radps.y.abs()],
            yaw_rate_radps: filtered_radps.z.abs(),
            altitude_m: state.position_m.z,
        }
    }
}

/// The roll and pitch `setpoint` holds, in rad, in the stabilizer's frame: an attitude
/// setpoint's (a NaN axis is level, as the stabilizer reads it), a blend's hover side's, and
/// level for a rate or plane setpoint, which holds no hover attitude. Guidance's target,
/// before the stabilizer caps it at its maximum tilt: a target past the cap never settles, and
/// the stage times out.
pub(super) fn attitude_target(setpoint: &Setpoint) -> Vec2 {
    let attitude = match setpoint {
        Setpoint::Attitude(attitude) => attitude,
        Setpoint::Blend(blend) => &blend.hover,
        _ => return Vec2::zeros(),
    };
    Vec2::new(
        zero_if_nan(attitude.roll_rad),
        zero_if_nan(attitude.pitch_rad),
    )
}

/// `value`, or 0 for NaN: the neutral reading of a missing angle or speed.
fn zero_if_nan(value: f32) -> f32 {
    if value.is_nan() { 0.0 } else { value }
}

/// What a stage's procedure decides on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Decide {
    /// The stage's start.
    Start,
    /// A finished probe's measurement, and whether the host stalled during it.
    Measured {
        measurement: Measurement,
        stalled: bool,
    },
    /// A probe stopped because the host turned slow, with no measurement.
    Stalled,
    /// A probe that gave up.
    Aborted { abort: ProbeAbort },
    /// A probe or a hold on candidate gains that took the vehicle past the envelope.
    Breached { reason: AutotuneAbort },
    /// A hold ended, `settled` on its candidate or timed out.
    Held { settled: bool },
    /// The settle before a probe timed out on the committed gains.
    Unsettled,
}
