//! What to fly, from the ground station or an offboard app: see [`SetpointService`].

use log::debug;
use mavlink::common::*;
use stable_flight_link::stable_flight::nalgebra::{Quaternion, UnitQuaternion};
use stable_flight_link::stable_flight::{
    AbsoluteAttitudeCommand, AttitudeCommand, Command, ControlInput, Vec2, WaypointCommand,
    euler_angles,
};
use stable_flight_link::{Control, Uplink};

use crate::outbox::Outbox;

#[cfg(test)]
mod tests;

/// `MANUAL_CONTROL`'s full-scale axis value.
const AXIS_FULL_SCALE: f32 = 1000.0;

/// `SET_POSITION_TARGET_*`'s ignore bits for the position, all of which must be clear.
const POSITION_IGNORE: PositionTargetTypemask =
    PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE
        .union(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Y_IGNORE)
        .union(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Z_IGNORE);
/// The ignore bits for the horizontal velocity, used when both are clear.
const VELOCITY_IGNORE: PositionTargetTypemask =
    PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VX_IGNORE
        .union(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VY_IGNORE);

/// The ignore bit for the yaw, which is used when clear.
const YAW_IGNORE: PositionTargetTypemask =
    PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_IGNORE;

/// `SET_ATTITUDE_TARGET` type-mask bits the flight controller can't fly without: the attitude
/// and the throttle must be included, and thrust must be the scalar `thrust`, not a vector.
const ATTITUDE_TARGET_UNSUPPORTED: AttitudeTargetTypemask =
    AttitudeTargetTypemask::ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
        .union(AttitudeTargetTypemask::ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE)
        .union(AttitudeTargetTypemask::ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET);

/// Smallest `SET_ATTITUDE_TARGET` quaternion norm accepted; below it the rotation is
/// undefined.
const MIN_QUATERNION_NORM: f32 = 1e-6;

/// Translates setpoints into `Control::Command`s for the flight controller. It only
/// translates: arming is [`CommandService`](crate::commands::CommandService)'s, and the flight
/// controller decides when a stream has gone stale.
///
/// - `MANUAL_CONTROL` (QGC's joystick or virtual joystick, 25 Hz) → an attitude command:
///   roll = y, pitch = −x (stick forward, x > 0, is nose down), yaw = r, throttle = z, each
///   divided by 1000. An axis marked invalid (`INT16_MAX`) is NaN, which the flight controller
///   treats as neutral (its input policy). Buttons are ignored.
/// - `SET_POSITION_TARGET_LOCAL_NED` in `MAV_FRAME_LOCAL_NED` → a waypoint: north and east from
///   x and y, altitude from −z, and the horizontal velocity as its fly-through velocity when
///   the type mask includes it, and `yaw` (radians from north) as its heading when the mask
///   includes it. The position must be included; acceleration and yaw rate are ignored. The
///   message has no speed, so the waypoint's cruise speed is
///   `offboard_speed_mps`. Any other frame, or a mask without the position, is dropped (the
///   message has no ack) and logged at debug level.
/// - `SET_ATTITUDE_TARGET` (offboard apps) → an absolute attitude: roll, pitch, and heading
///   from the quaternion `q` (`[w, x, y, z]`, body FRD from local NED, normalized here), and
///   `thrust` as collective thrust. Body rates are ignored whatever the mask says (the flight
///   controller has no rate feedforward). A mask that ignores the attitude or the throttle, or
///   sets `THRUST_BODY_SET`, or a non-finite or zero quaternion, is dropped and logged at
///   debug level.
///
/// A joystick and an offboard app sending at once both go through; the last message wins.
#[derive(Debug)]
pub(crate) struct SetpointService {
    offboard_speed_mps: f32,
}

impl SetpointService {
    /// Flies position targets at most `offboard_speed_mps`.
    pub(crate) fn new(offboard_speed_mps: f32) -> Self {
        Self { offboard_speed_mps }
    }

    /// `MANUAL_CONTROL`: sticks for an attitude command.
    pub(crate) fn on_manual_control(&self, data: &MANUAL_CONTROL_DATA, out: &mut Outbox) {
        let input = ControlInput::Attitude(AttitudeCommand {
            throttle: axis(data.z),
            roll: axis(data.y),
            pitch: -axis(data.x),
            yaw: axis(data.r),
        });
        send(input, out);
    }

    /// `SET_ATTITUDE_TARGET`: an absolute attitude, if the mask and quaternion are supported.
    pub(crate) fn on_attitude_target(&self, data: &SET_ATTITUDE_TARGET_DATA, out: &mut Outbox) {
        if data.type_mask.intersects(ATTITUDE_TARGET_UNSUPPORTED) {
            debug!("dropped attitude target with mask {:?}", data.type_mask);
            return;
        }
        let Some(attitude) = unit_quaternion(data.q) else {
            debug!("dropped attitude target with quaternion {:?}", data.q);
            return;
        };
        let (roll_rad, pitch_rad, heading_rad) = euler_angles(&attitude);
        send(
            ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
                roll_rad,
                pitch_rad,
                heading_rad,
                thrust: data.thrust,
            }),
            out,
        );
    }

    /// `SET_POSITION_TARGET_LOCAL_NED`: a waypoint, if the frame and mask are supported.
    pub(crate) fn on_position_target(
        &self,
        data: &SET_POSITION_TARGET_LOCAL_NED_DATA,
        out: &mut Outbox,
    ) {
        if data.coordinate_frame != MavFrame::MAV_FRAME_LOCAL_NED {
            debug!("dropped position target in {:?}", data.coordinate_frame);
            return;
        }
        if data.type_mask.intersects(POSITION_IGNORE) {
            debug!(
                "dropped position target without a position: {:?}",
                data.type_mask
            );
            return;
        }
        let velocity_mps =
            (!data.type_mask.intersects(VELOCITY_IGNORE)).then(|| Vec2::new(data.vx, data.vy));
        let heading_rad = (!data.type_mask.intersects(YAW_IGNORE)).then_some(data.yaw);
        send(
            ControlInput::Waypoint(WaypointCommand {
                position_m: Vec2::new(data.x, data.y),
                velocity_mps,
                altitude_m: -data.z,
                cruise_speed_mps: self.offboard_speed_mps,
                heading_rad,
            }),
            out,
        );
    }
}

/// A `MANUAL_CONTROL` axis normalized to [-1, 1] (not clamped: the flight controller
/// saturates); NaN when marked invalid.
fn axis(value: i16) -> f32 {
    if value == i16::MAX {
        return f32::NAN;
    }
    f32::from(value) / AXIS_FULL_SCALE
}

/// MAVLink's `[w, x, y, z]` quaternion, normalized; `None` when it has no rotation (a
/// non-finite or zero component set).
fn unit_quaternion([w, x, y, z]: [f32; 4]) -> Option<UnitQuaternion<f32>> {
    let q = Quaternion::new(w, x, y, z);
    if !q.coords.iter().all(|c| c.is_finite()) {
        return None;
    }
    UnitQuaternion::try_new(q, MIN_QUATERNION_NORM)
}

/// Queues `input` for the flight controller.
fn send(input: ControlInput, out: &mut Outbox) {
    out.to_fc.push(uplink(input));
}

/// `input` as a command for the flight controller. The command's `arm` is ignored there
/// (arming is `Control::Arm`'s), so it is left `false`.
pub(crate) fn uplink(input: ControlInput) -> Uplink {
    Uplink::Control(Control::Command(Command::new(false, input)))
}
