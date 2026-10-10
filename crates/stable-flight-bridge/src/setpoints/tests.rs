use assert2::check;

use stable_flight_link::stable_flight::wrap_angle;

use super::*;

fn sent(out: &Outbox) -> Vec<ControlInput> {
    out.to_fc
        .iter()
        .map(|u| match u {
            Uplink::Control(Control::Command(command)) => {
                check!(!command.arm, "arming is `Control::Arm`'s");
                command.input
            }
            other => panic!("expected a command, got {other:?}"),
        })
        .collect()
}

fn manual(x: i16, y: i16, z: i16, r: i16) -> MANUAL_CONTROL_DATA {
    MANUAL_CONTROL_DATA {
        target: 1,
        x,
        y,
        z,
        r,
        ..Default::default()
    }
}

#[test]
fn maps_joystick_axes_to_attitude_sticks() {
    // (case, x, y, z, r, expected (throttle, roll, pitch, yaw))
    let cases = [
        ("centered, idle", 0, 0, 0, 0, (0.0, 0.0, 0.0, 0.0)),
        (
            "stick forward is nose down",
            1000,
            0,
            500,
            0,
            (0.5, 0.0, -1.0, 0.0),
        ),
        (
            "stick back is nose up",
            -500,
            0,
            500,
            0,
            (0.5, 0.0, 0.5, 0.0),
        ),
        (
            "right roll, right yaw",
            0,
            250,
            1000,
            -1000,
            (1.0, 0.25, 0.0, -1.0),
        ),
    ];
    for (case, x, y, z, r, (throttle, roll, pitch, yaw)) in cases {
        let mut out = Outbox::default();
        SetpointService::new(5.0).on_manual_control(&manual(x, y, z, r), &mut out);
        check!(
            sent(&out)
                == [ControlInput::Attitude(AttitudeCommand {
                    throttle,
                    roll,
                    pitch,
                    yaw,
                    mode: 0,
                })],
            "{case}"
        );
    }
}

#[test]
fn an_invalid_axis_is_nan() {
    let mut out = Outbox::default();
    SetpointService::new(5.0).on_manual_control(&manual(0, i16::MAX, 500, 0), &mut out);
    let [ControlInput::Attitude(sticks)] = sent(&out)[..] else {
        panic!("expected one attitude command");
    };
    check!(sticks.roll.is_nan());
    check!(sticks.throttle == 0.5);
}

fn target(frame: MavFrame, ignore: PositionTargetTypemask) -> SET_POSITION_TARGET_LOCAL_NED_DATA {
    SET_POSITION_TARGET_LOCAL_NED_DATA {
        target_system: 1,
        target_component: 1,
        coordinate_frame: frame,
        type_mask: ignore,
        x: 10.0,
        y: -4.0,
        z: -5.0, // NED: 5 m up
        vx: 1.0,
        vy: 2.0,
        vz: 3.0,
        ..Default::default()
    }
}

#[test]
fn flies_local_ned_position_targets_as_waypoints() {
    use MavFrame::{MAV_FRAME_BODY_FRD, MAV_FRAME_GLOBAL, MAV_FRAME_LOCAL_NED as LOCAL};
    use PositionTargetTypemask as M;
    let accel_yaw = M::POSITION_TARGET_TYPEMASK_AX_IGNORE
        | M::POSITION_TARGET_TYPEMASK_AY_IGNORE
        | M::POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | M::POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | M::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    // (case, frame, ignored, expected velocity (None: dropped entirely, Some(None): no velocity))
    let cases = [
        (
            "position only",
            LOCAL,
            accel_yaw | VELOCITY_IGNORE,
            Some(None),
        ),
        (
            "position and velocity",
            LOCAL,
            accel_yaw,
            Some(Some(Vec2::new(1.0, 2.0))),
        ),
        (
            "acceleration and yaw requested: ignored",
            LOCAL,
            VELOCITY_IGNORE,
            Some(None),
        ),
        ("no position", LOCAL, POSITION_IGNORE, None),
        (
            "no altitude",
            LOCAL,
            M::POSITION_TARGET_TYPEMASK_Z_IGNORE,
            None,
        ),
        ("body frame", MAV_FRAME_BODY_FRD, accel_yaw, None),
        ("global frame", MAV_FRAME_GLOBAL, accel_yaw, None),
    ];
    for (case, frame, ignore, expected) in cases {
        let mut out = Outbox::default();
        SetpointService::new(3.0).on_position_target(&target(frame, ignore), &mut out);
        let inputs = sent(&out);
        match expected {
            None => {
                check!(inputs.is_empty(), "{case}: dropped");
            }
            Some(velocity_mps) => {
                let [ControlInput::Waypoint(waypoint)] = inputs[..] else {
                    panic!("{case}: expected one waypoint, got {inputs:?}");
                };
                check!(waypoint.position_m == Vec2::new(10.0, -4.0), "{case}");
                check!(waypoint.altitude_m == 5.0, "{case}: altitude is up");
                check!(waypoint.velocity_mps == velocity_mps, "{case}");
                check!(waypoint.cruise_speed_mps == 3.0, "{case}");
            }
        }
    }
}

fn attitude_target(q: [f32; 4], type_mask: AttitudeTargetTypemask) -> SET_ATTITUDE_TARGET_DATA {
    SET_ATTITUDE_TARGET_DATA {
        target_system: 1,
        target_component: 1,
        q,
        body_roll_rate: 0.5,
        body_pitch_rate: -0.5,
        body_yaw_rate: 1.0,
        thrust: 0.45,
        type_mask,
        ..Default::default()
    }
}

/// The absolute attitude sent for `data`, or `None` if it was dropped.
fn absolute_attitude(data: &SET_ATTITUDE_TARGET_DATA) -> Option<AbsoluteAttitudeCommand> {
    let mut out = Outbox::default();
    SetpointService::new(5.0).on_attitude_target(data, &mut out);
    match sent(&out)[..] {
        [] => None,
        [ControlInput::AbsoluteAttitude(command)] => Some(command),
        ref other => panic!("expected one absolute attitude, got {other:?}"),
    }
}

#[test]
fn converts_attitude_target_quaternions_to_angles() {
    use std::f32::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI};
    let (s, c) = (0.15_f32.sin(), 0.15_f32.cos()); // half of 0.3 rad
    let combined = UnitQuaternion::from_euler_angles(0.2, -0.1, 2.5);
    let rates = AttitudeTargetTypemask::empty(); // rates "included": still ignored
    // (case, q [w, x, y, z], expected (roll, pitch, heading))
    let cases = [
        (
            "identity: level, north",
            [1.0, 0.0, 0.0, 0.0],
            (0.0, 0.0, 0.0),
        ),
        ("roll right", [c, s, 0.0, 0.0], (0.3, 0.0, 0.0)),
        ("nose up", [c, 0.0, s, 0.0], (0.0, 0.3, 0.0)),
        (
            "heading east",
            [FRAC_1_SQRT_2, 0.0, 0.0, FRAC_1_SQRT_2],
            (0.0, 0.0, FRAC_PI_2),
        ),
        ("heading south", [0.0, 0.0, 0.0, 1.0], (0.0, 0.0, PI)),
        (
            "combined",
            [combined.w, combined.i, combined.j, combined.k],
            (0.2, -0.1, 2.5),
        ),
        (
            "not unit: normalized",
            [2.0 * c, 2.0 * s, 0.0, 0.0],
            (0.3, 0.0, 0.0),
        ),
    ];
    for (case, q, (roll, pitch, heading)) in cases {
        let Some(command) = absolute_attitude(&attitude_target(q, rates)) else {
            panic!("{case}: dropped");
        };
        check!(
            (command.roll_rad - roll).abs() < 1e-5,
            "{case}: {command:?}"
        );
        check!(
            (command.pitch_rad - pitch).abs() < 1e-5,
            "{case}: {command:?}"
        );
        check!(
            wrap_angle(command.heading_rad - heading).abs() < 1e-5,
            "{case}: {command:?}"
        );
        check!(command.thrust == 0.45, "{case}");
    }
}

#[test]
fn drops_attitude_targets_it_cant_fly() {
    use AttitudeTargetTypemask as M;
    let level = [1.0, 0.0, 0.0, 0.0];
    let rates = M::ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
        | M::ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
        | M::ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE;
    // (case, q, mask, flown)
    let cases = [
        ("attitude and thrust, rates ignored", level, rates, true),
        ("rates requested: flown as angles", level, M::empty(), true),
        (
            "no attitude",
            level,
            M::ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,
            false,
        ),
        (
            "no throttle",
            level,
            rates | M::ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
            false,
        ),
        (
            "thrust as a body vector",
            level,
            rates | M::ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET,
            false,
        ),
        ("zero quaternion", [0.0; 4], rates, false),
        ("NaN quaternion", [f32::NAN, 0.0, 0.0, 0.0], rates, false),
        (
            "infinite quaternion",
            [f32::INFINITY, 0.0, 0.0, 0.0],
            rates,
            false,
        ),
    ];
    for (case, q, mask, flown) in cases {
        let command = absolute_attitude(&attitude_target(q, mask));
        check!(command.is_some() == flown, "{case}: {command:?}");
    }
}
