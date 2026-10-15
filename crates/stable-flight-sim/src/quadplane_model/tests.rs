use assert2::check;

use super::*;
use crate::airframes;

fn model() -> QuadPlaneModel {
    airframes::quadplane().model
}

fn lift(lift: LiftRotors) -> QuadPlaneModel {
    QuadPlaneModel { lift, ..model() }
}

#[test]
fn accepts_the_airframe() {
    check!(model().validate() == Ok(()));
}

#[test]
fn rejects_out_of_range_parameters() {
    let rotors = model().lift;
    // (case, model, error)
    let cases = [
        (
            "wing",
            QuadPlaneModel {
                wing: FixedWingModel {
                    span_m: 0.0,
                    ..model().wing
                },
                ..model()
            },
            ModelError::NotPositive("span_m"),
        ),
        (
            "moment arm",
            lift(LiftRotors {
                moment_arm_m: 0.0,
                ..rotors
            }),
            ModelError::NotPositive("lift.moment_arm_m"),
        ),
        (
            "max thrust",
            lift(LiftRotors {
                max_thrust_n: f32::NAN,
                ..rotors
            }),
            ModelError::NotPositive("lift.max_thrust_n"),
        ),
        (
            "time constant",
            lift(LiftRotors {
                motor_time_constant_s: -0.05,
                ..rotors
            }),
            ModelError::NotPositive("lift.motor_time_constant_s"),
        ),
        (
            "yaw torque",
            lift(LiftRotors {
                yaw_torque_per_thrust_m: -0.01,
                ..rotors
            }),
            ModelError::Negative("lift.yaw_torque_per_thrust_m"),
        ),
        (
            "drag",
            lift(LiftRotors {
                drag_n_per_mps: f32::INFINITY,
                ..rotors
            }),
            ModelError::Negative("lift.drag_n_per_mps"),
        ),
        (
            "pusher slot",
            QuadPlaneModel {
                pusher_slot: 8,
                ..model()
            },
            ModelError::PusherSlot(8),
        ),
    ];
    for (case, model, error) in cases {
        check!(model.validate() == Err(error), "{case}");
    }
}

#[test]
fn accepts_zero_yaw_torque_and_drag() {
    let model = lift(LiftRotors {
        yaw_torque_per_thrust_m: 0.0,
        drag_n_per_mps: 0.0,
        ..model().lift
    });
    check!(model.validate() == Ok(()));
}
