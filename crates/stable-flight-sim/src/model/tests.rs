use assert2::check;

use super::*;
use crate::airframes;

fn valid() -> QuadModel {
    airframes::quad_x().model
}

#[test]
fn accepts_the_quad_x_model() {
    check!(valid().validate() == Ok(()));
}

/// An edit applied to a valid model.
type Change = fn(&mut QuadModel);

#[test]
fn rejects_each_out_of_range_parameter() {
    // (change to a valid model, expected error)
    let cases: [(Change, ModelError); 11] = [
        (|m| m.mass_kg = 0.0, ModelError::NotPositive("mass_kg")),
        (|m| m.mass_kg = f32::NAN, ModelError::NotPositive("mass_kg")),
        (
            |m| m.inertia_kgm2.z = -0.02,
            ModelError::NotPositive("inertia_kgm2.z"),
        ),
        (
            |m| m.moment_arm_m = 0.0,
            ModelError::NotPositive("moment_arm_m"),
        ),
        (
            |m| m.max_thrust_n = f32::INFINITY,
            ModelError::NotPositive("max_thrust_n"),
        ),
        (
            |m| m.motor_time_constant_s = 0.0,
            ModelError::NotPositive("motor_time_constant_s"),
        ),
        (
            |m| m.yaw_torque_per_thrust_m = -0.01,
            ModelError::Negative("yaw_torque_per_thrust_m"),
        ),
        (
            |m| m.drag_n_per_mps = f32::NAN,
            ModelError::Negative("drag_n_per_mps"),
        ),
        (
            |m| m.angular_damping_nm_per_radps.x = -1.0,
            ModelError::Negative("angular_damping_nm_per_radps.x"),
        ),
        (
            |m| m.ground_friction = -0.5,
            ModelError::Negative("ground_friction"),
        ),
        (|m| m.substeps = 0, ModelError::NoSubsteps),
    ];
    for (change, expected) in cases {
        let mut model = valid();
        change(&mut model);
        check!(model.validate() == Err(expected));
    }
}

#[test]
fn accepts_zero_for_non_negative_parameters() {
    let model = QuadModel {
        yaw_torque_per_thrust_m: 0.0,
        drag_n_per_mps: 0.0,
        angular_damping_nm_per_radps: Vec3::zeros(),
        ground_friction: 0.0,
        ..valid()
    };
    check!(model.validate() == Ok(()));
}
