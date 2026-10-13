use approx::assert_relative_eq;
use assert2::check;
use stable_flight::euler_angles;

use super::*;
use crate::airframes;

const H: f32 = 0.001;

fn model() -> QuadModel {
    airframes::quad_x().model
}

fn calm(thrust_n: f32, torque_nm: Vec3) -> Loads {
    Loads {
        thrust_n,
        torque_nm,
        wind_ned_mps: Vec3::zeros(),
    }
}

/// Steps `body` for `seconds` under `loads`.
fn run(body: &mut Body, model: &QuadModel, loads: &Loads, seconds: f32) {
    for _ in 0..(seconds / H).round() as usize {
        body.step(model, loads, H);
    }
}

#[test]
fn falls_freely_without_thrust_or_drag() {
    let model = QuadModel {
        drag_n_per_mps: 0.0,
        ..model()
    };
    let mut body = Body::default();
    run(&mut body, &model, &calm(0.0, Vec3::zeros()), 1.0);
    assert_relative_eq!(body.velocity_ned_mps.z, GRAVITY, max_relative = 1e-4);
    // Semi-implicit Euler overshoots ½gt² by ½g·h·t.
    assert_relative_eq!(body.position_ned_m.z, 0.5 * GRAVITY, max_relative = 2e-3);
}

#[test]
fn thrust_equal_to_weight_hovers() {
    let model = model();
    let mut body = Body::default();
    run(
        &mut body,
        &model,
        &calm(model.mass_kg * GRAVITY, Vec3::zeros()),
        1.0,
    );
    assert_relative_eq!(body.velocity_ned_mps, Vec3::zeros(), epsilon = 1e-4);
}

#[test]
fn drag_sets_the_terminal_speed_in_wind() {
    let model = model();
    let wind_ned_mps = Vec3::new(0.0, 5.0, 0.0);
    let loads = Loads {
        thrust_n: model.mass_kg * GRAVITY,
        torque_nm: Vec3::zeros(),
        wind_ned_mps,
    };
    let mut body = Body::default();
    run(&mut body, &model, &loads, 30.0);
    // With no other horizontal force, the body drifts with the air mass.
    assert_relative_eq!(body.velocity_ned_mps, wind_ned_mps, epsilon = 1e-2);
}

#[test]
fn torque_spins_each_axis_up_at_torque_over_inertia() {
    let model = QuadModel {
        angular_damping_nm_per_radps: Vec3::zeros(),
        ..model()
    };
    for axis in 0..3 {
        let mut torque_nm = Vec3::zeros();
        torque_nm[axis] = 0.01;
        let mut body = Body::default();
        run(&mut body, &model, &calm(0.0, torque_nm), 0.1);
        let expected = torque_nm[axis] / model.inertia_kgm2[axis] * 0.1;
        assert_relative_eq!(
            body.angular_velocity_radps[axis],
            expected,
            max_relative = 1e-3
        );
    }
}

#[test]
fn damping_slows_a_free_spin() {
    let model = model();
    let mut body = Body {
        angular_velocity_radps: Vec3::new(0.0, 0.0, 2.0),
        ..Body::default()
    };
    run(&mut body, &model, &calm(0.0, Vec3::zeros()), 1.0);
    let decay = libm::expf(-model.angular_damping_nm_per_radps.z / model.inertia_kgm2.z);
    assert_relative_eq!(
        body.angular_velocity_radps.z,
        2.0 * decay,
        max_relative = 1e-2
    );
}

/// Body rates are FRD and turn the attitude about body axes, so the Euler angles move the way
/// the frame contract says: roll right, nose up, nose right.
#[test]
fn body_rates_turn_the_attitude_the_contract_way() {
    let model = QuadModel {
        angular_damping_nm_per_radps: Vec3::zeros(),
        ..model()
    };
    // (body rates rad/s, expected roll, pitch, heading after 0.5 s)
    let cases = [
        (Vec3::new(0.4, 0.0, 0.0), (0.2, 0.0, 0.0)),
        (Vec3::new(0.0, 0.4, 0.0), (0.0, 0.2, 0.0)),
        (Vec3::new(0.0, 0.0, 0.4), (0.0, 0.0, 0.2)),
    ];
    for (rates, (roll, pitch, heading)) in cases {
        let mut body = Body {
            angular_velocity_radps: rates,
            ..Body::default()
        };
        run(&mut body, &model, &calm(0.0, Vec3::zeros()), 0.5);
        let angles = euler_angles(&body.attitude);
        assert_relative_eq!(angles.0, roll, epsilon = 1e-3);
        assert_relative_eq!(angles.1, pitch, epsilon = 1e-3);
        assert_relative_eq!(angles.2, heading, epsilon = 1e-3);
    }
}

/// A body banked right and lifting its weight accelerates east (north-east-down: +y).
#[test]
fn tilted_thrust_accelerates_toward_the_bank() {
    let model = QuadModel {
        drag_n_per_mps: 0.0,
        ..model()
    };
    let mut body = Body {
        attitude: UnitQuaternion::from_euler_angles(0.2, 0.0, 0.0),
        ..Body::default()
    };
    let weight_n = model.mass_kg * GRAVITY;
    body.step(&model, &calm(weight_n, Vec3::zeros()), H);
    check!(body.velocity_ned_mps.y > 0.0);
    check!(body.velocity_ned_mps.x.abs() < 1e-6);
}

#[test]
fn flip_z_is_its_own_inverse() {
    let v = Vec3::new(1.0, -2.0, 3.0);
    check!(flip_z(v) == Vec3::new(1.0, -2.0, -3.0));
    check!(flip_z(flip_z(v)) == v);
}
