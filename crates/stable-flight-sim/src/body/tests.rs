use approx::assert_relative_eq;
use assert2::check;
use stable_flight::euler_angles;

use super::*;

const H: f32 = 0.001;

const MASS: MassProperties = MassProperties {
    mass_kg: 1.0,
    inertia_kgm2: Vec3::new(0.01, 0.01, 0.018),
};

/// An upward force of `lift_n` (NED −z) and `torque_nm`, undamped.
fn loads(lift_n: f32, torque_nm: Vec3) -> Loads {
    Loads {
        force_ned_n: Vec3::new(0.0, 0.0, -lift_n),
        torque_nm,
        damping_nm: Vec3::zeros(),
    }
}

/// Steps `body` for `seconds`, with the loads `loads_for` gives at each step.
fn run(body: &mut Body, seconds: f32, loads_for: impl Fn(&Body) -> Loads) {
    for _ in 0..(seconds / H).round() as usize {
        let loads = loads_for(body);
        body.step(&MASS, &loads, H);
    }
}

#[test]
fn falls_freely_without_force() {
    let mut body = Body::default();
    run(&mut body, 1.0, |_| loads(0.0, Vec3::zeros()));
    assert_relative_eq!(body.velocity_ned_mps.z, GRAVITY, max_relative = 1e-4);
    // Semi-implicit Euler overshoots ½gt² by ½g·h·t.
    assert_relative_eq!(body.position_ned_m.z, 0.5 * GRAVITY, max_relative = 2e-3);
}

#[test]
fn lift_equal_to_weight_hovers() {
    let mut body = Body::default();
    run(&mut body, 1.0, |_| {
        loads(MASS.mass_kg * GRAVITY, Vec3::zeros())
    });
    assert_relative_eq!(body.velocity_ned_mps, Vec3::zeros(), epsilon = 1e-4);
}

#[test]
fn torque_spins_each_axis_up_at_torque_over_inertia() {
    for axis in 0..3 {
        let mut torque_nm = Vec3::zeros();
        torque_nm[axis] = 0.01;
        let mut body = Body::default();
        run(&mut body, 0.1, |_| loads(0.0, torque_nm));
        let expected = torque_nm[axis] / MASS.inertia_kgm2[axis] * 0.1;
        assert_relative_eq!(
            body.angular_velocity_radps[axis],
            expected,
            max_relative = 1e-3
        );
    }
}

#[test]
fn damping_slows_a_free_spin() {
    let damping_nm_per_radps = 0.004;
    let mut body = Body {
        angular_velocity_radps: Vec3::new(0.0, 0.0, 2.0),
        ..Body::default()
    };
    run(&mut body, 1.0, |body| Loads {
        damping_nm: body.angular_velocity_radps * damping_nm_per_radps,
        ..loads(0.0, Vec3::zeros())
    });
    let decay = libm::expf(-damping_nm_per_radps / MASS.inertia_kgm2.z);
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
        run(&mut body, 0.5, |_| loads(0.0, Vec3::zeros()));
        let angles = euler_angles(&body.attitude);
        assert_relative_eq!(angles.0, roll, epsilon = 1e-3);
        assert_relative_eq!(angles.1, pitch, epsilon = 1e-3);
        assert_relative_eq!(angles.2, heading, epsilon = 1e-3);
    }
}

#[test]
fn air_relative_velocity_is_against_the_wind() {
    let body = Body {
        velocity_ned_mps: Vec3::new(3.0, 0.0, -1.0),
        ..Body::default()
    };
    let air = body.air_relative_velocity_ned_mps(Vec3::new(1.0, 2.0, 0.0));
    check!(air == Vec3::new(2.0, -2.0, -1.0));
}

#[test]
fn flip_z_is_its_own_inverse() {
    let v = Vec3::new(1.0, -2.0, 3.0);
    check!(flip_z(v) == Vec3::new(1.0, -2.0, -3.0));
    check!(flip_z(flip_z(v)) == v);
}
