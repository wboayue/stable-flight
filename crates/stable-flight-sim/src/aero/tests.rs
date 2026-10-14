use approx::assert_relative_eq;
use assert2::check;

use super::*;
use crate::airframes;

fn model() -> FixedWingModel {
    airframes::plane().model
}

/// Air at `speed_mps` along body `x`, tilted by `alpha_rad` (from below) and `beta_rad` (from
/// the right).
fn air(speed_mps: f32, alpha_rad: f32, beta_rad: f32) -> Vec3 {
    Vec3::new(
        libm::cosf(alpha_rad) * libm::cosf(beta_rad),
        libm::sinf(beta_rad),
        libm::sinf(alpha_rad) * libm::cosf(beta_rad),
    ) * speed_mps
}

#[test]
fn no_airspeed_no_aero() {
    let still = wrench(
        &model(),
        Vec3::zeros(),
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::repeat(0.3),
    );
    check!(still == Aero::default());
}

#[test]
fn flow_angles_follow_the_air() {
    // (alpha rad, beta rad)
    let cases = [
        (0.0, 0.0),
        (0.1, 0.0),
        (-0.2, 0.0),
        (0.0, 0.15),
        (0.05, -0.1),
    ];
    for (alpha_rad, beta_rad) in cases {
        let aero = wrench(
            &model(),
            air(15.0, alpha_rad, beta_rad),
            Vec3::zeros(),
            Vec3::zeros(),
        );
        assert_relative_eq!(aero.alpha_rad, alpha_rad, epsilon = 1e-5);
        assert_relative_eq!(aero.beta_rad, beta_rad, epsilon = 1e-5);
    }
}

/// Head-on air at zero α: lift up (body −z), drag back (body −x), from the coefficients.
#[test]
fn head_on_lift_and_drag() {
    let model = model();
    let c = &model.aero;
    let aero = wrench(&model, air(15.0, 0.0, 0.0), Vec3::zeros(), Vec3::zeros());
    let pressure_area_n = 0.5 * AIR_DENSITY_KGPM3 * 225.0 * model.wing_area_m2;
    let aspect_ratio = model.span_m * model.span_m / model.wing_area_m2;
    let induced = c.c_lift_0 * c.c_lift_0 / (PI * c.oswald * aspect_ratio);
    assert_relative_eq!(
        aero.force_n.z,
        -pressure_area_n * c.c_lift_0,
        max_relative = 1e-4
    );
    assert_relative_eq!(
        aero.force_n.x,
        -pressure_area_n * (c.c_drag_0 + induced),
        max_relative = 1e-4
    );
    check!(aero.force_n.y == 0.0);
}

/// Each surface, flow angle, and rate pushes its moment the frame contract's way, through the
/// coefficients' signs: the change from head-on air with nothing deflected or turning.
#[test]
fn moments_point_the_contract_way() {
    // (case, alpha rad, beta rad, rates rad/s, deflections rad, moment sign: roll, pitch, yaw;
    // 0 = not checked)
    let cases = [
        (
            "aileron rolls right",
            0.0,
            0.0,
            Vec3::zeros(),
            Vec3::new(0.2, 0.0, 0.0),
            [1.0, 0.0, 0.0],
        ),
        (
            "elevator raises the nose",
            0.0,
            0.0,
            Vec3::zeros(),
            Vec3::new(0.0, 0.2, 0.0),
            [0.0, 1.0, 0.0],
        ),
        (
            "rudder yaws right",
            0.0,
            0.0,
            Vec3::zeros(),
            Vec3::new(0.0, 0.0, 0.2),
            [0.0, 0.0, 1.0],
        ),
        (
            "roll rate damped",
            0.0,
            0.0,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::zeros(),
            [-1.0, 0.0, 0.0],
        ),
        (
            "pitch rate damped",
            0.0,
            0.0,
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::zeros(),
            [0.0, -1.0, 0.0],
        ),
        (
            "yaw rate damped",
            0.0,
            0.0,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::zeros(),
            [0.0, 0.0, -1.0],
        ),
        (
            "nose into the wind, wing away",
            0.0,
            0.2,
            Vec3::zeros(),
            Vec3::zeros(),
            [-1.0, 0.0, 1.0],
        ),
        (
            "alpha pitches the nose down",
            0.2,
            0.0,
            Vec3::zeros(),
            Vec3::zeros(),
            [0.0, -1.0, 0.0],
        ),
    ];
    let head_on = wrench(&model(), air(15.0, 0.0, 0.0), Vec3::zeros(), Vec3::zeros()).torque_nm;
    for (case, alpha_rad, beta_rad, rates_radps, deflection_rad, signs) in cases {
        let aero = wrench(
            &model(),
            air(15.0, alpha_rad, beta_rad),
            rates_radps,
            deflection_rad,
        );
        let change_nm = aero.torque_nm - head_on;
        for (axis, sign) in signs.iter().enumerate() {
            if *sign != 0.0 {
                check!(change_nm[axis] * sign > 0.0, "{case}: {change_nm:?}");
            }
        }
    }
}

/// The pitch moment at zero α and no rates or deflection is the coefficient's, and a rate's
/// damping grows with airspeed (its term is per unit of it).
#[test]
fn moments_scale_from_the_coefficients() {
    let model = model();
    let c = &model.aero;
    let pressure_area_n = 0.5 * AIR_DENSITY_KGPM3 * 100.0 * model.wing_area_m2;
    let level = wrench(&model, air(10.0, 0.0, 0.0), Vec3::zeros(), Vec3::zeros());
    assert_relative_eq!(
        level.torque_nm.y,
        pressure_area_n * model.chord_m * c.c_pitch_0,
        max_relative = 1e-4
    );
    let roll_damping = |speed_mps: f32| {
        wrench(
            &model,
            air(speed_mps, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::zeros(),
        )
        .torque_nm
        .x
    };
    assert_relative_eq!(
        roll_damping(20.0),
        2.0 * roll_damping(10.0),
        max_relative = 1e-4
    );
}

/// Lift is the linear slope inside the stall, a flat plate's past it, continuous between, and
/// finite at any angle.
#[test]
fn lift_falls_off_past_the_stall() {
    let c = model().aero;
    let linear = |alpha_rad: f32| c.c_lift_0 + c.c_lift_alpha * alpha_rad;
    let flat_plate = |alpha_rad: f32| {
        2.0 * alpha_rad.signum() * libm::sinf(alpha_rad).powi(2) * libm::cosf(alpha_rad)
    };
    // (alpha rad, expected lift coefficient)
    let cases = [
        (0.0, linear(0.0)),
        (0.2, linear(0.2)),
        (-0.2, linear(-0.2)),
        (0.8, flat_plate(0.8)),
        (-0.8, flat_plate(-0.8)),
        (1.5, flat_plate(1.5)),
    ];
    for (alpha_rad, expected) in cases {
        assert_relative_eq!(lift_coefficient(&c, alpha_rad), expected, epsilon = 1e-3);
    }
    // Past the stall, lift is well below the linear slope's.
    let past = c.alpha_stall_rad + 0.1;
    check!(lift_coefficient(&c, past) < 0.5 * linear(past));
    // Continuous through the blend, and finite all the way round.
    let mut previous = lift_coefficient(&c, -PI);
    for step in 1..=3600 {
        let alpha_rad = -PI + step as f32 * (2.0 * PI / 3600.0);
        let lift = lift_coefficient(&c, alpha_rad);
        check!(lift.is_finite(), "at {alpha_rad} rad");
        check!((lift - previous).abs() < 0.2, "jump at {alpha_rad} rad");
        previous = lift;
    }
}
