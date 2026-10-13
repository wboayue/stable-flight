use approx::assert_relative_eq;
use assert2::check;

use super::*;
use crate::airframes;

const H: f32 = 0.001;

fn model() -> QuadModel {
    airframes::quad_x().model
}

fn weight_n(model: &QuadModel) -> f32 {
    model.mass_kg * GRAVITY
}

#[test]
fn airborne_body_is_untouched() {
    let model = model();
    let airborne = Body {
        position_ned_m: Vec3::new(0.0, 0.0, -0.01),
        velocity_ned_mps: Vec3::new(1.0, 0.0, 2.0),
        ..Body::default()
    };
    let mut body = airborne.clone();
    check!(!contact(&mut body, &model, 0.0, H));
    check!(body == airborne);
}

#[test]
fn touching_down_stops_the_sink_and_levels() {
    let model = model();
    let mut body = Body {
        position_ned_m: Vec3::new(3.0, 4.0, 0.02),
        velocity_ned_mps: Vec3::new(0.0, 0.0, 1.5),
        attitude: UnitQuaternion::from_euler_angles(0.1, -0.05, 1.0),
        angular_velocity_radps: Vec3::new(0.3, 0.2, 0.1),
        ..Body::default()
    };
    check!(contact(&mut body, &model, 0.5 * weight_n(&model), H));
    check!(body.position_ned_m == Vec3::new(3.0, 4.0, 0.0));
    check!(body.velocity_ned_mps == Vec3::zeros());
    check!(body.angular_velocity_radps == Vec3::zeros());
    let (roll_rad, pitch_rad, heading_rad) = euler_angles(&body.attitude);
    check!(roll_rad.abs() < 1e-6);
    check!(pitch_rad.abs() < 1e-6);
    assert_relative_eq!(heading_rad, 1.0, epsilon = 1e-5);
}

#[test]
fn thrust_above_weight_lifts_off() {
    let model = model();
    let mut body = Body {
        position_ned_m: Vec3::new(0.0, 0.0, 0.001),
        velocity_ned_mps: Vec3::new(0.0, 0.0, -0.2),
        ..Body::default()
    };
    check!(!contact(&mut body, &model, 1.1 * weight_n(&model), H));
    check!(body.position_ned_m.z == 0.0);
    // Climbing (NED −z) is kept.
    check!(body.velocity_ned_mps.z == -0.2);
}

#[test]
fn friction_slows_and_stops_a_slide() {
    let model = model();
    // (thrust as a fraction of weight, initial speed m/s, speed after one step)
    let slowdown = model.ground_friction * GRAVITY * H;
    let cases = [
        (0.0, 1.0, 1.0 - slowdown),
        (0.5, 1.0, 1.0 - 0.5 * slowdown),
        (0.0, 0.5 * slowdown, 0.0),
    ];
    for (thrust, speed_mps, expected_mps) in cases {
        let mut body = Body {
            velocity_ned_mps: Vec3::new(0.6, 0.8, 0.0) * speed_mps,
            ..Body::default()
        };
        check!(contact(&mut body, &model, thrust * weight_n(&model), H));
        assert_relative_eq!(body.velocity_ned_mps.norm(), expected_mps, epsilon = 1e-6);
    }
}
