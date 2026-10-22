use approx::assert_relative_eq;
use assert2::check;

use super::*;

const DT: f32 = 0.01;

fn config(p: f32, i: f32, ff: f32) -> PlaneRateControllerConfig {
    let gains = RateGains { p, i, ff };
    PlaneRateControllerConfig {
        roll: gains,
        pitch: gains,
        yaw: gains,
        max_integral: 2.0,
        trim: Vec3::zeros(),
    }
}

/// Trim adds to each axis, scaled by s² like the feedback, and clamps with the rest; a
/// non-finite error leaves an axis its trim, a non-finite scale nothing:
/// (P, trim, setpoint, rate, scale, expected).
#[test]
fn adds_trim_scaled_by_s_squared() {
    let cases = [
        // At zero error, the trim alone.
        (0.0, 0.1, 0.0, 0.0, 1.0, 0.1),
        (0.0, -0.1, 0.0, 0.0, 1.0, -0.1),
        // Half the trim airspeed: four times the moment; twice: a quarter.
        (0.0, 0.1, 0.0, 0.0, 2.0, 0.4),
        (0.0, 0.1, 0.0, 0.0, 0.5, 0.025),
        // With feedback: 0.1 · 1 rad/s + 0.1.
        (0.1, 0.1, 1.0, 0.0, 1.0, 0.2),
        // Clamped with the feedback: 0.4 · 4 + 0.5 · 4.
        (0.4, 0.5, 1.0, 0.0, 2.0, 1.0),
        (0.0, 0.5, f32::NAN, 0.0, 1.0, 0.5),
        (0.0, 0.5, 0.0, f32::INFINITY, 2.0, 1.0),
        (0.0, 0.5, 0.0, 0.0, f32::NAN, 0.0),
    ];
    for (p, trim, desired, current, scale, expected) in cases {
        let config = PlaneRateControllerConfig {
            trim: Vec3::repeat(trim),
            ..config(p, 0.0, 0.0)
        };
        let mut controller = PlaneRateController::default();
        let out = controller.control(
            &config,
            DT,
            Vec3::repeat(desired),
            Vec3::repeat(current),
            scale,
            true,
        );
        for axis in 0..3 {
            assert_relative_eq!(out[axis], expected, epsilon = 1e-6);
        }
    }
}

/// Feedback scales with s², feedforward with s: (gains, setpoint, rate, scale, expected).
#[test]
fn scales_feedback_by_s_squared_and_feedforward_by_s() {
    let cases = [
        // P only: 0.1 · 1 rad/s error.
        ((0.1, 0.0, 0.0), 1.0, 0.0, 1.0, 0.1),
        ((0.1, 0.0, 0.0), 1.0, 0.0, 2.0, 0.4),
        ((0.1, 0.0, 0.0), 1.0, 0.0, 0.5, 0.025),
        // FF only: 0.3 · 1 rad/s setpoint, error irrelevant.
        ((0.0, 0.0, 0.3), 1.0, 1.0, 1.0, 0.3),
        ((0.0, 0.0, 0.3), 1.0, 1.0, 2.0, 0.6),
        ((0.0, 0.0, 0.3), 1.0, 1.0, 0.5, 0.15),
        // Both, then clamped to 1.
        ((0.1, 0.0, 0.3), 1.0, 0.0, 1.0, 0.4),
        ((1.0, 0.0, 1.0), 2.0, 0.0, 2.0, 1.0),
        ((1.0, 0.0, 1.0), -2.0, 0.0, 2.0, -1.0),
    ];
    for ((p, i, ff), desired, current, scale, expected) in cases {
        let mut controller = PlaneRateController::default();
        let out = controller.control(
            &config(p, i, ff),
            DT,
            Vec3::new(desired, 0.0, 0.0),
            Vec3::new(current, 0.0, 0.0),
            scale,
            true,
        );
        assert_relative_eq!(out.x, expected, epsilon = 1e-6);
        check!(out.y == 0.0 && out.z == 0.0);
    }
}

/// The integral accumulates only while integrating, is clamped to `max_integral`, and is
/// cleared and rescaled on request.
#[test]
fn integrates_only_when_allowed_and_clamps() {
    let config = config(0.0, 0.5, 0.0);
    let error = Vec3::new(1.0, -1.0, 0.0);
    let mut controller = PlaneRateController::default();

    // Held below min airspeed: no I term.
    let held = controller.control(&config, DT, error, Vec3::zeros(), 1.0, false);
    check!(held == Vec3::zeros());

    for _ in 0..10 {
        controller.control(&config, DT, error, Vec3::zeros(), 1.0, true);
    }
    let out = controller.control(&config, DT, Vec3::zeros(), Vec3::zeros(), 1.0, false);
    // 10 integrating ticks · 0.01 s · 1 rad/s error = 0.1 rad, times I 0.5.
    assert_relative_eq!(out.x, 0.05, epsilon = 1e-5);
    assert_relative_eq!(out.y, -0.05, epsilon = 1e-5);

    for _ in 0..1_000 {
        controller.control(&config, DT, error, Vec3::zeros(), 1.0, true);
    }
    let saturated = controller.control(&config, DT, Vec3::zeros(), Vec3::zeros(), 1.0, false);
    assert_relative_eq!(saturated.x, 0.5 * 2.0, epsilon = 1e-5); // max_integral 2

    let doubled = PlaneRateControllerConfig {
        roll: RateGains {
            i: 1.0,
            ..config.roll
        },
        ..config
    };
    controller.rescale_integral(&config, &doubled);
    let rescaled = controller.control(&doubled, DT, Vec3::zeros(), Vec3::zeros(), 1.0, false);
    assert_relative_eq!(rescaled.x, saturated.x, epsilon = 1e-5);

    controller.clear_integral();
    let cleared = controller.control(&config, DT, Vec3::zeros(), Vec3::zeros(), 1.0, false);
    check!(cleared == Vec3::zeros());
}

/// A NaN setpoint or rate silences that axis only and keeps NaN out of its integral; a NaN
/// scale silences all.
#[test]
fn non_finite_inputs_silence_their_axis() {
    let config = config(0.1, 0.5, 0.2);
    let mut controller = PlaneRateController::default();
    let out = controller.control(
        &config,
        DT,
        Vec3::new(f32::NAN, 1.0, 1.0),
        Vec3::new(0.0, f32::NAN, 0.0),
        1.0,
        true,
    );
    check!(out.x == 0.0 && out.y == 0.0 && out.z > 0.0);
    let after = controller.control(&config, DT, Vec3::zeros(), Vec3::zeros(), 1.0, false);
    check!(after.x == 0.0 && after.y == 0.0 && after.z.is_finite());

    let silent = controller.control(
        &config,
        DT,
        Vec3::repeat(1.0),
        Vec3::zeros(),
        f32::NAN,
        true,
    );
    check!(silent == Vec3::zeros());
}

#[test]
fn validates_gains_and_limit() {
    type Break = fn(&mut PlaneRateControllerConfig);
    let cases: [(Break, &str); 7] = [
        (|c| c.roll.p = -0.1, "roll.p"),
        (|c| c.pitch.i = f32::NAN, "pitch.i"),
        (|c| c.yaw.ff = f32::INFINITY, "yaw.ff"),
        (|c| c.max_integral = -1.0, "max_integral"),
        (|c| c.trim.x = 1.1, "trim.x"),
        (|c| c.trim.y = -1.1, "trim.y"),
        (|c| c.trim.z = f32::NAN, "trim.z"),
    ];
    check!(PlaneRateControllerConfig::default().validate().is_ok());
    for (corrupt, field) in cases {
        let mut config = PlaneRateControllerConfig::default();
        corrupt(&mut config);
        check!(config.validate().map_err(|e| e.field) == Err(field));
    }
}
