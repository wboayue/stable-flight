use approx::assert_relative_eq;
use assert2::check;

use super::*;

fn on(gain: f32) -> ForwardThrust {
    ForwardThrust::new(ForwardThrustConfig {
        gain,
        ..ForwardThrustConfig::default()
    })
}

#[test]
fn splits_nose_down_pitch_between_tilt_and_pusher() {
    // (case, gain, pitch in, pitch out, thrust); pitches in degrees, the default 3° limit.
    let tan = |deg: f32| libm::tanf(deg * DEG);
    let cases = [
        ("level", 1.2, 0.0, 0.0, 0.0),
        ("nose up passes", 1.2, 20.0, 20.0, 0.0),
        ("inside the limit", 1.2, -2.0, -2.0, 1.2 * tan(2.0)),
        ("at the limit", 1.2, -3.0, -3.0, 1.2 * tan(3.0)),
        ("past the limit", 1.2, -20.0, -3.0, 1.2 * tan(20.0)),
        ("tilt counted to 45°", 0.5, -60.0, -3.0, 0.5),
        ("thrust capped at 1", 2.0, -40.0, -3.0, 1.0),
        ("off passes", 0.0, -20.0, -20.0, 0.0),
    ];
    for (case, gain, pitch_deg, want_pitch_deg, want_thrust) in cases {
        let split = on(gain).split(pitch_deg * DEG);
        assert_relative_eq!(split.pitch_rad, want_pitch_deg * DEG, epsilon = 1e-6);
        assert_relative_eq!(split.thrust, want_thrust, epsilon = 1e-6);
        check!(split.thrust >= 0.0, "{case}");
    }
}

#[test]
fn nan_pitch_gives_no_thrust_and_stays_level() {
    let split = on(1.2).split(f32::NAN);
    check!(split.pitch_rad.is_nan());
    check!(split.thrust == 0.0);
}

#[test]
fn a_zero_limit_holds_the_pitch_level() {
    let forward = ForwardThrust::new(ForwardThrustConfig {
        gain: 1.0,
        pitch_limit_rad: 0.0,
    });
    let split = forward.split(-10.0 * DEG);
    check!(split.pitch_rad == 0.0);
    assert_relative_eq!(split.thrust, libm::tanf(10.0 * DEG), epsilon = 1e-6);
}

#[test]
fn enabled_only_with_a_positive_gain() {
    check!(!ForwardThrust::new(ForwardThrustConfig::default()).enabled());
    check!(on(0.1).enabled());
}

#[test]
fn rejects_out_of_range_config() {
    // (field, config)
    let base = ForwardThrustConfig::default();
    let cases = [
        ("gain", ForwardThrustConfig { gain: -0.1, ..base }),
        (
            "gain",
            ForwardThrustConfig {
                gain: f32::NAN,
                ..base
            },
        ),
        (
            "gain",
            ForwardThrustConfig {
                gain: f32::INFINITY,
                ..base
            },
        ),
        (
            "pitch_limit_rad",
            ForwardThrustConfig {
                pitch_limit_rad: -0.01,
                ..base
            },
        ),
        (
            "pitch_limit_rad",
            ForwardThrustConfig {
                pitch_limit_rad: 90.0 * DEG,
                ..base
            },
        ),
        (
            "pitch_limit_rad",
            ForwardThrustConfig {
                pitch_limit_rad: f32::NAN,
                ..base
            },
        ),
    ];
    for (field, config) in cases {
        let error = config.validate().err();
        check!(error.map(|e| e.field) == Some(field), "{config:?}");
        check!(
            ForwardThrust::new(base).set_config(config).is_err(),
            "{config:?}"
        );
    }
    check!(base.validate().is_ok());
}
