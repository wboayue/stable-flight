use assert2::check;
use nalgebra::UnitQuaternion;
use std::f32::consts::FRAC_PI_2;

use super::*;

/// The state of a vehicle at `heading_rad`, tilted `pitch_rad` nose up, moving at
/// `velocity_mps` (north, east) through air moving at `wind_mps`, with `config`'s drag as its
/// only horizontal push in its thrust frame: the drag model run forward, separately from the
/// estimator's inversion.
fn pushed(
    config: &WindConfig,
    heading_rad: f32,
    pitch_rad: f32,
    velocity_mps: Vec2,
    wind_mps: Vec2,
) -> VehicleState {
    let attitude = UnitQuaternion::from_euler_angles(0.0, pitch_rad, heading_rad);
    let velocity_ned = Vec3::new(velocity_mps.x, velocity_mps.y, 0.0);
    let relative_body =
        attitude.inverse_transform_vector(&(velocity_ned - Vec3::new(wind_mps.x, wind_mps.y, 0.0)));
    let drag = |i: usize| {
        let k = if config.ballistic_kgpm2[i] > 0.0 {
            0.5 * AIR_DENSITY_KGPM3 / config.ballistic_kgpm2[i]
        } else {
            0.0
        };
        let v = relative_body[i];
        -(k * v.abs() + config.momentum_per_s[i]) * v
    };
    // Thrust holds the vehicle up along body z; the drag is the rest.
    let force_body = Vec3::new(drag(0), drag(1), -GRAVITY / libm::cosf(pitch_rad));
    let specific_ned = attitude.transform_vector(&force_body);
    let acceleration_ned = specific_ned + Vec3::new(0.0, 0.0, GRAVITY);
    VehicleState {
        attitude,
        velocity_mps: Vec3::new(velocity_mps.x, velocity_mps.y, 0.0),
        acceleration_mps2: Vec3::new(acceleration_ned.x, acceleration_ned.y, -acceleration_ned.z),
        ..Default::default()
    }
}

/// `seconds` of 4 ms ticks of `state` through a fresh estimator.
fn estimate_after(config: &WindConfig, state: &VehicleState, seconds: f32) -> Option<WindEstimate> {
    let mut estimator = WindEstimator::default();
    for _ in 0..(seconds / 0.004).round() as usize {
        estimator.update(0.004, config, state);
    }
    estimator.estimate()
}

/// Settled standard deviation band, in m/s: the default noises (0.5 m/s² over a 0.3/s slope,
/// 0.1 m/s per √s) settle at 0.29 m/s along an axis with drag.
const SETTLED_SD_MPS: f32 = 0.35;

fn momentum(per_s: f32) -> WindConfig {
    WindConfig {
        momentum_per_s: Vec2::new(per_s, per_s),
        ..Default::default()
    }
}

fn ballistic(kgpm2: f32) -> WindConfig {
    WindConfig {
        ballistic_kgpm2: Vec2::new(kgpm2, kgpm2),
        ..Default::default()
    }
}

/// With drag on both axes, the estimate converges on the wind whatever the heading, tilt,
/// motion, or drag model.
#[test]
fn converges_on_the_wind_with_drag_on_both_axes() {
    let both = WindConfig {
        ballistic_kgpm2: Vec2::new(20.0, 20.0),
        momentum_per_s: Vec2::new(0.2, 0.2),
        ..Default::default()
    };
    // (case, config, heading, pitch, velocity, wind)
    let cases = [
        (
            "momentum, wind east",
            momentum(0.3),
            0.0,
            0.0,
            Vec2::zeros(),
            Vec2::new(0.0, 2.0),
        ),
        (
            "momentum, turned",
            momentum(0.3),
            1.0,
            0.0,
            Vec2::zeros(),
            Vec2::new(1.5, -1.0),
        ),
        (
            "momentum, tilted",
            momentum(0.3),
            0.5,
            0.3,
            Vec2::zeros(),
            Vec2::new(-2.0, 1.0),
        ),
        (
            "momentum, moving",
            momentum(0.3),
            0.0,
            0.0,
            Vec2::new(3.0, 1.0),
            Vec2::new(-1.0, 2.0),
        ),
        (
            "calm, moving",
            momentum(0.3),
            0.0,
            0.0,
            Vec2::new(3.0, 0.0),
            Vec2::zeros(),
        ),
        (
            "ballistic",
            ballistic(10.0),
            2.0,
            0.0,
            Vec2::zeros(),
            Vec2::new(3.0, 3.0),
        ),
        (
            "both",
            both,
            -1.0,
            0.0,
            Vec2::new(1.0, 0.0),
            Vec2::new(0.0, -4.0),
        ),
    ];
    for (case, config, heading, pitch, velocity, wind) in cases {
        let state = pushed(&config, heading, pitch, velocity, wind);
        let estimate = estimate_after(&config, &state, 10.0).expect("updated");
        let error = (estimate.velocity_mps - wind).norm();
        check!(error < 0.01, "{case}: {estimate:?}, expected {wind}");
        check!(
            estimate.sd_along_mps(Vec2::x()) < SETTLED_SD_MPS,
            "{case}: {estimate:?}"
        );
    }
}

/// An axis without drag tells nothing: with only `x`'s drag, a wind along `y` stays unknown
/// (its standard deviation near where it began) while a wind along `x` converges; turning
/// the vehicle 90° then brings the `y` wind under `x`'s drag.
#[test]
fn an_axis_without_drag_leaves_its_wind_unknown_until_a_turn() {
    let x_only = WindConfig {
        ballistic_kgpm2: Vec2::new(5.0, 0.0),
        momentum_per_s: Vec2::new(0.3, 0.0),
        ..Default::default()
    };
    let east = Vec2::new(0.0, 2.0);
    let facing_north = pushed(&x_only, 0.0, 0.0, Vec2::zeros(), east);
    let facing_east = pushed(&x_only, FRAC_PI_2, 0.0, Vec2::zeros(), east);

    let mut estimator = WindEstimator::default();
    for _ in 0..2500 {
        estimator.update(0.004, &x_only, &facing_north);
    }
    let unseen = estimator.estimate().expect("updated");
    check!(unseen.velocity_mps.norm() < 0.01, "{unseen:?}");
    check!(
        unseen.sd_along_mps(Vec2::y()) > 0.9 * INITIAL_SD_MPS,
        "{unseen:?}"
    );
    check!(
        unseen.sd_along_mps(Vec2::x()) < SETTLED_SD_MPS,
        "{unseen:?}"
    );

    for _ in 0..2500 {
        estimator.update(0.004, &x_only, &facing_east);
    }
    let seen = estimator.estimate().expect("updated");
    check!((seen.velocity_mps - east).norm() < 0.05, "{seen:?}");
    check!(seen.sd_along_mps(Vec2::y()) < SETTLED_SD_MPS, "{seen:?}");
}

/// No drag coefficient: never updated. Non-finite ticks are skipped without poisoning the
/// estimate.
#[test]
fn needs_a_drag_model_and_skips_non_finite_ticks() {
    let wind = Vec2::new(0.0, 2.0);
    let state = pushed(&momentum(0.3), 0.0, 0.0, Vec2::zeros(), wind);
    check!(estimate_after(&WindConfig::default(), &state, 10.0) == None);

    let config = momentum(0.3);
    // (case, broken state)
    let cases = [
        (
            "NaN velocity",
            VehicleState {
                velocity_mps: Vec3::new(f32::NAN, 0.0, 0.0),
                ..state
            },
        ),
        (
            "NaN acceleration",
            VehicleState {
                acceleration_mps2: Vec3::new(0.0, f32::INFINITY, 0.0),
                ..state
            },
        ),
    ];
    for (case, broken) in cases {
        let mut estimator = WindEstimator::default();
        for tick in 0..2500 {
            let state = if tick % 3 == 0 { &broken } else { &state };
            estimator.update(0.004, &config, state);
        }
        let estimate = estimator.estimate().expect("updated");
        check!(
            (estimate.velocity_mps - wind).norm() < 0.01,
            "{case}: {estimate:?}"
        );
        check!(
            estimate_after(&config, &broken, 1.0) == None,
            "{case}: only broken ticks"
        );
    }
}

/// Once converged, one wild averaging period (past 5σ) is rejected.
#[test]
fn rejects_an_outlier_past_the_gate() {
    let config = momentum(0.3);
    let wind = Vec2::new(0.0, 2.0);
    let mut estimator = WindEstimator::default();
    for _ in 0..2500 {
        estimator.update(
            0.004,
            &config,
            &pushed(&config, 0.0, 0.0, Vec2::zeros(), wind),
        );
    }
    let wild = pushed(&config, 0.0, 0.0, Vec2::zeros(), Vec2::new(0.0, 40.0));
    for _ in 0..50 {
        estimator.update(0.004, &config, &wild);
    }
    let estimate = estimator.estimate().expect("updated");
    check!((estimate.velocity_mps - wind).norm() < 0.01, "{estimate:?}");
}

/// The update rate is a time, not a tick count: 2 ms and 10 ms ticks give the same estimate
/// after the same time; reset forgets it.
#[test]
fn updates_by_time_not_ticks_and_resets() {
    let config = momentum(0.3);
    let state = pushed(&config, 0.0, 0.0, Vec2::zeros(), Vec2::new(1.0, 1.0));
    let after = |dt: f32| {
        let mut estimator = WindEstimator::default();
        for _ in 0..(1.0 / dt).round() as usize {
            estimator.update(dt, &config, &state);
        }
        estimator
    };
    let (fast, slow) = (after(0.002), after(0.01));
    let (fast_estimate, slow_estimate) = (
        fast.estimate().expect("updated"),
        slow.estimate().expect("updated"),
    );
    check!((fast_estimate.covariance_m2ps2 - slow_estimate.covariance_m2ps2).norm() < 1e-3);
    let mut reset = fast;
    reset.reset();
    check!(reset.estimate() == None);
}

/// Every coefficient and noise is checked.
#[test]
fn validate_rejects_bad_coefficients_and_noise() {
    // (field, config)
    let bad = [
        (
            "ballistic_kgpm2.x",
            WindConfig {
                ballistic_kgpm2: Vec2::new(-1.0, 0.0),
                ..Default::default()
            },
        ),
        (
            "ballistic_kgpm2.y",
            WindConfig {
                ballistic_kgpm2: Vec2::new(0.0, f32::NAN),
                ..Default::default()
            },
        ),
        (
            "momentum_per_s.x",
            WindConfig {
                momentum_per_s: Vec2::new(f32::INFINITY, 0.0),
                ..Default::default()
            },
        ),
        (
            "momentum_per_s.y",
            WindConfig {
                momentum_per_s: Vec2::new(0.0, -0.1),
                ..Default::default()
            },
        ),
        (
            "force_noise_mps2",
            WindConfig {
                force_noise_mps2: 0.0,
                ..Default::default()
            },
        ),
        (
            "process_noise_mps_per_rt_s",
            WindConfig {
                process_noise_mps_per_rt_s: -1.0,
                ..Default::default()
            },
        ),
    ];
    for (field, config) in bad {
        check!(
            config.validate().map_err(|e| e.field) == Err(field),
            "{field}"
        );
    }
    check!(WindConfig::default().validate().is_ok());
    check!(!WindConfig::default().is_on());
    check!(momentum(0.1).is_on());
}

/// An axis with little drag counts for little: its measurement noise is the force noise over
/// the drag's slope, so 0.01/s along `y` (against 0.3/s along `x`) leaves the wind across it
/// loosely known after 10 s, where `x`'s settles.
#[test]
fn an_axis_with_little_drag_counts_for_little() {
    let lopsided = WindConfig {
        momentum_per_s: Vec2::new(0.3, 0.01),
        ..Default::default()
    };
    let state = pushed(&lopsided, 0.0, 0.0, Vec2::zeros(), Vec2::new(1.0, 2.0));
    let estimate = estimate_after(&lopsided, &state, 10.0).expect("updated");
    check!(
        estimate.sd_along_mps(Vec2::x()) < SETTLED_SD_MPS,
        "{estimate:?}"
    );
    check!(estimate.sd_along_mps(Vec2::y()) > 5.0, "{estimate:?}");
}
