use crate::{Vec2, Vec3};
use nalgebra::UnitQuaternion;

use super::*;
use std::f32::consts::PI;

// Builds a CopterStabilizer with non-default gains on every loop.
fn setup_controller() -> CopterStabilizer {
    let rate_gains = Gains {
        p: 1.0,
        i: 0.1,
        d: 0.05,
    };
    let attitude_gains = Gains {
        p: 2.0,
        i: 0.2,
        d: 0.1,
    };
    let config = CopterStabilizerConfig {
        body_rate: BodyRateControllerConfig {
            roll_gains: rate_gains,
            pitch_gains: rate_gains,
            yaw_gains: Gains {
                p: 1.0,
                i: 0.1,
                d: 0.05,
            },
            max_integral: 100.0,
        },
        attitude: AttitudeControllerConfig {
            roll_gains: attitude_gains,
            pitch_gains: attitude_gains,
            max_integral: 100.0,
            max_angles_rad: Vec2::new(PI / 4.0, PI / 4.0), // 45° for roll and pitch
            max_rate_radps: Vec2::new(3.0, 3.0),           // Max roll, pitch rates in rad/s
        },
        yaw: YawControllerConfig {
            gains: Gains {
                p: 1.2,
                i: 0.12,
                d: 0.06,
            },
            ..CopterStabilizerConfig::default().yaw
        },
        ..Default::default()
    };
    CopterStabilizer::new(config)
}

#[test]
fn test_rate_controller_zero_error() {
    let mut controller = setup_controller();
    let dt = 0.01;
    let desired_rates = Vec3::zeros();
    let current_rates = Vec3::zeros();

    let config = controller.config.body_rate;
    let output = controller
        .body_rate
        .control(&config, dt, desired_rates, current_rates);

    // With zero error (desired and current rates are the same), PID terms should be 0
    assert_eq!(
        output,
        Vec3::zeros(),
        "Output should be zero when error is zero"
    );
}

#[test]
fn test_rate_controller_positive_error() {
    let mut controller = setup_controller();
    let dt = 0.01;
    let desired_rates = Vec3::new(1.0, 1.0, 1.0);
    let current_rates = Vec3::zeros();

    let config = controller.config.body_rate;
    let output = controller
        .body_rate
        .control(&config, dt, desired_rates, current_rates);

    // Check that the output is positive for all axes when there is a positive error
    assert!(output.x > 0.0, "Roll moment should be positive");
    assert!(output.y > 0.0, "Pitch moment should be positive");
    assert!(output.z > 0.0, "Yaw moment should be positive");
}

#[test]
fn test_yaw_controller_zero_error() {
    let mut controller = setup_controller();
    let dt = 0.01;
    let desired_yaw = 0.0;
    let current_yaw = 0.0;

    let config = controller.config.yaw;
    let output = controller
        .yaw
        .control(&config, dt, desired_yaw, current_yaw);
    assert_eq!(output, 0.0, "Yaw rate should be zero when error is zero");
}

#[test]
fn test_yaw_controller_angle_wrapping() {
    let mut controller = setup_controller();
    let dt = 0.01;
    let desired_yaw = PI + 0.1;
    let current_yaw = -PI + 0.1;

    let config = controller.config.yaw;
    let output = controller
        .yaw
        .control(&config, dt, desired_yaw, current_yaw);
    assert!(
        output < 0.0,
        "Yaw rate should be negative for shortest path"
    );
}

#[test]
fn zero_rate_setpoint_at_rest_yields_no_output() {
    let mut controller = setup_controller();
    let dt = 0.01;
    let manual_setpoint = Setpoint::Rate(RateSetpoint {
        roll_rate: 0.0,
        pitch_rate: 0.0,
        yaw_rate: 0.0,
        thrust: 0.0,
    });
    let state = VehicleState::default();

    let output = controller.stabilize(dt, &manual_setpoint, &state);
    assert_eq!(
        output,
        ControlOutput::default(),
        "Output should be default for zero inputs"
    );
}

/// Out-of-range or NaN setpoint components stabilize exactly like their saturated
/// equivalents (the input policy), so a misbehaving guidance can't panic the loop.
#[test]
fn out_of_range_or_nan_setpoints_saturate_instead_of_panicking() {
    let nan = f32::NAN;
    let max = PI / 4.0; // the config's max_angles_rad
    // (thrust, roll_rad, pitch_rad, equivalent in range)
    let attitude_cases = [
        (1.2, 0.0, 0.0, (1.0, 0.0, 0.0)),
        (-0.1, 0.0, 0.0, (0.0, 0.0, 0.0)),
        (0.5, 1.21, -1.3, (0.5, max, -max)),
        (nan, nan, 0.2, (0.0, 0.0, 0.2)),
    ];
    let state = VehicleState::default();
    for (thrust, roll, pitch, (t, r, p)) in attitude_cases {
        let setpoint = |thrust, roll, pitch| {
            Setpoint::Attitude(AttitudeSetpoint {
                roll_rad: roll,
                pitch_rad: pitch,
                thrust,
                ..Default::default()
            })
        };
        let mut raw = CopterStabilizer::new(CopterStabilizerConfig::default());
        let mut saturated = CopterStabilizer::new(CopterStabilizerConfig::default());
        for _ in 0..5 {
            let a = raw.stabilize(0.004, &setpoint(thrust, roll, pitch), &state);
            let b = saturated.stabilize(0.004, &setpoint(t, r, p), &state);
            assert!(
                a == b,
                "({thrust}, {roll}, {pitch}) gave {a:?}, expected {b:?}"
            );
        }
    }

    for thrust in [1.5, -0.3, nan] {
        let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
        let output = stabilizer.stabilize(
            0.004,
            &Setpoint::Rate(RateSetpoint {
                thrust,
                ..Default::default()
            }),
            &state,
        );
        assert!(
            (0.0..=1.0).contains(&output.thrust),
            "rate thrust {thrust} gave {output:?}"
        );
    }
}

/// The stabilizer tracks the attitude quaternion; the Euler fields are not read.
#[test]
fn attitude_is_tracked_from_the_quaternion() {
    let (roll, pitch, heading) = (0.2, -0.1, 0.3);
    let consistent = VehicleState {
        attitude: UnitQuaternion::from_euler_angles(roll, pitch, heading),
        roll_rad: roll,
        pitch_rad: pitch,
        heading_rad: heading,
        ..Default::default()
    };
    // Euler fields level and north; quaternion as above.
    let quaternion_only = VehicleState {
        roll_rad: 0.0,
        pitch_rad: 0.0,
        heading_rad: 0.0,
        ..consistent.clone()
    };
    let setpoint = Setpoint::Attitude(AttitudeSetpoint {
        thrust: 0.5,
        ..Default::default()
    });
    let run = |state: &VehicleState| {
        CopterStabilizer::new(CopterStabilizerConfig::default()).stabilize(0.004, &setpoint, state)
    };

    let expected = run(&consistent);
    assert!(
        expected.moments != Vec3::zeros(),
        "tilted state must be corrected"
    );
    let tilted = run(&quaternion_only);
    assert!(tilted == expected, "gave {tilted:?}, expected {expected:?}");
}

/// The heading setpoint is absolute and unbounded: a NaN target holds no yaw correction, and
/// a target outside [-π, π) acts like its wrapped equivalent.
#[test]
fn heading_setpoint_is_wrapped_and_nan_is_neutral() {
    let state = VehicleState::default();
    let yaw_moment = |heading: f32| {
        let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
        let setpoint = Setpoint::Attitude(AttitudeSetpoint {
            heading_rad: heading,
            thrust: 0.5,
            ..Default::default()
        });
        (0..5)
            .map(|_| stabilizer.stabilize(0.004, &setpoint, &state).moments.z)
            .collect::<Vec<f32>>()
    };

    assert!(yaw_moment(f32::NAN) == [0.0; 5]);
    // (heading, wrapped equivalent)
    for (heading, wrapped) in [
        (5.0, 5.0 - 2.0 * PI),
        (-4.0, -4.0 + 2.0 * PI),
        (7.0, 7.0 - 2.0 * PI),
    ] {
        for (a, b) in yaw_moment(heading).into_iter().zip(yaw_moment(wrapped)) {
            approx::assert_relative_eq!(a, b, epsilon = 1e-3);
        }
    }
}

/// A NaN on one gyro axis zeroes that axis's moment only; the others keep tracking.
#[test]
fn nan_rate_estimate_zeroes_only_its_axis() {
    let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
    let state = VehicleState {
        angular_velocity_radps: Vec3::new(f32::NAN, 0.0, 0.0),
        ..Default::default()
    };
    let output = stabilizer.stabilize(
        0.004,
        &Setpoint::Rate(RateSetpoint {
            roll_rate: 0.5,
            pitch_rate: 0.5,
            yaw_rate: 0.5,
            thrust: 0.5,
        }),
        &state,
    );
    assert!(output.moments.x == 0.0, "gave {output:?}");
    assert!(
        output.moments.y > 0.0 && output.moments.z > 0.0,
        "gave {output:?}"
    );
}

/// A corrupted attitude quaternion is "no estimate": zero moments. nalgebra's own `euler_angles` reads its yaw as a finite 0, which
/// commanded a full yaw moment toward the heading setpoint.
#[test]
fn non_finite_attitude_quaternion_yields_zero_moments() {
    use nalgebra::Quaternion;

    let nan = f32::NAN;
    // (w, x, y, z)
    let cases = [
        [nan, nan, nan, nan],
        [1.0, nan, 0.0, 0.0],
        [f32::INFINITY, 0.0, 0.0, 0.0],
    ];
    let setpoint = Setpoint::Attitude(AttitudeSetpoint {
        thrust: 0.5,
        roll_rad: 0.3,
        pitch_rad: -0.2,
        heading_rad: 1.0,
    });
    for [w, x, y, z] in cases {
        let state = VehicleState {
            attitude: UnitQuaternion::new_unchecked(Quaternion::new(w, x, y, z)),
            ..Default::default()
        };
        let output = CopterStabilizer::new(CopterStabilizerConfig::default())
            .stabilize(0.004, &setpoint, &state);
        assert!(
            output.moments == Vec3::zeros(),
            "({w}, {x}, {y}, {z}) gave {output:?}"
        );
    }
}

/// A NaN rate setpoint means "hold zero rate" (damping), not "no moments".
#[test]
fn nan_rate_setpoint_holds_zero_rate() {
    let state = VehicleState {
        angular_velocity_radps: Vec3::new(0.5, -0.2, 0.1), // spinning: zero-rate hold must push back
        ..Default::default()
    };
    let rate = |roll_rate| {
        Setpoint::Rate(RateSetpoint {
            thrust: 0.5,
            roll_rate,
            ..Default::default()
        })
    };
    let mut with_nan = CopterStabilizer::new(CopterStabilizerConfig::default());
    let mut with_zero = CopterStabilizer::new(CopterStabilizerConfig::default());
    let a = with_nan.stabilize(0.004, &rate(f32::NAN), &state);
    let b = with_zero.stabilize(0.004, &rate(0.0), &state);
    assert!(
        a == b,
        "NaN rate setpoint gave {a:?}, zero-rate hold gives {b:?}"
    );
    assert!(a.moments != Vec3::zeros());
}

#[test]
fn thrust_curve_raises_linear_thrust_to_the_configured_exponent() {
    // (linear thrust, exponent, expected)
    let cases = [
        (0.0, 2.0, 0.0),
        (1.0, 2.0, 1.0),
        (0.5, 2.0, 0.25),
        (0.5, 1.5, 0.353_553_4),
        (0.5, 1.0, 0.5),
    ];
    for (linear_thrust, exponent, expected) in cases {
        let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig {
            thrust_curve_exponent: exponent,
            ..Default::default()
        });
        approx::assert_relative_eq!(
            stabilizer.fit_to_thrust_curve(linear_thrust),
            expected,
            epsilon = 1e-6
        );
    }
    // Out of range saturates; NaN means motors off (input policy).
    let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
    for (linear_thrust, expected) in [(f32::NAN, 0.0), (-0.5, 0.0), (1.5, 1.0)] {
        assert_eq!(stabilizer.fit_to_thrust_curve(linear_thrust), expected);
    }
}

#[test]
fn validate_rejects_negative_or_non_finite_full_scale_rates() {
    // (scales, offending field)
    let cases = [
        (
            Vec3::new(f32::INFINITY, 3.0, 3.0),
            "full_scale_rates_radps.x",
        ),
        (Vec3::new(3.0, f32::NAN, 3.0), "full_scale_rates_radps.y"),
        (Vec3::new(3.0, 3.0, -1.0), "full_scale_rates_radps.z"),
        (
            Vec3::new(3.0, 3.0, f32::NEG_INFINITY),
            "full_scale_rates_radps.z",
        ),
    ];
    for (scales, field) in cases {
        let config = CopterStabilizerConfig {
            full_scale_rates_radps: scales,
            ..Default::default()
        };
        assert_eq!(
            config.validate().map_err(|e| e.field),
            Err(field),
            "{scales:?}"
        );
    }
    // Boundary: zero (axis disabled) and large finite scales are valid.
    for scales in [Vec3::zeros(), Vec3::new(0.0, 3.0, 100.0)] {
        let config = CopterStabilizerConfig {
            full_scale_rates_radps: scales,
            ..Default::default()
        };
        assert!(config.validate().is_ok(), "{scales:?}");
    }
}

#[test]
#[should_panic(expected = "full_scale_rates_radps.x")]
fn new_rejects_an_infinite_full_scale_rate() {
    CopterStabilizer::new(CopterStabilizerConfig {
        full_scale_rates_radps: Vec3::new(f32::INFINITY, 3.0, 3.0),
        ..Default::default()
    });
}

/// Each axis scales by its own full-scale rate: full stick on every axis at rest gives the
/// body-rate loop desired rates equal to the per-axis scales.
#[test]
fn rate_setpoint_scales_each_axis_by_its_full_scale_rate() {
    // Small enough that no moment saturates, so a mis-scaled axis would show.
    let scales = Vec3::new(0.1, 0.2, 0.4);
    let config = CopterStabilizerConfig {
        full_scale_rates_radps: scales,
        ..Default::default()
    };
    let setpoint = Setpoint::Rate(RateSetpoint {
        roll_rate: 1.0,
        pitch_rate: 1.0,
        yaw_rate: 1.0,
        thrust: 0.5,
    });

    let output =
        CopterStabilizer::new(config).stabilize(0.004, &setpoint, &VehicleState::default());
    let expected =
        BodyRateController::default().control(&config.body_rate, 0.004, scales, Vec3::zeros());

    assert_eq!(output.moments, expected);
    assert!(expected.amax() < 1.0, "saturated: {expected:?}");
}

#[test]
fn validate_rejects_a_non_positive_or_non_finite_thrust_curve_exponent() {
    for exponent in [0.0, -2.0, f32::NAN, f32::INFINITY] {
        let config = CopterStabilizerConfig {
            thrust_curve_exponent: exponent,
            ..Default::default()
        };
        assert_eq!(
            config.validate().map_err(|e| e.field),
            Err("thrust_curve_exponent"),
            "exponent {exponent}"
        );
    }
}

/// A controller config error names the loop it came from.
#[test]
fn validate_reports_the_loop_of_a_controller_error() {
    for section in ["body_rate", "attitude", "yaw"] {
        let mut config = CopterStabilizerConfig::default();
        match section {
            "body_rate" => config.body_rate.max_integral = -1.0,
            "attitude" => config.attitude.max_integral = -1.0,
            _ => config.yaw.max_integral = -1.0,
        }
        let error = config.validate().unwrap_err();
        assert_eq!(
            (error.section, error.field),
            (Some(section), "max_integral")
        );
    }
}

/// An invalid config is rejected and the current one kept.
#[test]
fn set_config_rejects_an_invalid_config_and_keeps_the_current_one() {
    let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
    let current = *stabilizer.config();
    let mut bad = [current; 3];
    bad[0].body_rate.max_integral = -1.0;
    bad[1].full_scale_rates_radps.x = f32::NAN;
    bad[2].thrust_curve_exponent = 0.0;
    for config in bad {
        assert!(stabilizer.set_config(config).is_err(), "{config:?}");
        assert_eq!(*stabilizer.config(), current);
    }
}

/// A new config applies on the next tick and keeps controller state: the output matches a
/// body-rate controller that saw the old config, then the new one.
#[test]
fn set_config_applies_next_tick_and_keeps_controller_state() {
    let setpoint = Setpoint::Rate(RateSetpoint {
        roll_rate: 0.1,
        ..Default::default()
    });
    let state = VehicleState::default();
    let old = CopterStabilizerConfig::default();
    let desired = Vec3::new(0.1 * old.full_scale_rates_radps.x, 0.0, 0.0);
    let mut stabilizer = CopterStabilizer::new(old);
    let mut reference = BodyRateController::default();

    stabilizer.stabilize(0.004, &setpoint, &state);
    reference.control(&old.body_rate, 0.004, desired, Vec3::zeros());

    let mut new = old;
    new.body_rate.roll_gains.i = 5.0;
    stabilizer.set_config(new).expect("valid config");
    reference.rescale_integral(&old.body_rate, &new.body_rate);
    let output = stabilizer.stabilize(0.004, &setpoint, &state);

    let expected = reference.control(&new.body_rate, 0.004, desired, Vec3::zeros());
    assert_eq!(output.moments, expected);
}

/// Retuning an integral gain rescales the integral so the I term carries over (bumpless),
/// except across a zero gain, where it restarts from zero.
#[test]
fn retuning_an_integral_gain_carries_the_i_term_over() {
    const DT: f32 = 0.004;
    const TICKS: usize = 10;
    // In flight: at zero thrust the integrals are cleared every tick.
    let setpoint = Setpoint::Rate(RateSetpoint {
        roll_rate: 0.1,
        thrust: 0.5,
        ..Default::default()
    });
    let state = VehicleState::default();
    // (old_i, new_i)
    let cases = [(0.1, 0.2), (0.2, 0.05), (0.1, 0.0), (0.0, 0.1)];
    for (old_i, new_i) in cases {
        let mut old = CopterStabilizerConfig::default();
        old.body_rate.roll_gains = Gains {
            p: 0.1,
            i: old_i,
            d: 0.0,
        };
        let error = 0.1 * old.full_scale_rates_radps.x;
        let mut stabilizer = CopterStabilizer::new(old);
        for _ in 0..TICKS {
            stabilizer.stabilize(DT, &setpoint, &state);
        }

        let mut new = old;
        new.body_rate.roll_gains.i = new_i;
        stabilizer.set_config(new).expect("valid config");
        let output = stabilizer.stabilize(DT, &setpoint, &state);

        let carried = if old_i == 0.0 || new_i == 0.0 {
            0.0
        } else {
            old_i * TICKS as f32 * DT * error
        };
        let expected = 0.1 * error + carried + new_i * DT * error;
        approx::assert_relative_eq!(output.moments.x, expected, max_relative = 1e-5);
    }
}

/// After priming (as `FlightLoop` does on arming), the stabilizer flies exactly like a fresh
/// one: integrals and derivative history from before are gone.
#[test]
fn prime_clears_integrals_and_derivative_history() {
    let tilted = VehicleState {
        attitude: UnitQuaternion::from_euler_angles(0.3, -0.2, 0.0),
        angular_velocity_radps: Vec3::new(0.5, -0.4, 0.3),
        ..Default::default()
    };
    // one row per setpoint kind; each tick has a non-zero error on every axis
    let setpoints = [
        Setpoint::Attitude(AttitudeSetpoint {
            heading_rad: 1.0,
            thrust: 0.5,
            ..Default::default()
        }),
        Setpoint::Rate(RateSetpoint {
            roll_rate: 0.5,
            pitch_rate: -0.5,
            yaw_rate: 0.5,
            thrust: 0.5,
        }),
    ];
    for setpoint in setpoints {
        let mut wound = setup_controller();
        for _ in 0..50 {
            wound.stabilize(0.004, &setpoint, &tilted);
        }
        wound.prime(&tilted);

        let mut fresh = setup_controller();
        fresh.prime(&tilted);

        let rearmed = wound.stabilize(0.004, &setpoint, &tilted);
        let expected = fresh.stabilize(0.004, &setpoint, &tilted);
        assert_eq!(rearmed.moments, expected.moments, "{setpoint:?}");
    }
}

/// Rate flight clears the idle attitude and heading loops, so attitude flight after it starts
/// from fresh state, not from integrals wound up before the switch.
#[test]
fn rate_flight_clears_the_attitude_and_heading_loops() {
    let tilted = VehicleState {
        attitude: UnitQuaternion::from_euler_angles(0.3, -0.2, 0.0),
        ..Default::default()
    };
    let attitude = Setpoint::Attitude(AttitudeSetpoint {
        heading_rad: 1.0,
        thrust: 0.5,
        ..Default::default()
    });
    let rate = Setpoint::Rate(RateSetpoint::default());

    let mut stabilizer = setup_controller();
    stabilizer.prime(&tilted);
    for _ in 0..50 {
        stabilizer.stabilize(0.004, &attitude, &tilted);
    }
    stabilizer.stabilize(0.004, &rate, &tilted);

    assert!(stabilizer.attitude == AttitudeController::default());
    assert!(stabilizer.yaw == YawController::default());
}

#[test]
#[should_panic(expected = "dt must be positive and finite")]
fn zero_dt_panics() {
    setup_controller().stabilize(
        0.0,
        &Setpoint::Attitude(AttitudeSetpoint::default()),
        &VehicleState::default(),
    );
}

/// The stabilizer reports what its last call clipped: an attitude setpoint beyond
/// `max_angles_rad` (tilt), thrust outside [0, 1], and each rate-loop moment at ±1.
#[test]
fn reports_what_the_last_stabilize_clipped() {
    let level = VehicleState::default();
    let spinning = VehicleState {
        angular_velocity_radps: Vec3::new(-20.0, 0.0, 20.0),
        ..Default::default()
    };
    let attitude = |roll_rad, pitch_rad, thrust| {
        Setpoint::Attitude(AttitudeSetpoint {
            roll_rad,
            pitch_rad,
            thrust,
            ..Default::default()
        })
    };
    let rate = |thrust| {
        Setpoint::Rate(RateSetpoint {
            thrust,
            ..Default::default()
        })
    };
    let flags = |tilt, thrust, moments| Saturation {
        tilt,
        thrust,
        moments,
        ..Default::default()
    };
    // (setpoint, state, expected) with the default 45° max angles
    let cases = [
        (
            attitude(0.1, 0.0, 0.5),
            &level,
            flags(false, false, [false; 3]),
        ),
        (
            attitude(1.0, 0.0, 0.5),
            &level,
            flags(true, false, [false; 3]),
        ),
        (
            attitude(0.0, -1.0, 0.5),
            &level,
            flags(true, false, [false; 3]),
        ),
        (
            attitude(f32::NAN, 0.0, 0.5),
            &level,
            flags(false, false, [false; 3]),
        ),
        (
            attitude(0.0, 0.0, 1.5),
            &level,
            flags(false, true, [false; 3]),
        ),
        (rate(-0.2), &level, flags(false, true, [false; 3])),
        (
            rate(0.5),
            &spinning,
            flags(false, false, [true, false, true]),
        ),
    ];
    for (setpoint, state, expected) in cases {
        let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
        stabilizer.prime(state);
        stabilizer.stabilize(0.004, &setpoint, state);
        assert_eq!(stabilizer.saturation(), expected, "{setpoint:?}");
    }
}

/// Resting 3° off level (the yaw rate loop on a steady 0.1 rad/s too), a setpoint with thrust
/// lets the I terms build the moments up over time; one at zero or NaN thrust (on the ground
/// at idle) clears them every tick, so the moments stay at what P alone gives.
#[test]
fn zero_thrust_keeps_the_integrals_from_winding_up() {
    let resting = VehicleState {
        attitude: UnitQuaternion::from_euler_angles(0.0, 3.0_f32.to_radians(), 0.0),
        ..Default::default()
    };
    let attitude = |thrust| {
        Setpoint::Attitude(AttitudeSetpoint {
            thrust,
            ..Default::default()
        })
    };
    let rate = |thrust| {
        Setpoint::Rate(RateSetpoint {
            yaw_rate: 0.1,
            thrust,
            ..Default::default()
        })
    };
    // (setpoint, moment axis, integrals build)
    let cases = [
        (attitude(0.5), 1, true),
        (attitude(0.0), 1, false),
        (attitude(f32::NAN), 1, false),
        (rate(0.5), 2, true),
        (rate(0.0), 2, false),
    ];
    for (setpoint, axis, builds) in cases {
        let mut stabilizer = setup_controller();
        stabilizer.prime(&resting);
        let first = stabilizer.stabilize(0.004, &setpoint, &resting).moments[axis];
        let mut last = first;
        for _ in 0..500 {
            last = stabilizer.stabilize(0.004, &setpoint, &resting).moments[axis];
        }
        let grew = (last - first).abs() > 0.01;
        assert!(grew == builds, "{setpoint:?}: {first} → {last}");
    }
}
