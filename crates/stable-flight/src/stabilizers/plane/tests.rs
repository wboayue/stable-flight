use approx::assert_relative_eq;
use assert2::check;
use nalgebra::{Quaternion, UnitQuaternion};

use super::*;
use crate::{AttitudeSetpoint, RateSetpoint};

const DT: f32 = 0.004;

/// Rate loop with P only (no I or FF), so a moment's sign and size read the rate target.
fn p_only() -> PlaneStabilizerConfig {
    let mut config = PlaneStabilizerConfig::default();
    let p = |p| crate::controllers::RateGains { p, i: 0.0, ff: 0.0 };
    config.rate.roll = p(0.1);
    config.rate.pitch = p(0.1);
    config.rate.yaw = p(0.1);
    config
}

fn at(roll_deg: f32, pitch_deg: f32, airspeed_mps: Option<f32>) -> VehicleState {
    VehicleState {
        attitude: UnitQuaternion::from_euler_angles(
            roll_deg.to_radians(),
            pitch_deg.to_radians(),
            0.4,
        ),
        airspeed_mps,
        ..Default::default()
    }
}

fn plane(roll_deg: f32, pitch_deg: f32) -> Setpoint {
    Setpoint::Plane(PlaneSetpoint {
        roll_rad: roll_deg.to_radians(),
        pitch_rad: pitch_deg.to_radians(),
        thrust: 0.5,
        transition: false,
    })
}

/// Attitude errors drive their own axis with the right sign; holding level gives nothing.
/// (state roll°, pitch°, setpoint roll°, pitch°, expected sign of (x, y) moments)
#[test]
fn attitude_errors_drive_their_axis() {
    let cases = [
        ((0.0, 0.0), (0.0, 0.0), (0, 0)),
        ((0.0, 0.0), (10.0, 0.0), (1, 0)),
        ((0.0, 0.0), (-10.0, 0.0), (-1, 0)),
        ((0.0, 0.0), (0.0, 5.0), (0, 1)),
        ((0.0, 10.0), (0.0, 0.0), (0, -1)),
        // Banked 5°: rolls level, with the coordinated turn's small pitch-up rate.
        ((5.0, 0.0), (0.0, 0.0), (-1, 1)),
    ];
    let sign = |v: f32| if v.abs() < 1e-6 { 0 } else { v.signum() as i32 };
    for ((roll, pitch), (sp_roll, sp_pitch), (x, y)) in cases {
        let mut stabilizer = PlaneStabilizer::new(p_only());
        let out = stabilizer.stabilize(DT, &plane(sp_roll, sp_pitch), &at(roll, pitch, Some(15.0)));
        // Banked states carry a coordinated yaw rate target; roll and pitch are what's read here.
        check!(
            (sign(out.moments.x), sign(out.moments.y)) == (x, y),
            "{roll}/{pitch} → {sp_roll}/{sp_pitch}"
        );
    }
}

/// Holding a 30° coordinated turn at trim airspeed: the rate targets are the level-turn
/// kinematics, so a vehicle already turning at them needs no correction.
#[test]
fn coordinated_turn_feeds_forward_level_turn_rates() {
    let (bank, airspeed) = (30f32.to_radians(), 15.0);
    let turn = GRAVITY * bank.tan() / airspeed; // ψ̇
    let expected = Vec3::new(0.0, turn * bank.sin(), turn * bank.cos());

    let state = at(30.0, 0.0, Some(airspeed));
    let rates = coordinated_turn_radps(&state, airspeed);
    assert_relative_eq!(rates, expected, epsilon = 1e-4);

    // Turning at those rates already: no moments.
    let mut stabilizer = PlaneStabilizer::new(p_only());
    let turning = VehicleState {
        angular_velocity_radps: expected,
        ..state
    };
    let out = stabilizer.stabilize(DT, &plane(30.0, 0.0), &turning);
    assert_relative_eq!(out.moments, Vec3::zeros(), epsilon = 1e-4);
}

/// Coordination fades from 70° to 75° of bank and is off inverted; level flight has none.
/// (bank°, fraction of the unfaded yaw rate)
#[test]
fn coordination_fades_at_steep_bank() {
    let airspeed = 15.0;
    let unfaded = |bank: f32| GRAVITY * bank.to_radians().sin() / airspeed;
    for (bank, fraction) in [
        (0.0, 1.0),
        (60.0, 1.0),
        (70.0, 1.0),
        (72.5, 0.5),
        (75.0, 0.0),
        (180.0, 0.0),
    ] {
        let rates = coordinated_turn_radps(&at(bank, 0.0, None), airspeed);
        assert_relative_eq!(rates.z, unfaded(bank) * fraction, epsilon = 1e-4);
    }
    let nan = VehicleState {
        attitude: UnitQuaternion::new_unchecked(Quaternion::new(f32::NAN, 0.0, 0.0, 0.0)),
        ..Default::default()
    };
    check!(coordinated_turn_radps(&nan, airspeed) == Vec3::zeros());
}

/// Absolute rates fly at any attitude, saturated to their limits, with no coordination:
/// inverted and vertical included.
#[test]
fn absolute_rates_fly_at_any_attitude() {
    let config = p_only();
    let roll = Setpoint::PlaneRate(BodyRateSetpoint {
        roll_radps: 10.0, // beyond the 180°/s limit
        thrust: 0.7,
        ..Default::default()
    });
    for (roll_deg, pitch_deg) in [(0.0, 0.0), (180.0, 0.0), (0.0, 89.0), (70.0, -60.0)] {
        let mut stabilizer = PlaneStabilizer::new(config);
        let out = stabilizer.stabilize(DT, &roll, &at(roll_deg, pitch_deg, Some(15.0)));
        assert_relative_eq!(
            out.moments.x,
            0.1 * config.max_body_rate_radps.x,
            epsilon = 1e-5
        );
        check!(
            out.moments.y == 0.0 && out.moments.z == 0.0,
            "{roll_deg}/{pitch_deg}"
        );
        check!(out.thrust == 0.7);
    }
}

/// Upside down with a level setpoint, the attitude loop rolls back over (no stuck zero).
#[test]
fn recovers_from_inverted() {
    let mut stabilizer = PlaneStabilizer::new(p_only());
    let out = stabilizer.stabilize(DT, &plane(0.0, 0.0), &at(180.0, 0.0, Some(15.0)));
    check!(out.moments.x.abs() > 1e-3);
}

/// Feedback scales with (trim / airspeed)², clamped to the airspeed range; no airspeed means
/// trim. (airspeed, scale)
#[test]
fn scales_with_airspeed() {
    let config = p_only();
    let base = PlaneStabilizer::new(config)
        .stabilize(DT, &plane(10.0, 0.0), &at(0.0, 0.0, Some(15.0)))
        .moments
        .x;
    for (airspeed, scale) in [
        (Some(15.0), 1.0),
        (Some(30.0), 15.0 / 25.0), // clamped at max 25
        (Some(5.0), 15.0 / 10.0),  // clamped at min 10
        (None, 1.0),
        (Some(f32::NAN), 1.0),
    ] {
        let x = PlaneStabilizer::new(config)
            .stabilize(DT, &plane(10.0, 0.0), &at(0.0, 0.0, airspeed))
            .moments
            .x;
        assert_relative_eq!(x, base * scale * scale, max_relative = 1e-4);
    }
}

/// Below min airspeed the rate integrals hold; at or above it, and without an airspeed, they
/// run. (airspeed, integrates)
#[test]
fn integrals_hold_below_min_airspeed() {
    let mut config = PlaneStabilizerConfig::default();
    config.rate.roll = crate::controllers::RateGains {
        p: 0.0,
        i: 1.0,
        ff: 0.0,
    };
    let rate = Setpoint::PlaneRate(BodyRateSetpoint {
        roll_radps: 1.0,
        ..Default::default()
    });
    for (airspeed, integrates) in [(Some(5.0), false), (Some(10.0), true), (None, true)] {
        let mut stabilizer = PlaneStabilizer::new(config);
        let state = at(0.0, 0.0, airspeed);
        stabilizer.stabilize(DT, &rate, &state);
        let out = stabilizer.stabilize(DT, &rate, &state);
        check!((out.moments.x > 0.0) == integrates, "{airspeed:?}");
    }
}

/// `prime` clears the integrals; zero thrust doesn't (gliding).
#[test]
fn integrals_survive_zero_thrust_and_clear_on_prime() {
    let mut config = PlaneStabilizerConfig::default();
    config.rate.roll = crate::controllers::RateGains {
        p: 0.0,
        i: 1.0,
        ff: 0.0,
    };
    let glide = Setpoint::PlaneRate(BodyRateSetpoint {
        roll_radps: 1.0,
        thrust: 0.0,
        ..Default::default()
    });
    let state = at(0.0, 0.0, Some(15.0));
    let mut stabilizer = PlaneStabilizer::new(config);
    stabilizer.stabilize(DT, &glide, &state);
    check!(stabilizer.stabilize(DT, &glide, &state).moments.x > 0.0);
    stabilizer.prime(&state);
    let hold = Setpoint::PlaneRate(BodyRateSetpoint::default());
    check!(stabilizer.stabilize(DT, &hold, &state).moments.x == 0.0);
}

/// Rate flight leaves the attitude flight's rate integrals as it found them: after a
/// `PlaneRate` stretch, the `Plane` setpoint that follows flies as if the stretch never
/// happened, with a neutral tick between them or an integral gain changed during it.
/// (case, the ticks after the stretch, roll I set during the stretch)
#[test]
fn rate_flight_restores_the_attitude_integrals() {
    let mut config = PlaneStabilizerConfig::default();
    config.rate.roll = crate::controllers::RateGains {
        p: 0.0,
        i: 1.0,
        ff: 0.0,
    };
    let state = at(0.0, 0.0, Some(15.0));
    let bank = plane(5.0, 0.0);
    let roll = Setpoint::PlaneRate(BodyRateSetpoint {
        roll_radps: -2.0,
        ..Default::default()
    });
    let neutral = Setpoint::Attitude(AttitudeSetpoint::default());
    let cases: [(&str, &[&Setpoint], f32); 3] = [
        ("rate then attitude", &[&bank], 1.0),
        ("a neutral tick between", &[&neutral, &bank], 1.0),
        ("I doubled during the stretch", &[&bank], 2.0),
    ];
    for (case, after, rescaled_i) in cases {
        let (mut flown, mut twin) = (PlaneStabilizer::new(config), PlaneStabilizer::new(config));
        for _ in 0..50 {
            flown.stabilize(DT, &bank, &state);
            twin.stabilize(DT, &bank, &state);
        }
        let mut rolled = 0.0;
        for _ in 0..50 {
            rolled = flown.stabilize(DT, &roll, &state).moments.x;
        }
        check!(
            rolled < 0.0,
            "{case}: the stretch winds the integral the other way"
        );
        let rescaled = PlaneStabilizerConfig {
            rate: PlaneRateControllerConfig {
                roll: crate::controllers::RateGains {
                    i: rescaled_i,
                    ..config.rate.roll
                },
                ..config.rate
            },
            ..config
        };
        check!(flown.set_config(rescaled).is_ok());
        check!(twin.set_config(rescaled).is_ok());
        let (mut got, mut want) = (ControlOutput::default(), ControlOutput::default());
        for setpoint in after {
            got = flown.stabilize(DT, setpoint, &state);
            want = twin.stabilize(DT, setpoint, &state);
        }
        check!(want.moments.x > 0.0, "{case}");
        check!(got == want, "{case}");
    }
}

/// `prime` drops integrals saved for attitude flight along with the running ones.
#[test]
fn prime_drops_the_saved_integrals() {
    let mut config = PlaneStabilizerConfig::default();
    config.rate.roll.i = 1.0;
    let state = at(0.0, 0.0, Some(15.0));
    let mut stabilizer = PlaneStabilizer::new(config);
    for _ in 0..50 {
        stabilizer.stabilize(DT, &plane(5.0, 0.0), &state);
    }
    stabilizer.stabilize(
        DT,
        &Setpoint::PlaneRate(BodyRateSetpoint::default()),
        &state,
    );
    stabilizer.prime(&state);
    check!(stabilizer.attitude_rate.is_none());
    check!(stabilizer.rate == PlaneRateController::default());
}

/// Saturation flags: bank or pitch beyond the limit (`tilt`), thrust outside [0, 1], and a
/// moment at ±1.
#[test]
fn reports_saturation() {
    let state = at(0.0, 0.0, Some(15.0));
    // (setpoint, tilt, thrust)
    let cases = [
        (plane(10.0, 5.0), false, false),
        (plane(60.0, 0.0), true, false),
        (plane(0.0, -40.0), true, false),
        (
            Setpoint::Plane(PlaneSetpoint {
                thrust: 1.2,
                ..Default::default()
            }),
            false,
            true,
        ),
    ];
    for (setpoint, tilt, thrust) in cases {
        let mut stabilizer = PlaneStabilizer::new(PlaneStabilizerConfig::default());
        stabilizer.stabilize(DT, &setpoint, &state);
        let saturation = stabilizer.saturation();
        check!(
            (saturation.tilt, saturation.thrust) == (tilt, thrust),
            "{setpoint:?}"
        );
    }
    let mut config = p_only();
    config.rate.roll.p = 100.0;
    let mut stabilizer = PlaneStabilizer::new(config);
    stabilizer.stabilize(DT, &plane(40.0, 0.0), &state);
    check!(stabilizer.saturation().moments == [true, false, false]);
}

/// A transition ramp's pitch saturates at ±90°, not `max_pitch_rad` (#388); bank keeps its
/// limit. From 60° nose up with the default 30° limit: (bank°, pitch°, transition, sign of
/// the pitch moment if read, tilt flagged)
#[test]
fn transition_ramps_fly_pitch_past_the_cap() {
    let state = at(0.0, 60.0, Some(15.0));
    let cases = [
        (0.0, 70.0, true, Some(1.0), false),
        (0.0, 70.0, false, Some(-1.0), true),
        (0.0, 100.0, true, Some(1.0), true),
        (0.0, f32::NAN, true, Some(-1.0), false),
        (60.0, 60.0, true, None, true),
    ];
    for (roll_deg, pitch_deg, transition, pitch_sign, tilt) in cases {
        let setpoint = Setpoint::Plane(PlaneSetpoint {
            roll_rad: f32::to_radians(roll_deg),
            pitch_rad: f32::to_radians(pitch_deg),
            thrust: 0.5,
            transition,
        });
        let mut stabilizer = PlaneStabilizer::new(p_only());
        let output = stabilizer.stabilize(DT, &setpoint, &state);
        let case = (roll_deg, pitch_deg, transition);
        if let Some(sign) = pitch_sign {
            check!(output.moments.y.signum() == sign, "{case:?}: {output:?}");
        }
        check!(stabilizer.saturation().tilt == tilt, "{case:?}");
    }
}

/// The thrust curve, and neutral for a copter's setpoints.
#[test]
fn thrust_curve_and_neutral_for_copter_setpoints() {
    let state = at(0.0, 0.0, Some(15.0));
    let config = PlaneStabilizerConfig {
        thrust_curve_exponent: 2.0,
        ..Default::default()
    };
    let out = PlaneStabilizer::new(config).stabilize(DT, &plane(0.0, 0.0), &state);
    assert_relative_eq!(out.thrust, 0.25);

    for setpoint in [
        Setpoint::Attitude(AttitudeSetpoint {
            thrust: 0.5,
            ..Default::default()
        }),
        Setpoint::Rate(RateSetpoint {
            thrust: 0.5,
            ..Default::default()
        }),
        Setpoint::BodyRate(BodyRateSetpoint {
            thrust: 0.5,
            ..Default::default()
        }),
    ] {
        let out =
            PlaneStabilizer::new(PlaneStabilizerConfig::default()).stabilize(DT, &setpoint, &state);
        check!(out == ControlOutput::default(), "{setpoint:?}");
    }
}

#[test]
fn validates_config() {
    type Break = fn(&mut PlaneStabilizerConfig);
    let cases: [(Break, &str); 8] = [
        (|c| c.rate.roll.p = -1.0, "roll.p"),
        (|c| c.rate.trim.y = 2.0, "trim.y"),
        (|c| c.time_constant_s.y = 0.0, "time_constant_s.y"),
        (
            |c| c.max_body_rate_radps.z = f32::NAN,
            "max_body_rate_radps.z",
        ),
        (|c| c.max_bank_rad = 4.0, "max_bank_rad"),
        (|c| c.min_airspeed_mps = 0.0, "min_airspeed_mps"),
        (|c| c.trim_airspeed_mps = f32::INFINITY, "trim_airspeed_mps"),
        (|c| c.max_airspeed_mps = -1.0, "max_airspeed_mps"),
    ];
    check!(PlaneStabilizerConfig::default().validate().is_ok());
    for (corrupt, field) in cases {
        let mut config = PlaneStabilizerConfig::default();
        corrupt(&mut config);
        check!(config.validate().map_err(|e| e.field) == Err(field));
        let mut stabilizer = PlaneStabilizer::new(PlaneStabilizerConfig::default());
        check!(stabilizer.set_config(config).is_err());
    }
}
