use core::f32::consts::{PI, TAU};

use approx::assert_relative_eq;
use assert2::check;

use super::*;
use crate::Vec3;

/// A quarter-second tick: exact in binary, so time-based rows stay clear of `f32` rounding.
const COARSE_DT: f32 = 0.25;

/// The tick for kinematic turns.
const DT: f32 = 0.004;

/// Level, still, at `airspeed_mps`.
fn level(airspeed_mps: Option<f32>) -> VehicleState {
    VehicleState {
        airspeed_mps,
        ..Default::default()
    }
}

/// A runner with a recovery that settles after two coarse ticks and times out after four.
fn runner() -> ManeuverRunner {
    ManeuverRunner::new(ManeuverConfig {
        recovery: RecoveryConfig {
            settle_s: 2.0 * COARSE_DT,
            timeout_s: 4.0 * COARSE_DT,
            ..RecoveryConfig::default()
        },
        ..ManeuverConfig::default()
    })
}

/// One tick of an ideal vehicle: it turns at the commanded body rates and snaps to a commanded
/// attitude, with no attitude at all when `blind`.
fn step(state: &mut VehicleState, command: Option<ControlInput>, dt: f32, blind: bool) {
    match command {
        Some(ControlInput::AbsoluteRate(rates)) => {
            let radps = Vec3::new(rates.roll_radps, rates.pitch_radps, rates.yaw_radps);
            state.attitude *= UnitQuaternion::from_scaled_axis(radps * dt);
            state.angular_velocity_radps = radps;
        }
        Some(ControlInput::AbsoluteAttitude(attitude)) => {
            state.attitude =
                UnitQuaternion::from_euler_angles(attitude.roll_rad, attitude.pitch_rad, 0.0);
            state.angular_velocity_radps = Vec3::zeros();
        }
        _ => {}
    }
    if blind {
        state.attitude = UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(
            f32::NAN,
            f32::NAN,
            f32::NAN,
            f32::NAN,
        ));
    }
}

/// Flies `maneuver` from `state` until the runner is done, returning each tick's phase and
/// command.
fn fly(
    runner: &mut ManeuverRunner,
    maneuver: Maneuver,
    mut state: VehicleState,
    dt: f32,
    blind: bool,
) -> Vec<(ManeuverPhase, Option<ControlInput>)> {
    runner.start(maneuver, &state);
    let mut ticks = Vec::new();
    for _ in 0..100_000 {
        let command = runner.next(dt, &state);
        ticks.push((runner.phase(), command));
        if command.is_none() {
            break;
        }
        step(&mut state, command, dt, blind);
    }
    ticks
}

/// Ticks spent rotating.
fn rotating(ticks: &[(ManeuverPhase, Option<ControlInput>)]) -> usize {
    ticks
        .iter()
        .filter(|(phase, _)| matches!(phase, ManeuverPhase::Rotation { .. }))
        .count()
}

/// The commanded rate about `axis`, or `None` for anything but a rate command.
fn rate_about(command: &Option<ControlInput>, axis: Axis) -> Option<f32> {
    let Some(ControlInput::AbsoluteRate(rates)) = command else {
        return None;
    };
    Some(match axis {
        Axis::Roll => rates.roll_radps,
        Axis::Pitch => rates.pitch_radps,
        Axis::Yaw => rates.yaw_radps,
    })
}

/// A pause of `duration_s` at cruise thrust.
fn pause(duration_s: f32) -> Segment {
    Segment::Pause {
        duration_s,
        thrust: Thrust::Cruise,
    }
}

/// Out-of-range maneuvers are refused with the segment and reason: (case, result, error).
#[test]
fn refuses_maneuvers_it_cannot_fly() {
    let rotate = |rate, end| Segment::Rotate {
        axis: Axis::Roll,
        rate,
        end,
        thrust: Thrust::Maneuver,
        path: Path::Free,
    };
    let turn = SegmentEnd::Angle { rad: PI };
    let spin = RateLaw::Constant { radps: 1.0 };
    let bad = |index, reason| Err(ManeuverError::InvalidSegment { index, reason });
    let rate_bad = "rate not finite and nonzero";
    let radius_bad = "radius not finite and nonzero";
    let duration_bad = "duration not positive and finite";
    let cases = [
        ("no segments", Maneuver::new(&[]), Err(ManeuverError::Empty)),
        (
            "17 segments",
            Maneuver::new(&[pause(1.0); MAX_SEGMENTS + 1]),
            Err(ManeuverError::TooManySegments { count: 17 }),
        ),
        ("roll at zero", Maneuver::roll(0.0), bad(0, rate_bad)),
        ("roll at NaN", Maneuver::roll(f32::NAN), bad(0, rate_bad)),
        ("roll at ∞", Maneuver::roll(f32::INFINITY), bad(0, rate_bad)),
        ("loop of 0 m", Maneuver::loop_(0.0), bad(0, radius_bad)),
        (
            "loop of -∞ m",
            Maneuver::loop_(f32::NEG_INFINITY),
            bad(0, radius_bad),
        ),
        (
            "loop of NaN m",
            Maneuver::loop_(f32::NAN),
            bad(0, radius_bad),
        ),
        (
            "doublet at zero",
            Maneuver::doublet(Axis::Pitch, 0.0, 0.5),
            bad(0, rate_bad),
        ),
        (
            "doublet of 0 s",
            Maneuver::doublet(Axis::Pitch, 1.0, 0.0),
            bad(0, duration_bad),
        ),
        (
            "doublet of NaN s",
            Maneuver::doublet(Axis::Yaw, 1.0, f32::NAN),
            bad(0, duration_bad),
        ),
        (
            "a negative angle second",
            Maneuver::new(&[pause(1.0), rotate(spin, SegmentEnd::Angle { rad: -1.0 })]),
            bad(1, "angle not positive and finite"),
        ),
        (
            "an infinite pause",
            Maneuver::new(&[pause(f32::INFINITY)]),
            bad(0, duration_bad),
        ),
        (
            "a zero pause",
            Maneuver::new(&[pause(0.0)]),
            bad(0, duration_bad),
        ),
        (
            "a circle at a constant rate",
            Maneuver::new(&[Segment::Rotate {
                axis: Axis::Pitch,
                rate: spin,
                end: turn,
                thrust: Thrust::Maneuver,
                path: Path::Circle,
            }]),
            bad(0, "circle without a radius"),
        ),
        (
            "a circle about the roll axis",
            Maneuver::new(&[Segment::Rotate {
                axis: Axis::Roll,
                rate: RateLaw::Radius { radius_m: 10.0 },
                end: turn,
                thrust: Thrust::Maneuver,
                path: Path::Circle,
            }]),
            bad(0, "circle about the roll axis"),
        ),
    ];
    for (case, result, expected) in cases {
        check!(result.map(|_| ()) == expected, "{case}");
    }
    check!(Maneuver::new(&[rotate(spin, turn)]).is_ok());
    check!(Maneuver::new(&[pause(1.0); MAX_SEGMENTS]).is_ok());
    check!(Maneuver::loop_(-25.0).is_ok(), "an outside loop");
}

/// The catalog's segments: a roll and a loop turn a full turn, a doublet flies `+rate`,
/// `-rate`, then a pause.
#[test]
fn catalog_builds_its_segments() {
    let full = SegmentEnd::Angle { rad: TAU };
    check!(
        Maneuver::roll(-2.0).expect("valid").segments()
            == [Segment::Rotate {
                axis: Axis::Roll,
                rate: RateLaw::Constant { radps: -2.0 },
                end: full,
                thrust: Thrust::Maneuver,
                path: Path::Line,
            }]
    );
    check!(
        Maneuver::loop_(-25.0).expect("valid").segments()
            == [Segment::Rotate {
                axis: Axis::Pitch,
                rate: RateLaw::Radius { radius_m: -25.0 },
                end: full,
                thrust: Thrust::Maneuver,
                path: Path::Circle,
            }]
    );
    let half = |radps| Segment::Rotate {
        axis: Axis::Yaw,
        rate: RateLaw::Constant { radps },
        end: SegmentEnd::Duration { s: 0.4 },
        thrust: Thrust::Cruise,
        path: Path::Free,
    };
    check!(
        Maneuver::doublet(Axis::Yaw, 0.5, 0.4)
            .expect("valid")
            .segments()
            == [half(0.5), half(-0.5), pause(DOUBLET_PAUSE_S)]
    );
}

/// A roll and a loop end once the attitude has turned a full turn about their axis, either
/// way (left, outside) and from any start, at the tick the commanded rate gets there:
/// (case, maneuver, start roll and pitch in rad, airspeed, axis, rate in rad/s).
#[test]
fn turns_end_after_a_full_turn_of_the_attitude() {
    let cases = [
        (
            "roll right",
            Maneuver::roll(1.5),
            (0.0, 0.0),
            None,
            Axis::Roll,
            1.5_f32,
        ),
        (
            "roll left",
            Maneuver::roll(-3.0),
            (0.0, 0.0),
            None,
            Axis::Roll,
            -3.0,
        ),
        (
            "roll from a climb",
            Maneuver::roll(2.0),
            (0.0, 0.5),
            None,
            Axis::Roll,
            2.0,
        ),
        (
            "loop of 10 m",
            Maneuver::loop_(10.0),
            (0.0, 0.0),
            Some(15.0),
            Axis::Pitch,
            1.5,
        ),
        (
            "loop banked",
            Maneuver::loop_(20.0),
            (0.4, -0.1),
            Some(15.0),
            Axis::Pitch,
            0.75,
        ),
        (
            "outside loop",
            Maneuver::loop_(-10.0),
            (0.0, 0.0),
            Some(15.0),
            Axis::Pitch,
            -1.5,
        ),
    ];
    for (case, maneuver, (roll_rad, pitch_rad), airspeed_mps, axis, radps) in cases {
        let start = VehicleState {
            attitude: UnitQuaternion::from_euler_angles(roll_rad, pitch_rad, 0.3),
            ..level(airspeed_mps)
        };
        let ticks = fly(&mut runner(), maneuver.expect("valid"), start, DT, false);
        let expected = libm::ceilf(TAU / (radps.abs() * DT)) as usize;
        let flown = rotating(&ticks);
        check!(
            flown.abs_diff(expected) <= 1,
            "{case}: {flown} ticks, {expected} expected"
        );
        check!(
            ticks
                .iter()
                .take(flown)
                .all(|(_, c)| rate_about(c, axis) == Some(radps)),
            "{case}: the rate about its axis throughout"
        );
    }
}

/// The phases run in order: idle, each segment, recovery, done, and `next` gives nothing
/// before a start or after the end.
#[test]
fn phases_run_segments_then_recovery_then_done() {
    let mut runner = runner();
    let state = level(None);
    check!(runner.phase() == ManeuverPhase::Idle);
    check!(runner.next(COARSE_DT, &state).is_none());
    check!(!runner.is_active());
    let ticks = fly(
        &mut runner,
        Maneuver::doublet(Axis::Roll, 1.0, 0.5).expect("valid"),
        state.clone(),
        COARSE_DT,
        false,
    );
    let mut phases: Vec<ManeuverPhase> = ticks.iter().map(|(phase, _)| *phase).collect();
    phases.dedup();
    check!(
        phases
            == [
                ManeuverPhase::Rotation { segment: 0 },
                ManeuverPhase::Rotation { segment: 1 },
                ManeuverPhase::Rotation { segment: 2 },
                ManeuverPhase::Recovery,
                ManeuverPhase::Done,
            ]
    );
    check!(!runner.is_active());
    check!(runner.next(COARSE_DT, &state).is_none());
}

/// A doublet flies `+rate` and `-rate` for its half width each, then holds zero rate for the
/// pause, on time alone: the same ticks with or without an attitude.
/// (case, no attitude, half width in s)
#[test]
fn doublets_fly_both_halves_then_pause_on_time() {
    for (case, blind, half_s) in [
        ("with an attitude", false, 0.5),
        ("without one", true, 0.5),
        ("a quarter-second half", false, 0.25),
    ] {
        let ticks = fly(
            &mut runner(),
            Maneuver::doublet(Axis::Pitch, 0.8, half_s).expect("valid"),
            level(Some(15.0)),
            COARSE_DT,
            blind,
        );
        let half = (half_s / COARSE_DT) as usize;
        let pause = (DOUBLET_PAUSE_S / COARSE_DT) as usize;
        let rates: Vec<Option<f32>> = ticks
            .iter()
            .map(|(_, command)| rate_about(command, Axis::Pitch))
            .take(2 * half + pause)
            .collect();
        let mut expected = vec![Some(0.8); half];
        expected.extend(vec![Some(-0.8); half]);
        expected.extend(vec![Some(0.0); pause]);
        check!(rates == expected, "{case}");
        check!(
            matches!(
                ticks[2 * half + pause].1,
                Some(ControlInput::AbsoluteAttitude(_))
            ),
            "{case}: the recovery follows the pause"
        );
    }
}

/// An angle segment that can't read its turn ends at its timeout, a loop without a speed at
/// once, and a recovery that never levels at its own timeout: (case, maneuver, airspeed, no
/// attitude, ticks rotating, ticks recovering).
#[test]
fn turns_end_without_an_attitude_or_a_speed() {
    let recovery_ticks = 4;
    // 2π at 2 rad/s, twice over: 6.28 s, 26 quarter-second ticks.
    let cases = [
        (
            "a roll, blind",
            Maneuver::roll(2.0),
            None,
            true,
            26,
            recovery_ticks,
        ),
        ("a loop, still", Maneuver::loop_(10.0), None, false, 0, 1),
        // 2π at 1.5 rad/s, twice over: 8.38 s, 34 ticks.
        (
            "a loop, blind",
            Maneuver::loop_(10.0),
            Some(15.0),
            true,
            34,
            recovery_ticks,
        ),
    ];
    for (case, maneuver, airspeed_mps, blind, rotation, recovery) in cases {
        let ticks = fly(
            &mut runner(),
            maneuver.expect("valid"),
            level(airspeed_mps),
            COARSE_DT,
            blind,
        );
        let recovering = ticks
            .iter()
            .filter(|(phase, _)| *phase == ManeuverPhase::Recovery)
            .count();
        check!(rotating(&ticks) == rotation, "{case}");
        check!(recovering == recovery, "{case}");
        check!(
            ticks.last().map(|(phase, _)| *phase) == Some(ManeuverPhase::Done),
            "{case}"
        );
    }
}

/// The recovery commands level until the wings are level within the band and the rates
/// settled for `settle_s`: (case, roll°, pitch°, body rate rad/s, ends).
#[test]
fn recovery_ends_level_and_settled() {
    let cases = [
        ("level", 0.0, 0.0, 0.0, true),
        ("inside the band", 8.0, -8.0, 0.3, true),
        ("inverted", 180.0, 0.0, 0.0, false),
        ("banked", 15.0, 0.0, 0.0, false),
        ("nose down", 0.0, -15.0, 0.0, false),
        ("still rolling", 0.0, 0.0, 0.5, false),
        ("NaN rate", 0.0, 0.0, f32::NAN, false),
    ];
    let config = RecoveryConfig {
        thrust: 0.7,
        ..RecoveryConfig::default()
    };
    for (case, roll_deg, pitch_deg, radps, ends) in cases {
        let state = VehicleState {
            attitude: UnitQuaternion::from_euler_angles(
                f32::to_radians(roll_deg),
                f32::to_radians(pitch_deg),
                1.0,
            ),
            angular_velocity_radps: Vec3::new(radps, 0.0, 0.0),
            ..Default::default()
        };
        let mut recovery = Recovery::default();
        let first = recovery.next(&config, COARSE_DT, &state);
        let Some(ControlInput::AbsoluteAttitude(command)) = first else {
            panic!("{case}: the recovery begins level");
        };
        check!((command.roll_rad, command.pitch_rad, command.thrust) == (0.0, 0.0, 0.7));
        // Settled after 0.5 s: one more coarse tick.
        check!(
            recovery.next(&config, COARSE_DT, &state).is_none() == ends,
            "{case}"
        );
    }
}

/// `abort` cuts the segments short for the recovery; it leaves a recovery alone. `start`
/// replaces a maneuver in progress.
#[test]
fn abort_recovers_and_start_restarts() {
    let mut runner = runner();
    let state = level(None);
    runner.start(Maneuver::roll(1.0).expect("valid"), &state);
    runner.next(COARSE_DT, &state);
    runner.abort();
    check!(runner.phase() == ManeuverPhase::Recovery);
    check!(matches!(
        runner.next(COARSE_DT, &state),
        Some(ControlInput::AbsoluteAttitude(_))
    ));
    runner.abort();
    check!(runner.phase() == ManeuverPhase::Recovery);

    runner.start(Maneuver::roll(-1.0).expect("valid"), &state);
    check!(runner.phase() == ManeuverPhase::Rotation { segment: 0 });
    check!(rate_about(&runner.next(COARSE_DT, &state), Axis::Roll) == Some(-1.0));
}

/// Out-of-range settings name their field: (case, corrupt, section, field).
#[test]
fn validates_config() {
    type Break = fn(&mut ManeuverConfig);
    let cases: [(Break, Option<&str>, &str); 11] = [
        (|c| c.thrust = 1.5, None, "thrust"),
        (
            |c| c.path.velocity_gain_per_s = -1.0,
            Some("path"),
            "velocity_gain_per_s",
        ),
        (|c| c.cruise_thrust = f32::NAN, None, "cruise_thrust"),
        (|c| c.timeout_factor = 0.5, None, "timeout_factor"),
        (|c| c.timeout_factor = f32::INFINITY, None, "timeout_factor"),
        (
            |c| c.recovery.pitch_rad = 2.0,
            Some("recovery"),
            "pitch_rad",
        ),
        (|c| c.recovery.thrust = -0.1, Some("recovery"), "thrust"),
        (
            |c| c.recovery.level_band_rad = 0.0,
            Some("recovery"),
            "level_band_rad",
        ),
        (
            |c| c.recovery.max_body_rate_radps = f32::NAN,
            Some("recovery"),
            "max_body_rate_radps",
        ),
        (|c| c.recovery.settle_s = -1.0, Some("recovery"), "settle_s"),
        (
            |c| c.recovery.timeout_s = 0.0,
            Some("recovery"),
            "timeout_s",
        ),
    ];
    check!(ManeuverConfig::default().validate().is_ok());
    for (corrupt, section, field) in cases {
        let mut config = ManeuverConfig::default();
        corrupt(&mut config);
        let error = config.validate().expect_err(field);
        check!((error.section, error.field) == (section, field));
    }
}

#[test]
#[should_panic(expected = "invalid ManeuverConfig")]
fn panics_on_an_invalid_config() {
    let _ = ManeuverRunner::new(ManeuverConfig {
        thrust: 2.0,
        ..ManeuverConfig::default()
    });
}

/// A loop's rate follows the speed each tick, the airspeed first, else the ground speed.
#[test]
fn loop_rate_follows_the_speed() {
    let law = RateLaw::Radius { radius_m: 20.0 };
    let moving = |airspeed_mps| VehicleState {
        velocity_mps: Vec3::new(8.0, 6.0, 0.0),
        airspeed_mps,
        ..Default::default()
    };
    assert_relative_eq!(law.radps(&moving(Some(16.0))), 0.8);
    assert_relative_eq!(law.radps(&moving(None)), 0.5);
    assert_relative_eq!(law.radps(&moving(Some(f32::NAN))), 0.5);
}

/// Each segment flies its thrust: a roll the maneuver's, a doublet and its pause cruise's.
#[test]
fn segments_fly_their_thrust() {
    let config = ManeuverConfig {
        thrust: 0.9,
        cruise_thrust: 0.4,
        ..ManeuverConfig::default()
    };
    let thrusts = |maneuver: Maneuver| -> Vec<f32> {
        fly(
            &mut ManeuverRunner::new(config),
            maneuver,
            level(None),
            COARSE_DT,
            false,
        )
        .iter()
        .filter_map(|(_, command)| match command {
            Some(ControlInput::AbsoluteRate(rates)) => Some(rates.thrust),
            _ => None,
        })
        .collect()
    };
    let roll = thrusts(Maneuver::roll(PI).expect("valid"));
    check!(!roll.is_empty() && roll.iter().all(|&thrust| thrust == 0.9));
    let doublet = thrusts(Maneuver::doublet(Axis::Roll, 1.0, 0.5).expect("valid"));
    check!(doublet == [0.4; 8]);
}

/// A roll below its line pitches up on top of its roll rate, and the same roll with the path
/// hold at zero flies its roll rate alone, as a doublet always does: (case, maneuver, path
/// gain, expected pitch rate in rad/s). 2 m below at 4/s² is 8 m/s², 8/15 rad/s at 15 m/s.
#[test]
fn rates_add_the_path_correction() {
    let start = VehicleState {
        velocity_mps: Vec3::new(15.0, 0.0, 0.0),
        airspeed_mps: Some(15.0),
        ..Default::default()
    };
    let below = VehicleState {
        position_m: Vec3::new(0.0, 0.0, -2.0),
        ..start.clone()
    };
    let cases = [
        ("a roll", Maneuver::roll(1.0), 4.0, 8.0 / 15.0),
        ("a roll, no hold", Maneuver::roll(1.0), 0.0, 0.0),
        (
            "a doublet",
            Maneuver::doublet(Axis::Roll, 1.0, 0.5),
            4.0,
            0.0,
        ),
    ];
    for (case, maneuver, gain, pitch_radps) in cases {
        let mut runner = ManeuverRunner::new(ManeuverConfig {
            path: PathConfig {
                position_gain_per_s2: gain,
                velocity_gain_per_s: gain,
                line_gravity_share: 0.0,
            },
            ..ManeuverConfig::default()
        });
        runner.start(maneuver.expect("valid"), &start);
        let command = runner.next(COARSE_DT, &below);
        check!(rate_about(&command, Axis::Roll) == Some(1.0), "{case}");
        let pitch = rate_about(&command, Axis::Pitch).expect("a rate command");
        assert_relative_eq!(pitch, pitch_radps, epsilon = 1e-5);
    }
}
