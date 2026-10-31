use assert2::{assert, check};

use super::*;
use crate::controllers::Gains;
use crate::stabilizers::copter::CopterStabilizerConfig;
use crate::tuning::Measurement;
use crate::tuning::autotune::{COPTER_IDENTIFY, COPTER_RATE_STEP, COPTER_TILT_STEP};
use crate::tuning::plant::{Axes, PlantParams};
use crate::tuning::{IdentifyProbe, ProbeAbort, StepAbort, StepConfig};
use crate::{AttitudeSetpoint, ControlOutput, RateSetpoint, Setpoint, Stabilizer, Vec2, Vec3};

/// The flight loop's tick: 250 Hz.
const DT: f32 = 0.004;

/// Untested axes start this far off level (rad), so holding them is visible.
const OFFSET_RAD: f32 = 0.1;

/// Hand-tune-like gains that fly the `Axes` sim.
fn config() -> CopterStabilizerConfig {
    let mut config = CopterStabilizerConfig::default();
    let rate = Gains {
        p: 0.5,
        i: 0.2,
        d: 0.004,
    };
    config.body_rate.roll_gains = rate;
    config.body_rate.pitch_gains = rate;
    config.body_rate.yaw_gains = Gains {
        p: 0.6,
        i: 0.6,
        d: 0.0,
    };
    let attitude = Gains {
        p: 3.0,
        i: 0.0,
        d: 0.1,
    };
    config.attitude.roll_gains = attitude;
    config.attitude.pitch_gains = attitude;
    config.attitude.max_rate_radps = Vec2::new(3.0, 3.0);
    config.yaw.gains = Gains {
        p: 2.5,
        i: 0.0,
        d: 0.06,
    };
    config
}

fn level() -> Setpoint {
    Setpoint::Attitude(AttitudeSetpoint {
        thrust: 0.5,
        ..Default::default()
    })
}

fn axes(yaw_offset: f32) -> Axes {
    let yaw = PlantParams {
        offset: yaw_offset,
        ..PlantParams::rate_axis()
    };
    Axes::new(
        [PlantParams::rate_axis(), PlantParams::rate_axis(), yaw],
        Vec3::new(OFFSET_RAD, OFFSET_RAD, OFFSET_RAD),
        DT,
    )
}

/// One tick: the stabilizer on the sim's state, its moments applied.
fn tick(stabilizer: &mut CopterStabilizer, axes: &mut Axes, setpoint: &Setpoint) -> ControlOutput {
    let state = axes.state();
    let output = stabilizer.stabilize(DT, setpoint, &state);
    axes.step(output.moments);
    output
}

/// Flies `setpoint` until the stage stops running or `max_s` passes.
fn fly_stage(stabilizer: &mut CopterStabilizer, axes: &mut Axes, setpoint: &Setpoint, max_s: f32) {
    for _ in 0..(max_s / DT) as usize {
        tick(stabilizer, axes, setpoint);
        if !stabilizer
            .probe()
            .is_some_and(|(_, status)| status.is_running())
        {
            break;
        }
    }
}

/// The angles each target leaves to its controllers: roll and pitch, plus heading for every
/// target but the heading loop's own.
fn untested_axes(target: TunedLoop) -> &'static [usize] {
    match target {
        TunedLoop::RollRate | TunedLoop::Roll => &[1, 2],
        TunedLoop::PitchRate | TunedLoop::Pitch => &[0, 2],
        TunedLoop::YawRate | TunedLoop::Heading => &[0, 1],
    }
}

/// Stopping a stage hands back only its loop: the other loops' state is untouched.
#[test]
fn stop_hands_back_only_the_tested_loop() {
    let config = config();
    let mut stabilizer = CopterStabilizer::new(config);
    let mut axes = axes(0.0);
    for _ in 0..100 {
        tick(&mut stabilizer, &mut axes, &level());
    }
    stabilizer.start_probe(TunedLoop::RollRate, ProbeConfig::Step(COPTER_RATE_STEP));
    for _ in 0..20 {
        tick(&mut stabilizer, &mut axes, &level());
    }
    let attitude = stabilizer.attitude.clone();
    let yaw = stabilizer.yaw.clone();
    let i_terms = [0, 1, 2].map(|axis| stabilizer.body_rate.integral_term(axis));

    let stopped = stabilizer.stop_probe();

    check!(matches!(
        stopped,
        Some((TunedLoop::RollRate, ProbeStatus::Running(_)))
    ));
    check!(stabilizer.probe() == None);
    check!(stabilizer.attitude == attitude);
    check!(stabilizer.yaw == yaw);
    for (axis, i_term) in i_terms.into_iter().enumerate() {
        check!(stabilizer.body_rate.integral_term(axis) == i_term);
    }
}

/// Starting a stage while one runs hands the first loop back: its axis flies its own
/// setpoint again (the step would ask for 1 rad/s more).
#[test]
fn starting_a_stage_hands_back_the_running_one() {
    let mut stabilizer = CopterStabilizer::new(config());
    stabilizer.start_probe(TunedLoop::RollRate, ProbeConfig::Step(COPTER_RATE_STEP));
    stabilizer.start_probe(TunedLoop::PitchRate, ProbeConfig::Step(COPTER_RATE_STEP));

    let mut state = crate::VehicleState::default();
    state.angular_velocity_radps.x = 0.5;
    let output = stabilizer.stabilize(DT, &Setpoint::Rate(RateSetpoint::default()), &state);

    check!(matches!(
        stabilizer.probe(),
        Some((TunedLoop::PitchRate, _))
    ));
    let gains = config().body_rate.roll_gains;
    let expected = -gains.p * 0.5 - gains.i * DT * 0.5;
    approx::assert_relative_eq!(output.moments.x, expected, max_relative = 1e-5);
}

/// A rate tick ends an angle or heading stage as interrupted (its loop is idle in acro);
/// a rate stage keeps running under rate ticks.
#[test]
fn rate_ticks_interrupt_angle_stages_only() {
    // (target, interrupted)
    let cases = [
        (TunedLoop::Roll, true),
        (TunedLoop::Pitch, true),
        (TunedLoop::Heading, true),
        (TunedLoop::RollRate, false),
        (TunedLoop::YawRate, false),
    ];
    let acro = Setpoint::Rate(RateSetpoint {
        thrust: 0.5,
        ..Default::default()
    });
    for (target, interrupted) in cases {
        let mut stabilizer = CopterStabilizer::new(config());
        let mut axes = axes(0.0);
        stabilizer.start_probe(target, ProbeConfig::Step(step_for(target)));
        for _ in 0..10 {
            tick(&mut stabilizer, &mut axes, &level());
        }

        tick(&mut stabilizer, &mut axes, &acro);
        tick(&mut stabilizer, &mut axes, &level());

        let status = stabilizer.probe().map(|(_, status)| status);
        if interrupted {
            check!(status == Some(ProbeStatus::Interrupted), "{target:?}");
        } else {
            check!(
                status.is_some_and(|status| status.is_running()),
                "{target:?}"
            );
        }
    }
}

/// Priming (re-arming) ends any stage.
#[test]
fn prime_ends_the_stage() {
    let mut stabilizer = CopterStabilizer::new(config());
    stabilizer.start_probe(TunedLoop::Roll, ProbeConfig::Step(COPTER_TILT_STEP));
    stabilizer.prime(&crate::VehicleState::default());
    assert!(stabilizer.probe() == None);
}

/// An aborted stage hands its loop back at once; stopping it afterwards returns the abort and
/// clears the record.
#[test]
fn an_aborted_stage_is_handed_back_and_stop_reports_it() {
    let mut stabilizer = CopterStabilizer::new(config());
    stabilizer.start_probe(TunedLoop::RollRate, ProbeConfig::Step(COPTER_RATE_STEP));
    let rate = Setpoint::Rate(RateSetpoint::default());
    stabilizer.stabilize(DT, &rate, &crate::VehicleState::default());

    // Rolling left while the step asks for right: past half a step the wrong way, it aborts.
    let mut reversed = crate::VehicleState::default();
    reversed.angular_velocity_radps.x = -COPTER_RATE_STEP.amplitude;
    for _ in 0..50 {
        stabilizer.stabilize(DT, &rate, &reversed);
    }
    let aborted = Some((
        TunedLoop::RollRate,
        ProbeStatus::Aborted(ProbeAbort::Step(StepAbort::Reversed)),
    ));
    check!(stabilizer.probe() == aborted);

    check!(stabilizer.stop_probe() == aborted);
    check!(stabilizer.probe() == None);
}

/// The probe's step for `target`: the copter presets.
fn step_for(target: TunedLoop) -> StepConfig {
    if target.is_rate() {
        COPTER_RATE_STEP
    } else {
        COPTER_TILT_STEP
    }
}

/// A step replaces the loop's setpoint with the measurement plus the step, before its
/// controller runs: the first tick's moments match a stabilizer without a probe asked for that
/// setpoint directly (the heading wraps past π).
#[test]
fn a_step_replaces_the_loops_setpoint() {
    let config = config();
    let full_scale = config.full_scale_rates_radps;
    let level = AttitudeSetpoint {
        thrust: 0.5,
        ..Default::default()
    };
    let state = crate::VehicleState {
        angular_velocity_radps: Vec3::new(0.2, -0.1, 0.05),
        attitude: nalgebra::UnitQuaternion::from_euler_angles(0.05, -0.03, 3.1),
        ..Default::default()
    };
    let rate = COPTER_RATE_STEP.amplitude;
    let angle = COPTER_TILT_STEP.amplitude;
    // The attitude as the angle loops measure it: setpoint minus their error.
    let error = crate::math::attitude_error(&state.attitude, 0.0, 0.0, 3.1);
    let (measured_pitch, measured_heading) = (-error.y, 3.1 - error.z);
    // Within second-order terms of the Euler angles it measured before.
    approx::assert_abs_diff_eq!(measured_pitch, -0.03, epsilon = 2e-3);
    approx::assert_abs_diff_eq!(measured_heading, 3.1, epsilon = 2e-3);
    // (target, the tick's setpoint, the setpoint a probe-free stabilizer is asked for)
    let cases = [
        (
            TunedLoop::RollRate,
            Setpoint::Rate(RateSetpoint::default()),
            Setpoint::Rate(RateSetpoint {
                roll_rate: (0.2 + rate) / full_scale.x,
                ..Default::default()
            }),
        ),
        (
            TunedLoop::YawRate,
            Setpoint::Rate(RateSetpoint::default()),
            Setpoint::Rate(RateSetpoint {
                yaw_rate: (0.05 + rate) / full_scale.z,
                ..Default::default()
            }),
        ),
        (
            TunedLoop::Pitch,
            Setpoint::Attitude(level),
            Setpoint::Attitude(AttitudeSetpoint {
                pitch_rad: measured_pitch + angle,
                ..level
            }),
        ),
        (
            TunedLoop::Heading,
            Setpoint::Attitude(AttitudeSetpoint {
                heading_rad: 3.1,
                ..level
            }),
            Setpoint::Attitude(AttitudeSetpoint {
                heading_rad: measured_heading + angle,
                ..level
            }),
        ),
    ];
    for (target, setpoint, direct) in cases {
        let mut stabilizer = CopterStabilizer::new(config);
        stabilizer.start_probe(target, ProbeConfig::Step(step_for(target)));
        let mut reference = CopterStabilizer::new(config);

        let output = stabilizer.stabilize(DT, &setpoint, &state);
        let expected = reference.stabilize(DT, &direct, &state);

        for axis in 0..3 {
            approx::assert_relative_eq!(
                output.moments[axis],
                expected.moments[axis],
                max_relative = 1e-4,
                epsilon = 1e-6
            );
        }
        check!(
            stabilizer
                .probe()
                .is_some_and(|(_, status)| status.is_running())
        );
    }
}

/// Every target's step runs to `Done` on the sim and follows the step (hand-tune-like gains),
/// while the untested axes hold.
#[test]
fn each_step_measures_its_loops_response() {
    let targets = [
        TunedLoop::RollRate,
        TunedLoop::PitchRate,
        TunedLoop::Roll,
        TunedLoop::Pitch,
        TunedLoop::Heading,
    ];
    for target in targets {
        let mut stabilizer = CopterStabilizer::new(config());
        let mut axes = axes(0.0);
        for _ in 0..(3.0 / DT) as usize {
            tick(&mut stabilizer, &mut axes, &level());
        }
        stabilizer.start_probe(target, ProbeConfig::Step(step_for(target)));

        fly_stage(&mut stabilizer, &mut axes, &level(), 2.0);

        let Some((_, ProbeStatus::Done(Measurement::Step(metrics)))) = stabilizer.probe() else {
            panic!("{target:?}: {:?}", stabilizer.probe());
        };
        check!(
            (metrics.final_value - 1.0).abs() < 0.15,
            "{target:?}: {metrics:?}"
        );
        // The heading loop (P 2.5) is still rising at the end of the angle window.
        if target != TunedLoop::Heading {
            check!(metrics.rise_time_s.is_some(), "{target:?}: {metrics:?}");
        }
        for &axis in untested_axes(target) {
            let angle = axes.angles_rad()[axis];
            check!(angle.abs() < 0.02, "{target:?}: axis {axis} at {angle} rad");
        }
    }
}

/// Stopping a step mid-way hands the loop back with its I term as it was: a step has no trim
/// to seed.
#[test]
fn a_step_hands_back_keeping_the_integral() {
    let config = config();
    let mut stabilizer = CopterStabilizer::new(config);
    let mut axes = axes(0.08);
    for _ in 0..(3.0 / DT) as usize {
        tick(&mut stabilizer, &mut axes, &level());
    }
    stabilizer.start_probe(TunedLoop::YawRate, ProbeConfig::Step(COPTER_RATE_STEP));
    for _ in 0..10 {
        tick(&mut stabilizer, &mut axes, &level());
    }
    let i_term = stabilizer.body_rate.integral_term(2);
    check!(i_term.abs() > 0.01, "yaw I term {i_term}");

    stabilizer.stop_probe();

    check!(stabilizer.body_rate.integral_term(2) == i_term);
}

/// The angle and heading loops' integrals (roll, pitch, heading), in rad·s.
fn attitude_integrals(stabilizer: &CopterStabilizer) -> Vec3 {
    let angles = stabilizer.attitude.integral();
    Vec3::new(angles.x, angles.y, stabilizer.yaw.integral())
}

/// A rate step holds the integral of the angle or heading loop over it, which the step tilts
/// or turns away from its setpoint (#579); an identification, which adds to that loop's output,
/// leaves it integrating.
#[test]
fn a_rate_step_holds_the_integral_of_the_loop_over_it() {
    let mut config = config();
    for gains in [
        &mut config.attitude.roll_gains,
        &mut config.attitude.pitch_gains,
        &mut config.yaw.gains,
    ] {
        gains.i = 0.5;
    }
    let step = ProbeConfig::Step(COPTER_RATE_STEP);
    let identify = ProbeConfig::Identify(COPTER_IDENTIFY);
    // (target, probe, whether the loop over it keeps its integral)
    let cases = [
        (TunedLoop::RollRate, step, true),
        (TunedLoop::PitchRate, step, true),
        (TunedLoop::YawRate, step, true),
        (TunedLoop::RollRate, identify, false),
    ];
    for (target, probe, held) in cases {
        let mut stabilizer = CopterStabilizer::new(config);
        let mut axes = axes(0.0);
        for _ in 0..(3.0 / DT) as usize {
            tick(&mut stabilizer, &mut axes, &level());
        }
        let axis = match target.location() {
            Location::Rate(axis) => axis,
            other => unreachable!("rate targets only: {other:?}"),
        };
        let before = attitude_integrals(&stabilizer)[axis];
        stabilizer.start_probe(target, probe);

        // Inside both probes: the step lasts 0.25 s, the identification 3 s.
        for _ in 0..(0.2 / DT) as usize {
            tick(&mut stabilizer, &mut axes, &level());
        }
        check!(
            stabilizer
                .probe()
                .is_some_and(|(_, status)| status.is_running())
        );

        let after = attitude_integrals(&stabilizer)[axis];
        check!(
            (after == before) == held,
            "{target:?} {probe:?}: {before} → {after}"
        );
    }
}

/// A step's saturated time counts the ticks its loop's path clipped: a rate loop whose P pins
/// the moment, an angle loop whose rate output sits at `max_rate_radps`.
#[test]
fn a_step_counts_the_ticks_its_loop_clipped() {
    let mut stiff_rate = config();
    stiff_rate.body_rate.roll_gains.p = 50.0;
    let mut slow_angle = config();
    slow_angle.attitude.max_rate_radps = Vec2::new(0.1, 0.1);
    // (config, target, clipped for at least this long, in s)
    let cases = [
        (config(), TunedLoop::RollRate, 0.0),
        (stiff_rate, TunedLoop::RollRate, 0.1),
        (slow_angle, TunedLoop::Roll, 0.3),
    ];
    for (config, target, min_saturated_s) in cases {
        let mut stabilizer = CopterStabilizer::new(config);
        let mut axes = axes(0.0);
        for _ in 0..(3.0 / DT) as usize {
            tick(&mut stabilizer, &mut axes, &level());
        }
        stabilizer.start_probe(target, ProbeConfig::Step(step_for(target)));

        fly_stage(&mut stabilizer, &mut axes, &level(), 2.0);

        let saturated_s = match stabilizer.probe() {
            Some((_, ProbeStatus::Done(Measurement::Step(metrics)))) => metrics.saturated_s,
            other => panic!("{target:?}: {other:?}"),
        };
        if min_saturated_s == 0.0 {
            // The D kick clips at most the first couple of ticks.
            check!(saturated_s <= 2.0 * DT, "{target:?}: {saturated_s}");
        } else {
            check!(saturated_s >= min_saturated_s, "{target:?}: {saturated_s}");
        }
    }
}

/// An identification on a rate loop adds its square wave to the rate setpoint the angle loop
/// sets, so the angle loop still holds the attitude (#561): tilted, the roll moment moves from
/// the unprobed one by the rate P and I on the wave's first offset alone.
#[test]
fn a_rate_identification_adds_to_the_angle_loops_rate_setpoint() {
    let state = crate::VehicleState {
        attitude: nalgebra::UnitQuaternion::from_euler_angles(OFFSET_RAD, 0.0, 0.0),
        ..Default::default()
    };
    let mut unprobed = CopterStabilizer::new(config());
    let mut probed = CopterStabilizer::new(config());
    probed.start_probe(TunedLoop::RollRate, ProbeConfig::Identify(COPTER_IDENTIFY));

    let held = unprobed.stabilize(DT, &level(), &state).moments.x;
    let excited = probed.stabilize(DT, &level(), &state).moments.x;

    let offset_radps = IdentifyProbe::new(COPTER_IDENTIFY).step(DT, 0.0, 0.0);
    let gains = config().body_rate.roll_gains;
    let expected = held + gains.p * offset_radps + gains.i * DT * offset_radps;
    approx::assert_relative_eq!(excited, expected, max_relative = 1e-5);
}
