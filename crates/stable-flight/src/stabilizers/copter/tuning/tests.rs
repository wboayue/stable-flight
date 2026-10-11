use assert2::{assert, check};

use super::*;
use crate::controllers::Gains;
use crate::stabilizers::copter::CopterStabilizerConfig;
use crate::tuning::Measurement;
use crate::tuning::autotune::{COPTER_ANGLE_STEP, COPTER_RATE_STEP};
use crate::tuning::plant::{Axes, PlantParams};
use crate::tuning::{ProbeAbort, RelayAbort, RelayConfig, StepConfig};
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

/// Relay for a rate loop: moment out, rad/s in.
fn rate_relay() -> RelayConfig {
    RelayConfig {
        initial_relay: 0.05,
        max_relay: 0.4,
        relay_step: 1.5,
        hysteresis: 0.002,
        min_cycle_amplitude: 0.05,
        max_cycle_amplitude: 1.0,
        max_half_cycle_s: 0.3,
        max_bias: 0.3,
        bias_gain: 0.2,
        cycles: 4,
        tolerance: 0.1,
        timeout_s: 30.0,
    }
}

/// Relay for an angle or heading loop: rad/s out, rad in.
fn angle_relay() -> RelayConfig {
    RelayConfig {
        initial_relay: 0.2,
        max_relay: 2.0,
        relay_step: 1.5,
        hysteresis: 0.001,
        min_cycle_amplitude: 0.02,
        max_cycle_amplitude: 0.5,
        max_half_cycle_s: 1.0,
        max_bias: 1.0,
        bias_gain: 0.2,
        cycles: 4,
        tolerance: 0.1,
        timeout_s: 30.0,
    }
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

/// Every target measures its loop to `Done` and hands it back, while the untested loops keep
/// flying: their axes start off level and settle during the stage.
#[test]
fn each_stage_measures_its_loop_while_the_others_hold() {
    // (target, relay)
    let cases = [
        (TunedLoop::RollRate, rate_relay()),
        (TunedLoop::PitchRate, rate_relay()),
        (TunedLoop::YawRate, rate_relay()),
        (TunedLoop::Roll, angle_relay()),
        (TunedLoop::Pitch, angle_relay()),
        (TunedLoop::Heading, angle_relay()),
    ];
    for (target, relay) in cases {
        let mut stabilizer = CopterStabilizer::new(config());
        let mut axes = axes(0.0);
        stabilizer.start_probe(target, ProbeConfig::Relay(relay));

        fly_stage(&mut stabilizer, &mut axes, &level(), 30.0);

        let status = stabilizer.probe().map(|(_, status)| status);
        check!(
            matches!(status, Some(ProbeStatus::Done(_))),
            "{target:?}: {status:?}"
        );
        for &axis in untested_axes(target) {
            let angle = axes.angles_rad()[axis];
            check!(angle.abs() < 0.01, "{target:?}: axis {axis} at {angle} rad");
        }
        check!(stabilizer.stage.as_ref().is_some_and(|s| !s.is_running()));
    }
}

/// While a rate stage runs, the tested axis's moment is the relay's `bias ± h`: it takes two
/// values, symmetric about the bias, regardless of the controller's gains.
#[test]
fn a_rate_stage_replaces_the_moment_with_the_relay() {
    let mut config = config();
    config.body_rate.roll_gains.p = 50.0; // would saturate the moment if it still flew
    let mut stabilizer = CopterStabilizer::new(config);
    let mut axes = axes(0.0);
    stabilizer.start_probe(TunedLoop::RollRate, ProbeConfig::Relay(rate_relay()));

    let mut moments: Vec<f32> = Vec::new();
    for _ in 0..50 {
        let output = tick(&mut stabilizer, &mut axes, &level());
        moments.push(output.moments.x);
    }

    let relay = rate_relay().initial_relay;
    for moment in moments {
        check!((moment.abs() - relay).abs() < 1e-6, "moment {moment}");
    }
}

/// The handoff carries the trim: after a yaw-rate stage against a steady yaw torque, the
/// loop's I term is the tuner's bias, so the next moment is `P·e + bias` with no derivative
/// kick and no integral restart.
#[test]
fn handoff_carries_the_trim_into_the_integral() {
    const YAW_TORQUE: f32 = 0.08;
    let mut stabilizer = CopterStabilizer::new(config());
    let mut axes = axes(YAW_TORQUE);
    stabilizer.start_probe(TunedLoop::YawRate, ProbeConfig::Relay(rate_relay()));

    fly_stage(&mut stabilizer, &mut axes, &level(), 30.0);
    let Some((_, ProbeStatus::Done(Measurement::Relay(result)))) = stabilizer.probe() else {
        panic!("stage didn't finish: {:?}", stabilizer.probe());
    };
    // The duty check bounds the bias to about relay × tolerance of the trim.
    check!(
        (result.bias + YAW_TORQUE).abs() < 0.02,
        "bias {}",
        result.bias
    );

    let config = config();
    let i_term = stabilizer.body_rate.integral_term(&config.body_rate, 2);
    approx::assert_relative_eq!(i_term, result.bias, max_relative = 1e-5);

    // Next tick: the yaw loop flies again from the trim.
    let state = axes.state();
    let desired_yaw_rate = {
        let mut yaw = stabilizer.yaw.clone();
        let (_, _, heading) = crate::math::euler_angles(&state.attitude);
        yaw.control(&config.yaw, DT, 0.0, heading)
    };
    let output = stabilizer.stabilize(DT, &level(), &state);
    let error = desired_yaw_rate - state.angular_velocity_radps.z;
    let gains = config.body_rate.yaw_gains;
    let expected = gains.p * error + result.bias + gains.i * DT * error;
    approx::assert_relative_eq!(output.moments.z, expected, max_relative = 1e-4);
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
    stabilizer.start_probe(TunedLoop::RollRate, ProbeConfig::Relay(rate_relay()));
    for _ in 0..100 {
        tick(&mut stabilizer, &mut axes, &level());
    }
    let attitude = stabilizer.attitude.clone();
    let yaw = stabilizer.yaw.clone();
    let others = [1, 2].map(|axis| stabilizer.body_rate.integral_term(&config.body_rate, axis));
    let bias = stabilizer
        .stage
        .as_ref()
        .and_then(|s| s.probe.trim())
        .unwrap_or_default();

    let stopped = stabilizer.stop_probe();

    check!(matches!(
        stopped,
        Some((TunedLoop::RollRate, ProbeStatus::Running(_)))
    ));
    check!(stabilizer.probe() == None);
    check!(stabilizer.attitude == attitude);
    check!(stabilizer.yaw == yaw);
    for (index, axis) in [1, 2].into_iter().enumerate() {
        check!(stabilizer.body_rate.integral_term(&config.body_rate, axis) == others[index]);
    }
    let roll_i_term = stabilizer.body_rate.integral_term(&config.body_rate, 0);
    approx::assert_relative_eq!(roll_i_term, bias, max_relative = 1e-5, epsilon = 1e-7);
}

/// Starting a stage while one runs hands the first loop back: its axis flies on its gains
/// again (the relay would output ±h).
#[test]
fn starting_a_stage_hands_back_the_running_one() {
    let mut stabilizer = CopterStabilizer::new(config());
    stabilizer.start_probe(TunedLoop::RollRate, ProbeConfig::Relay(rate_relay()));
    stabilizer.start_probe(TunedLoop::PitchRate, ProbeConfig::Relay(rate_relay()));

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
    // (target, relay, interrupted)
    let cases = [
        (TunedLoop::Roll, angle_relay(), true),
        (TunedLoop::Pitch, angle_relay(), true),
        (TunedLoop::Heading, angle_relay(), true),
        (TunedLoop::RollRate, rate_relay(), false),
        (TunedLoop::YawRate, rate_relay(), false),
    ];
    let acro = Setpoint::Rate(RateSetpoint {
        thrust: 0.5,
        ..Default::default()
    });
    for (target, relay, interrupted) in cases {
        let mut stabilizer = CopterStabilizer::new(config());
        let mut axes = axes(0.0);
        stabilizer.start_probe(target, ProbeConfig::Relay(relay));
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
    stabilizer.start_probe(TunedLoop::Roll, ProbeConfig::Relay(angle_relay()));
    stabilizer.prime(&crate::VehicleState::default());
    assert!(stabilizer.probe() == None);
}

/// The stage's bias starts at the loop's I term, so the relay is centered on the trim.
#[test]
fn a_stage_starts_from_the_loops_trim() {
    let mut stabilizer = CopterStabilizer::new(config());
    let mut axes = axes(0.08);
    for _ in 0..(5.0 / DT) as usize {
        tick(&mut stabilizer, &mut axes, &level());
    }
    let config = config();
    let trim = stabilizer.body_rate.integral_term(&config.body_rate, 2);
    check!(trim < -0.05, "yaw trim {trim}");

    stabilizer.start_probe(TunedLoop::YawRate, ProbeConfig::Relay(rate_relay()));

    let bias = stabilizer.stage.as_ref().and_then(|s| s.probe.trim());
    check!(bias == Some(trim));
}

/// An aborted stage hands its loop back at once; stopping it afterwards only clears the
/// record (no second handoff), returning the abort.
#[test]
fn an_aborted_stage_is_handed_back_and_stop_reports_it() {
    let mut stabilizer = CopterStabilizer::new(config());
    let mut axes = axes(0.0);
    let relay = RelayConfig {
        timeout_s: 0.1,
        ..rate_relay()
    };
    stabilizer.start_probe(TunedLoop::RollRate, ProbeConfig::Relay(relay));

    fly_stage(&mut stabilizer, &mut axes, &level(), 1.0);
    let aborted = Some((
        TunedLoop::RollRate,
        ProbeStatus::Aborted(ProbeAbort::Relay(RelayAbort::Timeout)),
    ));
    check!(stabilizer.probe() == aborted);

    // Flying on after the handoff moves the roll integral; a second handoff would reset it.
    for _ in 0..50 {
        tick(&mut stabilizer, &mut axes, &level());
    }
    let config = config();
    let roll_i_term = stabilizer.body_rate.integral_term(&config.body_rate, 0);

    check!(stabilizer.stop_probe() == aborted);
    check!(stabilizer.body_rate.integral_term(&config.body_rate, 0) == roll_i_term);
}

/// The probe's step for `target`: the copter presets.
fn step_for(target: TunedLoop) -> StepConfig {
    if target.is_rate() {
        COPTER_RATE_STEP
    } else {
        COPTER_ANGLE_STEP
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
    let angle = COPTER_ANGLE_STEP.amplitude;
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
                pitch_rad: -0.03 + angle,
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
                heading_rad: 3.1 + angle,
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
    let i_term = stabilizer.body_rate.integral_term(&config.body_rate, 2);
    check!(i_term.abs() > 0.01, "yaw I term {i_term}");

    stabilizer.stop_probe();

    check!(stabilizer.body_rate.integral_term(&config.body_rate, 2) == i_term);
}

/// A step's saturated time counts the ticks its loop's path clipped: a rate loop whose P pins
/// the moment, an angle loop whose rate output sits at `max_rate_radps`.
#[test]
fn a_step_counts_the_ticks_its_loop_clipped() {
    let mut stiff_rate = config();
    stiff_rate.body_rate.roll_gains.p = 50.0;
    let mut slow_angle = config();
    slow_angle.attitude.max_rate_radps = Vec2::new(0.2, 0.2);
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
