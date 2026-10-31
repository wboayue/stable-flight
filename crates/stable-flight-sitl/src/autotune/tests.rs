use assert2::check;
use stable_flight::commanders::RouteEnd;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::stabilizers::copter::{CopterStabilizerConfig, TunedLoop};
use stable_flight::tuning::autotune::{AutotuneAbort, AutotuneEvent, Outcome};
use stable_flight::tuning::{Measurement, RateModel, StepMetrics, StepSummary};
use stable_flight::view::tailsitter_rotation;
use stable_flight::{ControlInput, Vec3, VehicleState, WaypointCommand};

use super::*;

/// A script hovering at 5 m over its start, holding there as the autotune examples' do.
fn hover_script() -> ScriptCommander {
    let hover = WaypointCommand {
        altitude_m: 5.0,
        ..Default::default()
    };
    ScriptCommander::from_waypoints(vec![hover])
        .expect("one waypoint")
        .with_route_end(RouteEnd::Hold)
}

/// At rest on the ground at the origin.
fn at_rest(altitude_m: f32) -> VehicleState {
    VehicleState {
        position_m: Vec3::new(0.0, 0.0, altitude_m),
        ..Default::default()
    }
}

/// The commander flies the script until `land` is set, then lands where it is and disarms
/// once the touchdown detector sees the vehicle down.
#[test]
fn then_land_flies_the_script_then_lands_and_disarms() {
    let (mut commander, switch) = ThenLand::new(hover_script());
    let state = at_rest(5.0);

    let scripted = commander.next_command(0.004, &state);
    check!(!matches!(scripted.input, ControlInput::Land(_)));

    switch.land();
    check!(switch.is_landing());
    let landing = commander.next_command(0.004, &state);
    check!(matches!(landing.input, ControlInput::Land(_)));
    check!(landing.arm);
    // The touchdown detector counts a stop only after the descent has begun.
    let sinking = VehicleState {
        velocity_mps: Vec3::new(0.0, 0.0, -0.5),
        ..state
    };
    check!(commander.next_command(0.004, &sinking).arm);
    let disarmed = (0..500).any(|_| !commander.next_command(0.004, &state).arm);
    check!(disarmed, "still armed 2 s after stopping");
}

/// The autotune starts once the script reaches its hover point and the vehicle is within
/// 0.5 m of its altitude: (altitude, at the hover point).
#[test]
fn the_hover_point_needs_the_last_waypoint_and_its_altitude() {
    let (mut commander, _) = ThenLand::new(hover_script());
    check!(
        !commander.at_hover_point(&at_rest(5.0)),
        "before the script ran"
    );
    for _ in 0..10 {
        commander.next_command(0.004, &at_rest(5.0));
    }
    for (altitude_m, at) in [(5.0, true), (5.4, true), (5.6, false), (4.0, false)] {
        check!(
            commander.at_hover_point(&at_rest(altitude_m)) == at,
            "altitude {altitude_m}"
        );
    }
}

/// Inverted is a tilt past 90° in the probed frame: (frame, roll and pitch in rad of that
/// frame, inverted). Nose-up hover reads level in a tailsitter's view, though its body frame
/// sits at 90°, where a little pitch either way would flip a body-frame check.
#[test]
fn inverted_is_a_tilt_past_ninety_degrees_in_the_frame() {
    let body = UnitQuaternion::identity();
    let view = tailsitter_rotation();
    for (name, frame, roll_rad, pitch_rad, inverted) in [
        ("body", body, 0.0, 0.0, false),
        ("body", body, 1.5, 0.0, false),
        ("body", body, 1.7, 0.0, true),
        ("body", body, 3.1, 0.0, true),
        ("view", view, 0.0, 0.0, false),
        ("view", view, 0.0, 0.05, false),
        ("view", view, 0.0, -0.05, false),
        ("view", view, 1.5, 0.0, false),
        ("view", view, 1.7, 0.0, true),
        ("view", view, 3.1, 0.0, true),
    ] {
        // The attitude whose `frame` is rolled and pitched by these angles.
        let state = VehicleState {
            attitude: UnitQuaternion::from_euler_angles(roll_rad, pitch_rad, 0.0) * frame.inverse(),
            ..Default::default()
        };
        check!(
            is_inverted(&state, &frame) == inverted,
            "{name} roll {roll_rad} pitch {pitch_rad}"
        );
    }
}

/// A step and a summary read in ms and fractions of the step, `-` where there's no sample.
#[test]
fn formats_steps_and_summaries() {
    let metrics = StepMetrics {
        amplitude: -0.5,
        overshoot: 0.04,
        rise_time_s: Some(0.08),
        settling_time_s: None,
        bounce_back: 0.01,
        final_value: 0.98,
        saturated_s: 0.012,
        ..Default::default()
    };
    check!(
        format_step(&metrics)
            == "-0.500: overshoot 0.04, rise 80 ms, settle -, bounce 0.01, final 0.98, clipped 12 ms"
    );
    let mut summary = StepSummary::default();
    summary.add(&metrics);
    check!(
        format_summary(&summary)
            == "n 1: overshoot 0.04 [0.04-0.04] (+ -, - 0.04), rise 80.00 [80.00-80.00] ms, \
                settle - ms, bounce 0.01 [0.01-0.01], final 0.98 [0.98-0.98]; not risen 0, \
                not settled 1"
    );
}

/// The snippet carries every tuned loop's gains, the yaw integral limit, and its source.
#[test]
fn the_snippet_carries_every_tuned_gain() {
    let mut config = CopterStabilizerConfig::default();
    TunedLoop::RollRate.gains_mut(&mut config).p = 0.1234;
    TunedLoop::Pitch.gains_mut(&mut config).p = 6.5;
    let snippet = airframe_snippet(&config, "copter-autotune-test");
    check!(snippet.starts_with("// copter-autotune-test\n"));
    check!(snippet.contains("roll_gains: Gains { p: 0.1234,"));
    check!(snippet.contains("pitch_gains: Gains { p: 6.5000,"));
    check!(snippet.contains(&format!(
        "yaw_max_integral: {:.1}",
        config.body_rate.yaw_max_integral
    )));
}

/// Every event logs without panicking, including the completion's snippet.
#[test]
fn logs_every_event() {
    let config = CopterStabilizerConfig::default();
    let gains = TunedLoop::RollRate.gains(&config);
    let steps = StepSummary::default();
    let model = RateModel {
        rate_weights: [1.0, 0.0],
        moment_weights: [0.1, 0.0, 0.0],
        delay_ticks: 3,
        dt_s: 0.004,
    };
    let target = TunedLoop::RollRate;
    let events = [
        AutotuneEvent::StageStarted(target),
        AutotuneEvent::Measured {
            target,
            measurement: Measurement::Step(StepMetrics::default()),
        },
        AutotuneEvent::Measured {
            target,
            measurement: Measurement::Identify(model),
        },
        AutotuneEvent::Stalled {
            target,
            measurement: Some(Measurement::Step(StepMetrics::default())),
        },
        AutotuneEvent::Stalled {
            target,
            measurement: None,
        },
        AutotuneEvent::Tried {
            target,
            gains,
            passed: true,
            steps,
        },
        AutotuneEvent::StageDone {
            target,
            gains,
            outcome: Outcome::Tuned,
            steps: Some(steps),
            baseline: Some(steps),
        },
        AutotuneEvent::Lowered { target, gains },
        AutotuneEvent::Unsettled { target, gains },
        AutotuneEvent::StageFailed {
            target,
            reason: AutotuneAbort::NotSettled,
        },
        AutotuneEvent::Complete,
        AutotuneEvent::Reverted,
    ];
    for event in &events {
        log_event(event, &config, "copter-autotune-test");
    }
}
