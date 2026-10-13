//! Step responses of every copter loop on `quad_x`, held to targets from ArduPilot's and PX4's
//! autotunes: each step, both ways, from a settled hover, through core's step probe.
//!
//! Sources (verified in source, #347):
//! - ArduPilot Copter-4.5 `libraries/AC_AutoTune/AC_AutoTune_Multi.cpp`, `AUTOTUNE_AGGR` 0.1
//!   (`:98`): rate P stops rising once the peak passes `1 + AGGR/2` of the target (`:918`); rate
//!   D keeps the bounce-back under `AGGR` of the peak (`:833`, `:888`); angle P stops once the
//!   angle passes `1 + AGGR/2` (`:997`).
//! - PX4 v1.15.0 `mc_autotune_attitude_control`: closed-loop rise time `MC_AT_RISE_TIME` 0.14 s
//!   (`mc_autotune_attitude_control_params.c:106`), 0.2 s on yaw
//!   (`mc_autotune_attitude_control.cpp:184-185`). PX4's rise time is its pole-placement time
//!   constant; it is compared here to the 10–90% rise, which is longer for the same response.
//! - Settling: inside the probe's 10% band before the window ends. Clipping: within the step
//!   plan's allowance, beyond which a step measures the limit, not the loop.
//!
//! Each loop steps as the autotune judges it (`SearchPlan::copter_rate`/`copter_angle`'s
//! steps; heading, which isn't searched, `StepTestPlan::copter`), except yaw rate (see
//! [`YAW_RATE_STEP_RADPS`]).

mod common;

use assert2::check;
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::autotune::{SearchPlan, StepTestPlan};
use stable_flight::tuning::{Measurement, ProbeConfig, ProbeStatus, StepConfig, StepMetrics};
use stable_flight_sim::airframes;

/// ArduPilot's default autotune aggressiveness.
const AGGR: f32 = 0.1;

/// Largest overshoot, as a fraction of the step (ArduPilot: `1 + AGGR/2`).
const MAX_OVERSHOOT: f32 = AGGR / 2.0;

/// Largest rate-loop bounce-back, as a fraction of the step (ArduPilot: `AGGR`).
const MAX_RATE_BOUNCE_BACK: f32 = AGGR;

/// Longest roll and pitch rate rise, in s (PX4 `MC_AT_RISE_TIME`).
const MAX_RATE_RISE_S: f32 = 0.14;

/// Longest yaw rate rise, in s (PX4's yaw rise time).
const MAX_YAW_RATE_RISE_S: f32 = 0.2;

/// Yaw rate step, in rad/s: `quad_x`'s yaw authority (prop drag, about 7 rad/s² per unit
/// moment) clips the standard 1 rad/s step past the allowance at any P that meets the rise
/// target, so its step is half that.
const YAW_RATE_STEP_RADPS: f32 = 0.5;

/// Simulated seconds a probe may run before the test gives up on it.
const PROBE_TIMEOUT_S: f32 = 5.0;

/// The steps and clip allowance `target` is held to.
fn plan(target: TunedLoop) -> StepTestPlan {
    match target {
        TunedLoop::RollRate | TunedLoop::PitchRate => SearchPlan::copter_rate().test,
        TunedLoop::Roll | TunedLoop::Pitch => SearchPlan::copter_angle().test,
        TunedLoop::Heading => StepTestPlan::copter(target),
        TunedLoop::YawRate => {
            let plan = StepTestPlan::copter(target);
            StepTestPlan {
                step: StepConfig {
                    amplitude: YAW_RATE_STEP_RADPS,
                    ..plan.step
                },
                ..plan
            }
        }
    }
}

/// `target`'s response to `step` from a settled `quad_x` hover.
fn respond(target: TunedLoop, step: StepConfig) -> StepMetrics {
    let status = common::probe(
        airframes::quad_x(),
        target,
        ProbeConfig::Step(step),
        PROBE_TIMEOUT_S,
    );
    let Some(ProbeStatus::Done(Measurement::Step(metrics))) = status else {
        panic!("{target:?} step {}: {status:?}", step.amplitude);
    };
    metrics
}

#[test]
fn every_loop_meets_the_autotune_targets_both_ways() {
    // (loop, longest rise s, largest bounce-back; None: no target)
    let cases = [
        (
            TunedLoop::RollRate,
            Some(MAX_RATE_RISE_S),
            Some(MAX_RATE_BOUNCE_BACK),
        ),
        (
            TunedLoop::PitchRate,
            Some(MAX_RATE_RISE_S),
            Some(MAX_RATE_BOUNCE_BACK),
        ),
        (
            TunedLoop::YawRate,
            Some(MAX_YAW_RATE_RISE_S),
            Some(MAX_RATE_BOUNCE_BACK),
        ),
        (TunedLoop::Roll, None, None),
        (TunedLoop::Pitch, None, None),
        (TunedLoop::Heading, None, None),
    ];
    for (target, max_rise_s, max_bounce_back) in cases {
        let plan = plan(target);
        for step in [plan.step, plan.step.reversed()] {
            let m = respond(target, step);
            let row = format!("{target:?} step {}: {m:?}", step.amplitude);
            check!(m.overshoot <= MAX_OVERSHOOT, "{row}");
            check!(m.settling_time_s.is_some(), "{row}");
            check!(m.saturated_s <= plan.max_saturated_s, "{row}");
            if let Some(max_rise_s) = max_rise_s {
                check!(
                    m.rise_time_s.is_some_and(|rise_s| rise_s <= max_rise_s),
                    "{row}"
                );
            }
            if let Some(max_bounce_back) = max_bounce_back {
                check!(m.bounce_back <= max_bounce_back, "{row}");
            }
        }
    }
}
