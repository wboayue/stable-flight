//! The step-response spec every tuned loop is held to: targets from ArduPilot's and PX4's
//! autotunes, each step both ways from a settled hover, through core's step probe.
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
//! Each loop steps as the autotune judges it (`SearchPlan::copter`'s steps; heading, which
//! isn't searched, `StepTestPlan::copter`).

use stable_flight::controllers::Gains;
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::autotune::{SearchPlan, StepTestPlan};
use stable_flight::tuning::{Measurement, ProbeConfig, ProbeStatus, StepConfig, StepMetrics};

use super::Plant;

/// ArduPilot's default autotune aggressiveness.
const AGGR: f32 = 0.1;

/// Largest overshoot, as a fraction of the step (ArduPilot: `1 + AGGR/2`).
pub const MAX_OVERSHOOT: f32 = AGGR / 2.0;

/// Largest rate-loop bounce-back, as a fraction of the step (ArduPilot: `AGGR`).
pub const MAX_RATE_BOUNCE_BACK: f32 = AGGR;

/// Longest roll and pitch rate rise, in s (PX4 `MC_AT_RISE_TIME`).
const MAX_RATE_RISE_S: f32 = 0.14;

/// Longest yaw rate rise, in s (PX4's yaw rise time).
const MAX_YAW_RATE_RISE_S: f32 = 0.2;

/// Simulated seconds a probe may run before the check gives up on it.
const PROBE_TIMEOUT_S: f32 = 5.0;

/// The loops the autotune (`StagePlan::copter_design_search`) designs, then searches; it then
/// searches yaw rate too.
pub const TUNED: [TunedLoop; 4] = [
    TunedLoop::RollRate,
    TunedLoop::PitchRate,
    TunedLoop::Roll,
    TunedLoop::Pitch,
];

/// Every copter loop.
pub const ALL: [TunedLoop; 6] = [
    TunedLoop::RollRate,
    TunedLoop::PitchRate,
    TunedLoop::YawRate,
    TunedLoop::Roll,
    TunedLoop::Pitch,
    TunedLoop::Heading,
];

/// The steps and clip allowance `target` is held to.
fn plan(target: TunedLoop) -> StepTestPlan {
    match target {
        TunedLoop::Heading => StepTestPlan::copter(target),
        _ => SearchPlan::copter(target).test,
    }
}

/// `target`'s longest rise in s and largest bounce-back; `None` where no target applies.
fn rise_and_bounce_back(target: TunedLoop) -> (Option<f32>, Option<f32>) {
    match target {
        TunedLoop::RollRate | TunedLoop::PitchRate => {
            (Some(MAX_RATE_RISE_S), Some(MAX_RATE_BOUNCE_BACK))
        }
        TunedLoop::YawRate => (Some(MAX_YAW_RATE_RISE_S), Some(MAX_RATE_BOUNCE_BACK)),
        TunedLoop::Roll | TunedLoop::Pitch | TunedLoop::Heading => (None, None),
    }
}

/// Steps each of `loops` both ways on `plant` and returns a line for each step that misses
/// the spec (empty when every step meets it), naming the loop, the step, the targets it
/// missed, and its metrics.
pub fn misses(plant: &Plant, loops: &[TunedLoop]) -> Vec<String> {
    let mut misses = Vec::new();
    for &target in loops {
        let plan = plan(target);
        for step in [plan.step, plan.step.reversed()] {
            let row = format!("{target:?} step {}", step.amplitude);
            match respond(plant, target, step) {
                None => misses.push(format!("{row}: no measurement")),
                Some(m) => {
                    let missed = missed_targets(target, &m);
                    if !missed.is_empty() {
                        misses.push(format!("{row}: {} in {m:?}", missed.join(", ")));
                    }
                }
            }
        }
    }
    misses
}

/// The targets `m`, a step of `target`, misses; empty when it meets them all.
pub fn missed_targets(target: TunedLoop, m: &StepMetrics) -> Vec<&'static str> {
    let (max_rise_s, _) = rise_and_bounce_back(target);
    missed_bands(target, m, plan(target).max_saturated_s, max_rise_s)
}

/// As [`missed_targets`], with the clip allowance `max_saturated_s` and the rise target
/// `max_rise_s` (`None`: none) of another plan's steps.
pub fn missed_bands(
    target: TunedLoop,
    m: &StepMetrics,
    max_saturated_s: f32,
    max_rise_s: Option<f32>,
) -> Vec<&'static str> {
    let (_, max_bounce_back) = rise_and_bounce_back(target);
    let checks = [
        ("overshoot", m.overshoot <= MAX_OVERSHOOT),
        ("unsettled", m.settling_time_s.is_some()),
        ("clipped", m.saturated_s <= max_saturated_s),
        (
            "rise",
            max_rise_s.is_none_or(|max_s| m.rise_time_s.is_some_and(|rise_s| rise_s <= max_s)),
        ),
        (
            "bounce-back",
            max_bounce_back.is_none_or(|max| m.bounce_back <= max),
        ),
    ];
    failed(&checks)
}

/// The names of the `checks` (name, met) not met.
pub fn failed(checks: &[(&'static str, bool)]) -> Vec<&'static str> {
    checks
        .iter()
        .filter(|(_, met)| !met)
        .map(|(name, _)| *name)
        .collect()
}

/// `target`'s response to `step` from a settled hover on `plant`; `None` if the probe didn't
/// measure it.
pub fn respond(plant: &Plant, target: TunedLoop, step: StepConfig) -> Option<StepMetrics> {
    respond_within(plant, target, step, PROBE_TIMEOUT_S)
}

/// As [`respond`], giving the probe up to `timeout_s` simulated seconds.
pub fn respond_within(
    plant: &Plant,
    target: TunedLoop,
    step: StepConfig,
    timeout_s: f32,
) -> Option<StepMetrics> {
    match super::probe(plant, target, ProbeConfig::Step(step), timeout_s) {
        Some(ProbeStatus::Done(Measurement::Step(metrics))) => Some(metrics),
        _ => None,
    }
}

/// The mean rise of `target`'s steps both ways on `plant`, in s, if both meet the spec.
pub fn rise_s(plant: &Plant, target: TunedLoop) -> Option<f32> {
    let step = plan(target).step;
    let [up, down] = [step, step.reversed()].map(|step| {
        let metrics = respond(plant, target, step)?;
        let met = missed_targets(target, &metrics).is_empty();
        met.then_some(metrics.rise_time_s).flatten()
    });
    Some((up? + down?) / 2.0)
}

/// Of `candidates` flown as `target`'s gains on `plant`, the one with the fastest [`rise_s`],
/// and that rise; `None` if none meets the spec.
pub fn fastest(
    plant: &Plant,
    target: TunedLoop,
    candidates: impl IntoIterator<Item = Gains>,
) -> Option<(f32, Gains)> {
    let mut best: Option<(f32, Gains)> = None;
    for gains in candidates {
        let Some(rise_s) = rise_s(&plant.flying(target, gains), target) else {
            continue;
        };
        if best.is_none_or(|(best_s, _)| rise_s < best_s) {
            best = Some((rise_s, gains));
        }
    }
    best
}
