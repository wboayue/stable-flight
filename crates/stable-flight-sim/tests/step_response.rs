//! Step responses held to the autotune targets in `common::spec`: every loop on `quad_x`, and
//! the tuned loops on every airframe.

mod common;

use assert2::check;
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::autotune::{COPTER_TILT_STEP, SearchPlan};
use stable_flight::tuning::{
    Measurement, ProbeConfig, ProbeStatus, StepConfig, StepMetrics, StepSummary,
};
use stable_flight_sim::{Wind, airframes};

use common::robustness::{Set, perturbations};
use common::{Plant, spec};

#[test]
fn every_quad_x_loop_meets_the_autotune_targets_both_ways() {
    let misses = spec::misses(&Plant::clean(airframes::quad_x()), &spec::ALL);
    check!(misses.is_empty(), "{misses:#?}");
}

#[test]
fn every_airframes_tuned_loops_meet_the_autotune_targets() {
    for (name, airframe) in airframes::all() {
        let misses = spec::misses(&Plant::clean(airframe), &spec::TUNED);
        check!(misses.is_empty(), "{name}: {misses:#?}");
    }
}

/// Rate I as a share of P that holds a CG offset's moment slowly enough for an angle step to
/// end past its target toward the offset (6% at angle P 6) and short of it the other way; at
/// 0.3 both end on it.
const SLOW_RATE_I_PER_P: f32 = 0.1;

/// Angle steps that a CG offset holds past their target and short of it, with no ringing,
/// pass the angle search's criterion, which judges overshoot from where each step ended: the
/// one held past alone too, though judged from the step it reads past the 0.05 limit (#564). On the Titan Cobra
/// in RealFlight such steps walked the angle search's P down to 1.3.
#[test]
fn angle_steps_held_past_their_target_pass_the_angle_criterion() {
    let mut airframe = airframes::quad_x();
    for target in [TunedLoop::RollRate, TunedLoop::PitchRate] {
        let gains = target.gains_mut(&mut airframe.stabilizer);
        gains.i = SLOW_RATE_I_PER_P * gains.p;
    }
    TunedLoop::Pitch.gains_mut(&mut airframe.stabilizer).p = 6.0;
    let plant = Plant {
        airframe,
        wind: Wind::default(),
        perturbations: perturbations(Set::CgOffset, &airframe),
    };
    let steps = [1.0, -1.0].map(|sign| {
        let config = ProbeConfig::Step(StepConfig {
            amplitude: sign * COPTER_TILT_STEP.amplitude,
            ..COPTER_TILT_STEP
        });
        let timeout_s = 2.0 * COPTER_TILT_STEP.duration_s;
        let Some(ProbeStatus::Done(Measurement::Step(metrics))) =
            common::probe(&plant, TunedLoop::Pitch, config, timeout_s)
        else {
            panic!("step {sign}: no result");
        };
        check!(metrics.bounce_back < 0.01, "step {sign}: {metrics:?}");
        metrics
    });
    let summary_of = |steps: &[StepMetrics]| {
        let mut summary = StepSummary::default();
        steps.iter().for_each(|metrics| summary.add(metrics));
        summary
    };
    let criterion = SearchPlan::copter_angle().criterion;
    let [held_past, held_short] = if steps[0].final_value > steps[1].final_value {
        steps
    } else {
        [steps[1], steps[0]]
    };
    check!(held_past.overshoot > 0.05, "{held_past:?}");
    check!(held_short.final_value < 0.97, "{held_short:?}");
    check!(criterion.passes(&summary_of(&[held_past])), "{held_past:?}");
    check!(criterion.passes(&summary_of(&steps)), "{steps:?}");
}

/// A step that meets every target: no overshoot, quick rise, settled, no bounce or clipping.
fn clean_step() -> StepMetrics {
    StepMetrics {
        amplitude: 1.0,
        peak: 1.0,
        rise_time_s: Some(0.05),
        settling_time_s: Some(0.08),
        final_value: 1.0,
        ..Default::default()
    }
}

/// An edit to [`clean_step`].
type Change = fn(&mut StepMetrics);

/// Each target flags just past its band, and a rise or bounce-back target only on the loops
/// that have one.
#[test]
fn missed_targets_flags_each_band() {
    // (loop, change to a clean step, targets missed)
    let cases: [(TunedLoop, Change, &[&str]); 9] = [
        (TunedLoop::RollRate, |_| {}, &[]),
        (TunedLoop::RollRate, |m| m.overshoot = 0.05, &[]),
        (TunedLoop::RollRate, |m| m.overshoot = 0.051, &["overshoot"]),
        (
            TunedLoop::RollRate,
            |m| m.settling_time_s = None,
            &["unsettled"],
        ),
        (TunedLoop::RollRate, |m| m.saturated_s = 1.0, &["clipped"]),
        (
            TunedLoop::RollRate,
            |m| m.rise_time_s = Some(0.141),
            &["rise"],
        ),
        (TunedLoop::RollRate, |m| m.rise_time_s = None, &["rise"]),
        (
            TunedLoop::RollRate,
            |m| m.bounce_back = 0.11,
            &["bounce-back"],
        ),
        // Angle loops have no rise or bounce-back target.
        (
            TunedLoop::Roll,
            |m| {
                m.rise_time_s = None;
                m.bounce_back = 0.5;
            },
            &[],
        ),
    ];
    for (target, change, missed) in cases {
        let mut m = clean_step();
        change(&mut m);
        check!(
            spec::missed_targets(target, &m) == missed,
            "{target:?} {m:?}"
        );
    }
}
