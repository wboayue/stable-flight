//! Step responses held to the autotune targets in `common::spec`: every loop on `quad_x`, and
//! the tuned loops on every airframe.

mod common;

use assert2::check;
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::StepMetrics;
use stable_flight_sim::airframes;

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
