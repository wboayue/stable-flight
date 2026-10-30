//! Attitude steps on `airframes::plane()` held to the plane spec in `common::plane_spec`.

mod common;

use assert2::check;
use stable_flight::tuning::StepMetrics;
use stable_flight_sim::airframes;

use common::plane::PlanePlant;
use common::plane_spec;

#[test]
fn roll_and_pitch_steps_meet_the_spec_both_ways() {
    let misses = plane_spec::misses(&PlanePlant::clean(airframes::plane()));
    check!(misses.is_empty(), "{misses:#?}");
}

/// A step that meets every target: no overshoot, quick rise, settled.
fn clean_step() -> StepMetrics {
    StepMetrics {
        amplitude: 1.0,
        peak: 1.0,
        rise_time_s: Some(0.5),
        settling_time_s: Some(0.7),
        final_value: 1.0,
        ..Default::default()
    }
}

/// An edit to [`clean_step`].
type Change = fn(&mut StepMetrics);

/// Each target flags just past its band: overshoot 5%, the clip allowance (42 ms), and a rise
/// of 2.2 time constants (0.88 s at 0.4 s).
#[test]
fn missed_targets_flags_each_band() {
    // (change to a clean step, targets missed)
    let cases: [(Change, &[&str]); 8] = [
        (|_| {}, &[]),
        (|m| m.overshoot = 0.05, &[]),
        (|m| m.overshoot = 0.051, &["overshoot"]),
        (|m| m.settling_time_s = None, &["unsettled"]),
        (|m| m.saturated_s = 0.042, &[]),
        (|m| m.saturated_s = 0.043, &["clipped"]),
        (|m| m.rise_time_s = Some(0.881), &["rise"]),
        (|m| m.rise_time_s = None, &["rise"]),
    ];
    for (change, expected) in cases {
        let mut m = clean_step();
        change(&mut m);
        check!(plane_spec::missed_targets(&m, 0.4) == expected, "{m:?}");
    }
}
