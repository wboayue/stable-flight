//! Autotune converges: `StagePlan::copter_search` from detuned starts, rate P too low, a little
//! too high, and far too high, tunes every stage to gains that agree with each other and with
//! `quad_x`'s (which came from this autotune).

mod common;

use common::spec::TUNED;
use common::{AUTOTUNE_TIMEOUT_S, Plant};

use assert2::check;
use stable_flight::stabilizers::copter::{CopterStabilizerConfig, TunedLoop};
use stable_flight::tuning::autotune::StagePlan;
use stable_flight_sim::airframes::{self, Airframe};

/// Rate P multiplier for the low start (the RealFlight example's `--rate-p-scale`).
const LOW_RATE_P_SCALE: f32 = 0.3;

/// Rate P multiplier for a start a little too high: its steps overshoot past the criterion
/// but don't clip.
const MODERATE_RATE_P_SCALE: f32 = 1.5;

/// Rate P multiplier for the high start: its steps clip.
const HIGH_RATE_P_SCALE: f32 = 3.0;

/// The angle P every run starts from: core's default, a third of `quad_x`'s.
const START_ANGLE_P: f32 = 3.0;

/// How far apart, as a fraction of the larger, two converged gains may be.
const AGREEMENT: f32 = 0.25;

/// `quad_x` with rate P scaled by `rate_p_scale` and angle P at [`START_ANGLE_P`].
fn detuned(rate_p_scale: f32) -> Airframe {
    let mut airframe = airframes::quad_x();
    for target in [TunedLoop::RollRate, TunedLoop::PitchRate] {
        target.gains_mut(&mut airframe.stabilizer).p *= rate_p_scale;
    }
    for target in [TunedLoop::Roll, TunedLoop::Pitch] {
        target.gains_mut(&mut airframe.stabilizer).p = START_ANGLE_P;
    }
    airframe
}

/// Whether `a` and `b` are within [`AGREEMENT`] of the larger.
fn agree(a: f32, b: f32) -> bool {
    (a - b).abs() <= AGREEMENT * a.max(b)
}

/// Each tuned loop's P in `config`.
fn tuned_p(config: &CopterStabilizerConfig) -> [f32; 4] {
    TUNED.map(|target| target.gains(config).p)
}

/// Autotunes `quad_x` from rate P × `rate_p_scale`, checks every stage ends `Tuned`, and
/// returns each tuned loop's P.
fn autotune_from(rate_p_scale: f32) -> [f32; 4] {
    let run = common::autotune(
        &Plant::clean(detuned(rate_p_scale)),
        &StagePlan::copter_search(),
        AUTOTUNE_TIMEOUT_S,
    );
    check!(run.all_tuned(), "rate P × {rate_p_scale}: {:?}", run.stages);
    tuned_p(&run.config)
}

/// Checks each of `p` agrees with `others`'.
fn check_agree(p: [f32; 4], others: &[[f32; 4]]) {
    for (index, target) in TUNED.iter().enumerate() {
        for other in others {
            check!(
                agree(p[index], other[index]),
                "{target:?}: {p:?} vs {other:?}"
            );
        }
    }
}

#[test]
fn converges_from_low_rate_gains() {
    let reference = tuned_p(&airframes::quad_x().stabilizer);
    check_agree(autotune_from(LOW_RATE_P_SCALE), &[reference]);
}

/// From a rate P a little too high, the search comes down and its result is kept though it
/// settles slower than the start, which settles fast by overshooting within the settle band
/// (#361).
#[test]
fn converges_from_moderately_high_rate_gains() {
    let reference = tuned_p(&airframes::quad_x().stabilizer);
    check_agree(autotune_from(MODERATE_RATE_P_SCALE), &[reference]);
}

/// From a rate P too high to step cleanly (×3 clips its steps), the relay's ceiling brings the
/// search down and the result is judged by the criterion, not by the start (#361).
#[test]
fn converges_from_high_rate_gains() {
    let reference = tuned_p(&airframes::quad_x().stabilizer);
    let low = autotune_from(LOW_RATE_P_SCALE);
    check_agree(autotune_from(HIGH_RATE_P_SCALE), &[reference, low]);
}
