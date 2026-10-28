//! Autotune converges: `StagePlan::copter_design_search` from detuned starts, rate P too low,
//! a little too high, and far too high, tunes every stage to gains that agree with each other.

mod common;

use common::spec::TUNED;
use common::{AUTOTUNE_TIMEOUT_S, Plant};

use assert2::check;
use stable_flight::controllers::Gains;
use stable_flight::stabilizers::copter::CopterStabilizerConfig;
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::autotune::{
    AutotuneAbort, AutotuneEvent, DesignPlan, Method, StagePlan,
};
use stable_flight_sim::airframes::{self, Airframe};

/// Rate P multiplier for the low start (the RealFlight example's `--rate-p-scale`).
const LOW_RATE_P_SCALE: f32 = 0.3;

/// Rate P multiplier for a start a little too high: its steps overshoot past the criterion
/// but don't clip.
const MODERATE_RATE_P_SCALE: f32 = 1.5;

/// Rate P multiplier for the high start: its steps clip.
const HIGH_RATE_P_SCALE: f32 = 3.0;

/// How far apart, as a fraction of the larger, two converged gains may be.
const AGREEMENT: f32 = 0.25;

/// `quad_x` with rate P scaled by `rate_p_scale` and angle P at core's default (a third of
/// `quad_x`'s).
fn detuned(rate_p_scale: f32) -> Airframe {
    let defaults = CopterStabilizerConfig::default();
    common::retuned(airframes::quad_x(), |target, gains| Gains {
        p: if target.is_rate() {
            gains.p * rate_p_scale
        } else {
            target.gains(&defaults).p
        },
        ..gains
    })
}

/// Whether `a` and `b` are within [`AGREEMENT`] of the larger.
fn agree(a: f32, b: f32) -> bool {
    (a - b).abs() <= AGREEMENT * a.max(b)
}

/// Each tuned loop's P in `config`.
fn tuned_p(config: &CopterStabilizerConfig) -> [f32; 4] {
    TUNED.map(|target| target.gains(config).p)
}

/// Autotunes `quad_x` from rate P × `rate_p_scale`, checks every loop is designed then
/// searched, and returns each tuned loop's P.
fn autotune_from(rate_p_scale: f32) -> [f32; 4] {
    let run = common::autotune(
        &Plant::clean(detuned(rate_p_scale)),
        &StagePlan::copter_design_search(),
        AUTOTUNE_TIMEOUT_S,
    );
    check!(
        run.all_designed_and_searched(),
        "rate P × {rate_p_scale}: {:?}",
        run.stages
    );
    tuned_p(&run.config)
}

/// From rate P too low, a little too high (its steps overshoot past the criterion), and far
/// too high (its steps clip), the tuned P agree: the fit reads the plant, not the gains it
/// flies on, and each search keeps a result judged by the criterion (#361).
#[test]
fn converges_from_any_start() {
    let low = autotune_from(LOW_RATE_P_SCALE);
    for scale in [MODERATE_RATE_P_SCALE, HIGH_RATE_P_SCALE] {
        let tuned = autotune_from(scale);
        for (index, target) in TUNED.iter().enumerate() {
            check!(
                agree(tuned[index], low[index]),
                "{target:?} from × {scale}: {tuned:?} vs {low:?}"
            );
        }
    }
}

/// Actuator delay past what the fit models (`MAX_DELAY_TICKS`, 40 ms), so its fit misreads
/// the axis and the design rings.
const MISREAD_DELAY_S: f32 = 0.05;

/// A rate design that rings doesn't hold the settle, goes back, and its loop is fitted again;
/// the third fails the stage and the loop flies its start. A delay the fit can't model stands
/// in for RealFlight's fits that misread the axis (#518).
#[test]
fn a_design_that_rings_goes_back_to_its_start() {
    let plant = Plant {
        perturbations: common::robustness::delay(MISREAD_DELAY_S),
        ..Plant::clean(airframes::quad_x())
    };
    let plan = [StagePlan {
        target: TunedLoop::RollRate,
        method: Method::Design(DesignPlan::copter()),
    }];

    let run = common::autotune(&plant, &plan, AUTOTUNE_TIMEOUT_S);

    check!(
        matches!(
            run.stages[..],
            [AutotuneEvent::StageFailed {
                reason: AutotuneAbort::NotSettled,
                ..
            }]
        ),
        "{:?}",
        run.stages
    );
    check!(run.config == plant.airframe.stabilizer);
}
