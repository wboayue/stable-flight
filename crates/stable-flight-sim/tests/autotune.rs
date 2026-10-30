//! Autotune converges: `StagePlan::copter_design_search` from detuned starts, rate P too low,
//! a little too high, and far too high, tunes every stage to gains that agree with each other.
//! Its rate loops rise close to the fastest P/D pair that meets the spec.

mod common;

use common::robustness::Start;
use common::spec::{self, TUNED};
use common::yaw::{YawWatch, watched_hover, yaw_trimmed};
use common::{AUTOTUNE_TIMEOUT_S, Plant};

use assert2::check;
use stable_flight::controllers::Gains;
use stable_flight::stabilizers::copter::CopterStabilizerConfig;
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::autotune::{
    AutotuneAbort, AutotuneEvent, COPTER_LOOP_SHAPING, DesignPlan, Envelope, Method, Outcome,
    SearchPlan, StagePlan, StepTestPlan,
};
use stable_flight::tuning::{LoopShapingConfig, StepConfig, StepMetrics};
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

/// The rate margin the misread is designed to, in degrees: with the gyro low-pass modeled and
/// D on the measured rate, the copter's 65° holds even on the misread; 30° is the widest that
/// rings (#373).
const MISREAD_MARGIN_DEG: f32 = 30.0;

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
        method: Method::Design(DesignPlan {
            design: LoopShapingConfig {
                rate_margin_deg: MISREAD_MARGIN_DEG,
                ..COPTER_LOOP_SHAPING
            },
            ..DesignPlan::copter()
        }),
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

/// How much slower than the fastest pair the design search's rate loop may rise, as a
/// fraction of the fastest pair's rise (#531's target). Measured: +8% on `quad_x`, +9% on
/// `heavy_quad`, each one 4 ms tick of rise.
const MAX_RISE_EXCESS: f32 = 0.25;

/// Multipliers on the tuned rate P and D, each, for the pairs the tuned loop is raced
/// against: 0.5 to 2, 22% apart. They span the fastest pairs that a sweep of P 0.2–6 by D
/// 0.001–0.4 finds (P × 0.92–1.47, D × 0.69–1.62 of the tuned, #531).
fn pair_scales() -> impl Iterator<Item = f32> + Clone {
    (0..8).map(|index| 0.5 * 1.219_f32.powi(index))
}

/// Autotunes `airframe` from core's default gains on the clean plant and checks its roll
/// rate loop rises within [`MAX_RISE_EXCESS`] of the fastest pair whose steps meet the spec.
/// Pitch rate isn't checked: it shares the fit and the design, and tunes to the same gains.
fn rises_near_the_fastest_rate_pair(airframe: Airframe) {
    let clean = Plant::clean(common::robustness::starting(airframe, Start::Low));
    let run = common::autotune(
        &clean,
        &StagePlan::copter_design_search(),
        AUTOTUNE_TIMEOUT_S,
    );
    let tuned = clean.with_config(run.config);
    let target = TunedLoop::RollRate;
    let tuned_gains = target.gains(&run.config);
    let pairs = pair_scales().flat_map(|p_scale| {
        pair_scales().map(move |d_scale| Gains {
            p: tuned_gains.p * p_scale,
            d: tuned_gains.d * d_scale,
            ..tuned_gains
        })
    });

    let tuned_rise_s = spec::rise_s(&tuned, target);
    let fastest = spec::fastest(&tuned, target, pairs);

    let Some((fastest_s, fastest_gains)) = fastest else {
        panic!("no pair meets the spec around {tuned_gains:?}");
    };
    check!(
        tuned_rise_s.is_some_and(|rise_s| rise_s <= (1.0 + MAX_RISE_EXCESS) * fastest_s),
        "tuned {tuned_gains:?} rises in {tuned_rise_s:?} s; {fastest_gains:?} in {fastest_s} s"
    );
}

/// On `quad_x`, the design search's rate loop rises within a quarter of the fastest pair
/// that meets the spec: the design's D at the fitted lag doesn't hold P back (#531).
#[test]
fn quad_x_design_search_rises_near_the_fastest_rate_pair() {
    rises_near_the_fastest_rate_pair(airframes::quad_x());
}

/// As [`quad_x_design_search_rises_near_the_fastest_rate_pair`], on `heavy_quad`.
#[test]
fn heavy_quad_design_search_rises_near_the_fastest_rate_pair() {
    rises_near_the_fastest_rate_pair(airframes::heavy_quad());
}

/// On `racer`, the gyro filter's lag is a large share of the loop's and its rate search
/// stops at P 0.55, rising in 28 ms against the fastest pair's 20 ms (#552).
#[test]
#[ignore = "racer rises 40% slower than its fastest pair under the gyro filter's lag (#552)"]
fn racer_design_search_rises_near_the_fastest_rate_pair() {
    rises_near_the_fastest_rate_pair(airframes::racer());
}

/// The copter angle step test measures the tuned angle loops of `quad_x` and `heavy_quad`
/// (angle P 9 and 8.54): its 5° steps clip for 5 ticks. At 10° they clip for 13 and 15, past
/// the allowance, so every step runs again until the stage fails `Invalid`, as after every
/// RealFlight QuadCopterX autotune (#310).
#[test]
fn angle_step_test_measures_the_tuned_angle_loops() {
    let plan = [TunedLoop::Roll, TunedLoop::Pitch].map(StagePlan::copter_step_test);
    for (name, airframe) in [
        ("quad_x", airframes::quad_x()),
        ("heavy_quad", airframes::heavy_quad()),
    ] {
        let run = common::autotune(&Plant::clean(airframe), &plan, AUTOTUNE_TIMEOUT_S);
        let measured =
            [TunedLoop::Roll, TunedLoop::Pitch].map(|target| (target, Ok(Outcome::Measured)));
        check!(run.outcomes() == measured, "{name}");
    }
}

/// QuadCopterX's hand-tuned yaw rate gains (sitl-realflight's `airframes::quadcopter_x`), a
/// start the yaw search meets in RealFlight.
const HAND_TUNED_YAW: Gains = Gains {
    p: 0.6,
    i: 0.6,
    d: 0.0,
};

/// How long the tuned yaw loop's steps are watched, in s: twice the search's window, since
/// the step against the trim levels off later.
const YAW_WATCH_S: f32 = 0.5;

/// Under a yaw trim as large as QuadCopterX's, the yaw rate search tunes from core's default
/// yaw gains and from QuadCopterX's hand tune to gains that agree, and the mixer never clips
/// the moments meanwhile. The tuned loop's steps clip within the allowance and settle, and judged
/// as the search judges a pair, on the two steps' means, overshoot and bounce back within the
/// spec. One at a time they don't: the trim spends 0.40 of the yaw moment in one direction,
/// so the step against it rises in up to 244 ms (PX4's yaw target is 0.2 s) and the step
/// with it overshoots by up to 0.08 (#296).
///
/// A start whose I builds the trim slowly can keep the search from passing anything:
/// `racer`'s own yaw gains (P 1.5, I 0.1) settle with part of the trim on P, and each lower
/// candidate drifts with the trim, overshooting by 0.14–0.28; from I 0.3 they tune to the
/// same P 0.91 as from 0.5 (#296).
#[test]
fn yaw_rate_search_tunes_against_a_yaw_trim() {
    let target = TunedLoop::YawRate;
    let plan = [StagePlan {
        target,
        method: Method::Search(SearchPlan::copter_yaw_rate()),
    }];
    let test = StepTestPlan::copter(target);
    let watched = StepConfig {
        duration_s: YAW_WATCH_S,
        ..test.step
    };
    for (name, airframe) in airframes::all() {
        let trimmed = yaw_trimmed(airframe);
        let starts = [
            target.gains(&CopterStabilizerConfig::default()),
            HAND_TUNED_YAW,
        ];
        let tuned = starts.map(|start| {
            let plant = trimmed.flying(target, start);
            let watch = YawWatch::default();
            let mut flight_loop = watched_hover(&plant, &watch);
            let clipped_before = watch.mixer_clipped_ticks.get();
            let run = common::autotune_on(
                &mut flight_loop,
                &plan,
                Envelope::default(),
                AUTOTUNE_TIMEOUT_S,
            );
            let row = format!("{name} from {start:?}");
            check!(run.outcomes() == [(target, Ok(Outcome::Tuned))], "{row}");
            check!(watch.mixer_clipped_ticks.get() == clipped_before, "{row}");
            let tuned = trimmed.with_config(run.config);
            let [up, down] = [watched, watched.reversed()]
                .map(|step| spec::respond(&tuned, target, step).expect("measured"));
            for m in [&up, &down] {
                check!(m.saturated_s <= test.max_saturated_s, "{row}: {m:?}");
                check!(m.settling_time_s.is_some(), "{row}: {m:?}");
            }
            let mean = |of: fn(&StepMetrics) -> f32| (of(&up) + of(&down)) / 2.0;
            check!(
                mean(|m| m.overshoot) <= spec::MAX_OVERSHOOT,
                "{row}: {up:?} {down:?}"
            );
            check!(
                mean(|m| m.bounce_back) <= spec::MAX_RATE_BOUNCE_BACK,
                "{row}: {up:?} {down:?}"
            );
            target.gains(&run.config).p
        });
        check!(agree(tuned[0], tuned[1]), "{name}: {tuned:?}");
    }
}
