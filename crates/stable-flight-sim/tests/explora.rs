//! The eXplora's hover ([`airframes::explora`]) in its nose-up view, calibrated to RealFlight's
//! autotune of it (#539): RealFlight's gains step here as they did there, and the design search
//! designs the gains RealFlight's did. Flown as RealFlight flew it: its exchange delay and gyro
//! noise, and the gyro low-pass at its 40 Hz default.

mod common;

use assert2::check;
use stable_flight::Vec3;
use stable_flight::controllers::Gains;
use stable_flight::stabilizers::copter::{
    CopterStabilizerConfig, DEFAULT_GYRO_CUTOFF_HZ, TunedLoop,
};
use stable_flight::tuning::StepSummary;
use stable_flight::tuning::autotune::{AutotuneEvent, Envelope, Outcome, StagePlan};
use stable_flight_sim::airframes;
use stable_flight_sim::{GyroNoise, Perturbations};

use common::tailsitter::{TailsitterPlant, hover_view_loop};
use common::{AUTOTUNE_TIMEOUT_S, AutotuneRun, HOVER_SETTLE_S};

/// RealFlight as the autotune met it: the elevons' rates fit 2–3 ticks of delay, one of them
/// the loop's hold, and the gyro reads about 0.01 rad/s of noise hovering (#539).
fn realflight() -> TailsitterPlant {
    TailsitterPlant {
        perturbations: Perturbations {
            actuator_delay_s: 0.006,
            gyro: GyroNoise {
                rms_radps: Vec3::repeat(0.01),
                vibration: None,
                seed: 7,
            },
            ..Perturbations::default()
        },
        ..TailsitterPlant::clean(airframes::explora())
    }
}

/// The hover's hand tune (`explora_vtol()`'s gains) on the default gyro low-pass, as RealFlight
/// run 2 flew it (#539).
fn hand_tune() -> CopterStabilizerConfig {
    CopterStabilizerConfig {
        gyro_cutoff_hz: DEFAULT_GYRO_CUTOFF_HZ,
        ..airframes::explora().stabilizer.hover
    }
}

/// RealFlight run 3's design search result (#539).
fn realflight_tune() -> CopterStabilizerConfig {
    let gains = |p, i, d| Gains { p, i, d };
    let mut config = hand_tune();
    config.body_rate.roll_gains = gains(0.6504, 0.1301, 0.0017);
    config.body_rate.pitch_gains = gains(1.4217, 0.2844, 0.0);
    config.body_rate.yaw_gains = gains(1.0191, 0.3057, 0.0);
    config.attitude.roll_gains = gains(2.3554, 0.0, 0.0);
    config.attitude.pitch_gains = gains(9.0, 0.0, 0.0);
    config
}

/// Runs `plan` on the [`realflight`] eXplora settled in a hover on `hover`.
fn autotune(hover: CopterStabilizerConfig, plan: &[StagePlan]) -> AutotuneRun {
    let plant = realflight();
    let mut flight_loop = hover_view_loop(&plant, hover, common::hover_follower());
    common::fly(&mut flight_loop, HOVER_SETTLE_S, |_, _| {});
    common::autotune_on(
        &mut flight_loop,
        plan,
        Envelope::default(),
        AUTOTUNE_TIMEOUT_S,
    )
}

/// `target`'s step test on `hover`.
fn steps(hover: CopterStabilizerConfig, target: TunedLoop) -> StepSummary {
    let run = autotune(hover, &[StagePlan::copter_step_test(target)]);
    match run.stages.as_slice() {
        [
            AutotuneEvent::StageDone {
                steps: Some(steps), ..
            },
        ] => *steps,
        other => panic!("{target:?}: {other:?}"),
    }
}

/// How a step test went in RealFlight (#539, runs 2 and 3), and how far the sim's may stray:
/// the final value within [`FINAL_BAND`] of RealFlight's range, the rise time (when RealFlight's
/// rose) within a factor of [`RISE_FACTOR`].
struct Flown {
    final_min: f32,
    final_max: f32,
    rise_s: Option<f32>,
}

/// Most a final value may stray outside RealFlight's range: 0.1 of the step, about the spread of
/// RealFlight's own finals across a test's steps (0.80–1.06 on the tuned pitch rate).
const FINAL_BAND: f32 = 0.1;

/// Most a rise time may stray from RealFlight's, as a factor either way: RealFlight's ticks run
/// unevenly (it skipped 50 over run 3), and its 2–3 tick elevon delay against the sim's 2 moves
/// a 61 ms rise by a third.
const RISE_FACTOR: f32 = 2.0;

/// The hand tune's and RealFlight's tuned gains' step tests fly as they did in RealFlight: the
/// hand tune's rate steps never rise and end short, and the tuned ones rise in about RealFlight's
/// time (yaw excepted: it didn't rise in RealFlight's window).
#[test]
fn realflights_gains_step_as_in_realflight() {
    let flown = |final_min, final_max, rise_s| Flown {
        final_min,
        final_max,
        rise_s,
    };
    // (gains, loop, RealFlight's steps)
    let cases = [
        (
            "hand",
            hand_tune(),
            TunedLoop::RollRate,
            flown(0.54, 0.56, None),
        ),
        (
            "hand",
            hand_tune(),
            TunedLoop::PitchRate,
            flown(0.80, 0.80, None),
        ),
        (
            "hand",
            hand_tune(),
            TunedLoop::YawRate,
            flown(0.24, 0.36, None),
        ),
        (
            "tuned",
            realflight_tune(),
            TunedLoop::RollRate,
            flown(0.88, 0.92, Some(0.173)),
        ),
        (
            "tuned",
            realflight_tune(),
            TunedLoop::PitchRate,
            flown(0.93, 1.06, Some(0.061)),
        ),
        (
            "tuned",
            realflight_tune(),
            TunedLoop::YawRate,
            flown(0.84, 0.84, None),
        ),
        (
            "tuned",
            realflight_tune(),
            TunedLoop::Roll,
            flown(0.99, 0.99, Some(0.546)),
        ),
        (
            "tuned",
            realflight_tune(),
            TunedLoop::Pitch,
            flown(1.01, 1.02, Some(0.164)),
        ),
    ];
    for (gains, hover, target, realflight) in cases {
        let row = format!("{gains} {target:?}");
        let steps = steps(hover, target);
        let (final_min, final_max) = (steps.final_value.min, steps.final_value.max);
        check!(
            final_min >= realflight.final_min - FINAL_BAND
                && final_max <= realflight.final_max + FINAL_BAND,
            "{row}: final {final_min}–{final_max}"
        );
        match realflight.rise_s {
            None if gains == "hand" => {
                check!(
                    steps.not_risen == steps.final_value.count,
                    "{row}: {steps:?}"
                );
            }
            None => {}
            Some(rise_s) => {
                let sim_s = steps.rise_time_s.max;
                check!(
                    steps.not_risen == 0
                        && sim_s <= RISE_FACTOR * rise_s
                        && sim_s >= rise_s / RISE_FACTOR,
                    "{row}: rise {sim_s} s against {rise_s} s, {steps:?}"
                );
            }
        }
    }
}

/// How far apart, as a fraction of the larger, a design may be from RealFlight's (as
/// `alia.rs`'s agreement).
const AGREEMENT: f32 = 0.25;

/// RealFlight run 3's designs: roll and pitch rate P, roll and pitch angle P (#539).
const REALFLIGHT_DESIGNS: [(TunedLoop, f32); 4] = [
    (TunedLoop::RollRate, 0.9394),
    (TunedLoop::PitchRate, 1.6868),
    (TunedLoop::Roll, 3.8791),
    (TunedLoop::Pitch, 9.0),
];

/// From the hand tune, the design search runs every stage as planned, and each loop's design
/// lands within [`AGREEMENT`] of RealFlight's.
#[test]
fn design_search_designs_realflights_gains() {
    let run = autotune(hand_tune(), &StagePlan::copter_design_search());
    check!(run.all_designed_and_searched(), "{:?}", run.stages);
    for (target, realflight_p) in REALFLIGHT_DESIGNS {
        let designed_p = run.stages.iter().find_map(|event| match event {
            AutotuneEvent::StageDone {
                target: done,
                gains,
                outcome: Outcome::Designed,
                ..
            } if *done == target => Some(gains.p),
            _ => None,
        });
        let Some(designed_p) = designed_p else {
            check!(false, "{target:?}: no design in {:?}", run.stages);
            continue;
        };
        check!(
            (designed_p - realflight_p).abs() <= AGREEMENT * designed_p.max(realflight_p),
            "{target:?}: designed {designed_p} against RealFlight's {realflight_p}"
        );
    }
}
