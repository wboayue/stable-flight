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
use stable_flight::tuning::autotune::{AutotuneEvent, Envelope, Outcome, StagePlan};
use stable_flight::tuning::{Measurement, ProbeConfig, ProbeStatus, StepSummary};
use stable_flight_sim::airframes;
use stable_flight_sim::{GyroNoise, Perturbations};

use common::tailsitter::{TailsitterPlant, hover_view_loop};
use common::{AUTOTUNE_TIMEOUT_S, AutotuneRun, HOVER_SETTLE_S, spec};

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

/// The hover's hand tune on the default gyro low-pass, as RealFlight run 2 flew it (#539).
fn hand_tune() -> CopterStabilizerConfig {
    let gains = |p, i, d| Gains { p, i, d };
    let mut config = CopterStabilizerConfig {
        gyro_cutoff_hz: DEFAULT_GYRO_CUTOFF_HZ,
        ..airframes::explora().stabilizer.hover
    };
    config.body_rate.roll_gains = gains(0.4, 0.2, 0.03);
    config.body_rate.pitch_gains = gains(0.3, 0.5, 0.0);
    config.body_rate.yaw_gains = gains(0.1, 0.05, 0.0);
    config.attitude.roll_gains = gains(4.0, 0.0, 0.0);
    config.attitude.pitch_gains = gains(4.0, 0.0, 0.0);
    config
}

/// RealFlight run 3's design search result, which `explora()` flies (#539).
fn realflight_tune() -> CopterStabilizerConfig {
    airframes::explora().stabilizer.hover
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
/// the final value within [`FINAL_BAND`] of RealFlight's range, and its rise as [`Rise`] says.
struct RealFlightSteps {
    final_min: f32,
    final_max: f32,
    rise: Rise,
}

/// What a row holds the sim's rise time to.
enum Rise {
    /// No step rose in RealFlight's window, nor may one here.
    Never,
    /// Not judged: no step rose in RealFlight's window, but the sim's may (the tuned yaw rate).
    Unjudged,
    /// Every step rose, in this many s; the sim's within a factor of [`RISE_FACTOR`].
    Within(f32),
}

/// Most a final value may stray outside RealFlight's range: 0.1 of the step, about the spread of
/// RealFlight's own finals across a test's steps (0.13 on the tuned pitch rate, 0.93–1.06).
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
    let steps_were = |final_min, final_max, rise| RealFlightSteps {
        final_min,
        final_max,
        rise,
    };
    // (gains, loop, RealFlight's steps)
    let cases = [
        (
            "hand",
            hand_tune(),
            TunedLoop::RollRate,
            steps_were(0.54, 0.56, Rise::Never),
        ),
        (
            "hand",
            hand_tune(),
            TunedLoop::PitchRate,
            steps_were(0.80, 0.80, Rise::Never),
        ),
        (
            "hand",
            hand_tune(),
            TunedLoop::YawRate,
            steps_were(0.24, 0.36, Rise::Never),
        ),
        (
            "tuned",
            realflight_tune(),
            TunedLoop::RollRate,
            steps_were(0.88, 0.92, Rise::Within(0.173)),
        ),
        (
            "tuned",
            realflight_tune(),
            TunedLoop::PitchRate,
            steps_were(0.93, 1.06, Rise::Within(0.061)),
        ),
        (
            "tuned",
            realflight_tune(),
            TunedLoop::YawRate,
            steps_were(0.84, 0.84, Rise::Unjudged),
        ),
        (
            "tuned",
            realflight_tune(),
            TunedLoop::Roll,
            steps_were(0.99, 0.99, Rise::Within(0.546)),
        ),
        (
            "tuned",
            realflight_tune(),
            TunedLoop::Pitch,
            steps_were(1.01, 1.02, Rise::Within(0.164)),
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
        match realflight.rise {
            Rise::Never => {
                check!(
                    steps.not_risen == steps.final_value.count,
                    "{row}: {steps:?}"
                );
            }
            Rise::Unjudged => {}
            Rise::Within(rise_s) => {
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

/// The loops the searched gains meet [`spec`] on: every loop the autotune tunes but yaw rate
/// (view yaw, the elevons apart), whose searched gains settle short at 0.87 on one step, as
/// RealFlight's tuned yaw rate ends short at 0.84 ([`realflights_gains_step_as_in_realflight`],
/// #539): the wash damps the elevons' roll.
const SPEC_HELD: [TunedLoop; 4] = [
    TunedLoop::RollRate,
    TunedLoop::PitchRate,
    TunedLoop::Roll,
    TunedLoop::Pitch,
];

/// From the hand tune, the design search runs every stage as planned, each loop's design lands
/// within [`AGREEMENT`] of RealFlight's, and the gains it ends on meet ArduPilot's and PX4's
/// step bands ([`spec`]) both ways on [`SPEC_HELD`].
#[test]
fn design_search_designs_realflights_gains() {
    let run = autotune(hand_tune(), &StagePlan::copter_design_search());
    check!(run.all_designed_and_searched(), "{:?}", run.stages);
    let misses = spec_misses(run.config, &SPEC_HELD);
    check!(misses.is_empty(), "{misses:#?}");
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

/// Steps each of `loops`, both ways, on the [`realflight`] eXplora hovering on `hover`, and
/// returns a line for each step that misses the spec (empty when all meet it).
fn spec_misses(hover: CopterStabilizerConfig, loops: &[TunedLoop]) -> Vec<String> {
    let mut misses = Vec::new();
    for &target in loops {
        let step = spec::plan(target).step;
        for step in [step, step.reversed()] {
            let row = format!("{target:?} step {}", step.amplitude);
            let plant = realflight();
            let mut flight_loop = hover_view_loop(&plant, hover, common::hover_follower());
            common::fly(&mut flight_loop, HOVER_SETTLE_S, |_, _| {});
            let status = common::probe_on(
                &mut flight_loop,
                target,
                ProbeConfig::Step(step),
                PROBE_TIMEOUT_S,
            );
            match status {
                Some(ProbeStatus::Done(Measurement::Step(m))) => {
                    let missed = spec::missed_targets(target, &m);
                    if !missed.is_empty() {
                        misses.push(format!("{row}: {} in {m:?}", missed.join(", ")));
                    }
                }
                other => misses.push(format!("{row}: {other:?}")),
            }
        }
    }
    misses
}

/// Simulated seconds a spec step may run: the step's window plus margin (as `spec`'s).
const PROBE_TIMEOUT_S: f32 = 5.0;
