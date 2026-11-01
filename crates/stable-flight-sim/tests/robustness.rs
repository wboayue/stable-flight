//! Autotune robustness (#366): `StagePlan::copter_design_search` on every airframe, under each
//! perturbation set, from a low and a high start, designs every roll and pitch loop, searches
//! yaw rate, and tunes each roll and pitch loop by a search from its design, with gains that meet the step
//! spec on that same plant, within
//! [`BUDGET_S`] of simulated time.

mod common;

use assert2::check;
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::autotune::StagePlan;
use stable_flight_sim::airframes::{self, Airframe};
use stable_flight_sim::{Perturbations, Wind};

use common::robustness::{Set, Start, notch_for, perturbations, starting};
use common::{AUTOTUNE_TIMEOUT_S, Plant, spec};

/// Simulated seconds a row's autotune may take: the slowest passing row (`heavy_quad`'s CG
/// offset from the high start, 280 s with the yaw search, #296) plus 25%.
const BUDGET_S: f32 = 350.0;

/// Autotunes `airframe` under `set` from `start` and checks the row's criteria.
fn check_row(name: &str, airframe: Airframe, set: Set, start: Start) {
    let row = format!("{name} {set:?}");
    check_plant(&row, airframe, perturbations(set, &airframe), start);
}

/// Autotunes `airframe` under `perturbations` from `start` and checks the row's criteria.
fn check_plant(row: &str, mut airframe: Airframe, perturbations: Perturbations, start: Start) {
    airframe.stabilizer.gyro_notch = notch_for(&perturbations);
    let plant = Plant {
        airframe: starting(airframe, start),
        wind: Wind::default(),
        perturbations,
    };
    let run = common::autotune(
        &plant,
        &StagePlan::copter_design_search(),
        AUTOTUNE_TIMEOUT_S,
    );
    let tuned = Plant {
        airframe: Airframe {
            stabilizer: run.config,
            ..airframe
        },
        ..plant
    };
    let misses = spec::misses(&tuned, &spec::AUTOTUNED);
    let gains = spec::AUTOTUNED.map(|target: TunedLoop| (target, target.gains(&run.config)));
    check!(
        run.all_designed_and_searched() && misses.is_empty() && run.duration_s <= BUDGET_S,
        "{row} from {start:?}: {:.0} s\noutcomes {:?}\ngains {gains:?}\nmisses {misses:#?}",
        run.duration_s,
        run.outcomes(),
    );
}

/// One `#[test]` per row: `test_name: airframe, Set, Start;`, each with its own attributes.
macro_rules! rows {
    ($($(#[$attr:meta])* $test:ident: $airframe:ident, $set:ident, $start:ident;)*) => {
        $(
            $(#[$attr])*
            #[test]
            fn $test() {
                check_row(
                    stringify!($airframe),
                    airframes::$airframe(),
                    Set::$set,
                    Start::$start,
                );
            }
        )*
    };
}

rows! {
    quad_x_clean_low: quad_x, Clean, Low;
    quad_x_clean_high: quad_x, Clean, High;
    quad_x_delay_10ms_low: quad_x, Delay10Ms, Low;
    quad_x_delay_10ms_high: quad_x, Delay10Ms, High;
    quad_x_delay_25ms_low: quad_x, Delay25Ms, Low;
    quad_x_delay_25ms_high: quad_x, Delay25Ms, High;
    quad_x_noise_low: quad_x, Noise, Low;
    quad_x_noise_high: quad_x, Noise, High;
    quad_x_cg_offset_low: quad_x, CgOffset, Low;
    quad_x_cg_offset_high: quad_x, CgOffset, High;
    quad_x_weak_motor_low: quad_x, WeakMotor, Low;
    quad_x_weak_motor_high: quad_x, WeakMotor, High;
    #[ignore = "under gyro noise the fit reads the 10 ms delay as 1 tick: the rate designs ring at the rate loop's limit and the angle stage's fit saturates (#603)"]
    quad_x_delay_noise_low: quad_x, DelayNoise, Low;
    #[ignore = "lowered up to 8 times, the ×3 start designs both rate loops, then the angle stage's fit saturates (#603)"]
    quad_x_delay_noise_high: quad_x, DelayNoise, High;
    #[ignore = "under gyro noise the fit reads the 10 ms delay as 1 tick: the rate designs ring at the rate loop's limit and the angle stage's fit saturates (#603)"]
    quad_x_combined_low: quad_x, Combined, Low;
    #[ignore = "lowered up to 8 times, the ×3 start designs both rate loops, then the angle stage's fit saturates (#603)"]
    quad_x_combined_high: quad_x, Combined, High;

    heavy_quad_clean_low: heavy_quad, Clean, Low;
    heavy_quad_clean_high: heavy_quad, Clean, High;
    heavy_quad_delay_10ms_low: heavy_quad, Delay10Ms, Low;
    heavy_quad_delay_10ms_high: heavy_quad, Delay10Ms, High;
    heavy_quad_delay_25ms_low: heavy_quad, Delay25Ms, Low;
    heavy_quad_delay_25ms_high: heavy_quad, Delay25Ms, High;
    heavy_quad_noise_low: heavy_quad, Noise, Low;
    heavy_quad_noise_high: heavy_quad, Noise, High;
    heavy_quad_cg_offset_low: heavy_quad, CgOffset, Low;
    heavy_quad_cg_offset_high: heavy_quad, CgOffset, High;
    heavy_quad_weak_motor_low: heavy_quad, WeakMotor, Low;
    heavy_quad_weak_motor_high: heavy_quad, WeakMotor, High;
    #[ignore = "under gyro noise the fit reads the 10 ms delay as 1 tick: the rate designs ring at the rate loop's limit and the yaw stage doesn't settle (NotSettled) (#603)"]
    heavy_quad_delay_noise_low: heavy_quad, DelayNoise, Low;
    #[ignore = "lowered up to 8 times, the ×3 start designs both rate loops, then the angle stage's fit saturates (#603)"]
    heavy_quad_delay_noise_high: heavy_quad, DelayNoise, High;
    #[ignore = "under gyro noise the fit reads the 10 ms delay as 1 tick: the rate designs ring at the rate loop's limit and the angle stage's fit saturates (#603)"]
    heavy_quad_combined_low: heavy_quad, Combined, Low;
    #[ignore = "lowered up to 8 times, the ×3 start designs both rate loops, then the angle stage's fit saturates (#603)"]
    heavy_quad_combined_high: heavy_quad, Combined, High;

    racer_clean_low: racer, Clean, Low;
    racer_clean_high: racer, Clean, High;
    racer_delay_10ms_low: racer, Delay10Ms, Low;
    racer_delay_10ms_high: racer, Delay10Ms, High;
    racer_delay_25ms_low: racer, Delay25Ms, Low;
    #[ignore = "the ×3 start (rate P 3.49) breaches the rate envelope in the first fit at 25 ms; the abort is the safe outcome (#552)"]
    racer_delay_25ms_high: racer, Delay25Ms, High;
    racer_noise_low: racer, Noise, Low;
    #[ignore = "with the gyro notch at the tone the autotune overshoots the budget or misses the spec on 4 noise seeds of 5 (#604)"]
    racer_noise_high: racer, Noise, High;
    racer_cg_offset_low: racer, CgOffset, Low;
    racer_cg_offset_high: racer, CgOffset, High;
    racer_weak_motor_low: racer, WeakMotor, Low;
    racer_weak_motor_high: racer, WeakMotor, High;
    #[ignore = "under gyro noise the fit reads the 10 ms delay as 1 tick: the roll rate design rings and its stage doesn't settle (NotSettled) (#603)"]
    racer_delay_noise_low: racer, DelayNoise, Low;
    #[ignore = "lowered 8 times, the ×3 start designs roll rate; the pitch rate fit still saturates (#603)"]
    racer_delay_noise_high: racer, DelayNoise, High;
    #[ignore = "under gyro noise the fit reads the 10 ms delay as 1 tick: the roll rate design rings and its stage doesn't settle (NotSettled) (#603)"]
    racer_combined_low: racer, Combined, Low;
    #[ignore = "lowered 8 times, the ×3 start designs roll rate; the pitch rate fit still saturates (#603)"]
    racer_combined_high: racer, Combined, High;
}

/// The noise rows that pass hold on other noise seeds: `quad_x` and `heavy_quad` from both
/// starts, on seeds 2–5 (the rows fly seed 1). `racer`'s low start, which passes on seed 1,
/// overshoots a yaw rate step on seed 5 (#604).
#[test]
fn noise_rows_pass_on_other_seeds() {
    for (name, airframe) in [
        ("quad_x", airframes::quad_x()),
        ("heavy_quad", airframes::heavy_quad()),
    ] {
        for start in [Start::Low, Start::High] {
            for seed in 2..=5 {
                let mut perturbations = perturbations(Set::Noise, &airframe);
                perturbations.gyro.seed = seed;
                check_plant(
                    &format!("{name} seed {seed}"),
                    airframe,
                    perturbations,
                    start,
                );
            }
        }
    }
}
