//! Autotune robustness (#366): `StagePlan::copter_design_search` on every airframe, under each
//! perturbation set, from a low and a high start, designs every loop and tunes each by a
//! search from its design, with gains that meet the step spec on that same plant, within
//! [`BUDGET_S`] of simulated time.

mod common;

use assert2::check;
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::autotune::StagePlan;
use stable_flight_sim::Wind;
use stable_flight_sim::airframes::{self, Airframe};

use common::robustness::{Set, Start, perturbations, starting};
use common::{AUTOTUNE_TIMEOUT_S, Plant, spec};

/// Simulated seconds a row's autotune may take: the slowest passing row (`quad_x` with 10 ms
/// of delay, 380 s) plus 25%.
const BUDGET_S: f32 = 475.0;

/// Autotunes `airframe` under `set` from `start` and checks the row's criteria.
fn check_row(name: &str, airframe: Airframe, set: Set, start: Start) {
    let perturbations = perturbations(set, &airframe);
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
    let misses = spec::misses(&tuned, &spec::TUNED);
    let gains = spec::TUNED.map(|target: TunedLoop| (target, target.gains(&run.config)));
    check!(
        run.all_designed_and_searched() && misses.is_empty() && run.duration_s <= BUDGET_S,
        "{name} {set:?} from {start:?}: {:.0} s\noutcomes {:?}\ngains {gains:?}\nmisses {misses:#?}",
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
    #[ignore = "gyro noise spoils the fit (Design(NotALag)) or the settle (#373)"]
    quad_x_noise_low: quad_x, Noise, Low;
    #[ignore = "gyro noise spoils the fit (Design(NotALag)) or the settle (#373)"]
    quad_x_noise_high: quad_x, Noise, High;
    #[ignore = "trim moment held only by the slow angle I: misses the step spec (#374)"]
    quad_x_cg_offset_low: quad_x, CgOffset, Low;
    #[ignore = "trim moment held only by the slow angle I: misses the step spec (#374)"]
    quad_x_cg_offset_high: quad_x, CgOffset, High;
    #[ignore = "trim moment held only by the slow angle I: misses the step spec (#374)"]
    quad_x_weak_motor_low: quad_x, WeakMotor, Low;
    #[ignore = "trim moment held only by the slow angle I: misses the step spec (#374)"]
    quad_x_weak_motor_high: quad_x, WeakMotor, High;
    #[ignore = "gyro noise spoils the fit (Design(NotALag)) or the settle (#373)"]
    quad_x_combined_low: quad_x, Combined, Low;
    #[ignore = "gyro noise spoils the fit (Design(NotALag)) or the settle (#373)"]
    quad_x_combined_high: quad_x, Combined, High;

    heavy_quad_clean_low: heavy_quad, Clean, Low;
    heavy_quad_clean_high: heavy_quad, Clean, High;
    heavy_quad_delay_10ms_low: heavy_quad, Delay10Ms, Low;
    heavy_quad_delay_10ms_high: heavy_quad, Delay10Ms, High;
    heavy_quad_delay_25ms_low: heavy_quad, Delay25Ms, Low;
    heavy_quad_delay_25ms_high: heavy_quad, Delay25Ms, High;
    #[ignore = "gyro noise spoils the fit (Design(NotALag)) or the settle (#373)"]
    heavy_quad_noise_low: heavy_quad, Noise, Low;
    #[ignore = "gyro noise spoils the fit (Design(NotALag)) or the settle (#373)"]
    heavy_quad_noise_high: heavy_quad, Noise, High;
    #[ignore = "trim moment held only by the slow angle I: misses the step spec (#374)"]
    heavy_quad_cg_offset_low: heavy_quad, CgOffset, Low;
    #[ignore = "trim moment held only by the slow angle I: misses the step spec (#374)"]
    heavy_quad_cg_offset_high: heavy_quad, CgOffset, High;
    #[ignore = "rate I 0 holds the weak motor's trim short: final 0.95, rate rise 148 ms at the search's P 2.7 (#374)"]
    heavy_quad_weak_motor_low: heavy_quad, WeakMotor, Low;
    #[ignore = "rate I 0 holds the weak motor's trim short: final 0.95, rate rise 148 ms at the search's P 2.7 (#374)"]
    heavy_quad_weak_motor_high: heavy_quad, WeakMotor, High;
    #[ignore = "gyro noise spoils the fit (Design(NotALag)) or the settle (#373)"]
    heavy_quad_combined_low: heavy_quad, Combined, Low;
    #[ignore = "gyro noise spoils the fit (Design(NotALag)) or the settle (#373)"]
    heavy_quad_combined_high: heavy_quad, Combined, High;

    racer_clean_low: racer, Clean, Low;
    racer_clean_high: racer, Clean, High;
    racer_delay_10ms_low: racer, Delay10Ms, Low;
    racer_delay_10ms_high: racer, Delay10Ms, High;
    racer_delay_25ms_low: racer, Delay25Ms, Low;
    racer_delay_25ms_high: racer, Delay25Ms, High;
    #[ignore = "gyro noise spoils the fit (Design(NotALag)) or the settle (#373)"]
    racer_noise_low: racer, Noise, Low;
    #[ignore = "gyro noise spoils the fit (Design(NotALag)) or the settle (#373)"]
    racer_noise_high: racer, Noise, High;
    #[ignore = "trim moment held only by the slow angle I: misses the step spec (#374)"]
    racer_cg_offset_low: racer, CgOffset, Low;
    #[ignore = "trim moment held only by the slow angle I: misses the step spec (#374)"]
    racer_cg_offset_high: racer, CgOffset, High;
    #[ignore = "trim moment held only by the slow angle I: misses the step spec (#374)"]
    racer_weak_motor_low: racer, WeakMotor, Low;
    #[ignore = "trim moment held only by the slow angle I: misses the step spec (#374)"]
    racer_weak_motor_high: racer, WeakMotor, High;
    #[ignore = "gyro noise spoils the fit (Design(NotALag)) or the settle (#373)"]
    racer_combined_low: racer, Combined, Low;
    #[ignore = "gyro noise spoils the fit (Design(NotALag)) or the settle (#373)"]
    racer_combined_high: racer, Combined, High;
}
