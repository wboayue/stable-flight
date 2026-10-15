//! The Alia 250 ([`airframes::alia250`]), a full-size multirotor: its autotune converges on
//! the large-aircraft plan (`common::large`), its tuned loops meet the autotune's overshoot
//! and bounce-back bands on that plan's steps, and it holds a hover through a gust.

mod common;

use assert2::check;
use stable_flight::Vec3;
use stable_flight::controllers::Gains;
use stable_flight::stabilizers::copter::CopterStabilizerConfig;
use stable_flight_sim::airframes::{self, Airframe};
use stable_flight_sim::{Gust, Wind};

use common::spec::{self, TUNED};
use common::{AUTOTUNE_TIMEOUT_S, HOVER_ALTITUDE_M, HOVER_SETTLE_S, Plant, large};

/// How far apart, as a fraction of the larger, a converged gain may be from the airframe's
/// (as `autotune.rs`).
const AGREEMENT: f32 = 0.25;

/// The Alia with every rate P at `rate_p`, every angle P at `angle_p`, and no rate D.
fn started_from(rate_p: f32, angle_p: f32) -> Airframe {
    common::retuned(airframes::alia250(), |target, gains| Gains {
        p: if target.is_rate() { rate_p } else { angle_p },
        d: 0.0,
        ..gains
    })
}

fn tuned_p(config: &CopterStabilizerConfig) -> [f32; 4] {
    TUNED.map(|target| target.gains(config).p)
}

/// From rate P 1 and 3 with angle P 1, the large-aircraft plan tunes every stage to the
/// airframe's gains. A higher angle P on untuned rate loops (2–3), or rate P 0.5, tilts past
/// the envelope's 35° before the search ends.
#[test]
fn autotune_converges_on_the_large_aircraft_plan() {
    let reference = tuned_p(&airframes::alia250().stabilizer);
    for rate_p in [1.0, 3.0] {
        let run = common::autotune_within(
            &Plant::clean(started_from(rate_p, 1.0)),
            &large::plan(),
            large::envelope(),
            4.0 * AUTOTUNE_TIMEOUT_S,
        );
        check!(run.all_tuned(), "rate P {rate_p}: {:?}", run.stages);
        let tuned = tuned_p(&run.config);
        for (index, target) in TUNED.iter().enumerate() {
            let (a, b) = (tuned[index], reference[index]);
            check!(
                (a - b).abs() <= AGREEMENT * a.max(b),
                "rate P {rate_p}, {target:?}: {tuned:?} vs {reference:?}"
            );
        }
    }
}

/// Each tuned loop's steps, both ways, meet ArduPilot's overshoot and bounce-back bands and
/// settle (`spec`). PX4's 0.14 s rate rise isn't held: the motors alone lag 0.17 s.
#[test]
fn tuned_loops_meet_the_overshoot_bands() {
    let plant = Plant::clean(airframes::alia250());
    let mut misses = Vec::new();
    for target in TUNED {
        let (step, max_saturated_s) = large::step(target);
        for step in [step, step.reversed()] {
            match spec::respond(&plant, target, step) {
                None => misses.push(format!("{target:?} {}: no measurement", step.amplitude)),
                Some(m) => {
                    let missed = spec::missed_bands(target, &m, max_saturated_s, None);
                    if !missed.is_empty() {
                        misses.push(format!("{target:?} {}: {missed:?} {m:?}", step.amplitude));
                    }
                }
            }
        }
    }
    check!(misses.is_empty(), "{misses:#?}");
}

/// Takes off to the hover point in `wind`, settles, and holds for `hold_s`: the largest
/// altitude error and distance from the point, and the distance at the end.
fn hold(wind: Wind, hold_s: f32) -> (f32, f32, f32) {
    let mut flight_loop = common::hover(&Plant::in_wind(airframes::alia250(), wind));
    let (mut altitude_m, mut horizontal_m, mut final_m) = (0f32, 0f32, 0f32);
    common::fly(&mut flight_loop, hold_s, |flight_loop, _| {
        let position = flight_loop.state().position_m;
        altitude_m = altitude_m.max((position.z - HOVER_ALTITUDE_M).abs());
        horizontal_m = horizontal_m.max(position.xy().norm());
        final_m = position.xy().norm();
    });
    (altitude_m, horizontal_m, final_m)
}

#[test]
fn holds_the_hover_in_calm_air() {
    let (altitude_m, horizontal_m, _) = hold(Wind::default(), 10.0);
    check!(altitude_m < common::CALM_ALTITUDE_M);
    check!(horizontal_m < common::CALM_HORIZONTAL_M);
}

/// `hover.rs`'s gust (5 m/s across for 2 s), held 30 s: measured 0.02 m of altitude, 0.33 m
/// away at most, 0.03 m away at the end; bands at about twice that.
#[test]
fn rides_out_a_gust_and_returns() {
    let wind = Wind {
        steady_mps: Vec3::zeros(),
        gust: Some(Gust {
            start_s: HOVER_SETTLE_S + 1.0,
            duration_s: 2.0,
            peak_mps: Vec3::new(0.0, 5.0, 0.0),
        }),
    };
    let (altitude_m, horizontal_m, final_m) = hold(wind, 30.0);
    check!(altitude_m < 0.05);
    check!(horizontal_m < 0.7);
    check!(final_m < 0.1);
}
