//! Step responses held to the autotune targets in `common::spec`: every loop on `quad_x`, and
//! the tuned loops on every airframe.

mod common;

use assert2::check;
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
