//! The hover wind estimate (`CopterGuidance`'s, from a drag model) against the sim's steady
//! wind: `quad_x` with its linear drag, and the tailsitter with its broadside flat plate.

mod common;

use assert2::check;
use stable_flight::estimation::wind::{WindConfig, WindEstimate};
use stable_flight::{Guidance, Vec2, Vec3};
use stable_flight_sim::airframes::TailsitterAirframe;
use stable_flight_sim::{Wind, airframes};

use common::tailsitter::{TailsitterPlant, tailsitter_loop};
use common::{Plant, fly, hover_follower, sim_loop_with};

/// Seconds hovering before the estimate is read: the quad's settles within 0.3 m/s in 0.8 s.
const HOVER_S: f32 = 10.0;

/// Settled standard deviation along an axis with drag, in m/s: measured 0.27 on the quad and
/// 0.11 on the tailsitter's belly; 0.35 leaves 1.3× margin.
const SETTLED_SD_MPS: f32 = 0.35;

/// A steady `wind_mps` (north, east), gust-free.
fn steady(north_mps: f32, east_mps: f32) -> Wind {
    Wind {
        steady_mps: Vec3::new(north_mps, east_mps, 0.0),
        gust: None,
    }
}

/// `quad_x` with its own linear drag as the estimator's momentum drag (drag per kg).
fn quad_with_its_drag() -> airframes::Airframe {
    let mut airframe = airframes::quad_x();
    let per_s = airframe.model.drag_n_per_mps / airframe.model.mass_kg;
    airframe.guidance.wind = WindConfig {
        momentum_per_s: Vec2::new(per_s, per_s),
        ..Default::default()
    };
    airframe
}

/// The tailsitter with its broadside flat plate as the belly's (view `x`) ballistic
/// coefficient, `m / ((c_drag_0 + 2)·S)` (the plate's drag at 90°), and no drag model along
/// the span.
fn tailsitter_with_its_plate() -> TailsitterAirframe {
    let mut airframe = airframes::tailsitter();
    let wing = &airframe.model.wing;
    let ballistic_kgpm2 = wing.mass_kg / ((wing.aero.c_drag_0 + 2.0) * wing.wing_area_m2);
    airframe.guidance.hover.wind = WindConfig {
        ballistic_kgpm2: Vec2::new(ballistic_kgpm2, 0.0),
        ..Default::default()
    };
    airframe
}

/// The tailsitter's estimate after [`HOVER_S`] holding the hover point in `wind`.
fn tailsitter_estimate(wind: Wind) -> WindEstimate {
    let plant = TailsitterPlant::in_wind(tailsitter_with_its_plate(), wind);
    let mut flight_loop = tailsitter_loop(&plant, hover_follower());
    fly(&mut flight_loop, HOVER_S, |_, _| {});
    flight_loop.guidance().wind().expect("estimated")
}

/// With its drag model exact, the quad's hover estimate matches a 3 m/s wind from any
/// direction: measured within 0.001 m/s; 0.05 m/s leaves margin for the filter's settling.
#[test]
fn quad_estimates_a_steady_wind() {
    // (case, wind)
    let cases = [
        ("toward the north", steady(3.0, 0.0)),
        ("toward the east", steady(0.0, 3.0)),
        ("toward the southwest", steady(-2.0, -2.0)),
    ];
    for (case, wind) in cases {
        let plant = Plant::in_wind(quad_with_its_drag(), wind);
        let mut flight_loop = sim_loop_with(&plant, hover_follower());
        fly(&mut flight_loop, HOVER_S, |_, _| {});
        let estimate = flight_loop.guidance().wind().expect("estimated");
        let error_mps = (estimate.velocity_mps - wind.steady_mps.xy()).norm();
        check!(error_mps < 0.05, "{case}: {estimate:?}");
        check!(
            estimate.sd_along_mps(Vec2::x()) < SETTLED_SD_MPS,
            "{case}: {estimate:?}"
        );
        check!(
            estimate.sd_along_mps(Vec2::y()) < SETTLED_SD_MPS,
            "{case}: {estimate:?}"
        );
    }
}

/// A 3 m/s wind on the tailsitter's belly (north, its hover heading) is estimated from the
/// plate's drag: measured 3.05 m/s (1.6% over); 0.15 m/s leaves 3× margin. Its span has no
/// drag model, so the wind across it stays unknown.
#[test]
fn tailsitter_estimates_a_wind_on_the_belly() {
    let estimate = tailsitter_estimate(steady(3.0, 0.0));
    check!((estimate.velocity_mps.x - 3.0).abs() < 0.15, "{estimate:?}");
    check!(
        estimate.sd_along_mps(Vec2::x()) < SETTLED_SD_MPS,
        "{estimate:?}"
    );
    check!(
        estimate.sd_along_mps(Vec2::y()) > 0.9 * stable_flight::estimation::wind::INITIAL_SD_MPS,
        "{estimate:?}"
    );
}

/// A 3 m/s wind along the span leaves the belly's estimate near 0 and the span's unknown.
/// The sim's plate takes the whole airflow's pressure with its angle from the belly and chord
/// alone, so a span wind pushes the belly as hard as a belly wind: measured −3.1 m/s across
/// the belly.
#[test]
#[ignore = "the sim's plate pushes the belly in a span wind (#387)"]
fn tailsitter_span_wind_stays_unknown() {
    let estimate = tailsitter_estimate(steady(0.0, 3.0));
    check!(estimate.velocity_mps.x.abs() < 0.3, "{estimate:?}");
    check!(estimate.sd_along_mps(Vec2::y()) > 5.0, "{estimate:?}");
}
