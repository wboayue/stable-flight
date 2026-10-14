//! Hover hold: take off to a waypoint and hold altitude, position, and heading.

mod common;

use assert2::check;
use stable_flight::commanders::waypoint::DEFAULT_ALTITUDE_ACCEPTANCE_M;
use stable_flight::{Vec2, Vec3};
use stable_flight_sim::{Gust, Wind, airframes};

use common::{HOVER_ALTITUDE_M as ALTITUDE_M, HOVER_SETTLE_S as SETTLE_S, Plant, fly};

/// Largest deviations from the hover point seen while holding.
#[derive(Debug, Default)]
struct Deviation {
    altitude_m: f32,
    /// Horizontal distance from the waypoint.
    horizontal_m: f32,
    /// Horizontal distance from where the hold began: wander, as opposed to a steady offset.
    drift_m: f32,
    heading_deg: f32,
    /// Horizontal distance from the waypoint at the end of the hold.
    final_m: f32,
}

/// Takes off to the hover point in `wind`, settles, then holds for `hold_s`, returning the
/// largest deviations during the hold.
fn hover(wind: Wind, hold_s: f32) -> Deviation {
    let mut flight_loop = common::hover(&Plant::in_wind(airframes::quad_x(), wind));
    check!(flight_loop.commander().is_complete());

    let horizontal = |position_m: Vec3| Vec2::new(position_m.x, position_m.y);
    let start = horizontal(flight_loop.state().position_m);
    let mut worst = Deviation::default();
    fly(&mut flight_loop, hold_s, |flight_loop, _| {
        let state = flight_loop.state();
        let position = horizontal(state.position_m);
        worst.altitude_m = worst
            .altitude_m
            .max((state.position_m.z - ALTITUDE_M).abs());
        worst.horizontal_m = worst.horizontal_m.max(position.norm());
        worst.drift_m = worst.drift_m.max((position - start).norm());
        worst.heading_deg = worst.heading_deg.max(
            stable_flight::wrap_angle(state.heading_rad)
                .to_degrees()
                .abs(),
        );
        worst.final_m = position.norm();
    });
    worst
}

#[test]
fn holds_altitude_position_and_heading_in_calm_air() {
    let worst = hover(Wind::default(), 10.0);
    check!(worst.altitude_m < 0.1, "{worst:?}");
    check!(worst.horizontal_m < 0.1, "{worst:?}");
    check!(worst.drift_m < 0.1, "{worst:?}");
    check!(worst.heading_deg < 1.0, "{worst:?}");
}

/// The lateral velocity I trims the drag of a steady wind, so the hold is on the waypoint
/// rather than downwind of it (#357).
#[test]
fn holds_the_waypoint_in_steady_wind() {
    let wind = Wind {
        steady_mps: Vec3::new(3.0, -4.0, 0.0),
        gust: None,
    };
    let worst = hover(wind, 10.0);
    check!(worst.altitude_m < 0.1, "{worst:?}");
    check!(worst.horizontal_m < 0.3, "{worst:?}");
    check!(worst.drift_m < 0.1, "{worst:?}");
    check!(worst.heading_deg < 1.0, "{worst:?}");
}

#[test]
fn rides_out_a_gust_and_returns() {
    let gust = Gust {
        start_s: SETTLE_S + 1.0,
        duration_s: 2.0,
        peak_mps: Vec3::new(0.0, 5.0, 0.0),
    };
    let wind = Wind {
        steady_mps: Vec3::zeros(),
        gust: Some(gust),
    };
    let worst = hover(wind, 9.0);
    check!(worst.horizontal_m < 1.0, "{worst:?}");
    check!(worst.altitude_m < 0.1, "{worst:?}");
    // Back near the waypoint 6 s after the gust: 0.17 m measured, the lateral I's slow tail
    // (#357; TODO(#367)).
    check!(worst.final_m < 0.25, "{worst:?}");
}

/// Holding a waypoint over the start completes only near its altitude, not on the ground
/// (#359): within the follower's altitude acceptance.
#[test]
fn hold_over_the_start_completes_at_its_altitude() {
    let mut flight_loop = common::sim_loop(common::hover_follower(), Wind::default());
    let mut completed_m = None;
    fly(&mut flight_loop, SETTLE_S, |flight_loop, _| {
        if completed_m.is_none() && flight_loop.commander().is_complete() {
            completed_m = Some(flight_loop.state().position_m.z);
        }
    });
    let completed_m = completed_m.expect("never completed");
    check!(
        (completed_m - ALTITUDE_M).abs() <= DEFAULT_ALTITUDE_ACCEPTANCE_M,
        "completed at {completed_m} m"
    );
}
