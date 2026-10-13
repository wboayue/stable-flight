//! Hover hold: take off to a waypoint and hold altitude, position, and heading.

mod common;

use assert2::check;
use stable_flight::{Vec2, Vec3};
use stable_flight_sim::{Gust, Wind, airframes};

use common::{HOVER_ALTITUDE_M as ALTITUDE_M, HOVER_SETTLE_S as SETTLE_S, fly};

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
    let mut flight_loop = common::hover(airframes::quad_x(), wind);
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

/// The lateral loop's default velocity gains have no I, so a steady wind leaves a steady
/// offset: drag over the position-to-tilt gain, about 2.4 m in 5 m/s here (TODO(#357)).
#[test]
fn holds_a_steady_offset_in_steady_wind() {
    let wind = Wind {
        steady_mps: Vec3::new(3.0, -4.0, 0.0),
        gust: None,
    };
    let worst = hover(wind, 10.0);
    check!(worst.altitude_m < 0.1, "{worst:?}");
    check!(worst.horizontal_m < 3.0, "{worst:?}");
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
    // Back on the waypoint 6 s after the gust.
    check!(worst.final_m < 0.1, "{worst:?}");
}
