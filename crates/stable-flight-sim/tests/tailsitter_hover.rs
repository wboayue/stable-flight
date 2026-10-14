//! Nose-up hover on `airframes::tailsitter()`: take off from the tail to a waypoint and hold
//! altitude, position, and heading in the hover view.

mod common;

use assert2::check;
use stable_flight::Vec3;
use stable_flight_sim::{Gust, Wind, airframes};

use common::tailsitter::{HoverDeviation, TailsitterPlant, tailsitter_loop};
use common::{HOVER_ALTITUDE_M as ALTITUDE_M, HOVER_SETTLE_S as SETTLE_S, fly, hover_follower};

/// Takes off to the hover point in `wind`, settles, then holds for `hold_s`, returning the
/// largest deviations during the hold.
fn hover(wind: Wind, hold_s: f32) -> HoverDeviation {
    let plant = TailsitterPlant::in_wind(airframes::tailsitter(), wind);
    let mut flight_loop = tailsitter_loop(&plant, hover_follower());
    fly(&mut flight_loop, SETTLE_S, |_, _| {});
    check!(flight_loop.commander().is_complete());

    let point_m = Vec3::new(0.0, 0.0, ALTITUDE_M);
    let mut worst = HoverDeviation::default();
    fly(&mut flight_loop, hold_s, |flight_loop, _| {
        worst.observe(flight_loop.state(), point_m);
    });
    worst
}

/// The copter's calm-air bands (`hover.rs`): a nose-up hover holds as tightly.
#[test]
fn holds_altitude_position_and_heading_in_calm_air() {
    let worst = hover(Wind::default(), 10.0);
    check!(worst.altitude_m < 0.1, "{worst:?}");
    check!(worst.horizontal_m < 0.1, "{worst:?}");
    check!(worst.tilt_deg < 1.0, "{worst:?}");
    check!(worst.heading_deg < 1.0, "{worst:?}");
}

/// Altitude band in a steady wind or a gust, in m: measured 0.27 m at most; 0.5 m leaves
/// about 2× margin.
const WIND_ALTITUDE_M: f32 = 0.5;

/// A steady 3 m/s wind on the belly (the whole wing broadside, about 15° of lean) or along
/// the span (differential thrust leaning it): the hold stays near the waypoint. Measured
/// 0.12 m and 0.57 m off; 1 m leaves about 2× margin. A 5 m/s wind with 4 m/s along the span
/// departs: the plane's weathercock moment (`c_yaw_beta` at 90° of sideslip) outgrows the
/// differential thrust (#387).
#[test]
fn holds_near_the_waypoint_in_steady_wind() {
    // (case, steady wind NEU m/s)
    let cases = [
        ("on the belly", Vec3::new(3.0, 0.0, 0.0)),
        ("along the span", Vec3::new(0.0, 3.0, 0.0)),
    ];
    for (case, steady_mps) in cases {
        let worst = hover(
            Wind {
                steady_mps,
                gust: None,
            },
            10.0,
        );
        check!(worst.altitude_m < WIND_ALTITUDE_M, "{case}: {worst:?}");
        check!(worst.horizontal_m < 1.0, "{case}: {worst:?}");
        // Measured 2.5° along the span; 5° leaves 2× margin.
        check!(worst.heading_deg < 5.0, "{case}: {worst:?}");
    }
}

/// A 5 m/s gust, on the belly or along the span, pushes the hold off and it comes back.
/// Measured 1.5 m off at most, and 0.30 m from the waypoint 6 s after the gust, the lateral
/// I's slow tail as on the copter (#357; TODO(#367)); 3 m and 0.5 m leave about 2× and 1.7×
/// margin.
#[test]
fn rides_out_a_gust_and_returns() {
    // (case, gust peak NEU m/s)
    let cases = [
        ("on the belly", Vec3::new(5.0, 0.0, 0.0)),
        ("along the span", Vec3::new(0.0, 5.0, 0.0)),
    ];
    for (case, peak_mps) in cases {
        let gust = Gust {
            start_s: SETTLE_S + 1.0,
            duration_s: 2.0,
            peak_mps,
        };
        let worst = hover(
            Wind {
                steady_mps: Vec3::zeros(),
                gust: Some(gust),
            },
            9.0,
        );
        check!(worst.altitude_m < WIND_ALTITUDE_M, "{case}: {worst:?}");
        check!(worst.horizontal_m < 3.0, "{case}: {worst:?}");
        check!(worst.final_m < 0.5, "{case}: {worst:?}");
    }
}
