//! Nose-up hover on `airframes::tailsitter()`: take off from the tail to a waypoint and hold
//! altitude, position, and heading in the hover view.

mod common;

use assert2::check;
use stable_flight::{ControlInput, Vec3, WaypointCommand, wrap_angle};
use stable_flight_sim::{Gust, Wind, airframes};
use std::f32::consts::FRAC_PI_2;

use common::tailsitter::{TailsitterPlant, tailsitter_loop, view_angles};
use common::transition::HoverDeviation;
use common::{
    CALM_ALTITUDE_M, CALM_ANGLE_DEG, CALM_HORIZONTAL_M, HOVER_ALTITUDE_M as ALTITUDE_M,
    HOVER_SETTLE_S as SETTLE_S, ThenFly, fly, hover_follower,
};

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
    check!(worst.altitude_m < CALM_ALTITUDE_M, "{worst:?}");
    check!(worst.horizontal_m < CALM_HORIZONTAL_M, "{worst:?}");
    check!(worst.tilt_deg < CALM_ANGLE_DEG, "{worst:?}");
    check!(worst.heading_deg < CALM_ANGLE_DEG, "{worst:?}");
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

/// The worst of a hold turning to a waypoint heading.
#[derive(Debug)]
struct Turn {
    /// The largest deviations from the hold point.
    worst: HoverDeviation,
    /// Seconds from the command until the view heading is within 5° of it; NaN if never.
    reached_s: f32,
    /// View heading error at the end, in degrees.
    final_error_deg: f32,
}

/// Settles on the hover point in `wind`, then holds it facing `heading_rad` for 6 s.
fn turn_to(heading_rad: f32, wind: Wind) -> Turn {
    let commander = ThenFly {
        hover: hover_follower(),
        elapsed_s: 0.0,
        after_s: SETTLE_S,
        input: Some(ControlInput::Waypoint(WaypointCommand {
            altitude_m: ALTITUDE_M,
            heading_rad: Some(heading_rad),
            ..Default::default()
        })),
    };
    let plant = TailsitterPlant::in_wind(airframes::tailsitter(), wind);
    let mut flight_loop = tailsitter_loop(&plant, commander);
    fly(&mut flight_loop, SETTLE_S, |_, _| {});

    let point_m = Vec3::new(0.0, 0.0, ALTITUDE_M);
    let mut turn = Turn {
        worst: HoverDeviation::default(),
        reached_s: f32::NAN,
        final_error_deg: f32::NAN,
    };
    fly(&mut flight_loop, TURN_HOLD_S, |flight_loop, elapsed_s| {
        let state = flight_loop.state();
        turn.worst.observe(state, point_m);
        let (_, _, heading) = view_angles(state);
        turn.final_error_deg = wrap_angle(heading - heading_rad).to_degrees().abs();
        if turn.reached_s.is_nan() && turn.final_error_deg < 5.0 {
            turn.reached_s = elapsed_s;
        }
    });
    turn
}

/// Seconds [`turn_to`] holds after the command.
const TURN_HOLD_S: f32 = 6.0;

/// A waypoint heading 90° off turns the hover view's heading, the belly, to it at
/// `heading_rate_radps` (60°/s: 1.5 s) while the hold stays as tight as in calm air. Measured
/// within 5° at 2.1 s; 3 s leaves the rate's 1.5 s plus 1 s for the yaw loop to settle.
#[test]
fn turns_the_belly_to_a_waypoint_heading() {
    let turn = turn_to(FRAC_PI_2, Wind::default());
    check!(turn.reached_s < 3.0, "{turn:?}");
    check!(turn.final_error_deg < CALM_ANGLE_DEG, "{turn:?}");
    check!(turn.worst.altitude_m < CALM_ALTITUDE_M, "{turn:?}");
    check!(turn.worst.horizontal_m < CALM_HORIZONTAL_M, "{turn:?}");
    check!(turn.worst.tilt_deg < CALM_ANGLE_DEG, "{turn:?}");
}

/// Turning a 3 m/s wind from the belly onto the span holds near the waypoint, as starting
/// with it on the span does (1 m, above). The sim departs at 60°/s (23 m off, 87° of roll)
/// and 30°/s (11 m, 66°), and holds at 15°/s (2.6 m, 15°); RealFlight's eXplora turned 90°
/// at 30°/s in 3 mph wind and held within 6 m (#417).
#[test]
#[ignore = "the sim departs turning a span wind onto the wing at 60°/s (#387)"]
fn turns_a_belly_wind_onto_the_span() {
    let wind = Wind {
        steady_mps: Vec3::new(3.0, 0.0, 0.0),
        gust: None,
    };
    let turn = turn_to(FRAC_PI_2, wind);
    check!(turn.worst.horizontal_m < 1.0, "{turn:?}");
    check!(turn.final_error_deg < 5.0, "{turn:?}");
}
