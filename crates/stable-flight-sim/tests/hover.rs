//! Hover hold: take off to a waypoint and hold altitude, position, and heading, and turn the
//! heading on it.

mod common;

use assert2::check;
use stable_flight::commanders::waypoint::DEFAULT_ALTITUDE_ACCEPTANCE_M;
use stable_flight::guidance::copter::DEFAULT_HEADING_RATE_RADPS;
use stable_flight::{ControlInput, Vec2, Vec3, wrap_angle};
use stable_flight_sim::airframes::{self, Airframe};
use stable_flight_sim::{Gust, Wind};

use common::yaw::{QUADCOPTER_X_YAW_RATE, quadcopter_x_yaw, yaw_trimmed};
use common::{
    CALM_ALTITUDE_M, CALM_ANGLE_DEG, CALM_HORIZONTAL_M, HOVER_ALTITUDE_M as ALTITUDE_M,
    HOVER_SETTLE_S as SETTLE_S, Plant, ThenFly, fly, waypoint,
};

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
    check!(worst.altitude_m < CALM_ALTITUDE_M, "{worst:?}");
    check!(worst.horizontal_m < CALM_HORIZONTAL_M, "{worst:?}");
    check!(worst.drift_m < CALM_HORIZONTAL_M, "{worst:?}");
    check!(worst.heading_deg < CALM_ANGLE_DEG, "{worst:?}");
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
    // (#357), which shaping the velocity target doesn't shorten (#367).
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

/// Most a heading turn may overshoot on QuadCopterX's yaw, as a share of the turn: RealFlight's
/// 7% plus 3 points (#383).
const MAX_TURN_OVERSHOOT: f32 = 0.10;

/// Most seconds from the end of the turn's slew until the heading stays within
/// [`TURN_SETTLE_DEG`] of the target on QuadCopterX's yaw: RealFlight's 1.6 s plus 0.4 s (#383,
/// #558).
const MAX_TURN_SETTLE_S: f32 = 2.0;

/// How close to the target a turn's heading counts as settled, in degrees.
const TURN_SETTLE_DEG: f32 = 1.0;

/// How long a turn is watched, in s: twice RealFlight's 4.2 s settle at yaw rate P 0.6, so a
/// ringing loop shows (#383).
const TURN_WATCH_S: f32 = 8.0;

/// How a heading turn on the hover point ends.
#[derive(Debug)]
struct Turn {
    /// Past the target, as a share of the turn.
    overshoot: f32,
    /// From the end of the slew until the heading stays within [`TURN_SETTLE_DEG`].
    settle_s: f32,
}

/// Hovers `airframe` under QuadCopterX's yaw trim, then turns the waypoint's heading by
/// `turn_deg` (positive turns right, from the spawn's north) and watches for [`TURN_WATCH_S`].
fn heading_turn(airframe: Airframe, turn_deg: f32) -> Turn {
    let target_rad = turn_deg.to_radians();
    let mut turn = waypoint(0.0, 0.0, ALTITUDE_M);
    turn.heading_rad = Some(target_rad);
    let commander = ThenFly {
        hover: common::hover_follower(),
        elapsed_s: 0.0,
        after_s: SETTLE_S,
        input: Some(ControlInput::Waypoint(turn)),
    };
    let mut flight_loop = common::sim_loop_with(&yaw_trimmed(airframe), commander);
    fly(&mut flight_loop, SETTLE_S, |_, _| {});

    // Guidance slews the heading target to the turn at its default rate.
    let slew_s = turn_deg.abs().to_radians() / DEFAULT_HEADING_RATE_RADPS;
    let mut overshoot_deg: f32 = 0.0;
    let mut last_outside_s = 0.0;
    fly(&mut flight_loop, TURN_WATCH_S, |flight_loop, elapsed_s| {
        let error_deg = wrap_angle(flight_loop.state().heading_rad - target_rad).to_degrees();
        // Past the target in the turn's direction.
        overshoot_deg = overshoot_deg.max(error_deg * turn_deg.signum());
        if error_deg.abs() > TURN_SETTLE_DEG {
            last_outside_s = elapsed_s;
        }
    });
    Turn {
        overshoot: overshoot_deg / turn_deg.abs(),
        settle_s: (last_outside_s - slew_s).max(0.0),
    }
}

/// QuadCopterX's yaw turns the heading 30° either way, against and with its yaw trim, and
/// settles without ringing: within RealFlight's measured overshoot and settle plus a margin
/// (#383). With yaw rate P 0.6, the loop overshoots 28% and rings for 4.1 s.
#[test]
fn quadcopter_x_heading_turn_settles_without_ringing() {
    for turn_deg in [30.0, -30.0] {
        let turn = heading_turn(quadcopter_x_yaw(QUADCOPTER_X_YAW_RATE), turn_deg);
        check!(
            turn.overshoot <= MAX_TURN_OVERSHOOT,
            "{turn_deg}°: {turn:?}"
        );
        check!(turn.settle_s <= MAX_TURN_SETTLE_S, "{turn_deg}°: {turn:?}");
    }
}
