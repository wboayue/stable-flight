//! Hover legs on the tailsitters stop on their waypoint: the drag of a leg's speed is fed
//! forward, so the lateral loop's I term doesn't learn it and carry it past (#609).

mod common;

use assert2::check;
use stable_flight::{ControlInput, Vec2, Vec3, WaypointCommand};
use stable_flight_sim::Wind;
use stable_flight_sim::airframes::{self, TailsitterAirframe};

use common::tailsitter::{TailsitterPlant, tailsitter_loop};
use common::{
    HOVER_ALTITUDE_M as ALTITUDE_M, HOVER_SETTLE_S as SETTLE_S, ThenFly, fly, hover_follower,
    waypoint,
};

/// Length of each leg, in m: the eXplora scripts' hover legs are 20–30 m.
const LEG_M: f32 = 25.0;

/// How a leg ended.
#[derive(Debug)]
struct Leg {
    /// Farthest past the waypoint along the leg, in m.
    overshoot_m: f32,
    /// Seconds from the command until the vehicle last left 0.2 m of the waypoint.
    settled_s: f32,
}

/// Settles on the hover point in `wind`, then flies `LEG_M` toward `direction` (north, east,
/// unit) at `cruise_mps`, the belly held north.
fn leg(airframe: TailsitterAirframe, direction: Vec2, cruise_mps: f32, wind: Wind) -> Leg {
    let end_m = direction * LEG_M;
    let commander = ThenFly {
        hover: hover_follower(),
        elapsed_s: 0.0,
        after_s: SETTLE_S,
        input: Some(ControlInput::Waypoint(WaypointCommand {
            cruise_speed_mps: cruise_mps,
            ..waypoint(end_m.x, end_m.y, ALTITUDE_M)
        })),
    };
    let mut flight_loop = tailsitter_loop(&TailsitterPlant::in_wind(airframe, wind), commander);
    fly(&mut flight_loop, SETTLE_S, |_, _| {});
    let mut leg = Leg {
        overshoot_m: 0.0,
        settled_s: 0.0,
    };
    fly(&mut flight_loop, 45.0, |flight_loop, elapsed_s| {
        let position_m = flight_loop.state().position_m.xy();
        leg.overshoot_m = leg.overshoot_m.max((position_m - end_m).dot(&direction));
        if (position_m - end_m).norm() > 0.2 {
            leg.settled_s = elapsed_s;
        }
    });
    leg
}

/// Overshoot band, in m: measured 0.25 m at most (the eXplora belly first at 1.5 m/s), where
/// main overshot 1.0–2.5 m on these legs at 3 m/s; 0.5 m leaves 2× margin.
const OVERSHOOT_M: f32 = 0.5;

/// Settling band, in s from the leg's command: measured 25.5 s at most, where main settled in
/// 23–27 s at 3 m/s after overshooting; 35 s leaves 1.4× margin.
const SETTLED_S: f32 = 35.0;

/// Belly first and along the span, at the eXplora scripts' 3 m/s and at 1.5 m/s, and belly
/// first through a span wind, each leg stops on its waypoint.
#[test]
fn hover_legs_stop_on_the_waypoint() {
    let north = Vec2::new(1.0, 0.0);
    let east = Vec2::new(0.0, 1.0);
    let calm = Wind::default();
    let span_wind = Wind {
        steady_mps: Vec3::new(0.0, 1.5, 0.0),
        gust: None,
    };
    // (case, airframe, direction, cruise m/s, wind)
    let cases = [
        (
            "tailsitter belly first, 3 m/s",
            airframes::tailsitter(),
            north,
            3.0,
            calm,
        ),
        (
            "tailsitter along the span, 3 m/s",
            airframes::tailsitter(),
            east,
            3.0,
            calm,
        ),
        (
            "tailsitter belly first, 1.5 m/s",
            airframes::tailsitter(),
            north,
            1.5,
            calm,
        ),
        (
            "tailsitter belly first, 3 m/s, 1.5 m/s span wind",
            airframes::tailsitter(),
            north,
            3.0,
            span_wind,
        ),
        (
            "eXplora belly first, 3 m/s",
            airframes::explora(),
            north,
            3.0,
            calm,
        ),
        (
            "eXplora along the span, 3 m/s",
            airframes::explora(),
            east,
            3.0,
            calm,
        ),
        (
            "eXplora belly first, 1.5 m/s",
            airframes::explora(),
            north,
            1.5,
            calm,
        ),
        (
            "eXplora along the span, 1.5 m/s",
            airframes::explora(),
            east,
            1.5,
            calm,
        ),
        (
            "eXplora belly first, 3 m/s, 1.5 m/s span wind",
            airframes::explora(),
            north,
            3.0,
            span_wind,
        ),
    ];
    for (case, airframe, direction, cruise_mps, wind) in cases {
        let leg = leg(airframe, direction, cruise_mps, wind);
        check!(leg.overshoot_m < OVERSHOOT_M, "{case}: {leg:?}");
        check!(leg.settled_s < SETTLED_S, "{case}: {leg:?}");
    }
}
