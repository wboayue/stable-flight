//! Saturation flags: each limit, hit from a settled hover, reports itself (and only the limits
//! it drags along) in the telemetry's merged `Saturation`.

mod common;

use assert2::check;
use stable_flight::{
    AbsoluteAttitudeCommand, AbsoluteRateCommand, Command, Commander, ControlInput, Saturation,
    Vec2, VehicleState, WaypointCommand,
};
use stable_flight_sim::airframes;

use common::{HOVER_ALTITUDE_M, Hover, Plant, fly, waypoint};

/// Hovers, then flies `input` from `after_s` on.
struct ThenFly {
    hover: Hover,
    elapsed_s: f32,
    after_s: f32,
    input: Option<ControlInput>,
}

impl Commander for ThenFly {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        self.elapsed_s += dt;
        match self.input {
            Some(input) if self.elapsed_s >= self.after_s => Command::new(true, input),
            _ => self.hover.next_command(dt, state),
        }
    }
}

/// Seconds each input flies before the flags are read: the onset, which is when limits clip,
/// falls inside the telemetry the loop keeps (`common::RECORDED` ticks).
const FLY_S: f32 = 0.2;

/// The flags of the last recorded ticks after hovering, then flying `input` (if any) for
/// [`FLY_S`].
fn flags(input: Option<ControlInput>) -> Saturation {
    let commander = ThenFly {
        hover: common::hover_follower(),
        elapsed_s: 0.0,
        after_s: common::HOVER_SETTLE_S,
        input,
    };
    let mut flight_loop = common::sim_loop_with(&Plant::clean(airframes::quad_x()), commander);
    fly(&mut flight_loop, common::HOVER_SETTLE_S + FLY_S, |_, _| {});
    let (.., telemetry) = flight_loop.into_parts();
    telemetry.iter().fold(Saturation::default(), |all, tick| {
        all.merge(tick.saturation)
    })
}

/// Body rates at `thrust`.
fn rates(roll_radps: f32, yaw_radps: f32, thrust: f32) -> Option<ControlInput> {
    Some(ControlInput::AbsoluteRate(AbsoluteRateCommand {
        roll_radps,
        pitch_radps: 0.0,
        yaw_radps,
        thrust,
    }))
}

/// The flags `set` sets (`Saturation` is non-exhaustive).
fn flagged(set: fn(&mut Saturation)) -> Saturation {
    let mut saturation = Saturation::default();
    set(&mut saturation);
    saturation
}

#[test]
fn each_limit_reports_itself() {
    let hover_thrust = airframes::quad_x().guidance.altitude.hover_thrust;
    // (scenario, input after the hover, expected flags)
    let cases = [
        ("holding the hover", None, Saturation::default()),
        (
            // A 15 m/s leg asks for more than guidance's 27° tilt cap; pitching over to it
            // clips the pitch moment.
            "a fast leg north",
            Some(ControlInput::Waypoint(WaypointCommand {
                position_m: Vec2::new(100.0, 0.0),
                altitude_m: HOVER_ALTITUDE_M,
                cruise_speed_mps: 15.0,
                ..Default::default()
            })),
            flagged(|s| {
                s.tilt = true;
                s.moments[1] = true;
            }),
        ),
        (
            "a climb of 55 m",
            Some(ControlInput::Waypoint(waypoint(0.0, 0.0, 60.0))),
            flagged(|s| s.thrust = true),
        ),
        (
            // Past the stabilizer's 45° cap; rolling to it clips the roll moment.
            "a 60° bank",
            Some(ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
                roll_rad: 60.0_f32.to_radians(),
                pitch_rad: 0.0,
                heading_rad: 0.0,
                thrust: hover_thrust,
            })),
            flagged(|s| {
                s.tilt = true;
                s.moments[0] = true;
            }),
        ),
        (
            "a 6 rad/s roll",
            rates(6.0, 0.0, hover_thrust),
            flagged(|s| s.moments[0] = true),
        ),
        (
            // Yaw's share of the motors (`yaw_ratio`) passes full throttle at full thrust: the
            // mixer gives up collective.
            "a yaw at full thrust",
            rates(0.0, 1.0, 1.0),
            flagged(|s| {
                s.moments[2] = true;
                s.mixer_thrust = true;
            }),
        ),
        (
            // Near idle the mixer can't shift the motors up enough for the moment.
            "a roll near idle",
            rates(6.0, 0.0, 0.05),
            flagged(|s| {
                s.moments[0] = true;
                s.mixer_moments = true;
            }),
        ),
    ];
    for (scenario, input, expected) in cases {
        check!(flags(input) == expected, "{scenario}");
    }
}
