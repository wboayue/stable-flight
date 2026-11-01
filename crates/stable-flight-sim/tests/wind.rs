//! The hover wind estimate (`CopterGuidance`'s, from a drag model) against the sim's steady
//! wind: `quad_x` with its linear drag, and the tailsitter with its broadside flat plate.

mod common;

use assert2::check;
use stable_flight::estimation::wind::{WindConfig, WindEstimate};
use stable_flight::{Guidance, Vec2, Vec3};
use stable_flight_sim::airframes::TailsitterAirframe;
use stable_flight_sim::{Wind, airframes};

use common::plane::Interlude;
use common::quadplane::{QuadPlanePlant, quadplane_loop};
use common::tailsitter::{TailsitterPlant, forward, tailsitter_loop};
use common::transition::{FORWARD_AT_S, TRANSITION_ALTITUDE_M};
use common::{Plant, fly, hover_at, hover_follower, sim_loop_with};
use stable_flight::{Command, ControlInput, Setpoint, WaypointCommand};

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
/// plate's drag: measured 3.06 m/s (2% over); 0.15 m/s leaves 2.4× margin. Its span has no
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

/// In calm air the tailsitter's belly estimate comes back near 0 after the takeoff. Climbing
/// off the ground nose first, its cambered wing lifts toward the belly, which reads as wind
/// until the climb slows below the estimator's 1 m/s skip; in still air afterward the estimate
/// returns. Measured −0.14 m/s; 0.2 m/s leaves 1.4× margin (#608).
#[test]
fn tailsitter_estimates_calm_air_after_its_takeoff() {
    let estimate = tailsitter_estimate(steady(0.0, 0.0));
    check!(estimate.velocity_mps.x.abs() < 0.2, "{estimate:?}");
}

/// A 3 m/s wind along the span leaves the belly's estimate near 0 and the span's unknown:
/// the wing's lift and drag take only the air in its plane of symmetry. Measured −0.22 m/s
/// across the belly; 0.3 m/s leaves 1.4× margin.
#[test]
fn tailsitter_span_wind_stays_unknown() {
    let estimate = tailsitter_estimate(steady(0.0, 3.0));
    check!(estimate.velocity_mps.x.abs() < 0.3, "{estimate:?}");
    check!(estimate.sd_along_mps(Vec2::y()) > 5.0, "{estimate:?}");
}

/// Through the forward transition and plane flight the tailsitter's estimate is left as the
/// hover had it: pitched over, the wing lifts and the drag model doesn't hold. Measured from
/// the transition's start to 15 s on.
#[test]
fn tailsitter_keeps_its_estimate_through_a_transition() {
    let airframe = tailsitter_with_its_plate();
    let plant = TailsitterPlant::clean(airframe);
    let mut flight_loop = tailsitter_loop(&plant, forward(&airframe));
    fly(&mut flight_loop, FORWARD_AT_S, |_, _| {});
    let hover = flight_loop
        .guidance()
        .wind()
        .expect("estimated in the hover");
    fly(&mut flight_loop, 15.0, |_, _| {});
    check!(flight_loop.guidance().wind() == Some(hover));
}

/// On a copter leg the quadplane's pusher takes the pitch, and its thrust along the body
/// isn't drag: the estimate is left as the hover had it while the pusher runs. The leg is
/// 100 m north, uncapped; 5 s in, the pusher is on (#427).
#[test]
fn quadplane_keeps_its_estimate_while_the_pusher_runs() {
    let mut airframe = airframes::quadplane();
    let lift = airframe.model.lift.drag_n_per_mps / airframe.model.wing.mass_kg;
    airframe.guidance.hover.wind = WindConfig {
        momentum_per_s: Vec2::new(lift, lift),
        ..Default::default()
    };
    let leg = WaypointCommand {
        position_m: Vec2::new(100.0, 0.0),
        altitude_m: TRANSITION_ALTITUDE_M,
        ..Default::default()
    };
    let commander = Interlude::new(
        hover_at(TRANSITION_ALTITUDE_M),
        FORWARD_AT_S,
        f32::INFINITY,
        Command::new(true, ControlInput::Waypoint(leg)),
    );
    let mut flight_loop = quadplane_loop(&QuadPlanePlant::clean(airframe), commander);
    fly(&mut flight_loop, FORWARD_AT_S, |_, _| {});
    let hover = flight_loop
        .guidance()
        .wind()
        .expect("estimated in the hover");
    fly(&mut flight_loop, 5.0, |_, _| {});
    check!(flight_loop.guidance().wind() == Some(hover));
    let (.., telemetry) = flight_loop.into_parts();
    let pusher = telemetry
        .iter()
        .last()
        .and_then(|tick| match tick.setpoint {
            Setpoint::Blend(blend) => Some(blend.plane.thrust),
            _ => None,
        });
    check!(pusher.is_some_and(|thrust| thrust > 0.0), "{pusher:?}");
}

/// Degrees between the wingtip heading and where the wind comes from, as core's wind heading
/// targets (`wind_heading_min_mps`).
const WINGTIP_FROM_WIND_DEG: f32 = 80.0;

/// Degrees from `heading_rad` to the nearer wingtip heading, [`WINGTIP_FROM_WIND_DEG`] off
/// where `wind_mps` comes from.
fn off_wingtip_deg(heading_rad: f32, wind_mps: Vec2) -> f32 {
    let from_rad = libm::atan2f(-wind_mps.y, -wind_mps.x);
    let off_deg = stable_flight::wrap_angle(heading_rad - from_rad)
        .abs()
        .to_degrees();
    (off_deg - WINGTIP_FROM_WIND_DEG).abs()
}

/// With `wind_heading_min_mps` set, the quad holding its hover point in a 3 m/s wind turns a
/// side into it (80° off where it comes from) from any direction: measured within 2° by
/// 3.7 s, and from 15 s on within 0.02° of it and 0.02 m of the point; 1° and 0.1 m leave
/// margin.
#[test]
fn quad_turns_a_side_into_a_steady_wind() {
    // (case, wind)
    let cases = [
        ("toward the north", steady(3.0, 0.0)),
        ("toward the east", steady(0.0, 3.0)),
        ("toward the southwest", steady(-2.0, -2.0)),
    ];
    for (case, wind) in cases {
        let mut airframe = quad_with_its_drag();
        airframe.guidance.wind_heading_min_mps = 1.0;
        let plant = Plant::in_wind(airframe, wind);
        let mut flight_loop = sim_loop_with(&plant, hover_follower());
        let (mut off_deg, mut away_m) = (0.0_f32, 0.0_f32);
        fly(&mut flight_loop, 30.0, |flight_loop, elapsed_s| {
            if elapsed_s > 15.0 {
                let state = flight_loop.state();
                off_deg = off_deg.max(off_wingtip_deg(state.heading_rad, wind.steady_mps.xy()));
                away_m = away_m.max(state.position_m.xy().norm());
            }
        });
        check!(off_deg < 1.0, "{case}: {off_deg}°");
        check!(away_m < 0.1, "{case}: {away_m} m");
    }
}

/// The tailsitter has no drag model along its span, so in a wind on its belly the wind's
/// direction stays unknown and the heading holds: measured 0° for 30 s at 3 m/s.
#[test]
fn tailsitter_holds_its_heading_while_the_wind_direction_is_unknown() {
    let mut airframe = tailsitter_with_its_plate();
    airframe.guidance.hover.wind_heading_min_mps = 1.0;
    let plant = TailsitterPlant::in_wind(airframe, steady(3.0, 0.0));
    let mut flight_loop = tailsitter_loop(&plant, hover_follower());
    let mut worst_deg = 0.0_f32;
    fly(&mut flight_loop, 30.0, |flight_loop, _| {
        let (_, _, heading_rad) = common::tailsitter::view_angles(flight_loop.state());
        worst_deg = worst_deg.max(heading_rad.to_degrees().abs());
    });
    check!(worst_deg < 1.0, "{worst_deg}°");
}
