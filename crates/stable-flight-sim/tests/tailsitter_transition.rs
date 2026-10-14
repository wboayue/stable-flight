//! Transitions on `airframes::tailsitter()`: forward from a hover to plane flight, back to a
//! hover, and the ways a forward transition ends early, against PX4's VTOL transition limits
//! (PX4 main `edeba2f`, `src/modules/vtol_att_control/vtol_att_control_params.yaml`).

mod common;

use core::f32::consts::FRAC_PI_2;

use assert2::check;
use stable_flight::{Command, ControlInput, FlightMode, ModeRefusal, Vec3};
use stable_flight_sim::{Gust, Wind, airframes};

use common::plane::{
    AIRSPEED_BAND_MPS, ALTITUDE_BAND_M, Interlude, LineDeviation, north_line_command,
};
use common::tailsitter::{
    FORWARD_AT_S, HoverDeviation, TRANSITION_ALTITUDE_M, TailsitterPlant, Trace, forward,
    forward_and_back, tailsitter_loop, trace,
};
use common::{
    CALM_ALTITUDE_M, CALM_ANGLE_DEG, CALM_HORIZONTAL_M, HOVER_SETTLE_S, TICK, hover_at, waypoint,
};

/// PX4's forward-transition timeout (`VT_TRANS_TIMEOUT`, `:92-97`): plane flight by then.
const FORWARD_TIMEOUT_S: f32 = 15.0;

/// PX4's longest back transition (`VT_B_TRANS_DUR`, `:35-40`): hovering by then.
const BACK_DURATION_S: f32 = 10.0;

/// PX4's quad-chute altitude loss (`VT_QC_T_ALT_LOSS`, `:145-160`), in m below the height the
/// transition began, watched until [`ALTITUDE_LOSS_AFTER_S`] after it ends. PX4 watches forward
/// transitions; every transition is held to it here.
const ALTITUDE_LOSS_M: f32 = 20.0;

/// Seconds after a transition ends that [`ALTITUDE_LOSS_M`] still applies (PX4, as above).
const ALTITUDE_LOSS_AFTER_S: f32 = 5.0;

/// When the back transition is requested: 23 s into plane flight, settled on the line.
const BACK_AT_S: f32 = 45.0;

/// Where the hover after the back transition holds, m north on the line: about where it comes
/// to a stop. The back transition begins at 349 m and hands over at 9 m/s (measured); the
/// hover holds its heading belly north, since moving along the span departs (#387).
const BACK_HOVER_NORTH_M: f32 = 380.0;

/// Seconds the plane flies after the forward transition before the hold is judged
/// (`plane_level`'s settling time).
const PLANE_SETTLE_S: f32 = 15.0;

/// Seconds the hover settles after the back transition before the hold is judged. It arrives
/// at 9 m/s, 30 m short: measured 0.14 m off after 15 s, 0.05 m after 20 s.
const BACK_SETTLE_S: f32 = 20.0;

/// Seconds a level hold is judged over.
const HOLD_S: f32 = 10.0;

/// Seconds a plane hold is judged over (`plane_level`'s window).
const PLANE_HOLD_S: f32 = 30.0;

/// When `Plane` is requested on the way up, in s: below `min_height_m` (measured 1.6 m).
const LOW_REQUEST_S: f32 = 1.0;

/// Distance off the line in plane flight, in m: no external band applies. Measured under 1 mm
/// in calm air; the plane's 1 m (`plane_level`).
const CROSS_TRACK_M: f32 = 1.0;

/// The tick a flight entered `mode` at or after `from_s`, or a failed check naming `what`.
fn entered_s(trace: &Trace, mode: FlightMode, from_s: f32, what: &str) -> f32 {
    let entered = trace.entered(mode, from_s);
    check!(entered.is_some(), "{what}: never flew {mode:?}");
    entered.map_or(f32::INFINITY, |sample| sample.time_s)
}

/// Checks the lowest height from `from_s` until `ALTITUDE_LOSS_AFTER_S` after `ended_s` is
/// within [`ALTITUDE_LOSS_M`] of the height at `from_s`.
fn check_altitude_loss(trace: &Trace, from_s: f32, ended_s: f32, what: &str) {
    let start_m = trace
        .at(from_s)
        .map_or(f32::NAN, |sample| sample.state.position_m.z);
    let lowest_m = trace.lowest_m(from_s, ended_s + ALTITUDE_LOSS_AFTER_S);
    check!(
        start_m - lowest_m <= ALTITUDE_LOSS_M,
        "{what}: fell {} m below {start_m} m",
        start_m - lowest_m
    );
}

/// Checks a hold against the copter's calm-air hover bands (`hover.rs`).
fn check_calm_hover(worst: &HoverDeviation, what: &str) {
    check!(worst.altitude_m < CALM_ALTITUDE_M, "{what}: {worst:?}");
    check!(worst.horizontal_m < CALM_HORIZONTAL_M, "{what}: {worst:?}");
    check!(worst.tilt_deg < CALM_ANGLE_DEG, "{what}: {worst:?}");
    check!(worst.heading_deg < CALM_ANGLE_DEG, "{what}: {worst:?}");
}

/// The airframe's hover tilt and plane pitch caps sit short of where the ramps hand over, so
/// the transitions below complete only by flying past them (#388).
#[test]
fn ramps_hand_over_past_the_stabilizer_caps() {
    let airframe = airframes::tailsitter();
    let transition = airframe.guidance.transition;
    let stabilizer = airframe.stabilizer;
    check!(stabilizer.hover.attitude.max_angles_rad.y < transition.forward_done_rad);
    check!(stabilizer.plane.max_pitch_rad < FRAC_PI_2 - transition.back_done_rad);
}

/// From a hover, `Plane` on the north line reaches plane flight within PX4's timeout and
/// altitude loss, then holds the line as the airplane does.
#[test]
fn transitions_forward_into_level_flight() {
    let airframe = airframes::tailsitter();
    let plant = TailsitterPlant::clean(airframe);
    let mut flight_loop = tailsitter_loop(&plant, forward(&airframe));
    let flight = trace(
        &mut flight_loop,
        FORWARD_AT_S + FORWARD_TIMEOUT_S + PLANE_SETTLE_S + PLANE_HOLD_S,
    );

    let plane_s = entered_s(&flight, FlightMode::Plane, FORWARD_AT_S, "forward");
    // Measured 2.2 s, handing over at the 10 m/s airspeed gate.
    check!(plane_s - FORWARD_AT_S <= FORWARD_TIMEOUT_S);
    check_altitude_loss(&flight, FORWARD_AT_S, plane_s, "forward");

    let airspeed_mps = airframe.stabilizer.plane.trim_airspeed_mps;
    let hold_s = plane_s + PLANE_SETTLE_S;
    let mut worst = LineDeviation::default();
    for sample in flight.window(hold_s, hold_s + PLANE_HOLD_S) {
        worst.observe(&sample.state, TRANSITION_ALTITUDE_M, airspeed_mps);
    }
    check!(worst.altitude_m <= ALTITUDE_BAND_M, "{worst:?}");
    check!(worst.airspeed_mps <= AIRSPEED_BAND_MPS, "{worst:?}");
    check!(worst.cross_track_m <= CROSS_TRACK_M, "{worst:?}");
}

/// From plane flight, `Copter` on a hover point ahead returns to a hover within PX4's back
/// transition time and altitude loss, then holds it within the copter's calm-air bands
/// (`hover.rs`). The climb on the way is #343's.
#[test]
fn transitions_back_into_a_hover() {
    let airframe = airframes::tailsitter();
    let plant = TailsitterPlant::clean(airframe);
    let commander = forward_and_back(&airframe, BACK_AT_S, BACK_HOVER_NORTH_M);
    let mut flight_loop = tailsitter_loop(&plant, commander);
    let flight = trace(
        &mut flight_loop,
        BACK_AT_S + BACK_DURATION_S + BACK_SETTLE_S + HOLD_S,
    );

    check!(flight.entered(FlightMode::ToCopter, BACK_AT_S).is_some());
    let copter_s = entered_s(&flight, FlightMode::Copter, BACK_AT_S, "back");
    // Measured 1.3 s.
    check!(copter_s - BACK_AT_S <= BACK_DURATION_S);
    check_altitude_loss(&flight, BACK_AT_S, copter_s, "back");

    let point_m = Vec3::new(BACK_HOVER_NORTH_M, 0.0, TRANSITION_ALTITUDE_M);
    let hold_s = copter_s + BACK_SETTLE_S;
    let mut worst = HoverDeviation::default();
    for sample in flight.window(hold_s, hold_s + HOLD_S) {
        worst.observe(&sample.state, point_m);
    }
    check_calm_hover(&worst, "back");
}

/// A forward transition that ends in a hover instead: asked to, timed out, or rolled past its
/// limit. Each ends as the transition says, within PX4's altitude loss.
#[test]
fn ends_a_forward_transition_early() {
    let airframe = airframes::tailsitter();
    // (case, plant, seconds after FORWARD_AT_S that Copter is requested, refusal, seconds
    // into the ramp that the hover takes over: none for the roll error, which the gust times)
    let mut tilted = airframe;
    // Short of the 45° that completes it: only the timeout ends it.
    tilted.guidance.transition.forward_max_tilt_rad = 40f32.to_radians();
    let span_gust = Wind {
        steady_mps: Vec3::zeros(),
        // 8 m/s along the span as the ramp begins: measured 47° of roll (5 m/s: 23°).
        gust: Some(Gust {
            start_s: FORWARD_AT_S,
            duration_s: 2.0,
            peak_mps: Vec3::new(0.0, 8.0, 0.0),
        }),
    };
    let cases = [
        (
            "Copter requested",
            TailsitterPlant::clean(airframe),
            0.5,
            None,
            Some(0.5),
        ),
        (
            "timed out",
            TailsitterPlant::clean(tilted),
            f32::INFINITY,
            Some(ModeRefusal::TransitionTimeout),
            Some(FORWARD_TIMEOUT_S),
        ),
        (
            "rolled past the limit",
            TailsitterPlant::in_wind(airframe, span_gust),
            f32::INFINITY,
            Some(ModeRefusal::RollError),
            None,
        ),
    ];
    // The timeout trips on the tick after it runs out.
    let tolerance_s = 2.0 * TICK.as_secs_f32();
    let origin_m = Vec3::new(0.0, 0.0, TRANSITION_ALTITUDE_M);
    for (case, plant, copter_after_s, refusal, hover_after_s) in cases {
        let hover = Command::new(
            true,
            ControlInput::Waypoint(waypoint(0.0, 0.0, TRANSITION_ALTITUDE_M)),
        );
        let commander = Interlude::new(
            forward(&plant.airframe),
            FORWARD_AT_S + copter_after_s,
            f32::INFINITY,
            hover,
        );
        let mut flight_loop = tailsitter_loop(&plant, commander);
        let flight = trace(
            &mut flight_loop,
            FORWARD_AT_S + FORWARD_TIMEOUT_S + HOVER_SETTLE_S,
        );

        let ramp_s = entered_s(&flight, FlightMode::ToPlane, FORWARD_AT_S, case);
        check!(
            flight.entered(FlightMode::Plane, ramp_s).is_none(),
            "{case}"
        );
        let ended = flight.entered(FlightMode::Copter, ramp_s);
        check!(
            ended.map(|sample| sample.refusal) == Some(refusal),
            "{case}"
        );
        let copter_s = ended.map_or(f32::INFINITY, |sample| sample.time_s);
        if let Some(after_s) = hover_after_s {
            check!(
                (copter_s - ramp_s - after_s).abs() < tolerance_s,
                "{case}: {copter_s} s"
            );
        }
        check_altitude_loss(&flight, ramp_s, copter_s, case);

        // Asked back, it hovers over the origin. Refused, it still wants `Plane` and flies its
        // line as a copter, so only its height is checked.
        if refusal.is_none() {
            let hold_s = copter_s + HOVER_SETTLE_S;
            let mut worst = HoverDeviation::default();
            for sample in flight.window(hold_s, hold_s + HOLD_S) {
                worst.observe(&sample.state, origin_m);
            }
            check_calm_hover(&worst, case);
        }
    }
}

/// `Plane` requested on the way up waits, refused, until the transition's height, then
/// transitions.
#[test]
fn waits_for_the_transition_height() {
    let airframe = airframes::tailsitter();
    let plant = TailsitterPlant::clean(airframe);
    let airspeed_mps = airframe.stabilizer.plane.trim_airspeed_mps;
    let commander = Interlude::new(
        hover_at(TRANSITION_ALTITUDE_M),
        LOW_REQUEST_S,
        f32::INFINITY,
        north_line_command(TRANSITION_ALTITUDE_M, airspeed_mps),
    );
    let mut flight_loop = tailsitter_loop(&plant, commander);
    let flight = trace(&mut flight_loop, 10.0);

    let min_height_m = airframe.guidance.transition.min_height_m;
    let refused = flight.at(LOW_REQUEST_S);
    check!(
        refused.map(|sample| (sample.mode, sample.refusal))
            == Some((FlightMode::Copter, Some(ModeRefusal::BelowMinAltitude)))
    );
    let ramp = flight.entered(FlightMode::ToPlane, LOW_REQUEST_S);
    check!(ramp.is_some_and(|sample| sample.state.position_m.z >= min_height_m));
    let to_plane_s = ramp.map_or(f32::INFINITY, |sample| sample.time_s);
    let plane_s = entered_s(&flight, FlightMode::Plane, to_plane_s, "forward");
    check!(plane_s - to_plane_s <= FORWARD_TIMEOUT_S);
    check_altitude_loss(&flight, to_plane_s, plane_s, "forward");
}
