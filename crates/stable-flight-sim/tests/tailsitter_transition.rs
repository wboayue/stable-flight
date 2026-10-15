//! Transitions on `airframes::tailsitter()`: forward from a hover to plane flight, back to a
//! hover, and the ways a forward transition ends early, against PX4's VTOL transition limits
//! (PX4 main `edeba2f`, `src/modules/vtol_att_control/vtol_att_control_params.yaml`).

mod common;

use core::f32::consts::FRAC_PI_2;

use assert2::check;
use stable_flight::guidance::vtol::TransitionConfig;
use stable_flight::{
    Command, ControlInput, FlightMode, ModeRefusal, TelemetryPublisher, TelemetryRecord, Vec3,
};
use stable_flight_sim::{Gust, Wind, airframes};

use common::plane::{
    AIRSPEED_BAND_MPS, ALTITUDE_BAND_M, Interlude, LineDeviation, north_line_command,
};
use common::tailsitter::{
    FORWARD_AT_S, HoverDeviation, TRANSITION_ALTITUDE_M, TailsitterPlant, Trace, forward,
    forward_and_back, tailsitter_loop, tailsitter_pipeline, trace,
};
use common::{
    CALM_ALTITUDE_M, CALM_ANGLE_DEG, CALM_HORIZONTAL_M, HOVER_SETTLE_S, TICK, fly, hover_at,
    waypoint,
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
/// to a stop. The back transition begins at 349 m and hands over at 380 m, at 2.5 m/s
/// (measured); the hover holds its heading belly north, since moving along the span departs
/// (#387).
const BACK_HOVER_NORTH_M: f32 = 380.0;

/// Seconds the plane flies after the forward transition before the hold is judged
/// (`plane_level`'s settling time).
const PLANE_SETTLE_S: f32 = 15.0;

/// Seconds the hover settles after the back transition before the hold is judged. It arrives
/// at 2.5 m/s over the point: measured 0.07 m off after 10 s.
const BACK_SETTLE_S: f32 = 10.0;

/// Most the back transition climbs above the height it began at, in m, until
/// [`ALTITUDE_LOSS_AFTER_S`] after it ends: no external band applies. Measured 9.5 m, under
/// the 10–12 m the eXplora climbed at hover thrust (#343), with 1 m of margin.
const BACK_CLIMB_M: f32 = 10.5;

/// The back transition's ramp thrusts, below the airframe's, that stall it nose-high: without
/// the sink guard both fall 30 m to the ground (measured, #343).
const STALLING_BACK_THRUSTS: [f32; 2] = [0.3, 0.5];

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

/// Checks the highest height from `from_s` until `ALTITUDE_LOSS_AFTER_S` after `ended_s` is
/// within [`BACK_CLIMB_M`] above the height at `from_s`.
fn check_climb(trace: &Trace, from_s: f32, ended_s: f32, what: &str) {
    let start_m = trace
        .at(from_s)
        .map_or(f32::NAN, |sample| sample.state.position_m.z);
    let highest_m = trace
        .window(from_s, ended_s + ALTITUDE_LOSS_AFTER_S)
        .map(|sample| sample.state.position_m.z)
        .fold(f32::NEG_INFINITY, f32::max);
    check!(
        highest_m - start_m <= BACK_CLIMB_M,
        "{what}: climbed {} m above {start_m} m",
        highest_m - start_m
    );
}

/// Seconds after the back handoff that [`HANDOFF_STEP`] watches.
const HANDOFF_WATCH_S: f32 = 2.0;

/// Largest change in the moments (norm) between two ticks after the back handoff: no external
/// band applies. Measured 0.022 on the fast ramp with the integrals kept through the thrust
/// handoff, 0.263 when its zero-thrust tick cleared them (#326).
const HANDOFF_STEP: f32 = 0.05;

/// The largest per-tick moment change in the [`HANDOFF_WATCH_S`] after the first switch from
/// `ToCopter` into `Copter`.
#[derive(Debug, Default)]
struct HandoffStep {
    previous_mode: Option<FlightMode>,
    watched_s: Option<f32>,
    previous_moments: Option<Vec3>,
    largest: f32,
}

impl TelemetryPublisher for HandoffStep {
    fn publish(&mut self, record: &TelemetryRecord) {
        let mode = record.flight_mode.mode;
        if self.watched_s.is_none()
            && mode == FlightMode::Copter
            && self.previous_mode == Some(FlightMode::ToCopter)
        {
            self.watched_s = Some(0.0);
        }
        self.previous_mode = Some(mode);
        let Some(watched_s) = self.watched_s.as_mut() else {
            return;
        };
        if *watched_s >= HANDOFF_WATCH_S {
            return;
        }
        *watched_s += record.dt.as_secs_f32();
        let moments = record.control.moments;
        if let Some(previous) = self.previous_moments {
            self.largest = self.largest.max((moments - previous).norm());
        }
        self.previous_moments = Some(moments);
    }
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
/// transition time and altitude loss and within [`BACK_CLIMB_M`], handing over with the sink
/// guard idle (not sinking past its limit), then holds it within the copter's calm-air bands
/// (`hover.rs`).
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
    check_climb(&flight, BACK_AT_S, copter_s, "back");
    // Measured 0.8 m/s of sink.
    let sink_limit_mps = airframe.guidance.transition.back_sink_limit_mps;
    let handover_climb_mps = flight
        .at(copter_s)
        .map_or(f32::NAN, |sample| sample.state.velocity_mps.z);
    check!(
        handover_climb_mps >= -sink_limit_mps,
        "sinking {handover_climb_mps} m/s"
    );

    let point_m = Vec3::new(BACK_HOVER_NORTH_M, 0.0, TRANSITION_ALTITUDE_M);
    let hold_s = copter_s + BACK_SETTLE_S;
    let mut worst = HoverDeviation::default();
    for sample in flight.window(hold_s, hold_s + HOLD_S) {
        worst.observe(&sample.state, point_m);
    }
    check_calm_hover(&worst, "back");
}

/// Handing thrust back to the hover after a back transition doesn't step the moments: the
/// altitude loop's zero thrust on the way isn't ground idle, so the stabilizer keeps its
/// integrals (#326). Flown on the core default ramp (50°/s, done at 45°), which arrives
/// climbing 6 m/s (#343) so the altitude loop cuts thrust to 0.
#[test]
fn back_handoff_keeps_the_hover_trim() {
    let mut airframe = airframes::tailsitter();
    let fast = TransitionConfig::default();
    airframe.guidance.transition = TransitionConfig {
        back_rate_radps: fast.back_rate_radps,
        back_done_rad: fast.back_done_rad,
        back_thrust: fast.back_thrust,
        ..airframe.guidance.transition
    };
    let plant = TailsitterPlant::clean(airframe);
    let commander = forward_and_back(&airframe, BACK_AT_S, BACK_HOVER_NORTH_M);
    let mut flight_loop = tailsitter_pipeline(&plant, commander, HandoffStep::default());
    fly(
        &mut flight_loop,
        BACK_AT_S + BACK_DURATION_S + HANDOFF_WATCH_S,
        |_, _| {},
    );

    let step = flight_loop.into_parts().5;
    check!(step.watched_s.is_some(), "never handed over: {step:?}");
    check!(step.largest <= HANDOFF_STEP, "{step:?}");
}

/// Thrust doesn't reach zero in the air from the back request until [`ALTITUDE_LOSS_AFTER_S`]
/// after the handover: a tailsitter's elevons fly in the propwash, and an altitude loop cutting
/// the motors to shed a climb leaves it without hover control (#343).
#[test]
fn back_transition_keeps_thrust_on() {
    let airframe = airframes::tailsitter();
    let plant = TailsitterPlant::clean(airframe);
    let commander = forward_and_back(&airframe, BACK_AT_S, BACK_HOVER_NORTH_M);
    let mut flight_loop = tailsitter_pipeline(&plant, commander, LeastThrust::default());
    fly(
        &mut flight_loop,
        BACK_AT_S + BACK_DURATION_S + ALTITUDE_LOSS_AFTER_S,
        |_, _| {},
    );

    let least = flight_loop.into_parts().5;
    check!(
        least.handed_over_s.is_some(),
        "never handed over: {least:?}"
    );
    // Measured 0.6, the ramp's thrust.
    check!(least.thrust > 0.0, "{least:?}");
}

/// The least setpoint thrust from the first `ToCopter` tick until [`ALTITUDE_LOSS_AFTER_S`]
/// after the switch into `Copter`.
#[derive(Debug)]
struct LeastThrust {
    watching: bool,
    handed_over_s: Option<f32>,
    thrust: f32,
}

impl Default for LeastThrust {
    fn default() -> Self {
        LeastThrust {
            watching: false,
            handed_over_s: None,
            thrust: f32::INFINITY,
        }
    }
}

impl TelemetryPublisher for LeastThrust {
    fn publish(&mut self, record: &TelemetryRecord) {
        let mode = record.flight_mode.mode;
        if mode == FlightMode::ToCopter && self.handed_over_s.is_none() {
            self.watching = true;
        }
        if !self.watching {
            return;
        }
        if mode == FlightMode::Copter {
            let handed_over_s = self.handed_over_s.get_or_insert(0.0);
            *handed_over_s += record.dt.as_secs_f32();
            if *handed_over_s > ALTITUDE_LOSS_AFTER_S {
                self.watching = false;
                return;
            }
        }
        self.thrust = self.thrust.min(record.setpoint.thrust());
    }
}

/// A back ramp with too little thrust to hold the nose up stalls and sinks; the sink guard
/// raises its thrust, so it still hands over within PX4's back transition time and altitude
/// loss.
#[test]
fn sink_guard_catches_a_stalling_back_ramp() {
    for back_thrust in STALLING_BACK_THRUSTS {
        let mut airframe = airframes::tailsitter();
        airframe.guidance.transition.back_thrust = back_thrust;
        let plant = TailsitterPlant::clean(airframe);
        let commander = forward_and_back(&airframe, BACK_AT_S, BACK_HOVER_NORTH_M);
        let mut flight_loop = tailsitter_loop(&plant, commander);
        let flight = trace(
            &mut flight_loop,
            BACK_AT_S + BACK_DURATION_S + HOVER_SETTLE_S,
        );

        let what = format!("back thrust {back_thrust}");
        let copter_s = entered_s(&flight, FlightMode::Copter, BACK_AT_S, &what);
        // Measured 3.3 s.
        check!(copter_s - BACK_AT_S <= BACK_DURATION_S, "{what}");
        check_altitude_loss(&flight, BACK_AT_S, copter_s, &what);
    }
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
