//! Transitions on `airframes::quadplane()`: forward from a hover to plane flight, back to a
//! hover, and the ways a forward transition ends early, against PX4's VTOL transition limits
//! (PX4 `c4e4ef98e9`, `src/modules/vtol_att_control/vtol_att_control_params.yaml`).

mod common;

use assert2::check;
use stable_flight::commanders::{
    FailsafeCommander, FailsafeConfig, HoldCommander, LinkedCommander,
};
use stable_flight::{Command, Commander, FlightMode, ModeRefusal, Vec3, VehicleState};
use stable_flight_sim::airframes;

use common::plane::{AIRSPEED_BAND_MPS, ALTITUDE_BAND_M, LineDeviation};
use common::quadplane::{QuadPlanePlant, forward, forward_and_back, quadplane_loop};
use common::transition::{
    BACK_DURATION_S, CROSS_TRACK_M, FORWARD_AT_S, FORWARD_TIMEOUT_S, Forward, HOLD_S,
    HoverDeviation, PLANE_HOLD_S, PLANE_SETTLE_S, TRANSITION_ALTITUDE_M, Trace,
    check_altitude_loss, entered_s, trace, transition_heights,
};
use common::{CALM_ALTITUDE_M, CALM_ANGLE_DEG, CALM_HORIZONTAL_M, HOVER_SETTLE_S, TICK};

/// When the back transition is requested: 32 s into plane flight, settled on the line.
const BACK_AT_S: f32 = 60.0;

/// Where the hover after the back transition holds, m north on the line: about where it comes
/// to a stop. The back transition hands over at 634 m, at 5.0 m/s (measured). A point behind it
/// has the copter fly the wing tail first, and one far ahead nose down, which sinks it (#427).
const BACK_HOVER_NORTH_M: f32 = 640.0;

/// Seconds the hover settles after the back transition before the hold is judged. It arrives
/// at 5 m/s: measured 0.06 m off after 20 s, 0.09 after 15.
const BACK_SETTLE_S: f32 = 20.0;

/// A pusher thrust too low to reach the airspeed gate: measured 2.3 m/s at most.
const WEAK_PUSHER_THRUST: f32 = 0.2;

/// Seconds into the forward transition that the link goes silent.
const SILENT_AFTER_S: f32 = 2.0;

/// Checks a hold over `point_m` against the copter's calm-air hover bands (`hover.rs`).
fn check_calm_hover(trace: &Trace, from_s: f32, point_m: Vec3, what: &str) {
    let mut worst = HoverDeviation::default();
    for sample in trace.window(from_s, from_s + HOLD_S) {
        worst.observe_level(&sample.state, point_m);
    }
    check!(worst.altitude_m < CALM_ALTITUDE_M, "{what}: {worst:?}");
    check!(worst.horizontal_m < CALM_HORIZONTAL_M, "{what}: {worst:?}");
    check!(worst.tilt_deg < CALM_ANGLE_DEG, "{what}: {worst:?}");
}

/// From a hover, `Plane` on the north line reaches plane flight within PX4's timeout and
/// altitude loss, then holds the line as the airplane does.
#[test]
fn transitions_forward_into_level_flight() {
    let airframe = airframes::quadplane();
    let plant = QuadPlanePlant::clean(airframe);
    let mut flight_loop = quadplane_loop(&plant, forward(&airframe));
    let flight = trace(
        &mut flight_loop,
        FORWARD_AT_S + FORWARD_TIMEOUT_S + PLANE_SETTLE_S + PLANE_HOLD_S,
    );

    let plane_s = entered_s(&flight, FlightMode::Plane, FORWARD_AT_S, "forward");
    // Measured 8.2 s, losing 0.3 m.
    check!(plane_s - FORWARD_AT_S <= FORWARD_TIMEOUT_S);
    check_altitude_loss(
        transition_heights(&flight, FORWARD_AT_S, plane_s),
        "forward",
    );

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
/// transition time and altitude loss, then holds it within the copter's calm-air bands.
#[test]
fn transitions_back_into_a_hover() {
    let airframe = airframes::quadplane();
    let plant = QuadPlanePlant::clean(airframe);
    let commander = forward_and_back(&airframe, BACK_AT_S, BACK_HOVER_NORTH_M);
    let mut flight_loop = quadplane_loop(&plant, commander);
    let flight = trace(
        &mut flight_loop,
        BACK_AT_S + BACK_DURATION_S + BACK_SETTLE_S + HOLD_S,
    );

    // The commander's clock sums `f32` ticks, so its switch runs a little early.
    let back_s = entered_s(&flight, FlightMode::ToCopter, BACK_AT_S - 1.0, "back");
    let copter_s = entered_s(&flight, FlightMode::Copter, back_s, "back");
    // Measured 7.3 s, losing 0.5 m.
    check!(copter_s - back_s <= BACK_DURATION_S);
    check_altitude_loss(transition_heights(&flight, back_s, copter_s), "back");

    let point_m = Vec3::new(BACK_HOVER_NORTH_M, 0.0, TRANSITION_ALTITUDE_M);
    check_calm_hover(&flight, copter_s + BACK_SETTLE_S, point_m, "back");
}
/// With too little pusher thrust to reach the airspeed gate, the forward transition waits on
/// the lift rotors and times out to `Copter` within PX4's altitude loss.
#[test]
fn times_out_short_of_the_airspeed_gate() {
    let mut airframe = airframes::quadplane();
    airframe.guidance.transition.forward_thrust = WEAK_PUSHER_THRUST;
    let plant = QuadPlanePlant::clean(airframe);
    let mut flight_loop = quadplane_loop(&plant, forward(&airframe));
    let flight = trace(
        &mut flight_loop,
        FORWARD_AT_S + FORWARD_TIMEOUT_S + HOVER_SETTLE_S,
    );

    let fastest_mps = flight
        .window(FORWARD_AT_S, FORWARD_AT_S + FORWARD_TIMEOUT_S)
        .filter_map(|sample| sample.state.airspeed_mps)
        .fold(0.0, f32::max);
    check!(fastest_mps < airframe.guidance.transition.min_airspeed_mps);
    let ramp_s = entered_s(&flight, FlightMode::ToPlane, FORWARD_AT_S, "forward");
    check!(flight.entered(FlightMode::Plane, ramp_s).is_none());
    let ended = flight.entered(FlightMode::Copter, ramp_s);
    check!(ended.map(|sample| sample.refusal) == Some(Some(ModeRefusal::TransitionTimeout)));
    let copter_s = ended.map_or(f32::INFINITY, |sample| sample.time_s);
    // The timeout trips on the tick after it runs out.
    let tolerance_s = 2.0 * TICK.as_secs_f32();
    check!((copter_s - ramp_s - FORWARD_TIMEOUT_S).abs() < tolerance_s);
    // Measured 6.6 m: the copter then flies the line nose down, and the wing sinks it (#427).
    check_altitude_loss(transition_heights(&flight, ramp_s, copter_s), "timeout");
}

/// A link flying [`Forward`] that goes silent [`SILENT_AFTER_S`] into the forward transition.
struct SilentLink {
    commander: Forward,
    elapsed_s: f32,
}

impl Commander for SilentLink {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        self.elapsed_s += dt;
        self.commander.next_command(dt, state)
    }
}

impl LinkedCommander for SilentLink {
    fn since_update_s(&self) -> Option<f32> {
        Some((self.elapsed_s - FORWARD_AT_S - SILENT_AFTER_S).max(0.0))
    }

    fn sync_arm(&mut self, _armed: bool) {}
}

/// The link going silent mid forward transition hands over to `HoldCommander`, which asks for
/// `Copter`: the transition ends back on the lift rotors within PX4's altitude loss, and the
/// hold stops it and keeps it there.
#[test]
fn link_loss_mid_transition_returns_to_a_hover() {
    let airframe = airframes::quadplane();
    let plant = QuadPlanePlant::clean(airframe);
    let link = SilentLink {
        commander: forward(&airframe),
        elapsed_s: 0.0,
    };
    let config = FailsafeConfig::default();
    let commander = FailsafeCommander::new(link, HoldCommander::default(), config)
        .expect("valid failsafe config");
    let mut flight_loop = quadplane_loop(&plant, commander);
    let flight = trace(
        &mut flight_loop,
        FORWARD_AT_S + FORWARD_TIMEOUT_S + HOVER_SETTLE_S + HOLD_S,
    );

    let ramp_s = entered_s(&flight, FlightMode::ToPlane, FORWARD_AT_S, "forward");
    let ended = flight.entered(FlightMode::Copter, ramp_s);
    check!(ended.map(|sample| sample.refusal) == Some(None));
    let copter_s = ended.map_or(f32::INFINITY, |sample| sample.time_s);
    let silent_s = FORWARD_AT_S + SILENT_AFTER_S;
    // The failsafe trips on the tick after the timeout runs out.
    check!(copter_s - silent_s <= config.timeout_s + 2.0 * TICK.as_secs_f32());
    // Measured 0.1 m.
    check_altitude_loss(transition_heights(&flight, ramp_s, copter_s), "link loss");

    // The hold latches where the fallback engaged.
    let point_m = flight
        .at(copter_s)
        .map_or(Vec3::repeat(f32::NAN), |sample| sample.state.position_m);
    check_calm_hover(&flight, copter_s + HOVER_SETTLE_S, point_m, "link loss");
}
