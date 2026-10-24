//! Transitions on `airframes::quadplane()`: forward from a hover to plane flight, back to a
//! hover, and the ways a forward transition ends early, against PX4's VTOL transition limits
//! (PX4 `c4e4ef98e9`, `src/modules/vtol_att_control/vtol_att_control_params.yaml`); and a
//! fallback at cruise speed on `airframes::alia250()`.

mod common;

use assert2::check;
use stable_flight::commanders::{
    FailsafeCommander, FailsafeConfig, HoldCommander, LinkedCommander,
};
use stable_flight::guidance::copter::ALTITUDE_LEASH_M;
use stable_flight::{
    Command, Commander, FlightMode, FlightRegime, ModeRefusal, TelemetryPublisher, TelemetryRecord,
    Vec3, VehicleState,
};
use stable_flight_sim::{Wind, airframes};

use common::plane::{AIRSPEED_BAND_MPS, ALTITUDE_BAND_M, LineDeviation};
use common::quadplane::{
    QuadPlanePlant, forward, forward_and_back, quadplane_loop, quadplane_pipeline,
};
use common::transition::{
    BACK_DURATION_S, CROSS_TRACK_M, DIVE_AT_S, DIVE_WATCH_S, DivingLeg, FORWARD_AT_S,
    FORWARD_TIMEOUT_S, Forward, HOLD_S, PLANE_FLOOR_M, PLANE_HOLD_S, PLANE_SETTLE_S, SINK_AT_S,
    SINK_WATCH_S, TRANSITION_ALTITUDE_M, Trace, check_altitude_loss, check_falls_back, entered_s,
    sinking_leg, trace, transition_heights,
};
use common::{CALM_ALTITUDE_M, CALM_ANGLE_DEG, CALM_HORIZONTAL_M, HOVER_SETTLE_S, TICK, fly};

/// When the back transition is requested: 32 s into plane flight, settled on the line.
const BACK_AT_S: f32 = 60.0;

/// Where the back transition hands over, m north on the line: at 634 m, at 5.0 m/s (measured).
const HANDOVER_NORTH_M: f32 = 634.0;

/// How far the hover point after the back transition sits from the handover, ahead or
/// behind, in m: copter flight to it at speed, tail first behind (#427).
const BACK_HOVER_OFFSET_M: f32 = 120.0;

/// Seconds the hover settles after the back transition before the hold is judged: 120 m off
/// at 5 m/s, measured 0.07 m off at most after 35 s.
const BACK_SETTLE_S: f32 = 35.0;

/// A pusher thrust too low to reach the airspeed gate: measured 2.3 m/s at most.
const WEAK_PUSHER_THRUST: f32 = 0.2;

/// Seconds into the forward transition that the link goes silent.
const SILENT_AFTER_S: f32 = 2.0;

/// Checks a hold over `point_m` against the copter's calm-air hover bands (`hover.rs`).
fn check_calm_hover(trace: &Trace, from_s: f32, point_m: Vec3, what: &str) {
    let worst = trace.level_hold(from_s, point_m);
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

/// From plane flight, `Copter` on a hover point where it stops, ahead of it, or behind it
/// returns to a hover within PX4's back transition time and altitude loss, keeps within that
/// loss flying to the point, and holds it within the copter's calm-air bands.
#[test]
fn transitions_back_into_a_hover() {
    let airframe = airframes::quadplane();
    let rows = [
        ("where it stops", HANDOVER_NORTH_M),
        ("ahead", HANDOVER_NORTH_M + BACK_HOVER_OFFSET_M),
        ("behind", HANDOVER_NORTH_M - BACK_HOVER_OFFSET_M),
    ];
    for (case, north_m) in rows {
        let plant = QuadPlanePlant::clean(airframe);
        let commander = forward_and_back(&airframe, BACK_AT_S, north_m);
        let mut flight_loop = quadplane_loop(&plant, commander);
        let flight = trace(
            &mut flight_loop,
            BACK_AT_S + BACK_DURATION_S + BACK_SETTLE_S + HOLD_S,
        );

        // The commander's clock sums `f32` ticks, so its switch runs a little early.
        let back_s = entered_s(&flight, FlightMode::ToCopter, BACK_AT_S - 1.0, case);
        let copter_s = entered_s(&flight, FlightMode::Copter, back_s, case);
        // Measured 7.3 s.
        check!(copter_s - back_s <= BACK_DURATION_S, "{case}");
        // Measured 0.5–0.6 m.
        let hold_s = copter_s + BACK_SETTLE_S;
        check_altitude_loss(flight.heights(back_s, hold_s), case);

        let point_m = Vec3::new(north_m, 0.0, TRANSITION_ALTITUDE_M);
        check_calm_hover(&flight, hold_s, point_m, case);
    }
}

/// With too little pusher thrust to reach the airspeed gate, the forward transition waits on
/// the lift rotors and times out to `Copter` within PX4's altitude loss, then hovers where it
/// timed out within the copter's calm-air bands, though `Plane` is still asked for toward a
/// waypoint 100 km north (#453).
#[test]
fn times_out_short_of_the_airspeed_gate() {
    let mut airframe = airframes::quadplane();
    airframe.guidance.transition.forward_thrust = WEAK_PUSHER_THRUST;
    let plant = QuadPlanePlant::clean(airframe);
    let mut flight_loop = quadplane_loop(&plant, forward(&airframe));
    let flight = trace(
        &mut flight_loop,
        FORWARD_AT_S + FORWARD_TIMEOUT_S + HOVER_SETTLE_S + HOLD_S,
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
    // Measured 0.01 m.
    check_altitude_loss(transition_heights(&flight, ramp_s, copter_s), "timeout");
    if let Some(ended) = ended {
        let hold_s = copter_s + HOVER_SETTLE_S;
        check_calm_hover(&flight, hold_s, ended.state.position_m, "timeout");
    }
}

/// A plane leg sinking through the plane minimum height falls back to copter flight no
/// further than `FALLBACK_LOSS_M` below it, then hovers where the hover took over within the
/// copter's calm-air bands, `Plane` still refused (#450).
#[test]
fn falls_back_below_the_plane_minimum_height() {
    let mut airframe = airframes::quadplane();
    airframe.guidance.transition.plane_min_height_m = PLANE_FLOOR_M;
    let plant = QuadPlanePlant::clean(airframe);
    let commander = sinking_leg(airframe.stabilizer.plane.trim_airspeed_mps);
    let mut flight_loop = quadplane_loop(&plant, commander);
    let flight = trace(
        &mut flight_loop,
        SINK_AT_S + SINK_WATCH_S + HOVER_SETTLE_S + HOLD_S,
    );

    if let Some(hover) = check_falls_back(&flight, SINK_AT_S + SINK_WATCH_S, "fallback") {
        let hold_s = hover.time_s + HOVER_SETTLE_S;
        check_calm_hover(&flight, hold_s, hover.state.position_m, "fallback");
    }
}

/// The Alia's dive pitch, nose up positive: with [`ALIA_DIVE_THRUST`], 6.2 m/s of sink at
/// 53 m/s by the floor (measured), past its 5 m/s `PGUID_SINK_MAX`.
const ALIA_DIVE_PITCH_DEG: f32 = -7.0;

/// The Alia's pusher thrust through the dive, `[0, 1]`.
const ALIA_DIVE_THRUST: f32 = 0.5;

/// The Alia diving through the plane minimum height at cruise speed falls back no further than
/// `FALLBACK_LOSS_M` below it: the lift rotors take all the lift at once (#493). The hover
/// after is the quadplane's to judge: the Alia hands over on its back timeout, still at speed.
#[test]
fn falls_back_at_cruise_speed_on_full_lift() {
    let mut airframe = airframes::alia250();
    airframe.guidance.transition.plane_min_height_m = PLANE_FLOOR_M;
    let plant = QuadPlanePlant::clean(airframe);
    let commander = DivingLeg::new(
        airframe.stabilizer.plane.trim_airspeed_mps,
        ALIA_DIVE_PITCH_DEG.to_radians(),
        ALIA_DIVE_THRUST,
        PLANE_FLOOR_M,
    );
    let mut flight_loop = quadplane_loop(&plant, commander);
    let flight = trace(&mut flight_loop, DIVE_AT_S + DIVE_WATCH_S);

    check_falls_back(&flight, DIVE_AT_S + DIVE_WATCH_S, "fallback");
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

    fn sync_regime(&mut self, _regime: FlightRegime) {}
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

/// The forward transition's hover thrust tick to tick, and how far above the altitude it holds
/// the vehicle rose.
#[derive(Debug, Default)]
struct HoverThrust {
    hold_m: Option<f32>,
    previous: Option<f32>,
    largest_step: f32,
    above_hold_m: f32,
}

impl TelemetryPublisher for HoverThrust {
    fn publish(&mut self, record: &TelemetryRecord) {
        if record.flight_mode.mode != FlightMode::ToPlane {
            self.previous = None;
            return;
        }
        let altitude_m = record.state.position_m.z;
        let hold_m = *self.hold_m.get_or_insert(altitude_m);
        self.above_hold_m = self.above_hold_m.max(altitude_m - hold_m);
        let thrust = record.setpoint.thrust();
        if let Some(previous) = self.previous {
            self.largest_step = self.largest_step.max((thrust - previous).abs());
        }
        self.previous = Some(thrust);
    }
}

/// An updraft lifting a forward transition past the leash above the altitude it holds moves
/// the lift rotors' thrust smoothly: the descent feedforward stepped in there, half of full
/// thrust in one tick, and chattered the Alia's lift rotors in X-Plane (#445).
#[test]
fn updraft_in_a_transition_moves_the_hover_thrust_smoothly() {
    let airframe = airframes::quadplane();
    let updraft = Wind {
        // Strong enough to lift the transition past the leash under total energy control,
        // which holds the plane side's height against 1 m/s (0.4 m above; #454).
        steady_mps: Vec3::new(0.0, 0.0, 2.5),
        gust: None,
    };
    let plant = QuadPlanePlant::in_wind(airframe, updraft);
    let mut flight_loop = quadplane_pipeline(&plant, forward(&airframe), HoverThrust::default());
    fly(
        &mut flight_loop,
        FORWARD_AT_S + FORWARD_TIMEOUT_S,
        |_, _| {},
    );
    let (.., hover) = flight_loop.into_parts();

    // Measured 2.0 m: the transition crosses the leash.
    check!(hover.above_hold_m > ALTITUDE_LEASH_M, "{hover:?}");
    // Measured 0.003 (0.544 with the step, in 1 m/s).
    check!(hover.largest_step < 0.05, "{hover:?}");
}
