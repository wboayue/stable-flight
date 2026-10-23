//! The Alia 250's plane side on its sim model ([`airframes::alia250`], fitted to X-Plane, #392):
//! level flight and a pitch-limited climb, which separate altitude and airspeed loops failed in
//! X-Plane and total energy control flies (#454); PX4's attitude step spec; altitude steps,
//! turn entry, and entry into plane flight, with the pitch side's margin.

mod common;

use std::cell::RefCell;
use std::rc::Rc;

use assert2::check;
use stable_flight::guidance::vtol::{
    LiftTransitionConfig, QuadPlaneGuidance, QuadPlaneGuidanceConfig,
};
use stable_flight::stabilizers::vtol::QuadPlaneStabilizer;
use stable_flight::tuning::StepConfig;
use stable_flight::{
    Commander, ControlInput, FlightLoop, FlightMode, TelemetryPublisher, TelemetryRecord, Vec2,
    WaypointCommand,
};
use stable_flight_sim::airframes::QuadPlaneAirframe;
use stable_flight_sim::{SimQuadPlane, Start, airframes};

use common::fly;
use common::plane::{Interlude, north_line, north_line_command, plane_command};
use common::plane_spec;
use common::quadplane::QuadPlanePlant;

/// Altitude, climb rate, and airspeed on each plane-flight tick.
#[derive(Clone, Default)]
struct Hold(Rc<RefCell<Vec<(f32, f32, f32)>>>);

impl TelemetryPublisher for Hold {
    fn publish(&mut self, record: &TelemetryRecord) {
        if record.flight_mode.mode == FlightMode::Plane {
            let state = &record.state;
            self.0.borrow_mut().push((
                state.position_m.z,
                state.velocity_mps.z,
                state.valid_airspeed_mps().unwrap_or(f32::NAN),
            ));
        }
    }
}

/// The Alia flying a line north at `altitude_m` and `airspeed_mps` from a level start at 300 m
/// and its trim airspeed, plane flight asked for at once (the transition's height floor off,
/// the airspeed already past its gate). Returns each plane tick's altitude, climb rate, and
/// airspeed.
fn line(altitude_m: f32, airspeed_mps: f32, seconds: f32) -> Vec<(f32, f32, f32)> {
    line_from(
        airframes::alia250().start,
        altitude_m,
        airspeed_mps,
        seconds,
    )
}

/// [`line`] from `start`.
fn line_from(
    start: Start,
    altitude_m: f32,
    airspeed_mps: f32,
    seconds: f32,
) -> Vec<(f32, f32, f32)> {
    let commander = north_line(altitude_m, airspeed_mps);
    let hold = Hold::default();
    flown(
        airframes::alia250(),
        start,
        commander,
        hold.clone(),
        seconds,
        |_, _| {},
    );
    hold.0.take()
}

/// `airframe` with the transition's height floors off: plane flight asked for, and flown, at
/// any height above where it was primed in the air.
fn floorless(airframe: QuadPlaneAirframe) -> QuadPlaneAirframe {
    QuadPlaneAirframe {
        guidance: QuadPlaneGuidanceConfig {
            transition: LiftTransitionConfig {
                min_height_m: 0.0,
                plane_min_height_m: 0.0,
                ..airframe.guidance.transition
            },
            ..airframe.guidance
        },
        ..airframe
    }
}

/// `airframe` ([`floorless`]) from `start`, flown by `commander` for `seconds`, publishing to
/// `telemetry` and calling `each` with each tick's altitude and airspeed and the seconds flown.
fn flown<C: Commander, T: TelemetryPublisher>(
    airframe: QuadPlaneAirframe,
    start: Start,
    commander: C,
    telemetry: T,
    seconds: f32,
    mut each: impl FnMut((f32, f32), f32),
) {
    let airframe = floorless(airframe);
    let vehicle = SimQuadPlane::new(airframe.model, &airframe.mixer)
        .expect("valid airframe")
        .with_start(start);
    let mut flight_loop = FlightLoop::new(
        commander,
        vehicle,
        QuadPlaneGuidance::new(airframe.guidance),
        QuadPlaneStabilizer::new(airframe.stabilizer),
        airframe.mixer,
        telemetry,
    );
    flight_loop.prime();
    fly(&mut flight_loop, seconds, |flight_loop, t| {
        let state = flight_loop.state();
        each(
            (state.position_m.z, state.airspeed_mps.unwrap_or(f32::NAN)),
            t,
        );
    });
}

/// Level flight holds without a limit cycle: within #454's ±5 m and 1.5 m/s of climb rate over
/// the last two minutes. Separate altitude and airspeed loops swung ±10–15 m and ±7–11 m/s at
/// about 8 s in X-Plane (#392), 290–305 m and 5.6 m/s here; total energy control holds
/// 299.7–300.1 m and 0.1 m/s.
#[test]
fn alia250_holds_level_flight() {
    let airframe = airframes::alia250();
    let speed = airframe.stabilizer.plane.trim_airspeed_mps;
    let ticks = line(airframe.start.position_m.z, speed, 180.0);
    // The last two minutes.
    let tail = &ticks[ticks.len().saturating_sub(common::ticks(120.0))..];
    // Plane flight reached: an empty tail would pass both checks.
    check!(!tail.is_empty());
    let (low, high) = tail
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), t| {
            (lo.min(t.0), hi.max(t.0))
        });
    let climb = tail.iter().map(|t| t.1.abs()).fold(0.0, f32::max);
    println!(
        "altitude {low:.1}..{high:.1} m, |climb| max {climb:.1} m/s, ticks {}",
        ticks.len()
    );
    check!(high - low < 10.0);
    check!(climb < 1.5);
}

/// Asked to climb 150 m at 40 m/s, the pitch limit can't make the climb alone: thrust rises
/// to full and it climbs, at least 1 m/s over the last minute (#454). Separate loops sank
/// instead, the airspeed loop cutting the pusher (X-Plane 262 → 149 m; here into the ground);
/// total energy control climbs 1.7 m/s, near the model's full-power climb (1.9 m/s at
/// 40.8 m/s; X-Plane 2.4 m/s).
#[test]
fn alia250_climbs_at_the_pitch_limit() {
    let ticks = line(450.0, 40.0, 90.0);
    let tail = &ticks[ticks.len().saturating_sub(common::ticks(60.0))..];
    check!(!tail.is_empty());
    let mean_climb = tail.iter().map(|t| t.1).sum::<f32>() / tail.len() as f32;
    let end = tail.last().map_or(f32::NAN, |t| t.0);
    println!("mean climb {mean_climb:.2} m/s, ends at {end:.0} m");
    check!(mean_climb > 1.0);
}

/// Most altitude lost from entering plane flight, in m, out of a forward transition from near
/// a hover: PX4's transition altitude loss (`VT_QC_T_ALT_LOSS`, 20 m,
/// `vtol_att_control_params.yaml`). Measured 12 m, handing over at 43.7 m/s. Handed over below
/// the wing's speed (37.5 m/s) it lost 37 m, and 296 m once the pitch loop tracked; its energy
/// control carried into plane flight from the transition lost 108 m (X-Plane: 90 m).
const ENTRY_LOSS_M: f32 = 20.0;

/// Entering plane flight out of the forward transition starts its energy control fresh: what
/// it built flying from no airspeed doesn't pitch the Alia down at entry (#454).
#[test]
fn alia250_enters_plane_flight_without_the_transitions_energy_state() {
    let loss_m = entry_loss_m(airframes::alia250());
    check!(loss_m <= ENTRY_LOSS_M, "{loss_m} m");
}

/// Altitude lost from entering plane flight on `airframe`, in m, out of a forward transition
/// from near a hover at 300 m.
fn entry_loss_m(airframe: QuadPlaneAirframe) -> f32 {
    let hold = Hold::default();
    let line = north_line(300.0, airframe.stabilizer.plane.trim_airspeed_mps);
    let start = Start::level(300.0, 5.0, 0.0);
    flown(airframe, start, line, hold.clone(), 150.0, |_, _| {});
    let ticks = hold.0.take();
    let entry_m = ticks.first().map_or(f32::NAN, |t| t.0);
    let lowest_m = ticks.iter().map(|t| t.0).fold(f32::INFINITY, f32::min);
    entry_m - lowest_m
}

/// Most altitude lost entering plane flight with the pitch side's gains raised (pitch rate P
/// 1.5×, TECS pitch damping 2×), in m. Measured 21 m; 30 m leaves about 1.5× margin. With a
/// pitch-up limit past the stall (24°, px4xplane's), damping 0.8 at P 0.52 already fell into a
/// full-elevator stall cycle to the ground (298 m, #392).
const MARGIN_ENTRY_LOSS_M: f32 = 30.0;

/// The pitch side has margin: raised gains still enter plane flight without the stall cycle.
#[test]
fn alia250_pitch_side_has_margin() {
    let mut airframe = airframes::alia250();
    airframe.stabilizer.plane.rate.pitch.p *= 1.5;
    airframe.guidance.plane.tecs.pitch_damping *= 2.0;
    let loss_m = entry_loss_m(airframe);
    check!(loss_m <= MARGIN_ENTRY_LOSS_M, "{loss_m} m");
}

/// The Alia's attitude steps: a 20° bank (its bank limit) and 5° of pitch, both ways, from the
/// line at trim.
const STEPS: [StepConfig; 2] = [
    StepConfig {
        amplitude: 20.0 * core::f32::consts::PI / 180.0,
        ..plane_spec::ROLL_STEP
    },
    StepConfig {
        amplitude: 5.0 * core::f32::consts::PI / 180.0,
        ..plane_spec::PITCH_STEP
    },
];

/// The roll step's overshoot allowance: measured 11% (the spec's is 5%), 15% with margin. It
/// runs through the model's lateral-directional terms, not yet measured: 16% with twice the
/// rudder feedforward, 7% without it.
// TODO(#458): the spec's 5% once yaw damping and sideslip are fitted to X-Plane.
const ROLL_OVERSHOOT: f32 = 0.15;

/// Roll and pitch steps meet PX4's step spec (`common::plane_spec`), rise against each axis's
/// time constant; roll's overshoot within [`ROLL_OVERSHOOT`].
#[test]
fn alia250_meets_the_plane_step_spec() {
    let plant = QuadPlanePlant::clean(floorless(airframes::alia250()));
    let overshoot = [ROLL_OVERSHOOT, common::spec::MAX_OVERSHOOT];
    let misses = plane_spec::misses_on(&plant, STEPS, overshoot);
    check!(misses.is_empty(), "{misses:#?}");
}

/// A step's overshoot past the new altitude (m, its direction), the seconds until it stays
/// within [`STEP_BAND_M`], and its largest airspeed error (m/s).
fn altitude_step(step_m: f32) -> (f32, f32, f32) {
    const LEAD_S: f32 = 30.0;
    let airspeed_mps = airframes::alia250().stabilizer.plane.trim_airspeed_mps;
    let commander = Interlude::new(
        north_line(300.0, airspeed_mps),
        LEAD_S,
        f32::INFINITY,
        north_line_command(300.0 + step_m, airspeed_mps),
    );
    let target_m = 300.0 + step_m;
    let (mut overshoot_m, mut settled_s, mut airspeed_error) = (0f32, 0f32, 0f32);
    let start = airframes::alia250().start;
    flown(
        airframes::alia250(),
        start,
        commander,
        (),
        150.0,
        |(z, v), t| {
            if t < LEAD_S {
                return;
            }
            overshoot_m = overshoot_m.max((z - target_m) * step_m.signum());
            if (z - target_m).abs() > STEP_BAND_M {
                settled_s = t - LEAD_S;
            }
            airspeed_error = airspeed_error.max((v - airspeed_mps).abs());
        },
    );
    (overshoot_m, settled_s, airspeed_error)
}

/// The altitude band a step settles into, in m.
const STEP_BAND_M: f32 = 2.0;

/// ±30 m altitude steps: overshoot, settle into [`STEP_BAND_M`], and airspeed error, each
/// measured with margin (no external band applies): (step m, most overshoot m, settle s, most
/// airspeed error m/s). Measured up 0.06 m / 18.6 s (the 1.8 m/s climb limit alone takes
/// 16.7 s) / 0.21 m/s, down 2.3 m / 12.6 s / 0.78 m/s. The pitch loop before #392's tuning
/// overshot the descent 7.4 m and took 23 s.
#[test]
fn alia250_steps_altitude() {
    for (step_m, overshoot_m, settle_s, airspeed_mps) in
        [(30.0, 1.0, 25.0, 1.0), (-30.0, 4.0, 18.0, 1.5)]
    {
        let (over, settled, error) = altitude_step(step_m);
        check!(over <= overshoot_m, "{step_m}: {over}");
        check!(settled <= settle_s, "{step_m}: {settled}");
        check!(error <= airspeed_mps, "{step_m}: {error}");
    }
}

/// Most altitude lost or gained entering a loiter's 20° bank from the line, in m. Measured
/// 0.34 m low and 0.12 m high; 1 m leaves about 3× margin. The pitch loop before #392's tuning
/// dipped 1.6 m (X-Plane, 2.6–4 m).
const TURN_ALTITUDE_M: f32 = 1.0;

/// Entering a turn holds altitude: the line north, then a loiter to its right.
#[test]
fn alia250_enters_a_turn_holding_altitude() {
    const LEAD_S: f32 = 30.0;
    let airframe = airframes::alia250();
    let airspeed_mps = airframe.stabilizer.plane.trim_airspeed_mps;
    let loiter = plane_command(ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(2000.0, airframe.guidance.plane.loiter_radius_m),
        velocity_mps: None,
        altitude_m: 300.0,
        cruise_speed_mps: airspeed_mps,
        heading_rad: None,
    }));
    let commander = Interlude::new(
        north_line(300.0, airspeed_mps),
        LEAD_S,
        f32::INFINITY,
        loiter,
    );
    let mut worst_m = 0f32;
    flown(
        airframe,
        airframe.start,
        commander,
        (),
        200.0,
        |(z, _), t| {
            if t >= LEAD_S {
                worst_m = worst_m.max((z - 300.0).abs());
            }
        },
    );
    check!(worst_m <= TURN_ALTITUDE_M, "{worst_m} m");
}
