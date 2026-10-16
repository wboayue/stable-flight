//! The Alia 250's plane side on its sim model ([`airframes::alia250`], fitted to X-Plane, #392):
//! level flight and a pitch-limited climb, which separate altitude and airspeed loops failed in
//! X-Plane and total energy control flies (#454).

mod common;

use std::cell::RefCell;
use std::rc::Rc;

use assert2::check;
use stable_flight::guidance::vtol::{QuadPlaneGuidance, QuadPlaneGuidanceConfig};
use stable_flight::stabilizers::vtol::QuadPlaneStabilizer;
use stable_flight::{FlightLoop, FlightMode, TelemetryPublisher, TelemetryRecord};
use stable_flight_sim::{SimQuadPlane, Start, airframes};

use common::fly;
use common::plane::north_line;

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
    let airframe = airframes::alia250();
    let guidance = QuadPlaneGuidanceConfig {
        transition: stable_flight::guidance::vtol::LiftTransitionConfig {
            min_height_m: 0.0,
            ..airframe.guidance.transition
        },
        ..airframe.guidance
    };
    let vehicle = SimQuadPlane::new(airframe.model, &airframe.mixer)
        .expect("valid airframe")
        .with_start(start);
    let hold = Hold::default();
    let mut flight_loop = FlightLoop::new(
        north_line(altitude_m, airspeed_mps),
        vehicle,
        QuadPlaneGuidance::new(guidance),
        QuadPlaneStabilizer::new(airframe.stabilizer),
        airframe.mixer,
        hold.clone(),
    );
    flight_loop.prime();
    fly(&mut flight_loop, seconds, |_, _| {});
    hold.0.take()
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
/// a hover. Measured 41 m: the transition hands over at 37.5 m/s, below the 40 m/s the wing
/// holds level at, so it dives for speed. 60 m leaves about 1.5× margin. Its energy control
/// carried into plane flight from the transition lost 108 m (X-Plane: 90 m at full nose-down).
// TODO(#392): PX4's 20 m (`VT_QC_T_ALT_LOSS`) once the transition hands over at a speed the
// wing flies.
const ENTRY_LOSS_M: f32 = 60.0;

/// Entering plane flight out of the forward transition starts its energy control fresh: what
/// it built flying from no airspeed doesn't pitch the Alia down at entry (#454).
#[test]
fn alia250_enters_plane_flight_without_the_transitions_energy_state() {
    let ticks = line_from(Start::level(300.0, 5.0, 0.0), 300.0, 45.0, 120.0);
    let entry_m = ticks.first().map_or(f32::NAN, |t| t.0);
    let lowest_m = ticks.iter().map(|t| t.0).fold(f32::INFINITY, f32::min);
    check!(entry_m - lowest_m <= ENTRY_LOSS_M, "{entry_m} → {lowest_m}");
}
