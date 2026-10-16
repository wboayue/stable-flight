//! The Alia 250's plane side on its sim model ([`airframes::alia250`], fitted to X-Plane, #392):
//! what X-Plane showed on the same controllers, here first so the fixes can be made on the sim.

mod common;

use std::cell::RefCell;
use std::rc::Rc;

use assert2::check;
use stable_flight::guidance::vtol::{QuadPlaneGuidance, QuadPlaneGuidanceConfig};
use stable_flight::stabilizers::vtol::QuadPlaneStabilizer;
use stable_flight::{FlightLoop, FlightMode, TelemetryPublisher, TelemetryRecord};
use stable_flight_sim::{SimQuadPlane, airframes};

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
        .with_start(airframe.start);
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

/// X-Plane's Alia limit-cycles in level flight on these controllers: ±10–15 m and ±7–11 m/s of
/// climb at about 8 s (#392, #454). The sim model does the same.
#[test]
#[ignore = "the altitude and airspeed loops limit-cycle on the Alia; TECS (#454)"]
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
    // #454's done-when band.
    check!(high - low < 10.0);
    check!(climb < 1.5);
}

/// Asked to climb 150 m at 40 m/s, X-Plane's Alia sank instead: pitch saturated at the
/// elevator's reach and the airspeed loop held the speed by cutting the pusher (#454; it climbs
/// 2.4 m/s at full power). The sim model does the same.
#[test]
#[ignore = "the airspeed loop takes the climb's power at the pitch limit; TECS (#454)"]
fn alia250_climbs_at_the_pitch_limit() {
    let ticks = line(450.0, 40.0, 90.0);
    // Over the last minute: at least 1 m/s on average (2.4 m/s at full power, X-Plane).
    let tail = &ticks[ticks.len().saturating_sub(common::ticks(60.0))..];
    check!(!tail.is_empty());
    let mean_climb = tail.iter().map(|t| t.1).sum::<f32>() / tail.len() as f32;
    let end = tail.last().map_or(f32::NAN, |t| t.0);
    println!("mean climb {mean_climb:.2} m/s, ends at {end:.0} m");
    check!(mean_climb > 1.0);
}
