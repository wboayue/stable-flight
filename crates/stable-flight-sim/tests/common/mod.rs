//! Wiring shared by the closed-loop tests: the `quad_x` pipeline on a `SimVehicle`, flown in
//! simulated time.

use std::time::Duration;

use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::mixers::TableMixer;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::testing::RecordingTelemetry;
use stable_flight::{Commander, FlightLoop, Vec2, WaypointCommand};
use stable_flight_sim::{SimVehicle, Wind, airframes};

/// Loop rate of the RealFlight examples.
pub const TICK: Duration = Duration::from_millis(4);

/// Ticks of telemetry kept.
pub const RECORDED: usize = 64;

pub type SimLoop<C> = FlightLoop<
    C,
    SimVehicle,
    CopterGuidance,
    CopterStabilizer,
    TableMixer<4>,
    RecordingTelemetry<RECORDED>,
>;

/// The `quad_x` airframe on the ground at the origin in `wind`, flown by `commander`, primed.
pub fn sim_loop<C: Commander>(commander: C, wind: Wind) -> SimLoop<C> {
    let airframe = airframes::quad_x();
    let vehicle = SimVehicle::new(airframe.model, &airframe.mixer)
        .expect("valid airframe")
        .with_wind(wind);
    let mut flight_loop = FlightLoop::new(
        commander,
        vehicle,
        CopterGuidance::new(airframe.guidance),
        CopterStabilizer::new(airframe.stabilizer),
        airframe.mixer,
        RecordingTelemetry::new(),
    );
    flight_loop.prime();
    flight_loop
}

/// Steps `flight_loop` for `seconds` of simulated time, calling `each` after every tick.
pub fn fly<C: Commander>(
    flight_loop: &mut SimLoop<C>,
    seconds: f32,
    mut each: impl FnMut(&SimLoop<C>),
) {
    let ticks = (seconds / TICK.as_secs_f32()).round() as usize;
    for _ in 0..ticks {
        flight_loop.step(TICK);
        each(flight_loop);
    }
}

/// A waypoint over `north_m`, `east_m` at `altitude_m`, stopping on it.
pub fn waypoint(north_m: f32, east_m: f32, altitude_m: f32) -> WaypointCommand {
    WaypointCommand {
        position_m: Vec2::new(north_m, east_m),
        altitude_m,
        ..Default::default()
    }
}
