//! Wiring shared by the closed-loop tests: the `quad_x` pipeline on a `SimVehicle`, flown in
//! simulated time.

// Each test binary compiles this module and uses only part of it.
#![allow(dead_code)]

use std::time::Duration;

use stable_flight::commanders::{RouteEnd, WaypointFollower};
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::mixers::TableMixer;
use stable_flight::stabilizers::copter::CopterStabilizerConfig;
use stable_flight::stabilizers::copter::{CopterStabilizer, TunedLoop};
use stable_flight::testing::RecordingTelemetry;
use stable_flight::tuning::autotune::{Autotune, AutotuneEvent, Envelope, StagePlan};
use stable_flight::tuning::{ProbeConfig, ProbeStatus};
use stable_flight::{Commander, Configurable, FlightLoop, Vec2, WaypointCommand};
use stable_flight_sim::airframes::{self, Airframe};
use stable_flight_sim::{SimVehicle, Wind};

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

/// Altitude of the [`hover`] point over the origin, in m.
pub const HOVER_ALTITUDE_M: f32 = 5.0;

/// Seconds [`hover`] allows to take off and settle on its point.
pub const HOVER_SETTLE_S: f32 = 15.0;

/// A commander that holds one waypoint.
pub type Hover = WaypointFollower<[WaypointCommand; 1]>;

/// The `quad_x` airframe on the ground at the origin in `wind`, flown by `commander`, primed.
pub fn sim_loop<C: Commander>(commander: C, wind: Wind) -> SimLoop<C> {
    sim_loop_with(airframes::quad_x(), commander, wind)
}

/// As [`sim_loop`], for `airframe` (e.g. `quad_x` with other gains).
pub fn sim_loop_with<C: Commander>(airframe: Airframe, commander: C, wind: Wind) -> SimLoop<C> {
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

/// `airframe` taken off and settled for [`HOVER_SETTLE_S`] on a held waypoint
/// [`HOVER_ALTITUDE_M`] over the origin, in `wind`.
pub fn hover(airframe: Airframe, wind: Wind) -> SimLoop<Hover> {
    let follower = WaypointFollower::new([waypoint(0.0, 0.0, HOVER_ALTITUDE_M)])
        .expect("one waypoint")
        .with_route_end(RouteEnd::Hold);
    let mut flight_loop = sim_loop_with(airframe, follower, wind);
    fly(&mut flight_loop, HOVER_SETTLE_S, |_| {});
    flight_loop
}

/// Probes `target` with `config` on `airframe` settled in a calm [`hover`], until the probe
/// ends or `timeout_s` passes; `None` on the timeout.
pub fn probe(
    airframe: Airframe,
    target: TunedLoop,
    config: ProbeConfig,
    timeout_s: f32,
) -> Option<ProbeStatus> {
    let mut flight_loop = hover(airframe, Wind::default());
    flight_loop.start_probe(target, config);
    let ticks = (timeout_s / TICK.as_secs_f32()).round() as usize;
    for _ in 0..ticks {
        flight_loop.step(TICK);
        match flight_loop.probe() {
            Some((_, ProbeStatus::Running(_))) => {}
            Some((_, status)) => return Some(status),
            None => return None,
        }
    }
    None
}

/// How an [`autotune`] run ended.
#[derive(Debug)]
pub struct AutotuneRun {
    /// Every `StageDone` and `StageFailed` event, in order.
    pub stages: Vec<AutotuneEvent>,
    /// The stabilizer config at the end.
    pub config: CopterStabilizerConfig,
    /// Simulated seconds the autotune ran.
    pub duration_s: f32,
}

/// Runs `plan` on `airframe` settled in a calm [`hover`], until it ends or `timeout_s` passes.
pub fn autotune(airframe: Airframe, plan: &[StagePlan], timeout_s: f32) -> AutotuneRun {
    let mut flight_loop = hover(airframe, Wind::default());
    let mut autotune = Autotune::new(plan, Envelope::default()).expect("valid plan");
    autotune.start(&flight_loop).expect("armed, idle");
    let mut stages = Vec::new();
    let mut duration_s = 0.0;
    let ticks = (timeout_s / TICK.as_secs_f32()).round() as usize;
    for _ in 0..ticks {
        flight_loop.step(TICK);
        duration_s += TICK.as_secs_f32();
        let event = autotune.update(&mut flight_loop, TICK.as_secs_f32());
        if let Some(event @ (AutotuneEvent::StageDone { .. } | AutotuneEvent::StageFailed { .. })) =
            event
        {
            stages.push(event);
        }
        if !autotune.is_running() {
            break;
        }
    }
    AutotuneRun {
        stages,
        config: *flight_loop.stabilizer().config(),
        duration_s,
    }
}
