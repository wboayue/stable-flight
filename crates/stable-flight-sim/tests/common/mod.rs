//! Wiring shared by the closed-loop tests: an airframe's pipeline on a `SimCopter` (a
//! [`Plant`]), flown in simulated time, and the step-response [`spec`].

// Each test binary compiles this module and uses only part of it.
#![allow(dead_code)]

pub mod large;
pub mod plane;
pub mod plane_spec;
pub mod quadplane;
pub mod robustness;
pub mod spec;
pub mod tailsitter;
pub mod transition;

use std::time::Duration;

use stable_flight::commanders::{RouteEnd, WaypointFollower};
use stable_flight::controllers::Gains;
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::mixers::TableMixer;
use stable_flight::stabilizers::copter::CopterStabilizerConfig;
use stable_flight::stabilizers::copter::{CopterStabilizer, TunedLoop};
use stable_flight::testing::RecordingTelemetry;
use stable_flight::tuning::autotune::{
    Autotune, AutotuneAbort, AutotuneEvent, Envelope, Outcome, StagePlan,
};
use stable_flight::tuning::{ProbeConfig, ProbeStatus};
use stable_flight::{
    Command, Commander, Configurable, ControlInput, FlightLoop, Guidance, Mixer, Stabilizer,
    TelemetryPublisher, Vec2, Vehicle, VehicleState, WaypointCommand,
};
use stable_flight_sim::airframes::{self, Airframe};
use stable_flight_sim::{Perturbations, SimCopter, Wind};

/// Loop rate of the RealFlight examples.
pub const TICK: Duration = Duration::from_millis(4);

/// Ticks of telemetry kept.
pub const RECORDED: usize = 64;

/// The copter pipeline on a [`SimCopter`], publishing to `T`.
pub type Pipeline<C, T> =
    FlightLoop<C, SimCopter, CopterGuidance, CopterStabilizer, TableMixer<4>, T>;

/// The pipeline recording its last [`RECORDED`] ticks.
pub type SimLoop<C> = Pipeline<C, RecordingTelemetry<RECORDED>>;

/// Most a waypoint descent may sink below its waypoint arriving, in m. The descent target
/// brakes at `descent_accel_mps2` to stop on the waypoint (ArduPilot's shaping, #445):
/// measured 0.11 m on `quad_x` from 30 to 10 m and 0.06 m on the tailsitter after a back
/// transition (0.52 m and 1.16 m when the target snapped onto the waypoint, #403), so 0.2 m
/// leaves about twice the larger.
pub const ARRIVAL_UNDERSHOOT_M: f32 = 0.2;

/// Altitude of the [`hover`] point over the origin, in m.
pub const HOVER_ALTITUDE_M: f32 = 5.0;

/// Seconds [`hover`] allows to take off and settle on its point.
pub const HOVER_SETTLE_S: f32 = 15.0;

/// Calm-air hover band: altitude off the point, in m, over a whole hold (`hover.rs`).
pub const CALM_ALTITUDE_M: f32 = 0.1;

/// Calm-air hover band: horizontal distance from the point (or wander from where the hold
/// began), in m, over a whole hold.
pub const CALM_HORIZONTAL_M: f32 = 0.1;

/// Calm-air hover band: heading off its hold (and a tailsitter's nose off vertical), in
/// degrees, over a whole hold.
pub const CALM_ANGLE_DEG: f32 = 1.0;

/// Most a waypoint leg's heading may be off the bearing to its waypoint once turned, in
/// degrees: PX4's heading acceptance at a waypoint (`MIS_YAW_ERR`,
/// `src/modules/navigator/mission_params.yaml:79-83`).
pub const FACING_DEG: f32 = 12.0;

/// How far `heading_rad` is off `bearing_rad` the shorter way round, in degrees.
pub fn heading_error_deg(heading_rad: f32, bearing_rad: f32) -> f32 {
    stable_flight::wrap_angle(heading_rad - bearing_rad)
        .abs()
        .to_degrees()
}

/// A commander that holds one waypoint.
pub type Hover = WaypointFollower<[WaypointCommand; 1]>;

/// What a test flies: an airframe, the air it flies through, and how its plant departs from
/// the clean model.
#[derive(Debug, Clone, Copy)]
pub struct Plant {
    pub airframe: Airframe,
    pub wind: Wind,
    pub perturbations: Perturbations,
}

impl Plant {
    /// `airframe` in calm air, unperturbed.
    pub fn clean(airframe: Airframe) -> Self {
        Plant {
            airframe,
            wind: Wind::default(),
            perturbations: Perturbations::default(),
        }
    }

    /// `airframe` in `wind`, unperturbed.
    pub fn in_wind(airframe: Airframe, wind: Wind) -> Self {
        Plant {
            wind,
            ..Plant::clean(airframe)
        }
    }

    /// The vehicle, at rest on the ground at the origin.
    pub fn vehicle(&self) -> SimCopter {
        SimCopter::new(self.airframe.model, &self.airframe.mixer)
            .expect("valid airframe")
            .with_wind(self.wind)
            .with_perturbations(self.perturbations)
            .expect("valid perturbations")
    }
}

/// The `quad_x` airframe on the ground at the origin in `wind`, flown by `commander`, primed.
pub fn sim_loop<C: Commander>(commander: C, wind: Wind) -> SimLoop<C> {
    sim_loop_with(&Plant::in_wind(airframes::quad_x(), wind), commander)
}

/// As [`sim_loop`], for `plant`.
pub fn sim_loop_with<C: Commander>(plant: &Plant, commander: C) -> SimLoop<C> {
    pipeline(plant, commander, RecordingTelemetry::new())
}

/// `plant` on the ground at the origin, flown by `commander`, publishing to `telemetry`,
/// primed.
pub fn pipeline<C: Commander, T: TelemetryPublisher>(
    plant: &Plant,
    commander: C,
    telemetry: T,
) -> Pipeline<C, T> {
    let airframe = plant.airframe;
    let mut flight_loop = FlightLoop::new(
        commander,
        plant.vehicle(),
        CopterGuidance::new(airframe.guidance),
        CopterStabilizer::new(airframe.stabilizer),
        airframe.mixer,
        telemetry,
    );
    flight_loop.prime();
    flight_loop
}

/// Ticks in `seconds` of simulated time.
pub fn ticks(seconds: f32) -> usize {
    (seconds / TICK.as_secs_f32()).round() as usize
}

/// Simulated seconds after `ticks` ticks.
pub fn seconds(ticks: usize) -> f32 {
    ticks as f32 * TICK.as_secs_f32()
}

/// Steps `flight_loop` (copter or plane) for `seconds` of simulated time, calling `each` after
/// every tick with the seconds flown so far in this call.
pub fn fly<C, V, G, S, M, T>(
    flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
    seconds: f32,
    mut each: impl FnMut(&FlightLoop<C, V, G, S, M, T>, f32),
) where
    C: Commander,
    V: Vehicle,
    G: Guidance,
    S: Stabilizer,
    M: Mixer,
    T: TelemetryPublisher,
{
    for tick in 1..=ticks(seconds) {
        flight_loop.step(TICK);
        each(flight_loop, self::seconds(tick));
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

/// Holds a waypoint [`HOVER_ALTITUDE_M`] over the origin.
pub fn hover_follower() -> Hover {
    hover_at(HOVER_ALTITUDE_M)
}

/// Holds a waypoint `altitude_m` over the origin.
pub fn hover_at(altitude_m: f32) -> Hover {
    WaypointFollower::new([waypoint(0.0, 0.0, altitude_m)])
        .expect("one waypoint")
        .with_route_end(RouteEnd::Hold)
}

/// Hovers, then flies `input` (if any) from `after_s` on.
pub struct ThenFly {
    pub hover: Hover,
    pub elapsed_s: f32,
    pub after_s: f32,
    pub input: Option<ControlInput>,
}

impl Commander for ThenFly {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        self.elapsed_s += dt;
        match self.input {
            Some(input) if self.elapsed_s >= self.after_s => Command::new(true, input),
            _ => self.hover.next_command(dt, state),
        }
    }
}

/// `plant` taken off and settled for [`HOVER_SETTLE_S`] on [`hover_follower`]'s point.
pub fn hover(plant: &Plant) -> SimLoop<Hover> {
    let mut flight_loop = sim_loop_with(plant, hover_follower());
    fly(&mut flight_loop, HOVER_SETTLE_S, |_, _| {});
    flight_loop
}

/// Probes `target` with `config` on `plant` settled in a [`hover`], until the probe ends or
/// `timeout_s` passes; `None` on the timeout.
pub fn probe(
    plant: &Plant,
    target: TunedLoop,
    config: ProbeConfig,
    timeout_s: f32,
) -> Option<ProbeStatus> {
    let mut flight_loop = hover(plant);
    flight_loop.start_probe(target, config);
    for _ in 0..ticks(timeout_s) {
        flight_loop.step(TICK);
        match flight_loop.probe() {
            Some((_, ProbeStatus::Running(_))) => {}
            Some((_, status)) => return Some(status),
            None => return None,
        }
    }
    None
}

/// `airframe` with each tuned loop's ([`spec::TUNED`]) gains replaced by `retune(target, gains)`.
pub fn retuned(mut airframe: Airframe, retune: impl Fn(TunedLoop, Gains) -> Gains) -> Airframe {
    for target in spec::TUNED {
        let gains = target.gains_mut(&mut airframe.stabilizer);
        *gains = retune(target, *gains);
    }
    airframe
}

/// Simulated seconds an autotune may run (`copter_search` takes about 430–620 s from core's
/// default gains on the clean airframes).
pub const AUTOTUNE_TIMEOUT_S: f32 = 1200.0;

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

impl AutotuneRun {
    /// Each stage's loop and outcome, in order, or why it failed.
    pub fn outcomes(&self) -> Vec<(TunedLoop, Result<Outcome, AutotuneAbort>)> {
        self.stages
            .iter()
            .map(|event| match event {
                AutotuneEvent::StageDone {
                    target, outcome, ..
                } => (*target, Ok(*outcome)),
                AutotuneEvent::StageFailed { target, reason } => (*target, Err(*reason)),
                _ => unreachable!("autotune() keeps stage ends only"),
            })
            .collect()
    }

    /// Whether every stage of `StagePlan::copter_search` ran and ended `Tuned`.
    pub fn all_tuned(&self) -> bool {
        self.outcomes() == spec::TUNED.map(|target| (target, Ok(Outcome::Tuned)))
    }

    /// Whether every stage of `StagePlan::copter_loop_shaping` ran: each loop designed
    /// (`Tuned`), then verified and kept (`Measured`).
    pub fn all_designed_and_kept(&self) -> bool {
        let designed = spec::TUNED.map(|target| (target, Ok(Outcome::Tuned)));
        let kept = spec::TUNED.map(|target| (target, Ok(Outcome::Measured)));
        self.outcomes() == [designed, kept].concat()
    }
}

/// Runs `plan` on `plant` settled in a [`hover`], until it ends or `timeout_s` passes.
pub fn autotune(plant: &Plant, plan: &[StagePlan], timeout_s: f32) -> AutotuneRun {
    autotune_within(plant, plan, Envelope::default(), timeout_s)
}

/// As [`autotune`], within `envelope`.
pub fn autotune_within(
    plant: &Plant,
    plan: &[StagePlan],
    envelope: Envelope,
    timeout_s: f32,
) -> AutotuneRun {
    let mut flight_loop = hover(plant);
    let mut autotune = Autotune::new(plan, envelope).expect("valid plan");
    autotune.start(&flight_loop).expect("armed, idle");
    let mut stages = Vec::new();
    let mut duration_s = 0.0;
    for tick in 1..=ticks(timeout_s) {
        flight_loop.step(TICK);
        duration_s = seconds(tick);
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
