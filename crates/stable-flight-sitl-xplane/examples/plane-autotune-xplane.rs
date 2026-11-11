//! Autotunes the Alia 250's cruise side in X-Plane: flies a script into plane flight (by
//! default `flights/alia_plane_tune.csv`, from Oceano County Airport (L52): a hover climb, a
//! transition, and plane legs offshore at 300 m), hands over to the attitude hold a plane
//! autotune flies in (`PlaneTuneHold`, at the thrust the last leg flew level on) once that leg
//! flies straight and level for 3 s, tunes roll, then pitch, by the plane design
//! (`StagePlan::plane_design`, `architecture.md#autotune`), and flies home in plane,
//! transitions back, hovers over the start, and lands there
//! (`stable_flight_sitl::autotune::PlaneTuneFlight`). Each axis is fitted from a square wave
//! on its rate loop (the fit logged), its gains designed on the fit, and the design checked on
//! attitude step pairs (each pair logged). Between probes, past 1000 m from where the hold
//! started, the hold turns back toward it. On completion it prints the tuned config as an
//! `airframes.rs` snippet. An envelope breach (`PlaneEnvelope::for_plane`) ends the run on the
//! last kept gains, and the Alia hovers where it is, on its lift rotors, then lands there.
//!
//! It flies [`airframes::alia250`] on [`QuadPlaneGuidance`] and [`QuadPlaneStabilizer`], its
//! lift rotors stopped in plane flight, with the Alia's own steps: 5° of pitch, a 15% roll
//! overshoot allowance, and roll steps (designed and tested) that may clip for 0.3 s (as
//! `stable-flight-sim`'s `tests/plane_autotune.rs` tunes it). A run flies about 10 minutes in
//! the hold.
//!
//! [`QuadPlaneGuidance`]: stable_flight::guidance::vtol::QuadPlaneGuidance
//! [`QuadPlaneStabilizer`]: stable_flight::stabilizers::vtol::QuadPlaneStabilizer
//!
//! `--rate-scale` starts from scaled plane rate gains (P, I, and feedforward, roll and pitch;
//! e.g. `0.3` or `3`), to check the tune recovers from them. `--param NAME=VALUE`
//! (repeatable) sets a param before takeoff, as the link would.
//!
//! `--step-test` appends step tests of roll and pitch rate and roll and pitch angle from the
//! hold: each step's response logged at `debug`, each loop's summary at `info`, and every step
//! test's summary again at the end. `--stages none --step-test` measures the committed gains.
//!
//! As `copter-autotune-xplane` does, a freeze (a tick over 50 ms after the last) makes the
//! running probe run again; while the host runs slow (`RunStats::host_slow`) the running probe
//! stops and the autotune waits; a stale state (`XPlaneVehicle::state_is_stale`) makes the
//! probe run again and ends the run past 2 s (`PlaneEnvelope::max_stale_s`), and, stale for
//! 30 s with X-Plane unpaused, stops the example with exit status 1. The run also ends with
//! exit status 1 when the vehicle stays inverted for 3 s or when a stage fails. Params and
//! telemetry go over the native link for the bridge, so QGC shows the gains change. Reset
//! X-Plane's flight before each run; X-Plane listens on UDP 49000; override it with `--host`.
//!
//! ```text
//! cargo run --release -p stable-flight-sitl-xplane --example plane-autotune-xplane -- --record tune.csv
//! cargo run --release -p stable-flight-sitl-xplane --example plane-autotune-xplane -- --step-test
//! cargo run --release -p stable-flight-sitl-xplane --example plane-autotune-xplane -- --stages none --step-test
//! cargo run --release -p stable-flight-sitl-xplane --example plane-autotune-xplane -- --rate-scale 3
//! ```

use std::process::ExitCode;
use std::time::Duration;

use clap::error::ErrorKind;
use clap::{CommandFactory, Parser, ValueEnum};
use env_logger::Builder;
use log::{debug, error, info, warn};

use stable_flight::commanders::{PlaneTuneHoldConfig, RouteEnd};
use stable_flight::guidance::vtol::QuadPlaneGuidance;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::params::Params;
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::stabilizers::plane::{PlaneLoop, PlaneStabilizerConfig};
use stable_flight::stabilizers::vtol::QuadPlaneStabilizer;
use stable_flight::tuning::autotune::{
    AutotuneEvent, AutotuneStatus, Outcome, PlaneAutotune, PlaneEnvelope, PlaneMethod, PlaneSide,
    SideConfig, SideTunable, StagePlan,
};
use stable_flight::tuning::{StepSummary, Tunable};
use stable_flight::{
    Configurable, FlightLoop, FlightMode, FlightModeReport, FlightRegime, Guidance, Mixer,
    Setpoint, Stabilizer, TelemetryPublisher, TelemetryRecord,
};
use stable_flight_link::LinkRx;
use stable_flight_link::fc::{FcLink, FcPoller};
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::autotune::{
    PlaneHome, PlaneTuneFlight, PlaneTuneSwitch, format_brief, is_inverted, log_event,
};
use stable_flight_sitl::cli::parse_param;
use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::recorder::{CsvRecorder, QuadPlaneColumns};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_xplane::xplane_bridge::{DatarefSource, XPlaneUdpBridge};
use stable_flight_sitl_xplane::{XPlaneConfiguration, XPlaneVehicle, airframes};

/// The flight loop's rate.
const LOOP_HZ: f64 = 250.0;
/// The flight loop's period, in s: the supervisor's `dt`.
const LOOP_DT_S: f32 = (1.0 / LOOP_HZ) as f32;
/// The longest wall-clock gap between ticks that isn't a freeze: `copter-autotune-xplane`'s,
/// past several of X-Plane's frames (8–17 ms apart).
const FREEZE_GAP: Duration = Duration::from_millis(50);
/// How long the vehicle's state may stay stale, X-Plane unpaused, before the run ends, exit
/// status 1: X-Plane isn't coming back, and an unattended run shouldn't wait on it.
const STALE_EXIT: Duration = Duration::from_secs(30);
/// How long the vehicle may stay inverted (tilted past 90°) before the run ends: past a tumble
/// through inverted.
const INVERTED_EXIT: Duration = Duration::from_secs(3);
/// How often a running stage's status is logged at `debug`, in ticks (1 s).
const STATUS_EVERY_TICKS: u32 = 250;
/// Seconds of plane flight before the hold may take over: the transition's wake and the
/// line's pitch settle, as the sim's plane autotune tests lead in.
const PLANE_LEAD_S: f32 = 15.0;
/// How long the last leg must fly straight and level before the hold takes over, in s: the
/// level-flight thrust's lag three times over.
const LEVEL_HOLD_S: f32 = 3.0;
/// Straight and level: wings within this bank, in rad.
const LEVEL_BANK_RAD: f32 = 3.0 * std::f32::consts::PI / 180.0;
/// Straight and level: climbing or sinking under this, in m/s.
const LEVEL_CLIMB_MPS: f32 = 0.5;
/// Straight and level: airspeed within this of trim, in m/s (the autotune's settle band).
const LEVEL_AIRSPEED_BAND_MPS: f32 = 1.0;
/// Seconds on the hold before the run starts: its trims settle.
const HOLD_LEAD_S: f32 = 5.0;
/// How slowly the level-flight thrust follows the plane setpoint's, in s (a first-order time
/// constant): the hold starts on the last second's mean, as the sim tests' does.
const LINE_THRUST_TIME_CONSTANT_S: f32 = 1.0;
/// How far from where it started the hold turns back, in m: the Alia's turn circle at its 20°
/// bank limit is 567 m across, and at 300 m the sim's hold circled through every probe.
const HOLD_RETURN_RADIUS_M: f32 = 1000.0;
/// The plane leg home flies at, and the back transition starts from, in m above the start.
const HOME_ALTITUDE_M: f32 = 150.0;
/// How close to the start the leg home transitions back, in m: about how far the Alia's back
/// transition carries it from 45 m/s.
const HOME_RADIUS_M: f32 = 500.0;
/// How far past the start the leg home turns back, in m: room for the 567 m turn and to come
/// down to the home altitude.
const HOME_OVERFLY_M: f32 = 1500.0;
/// The Alia's pitch step, in rad: its plane spec's 5° (`stable-flight-sim`'s
/// `tests/alia_plane.rs`). On the sim, a 10° step from the hold aborts as reversed.
const ALIA_PITCH_STEP_RAD: f32 = 5.0 * std::f32::consts::PI / 180.0;
/// The Alia's largest roll overshoot, as a fraction of the step: its plane spec's, where its
/// 15°/s roll rate limit holds every roll step to a ramp (sim: 14% measured).
const ALIA_MAX_ROLL_OVERSHOOT: f32 = 0.15;
/// How long the Alia's roll steps may clip, in s, in its step tests and its design, as
/// `stable-flight-sim`'s `tests/plane_autotune.rs` has it: holding the 15°/s roll rate limit
/// takes 0.90–0.92 of the aileron on its fits, past the design's 0.8 (#458).
const ALIA_ROLL_MAX_SATURATED_S: f32 = 0.3;

/// Autotune of the Alia 250's cruise side in X-Plane.
#[derive(Parser)]
struct Args {
    /// Waypoint CSV that flies into plane flight (columns: x,y,altitude,cruise_speed,regime);
    /// the hold takes over on its last leg.
    #[arg(default_value = "flights/alia_plane_tune.csv")]
    script: String,

    /// X-Plane's UDP address (`host:port`).
    #[arg(long, default_value = "127.0.0.1:49000")]
    host: String,

    /// Scales the plane rate gains (P, I, and feedforward, roll and pitch) before tuning, to
    /// start from bad gains. Positive.
    #[arg(long, default_value_t = 1.0, value_parser = positive)]
    rate_scale: f32,

    /// Which loops to tune.
    #[arg(long, value_enum, default_value_t = Stages::Design)]
    stages: Stages,

    /// After tuning, step-tests roll and pitch rate and roll and pitch angle and logs their
    /// responses.
    #[arg(long)]
    step_test: bool,

    /// Writes every tick (`QuadPlaneColumns`) to this CSV, to diagnose a run.
    #[arg(long)]
    record: Option<String>,

    /// Set a param before takeoff, as `NAME=VALUE` (repeatable).
    #[arg(long = "param", value_parser = parse_param)]
    params: Vec<(String, f32)>,

    /// Local address the link listens on.
    #[arg(long, default_value = DEFAULT_FC_ADDR)]
    link_bind: String,
    /// The bridge's link address.
    #[arg(long, default_value = DEFAULT_BRIDGE_ADDR)]
    link_peer: String,
}

/// `--stages`.
#[derive(Clone, Copy, ValueEnum)]
enum Stages {
    /// Roll, then pitch: each axis fitted, its gains designed on the fit, then verified.
    Design,
    /// None: the committed gains (with `--step-test`, to measure them).
    None,
}

/// The loops `--step-test` measures, in order: rate loops first.
const STEP_TESTED: [PlaneLoop; 4] = [
    PlaneLoop::RollRate,
    PlaneLoop::PitchRate,
    PlaneLoop::Roll,
    PlaneLoop::Pitch,
];

/// The run's plan for a plane side starting on `config`: the tuning stages, then the step
/// tests if asked for, each with the Alia's own steps.
fn plan(args: &Args, config: &PlaneStabilizerConfig) -> Vec<StagePlan<PlaneSide>> {
    let mut plan = match args.stages {
        Stages::None => Vec::new(),
        Stages::Design => [PlaneLoop::Roll, PlaneLoop::Pitch]
            .map(|target| design_plan(target, config))
            .into(),
    };
    if args.step_test {
        plan.extend(STEP_TESTED.map(|target| step_test_plan(target, config)));
    }
    plan
}

/// The design of `target` from `config`: the plane default, with the Alia's roll overshoot and
/// clip and pitch step.
fn design_plan(target: PlaneLoop, config: &PlaneStabilizerConfig) -> StagePlan<PlaneSide> {
    let mut plan = StagePlan::plane_design(target, config);
    let PlaneMethod::Design(design) = &mut plan.method else {
        unreachable!("a design's plan");
    };
    match target {
        PlaneLoop::Roll => {
            design.design.max_overshoot = ALIA_MAX_ROLL_OVERSHOOT;
            design.design.max_saturated_s = ALIA_ROLL_MAX_SATURATED_S;
        }
        PlaneLoop::Pitch => {
            design.verify.amplitude = ALIA_PITCH_STEP_RAD;
            design.design.amplitude_rad = ALIA_PITCH_STEP_RAD;
        }
        _ => {}
    }
    plan
}

/// The step test of `target` on `config`: the plane default, with the Alia's roll clip and
/// pitch step.
fn step_test_plan(target: PlaneLoop, config: &PlaneStabilizerConfig) -> StagePlan<PlaneSide> {
    let mut plan = StagePlan::plane_step_test(target, config);
    let PlaneMethod::StepTest(step_test) = &mut plan.method else {
        unreachable!("a step test's plan");
    };
    match target {
        PlaneLoop::Roll => step_test.max_saturated_s = ALIA_ROLL_MAX_SATURATED_S,
        PlaneLoop::Pitch => step_test.step.amplitude = ALIA_PITCH_STEP_RAD,
        _ => {}
    }
    plan
}

/// `config`'s plane rate gains (P, I, and feedforward, both axes) times `factor`.
fn scaled(config: PlaneStabilizerConfig, factor: f32) -> PlaneStabilizerConfig {
    let mut scaled = config;
    for gains in [&mut scaled.rate.roll, &mut scaled.rate.pitch] {
        gains.p *= factor;
        gains.i *= factor;
        gains.ff *= factor;
    }
    scaled
}

fn main() -> ExitCode {
    Builder::from_default_env().format_timestamp_millis().init();

    let args = Args::parse();
    if matches!(args.stages, Stages::None) && !args.step_test {
        Args::command()
            .error(
                ErrorKind::MissingRequiredArgument,
                "--stages none tunes nothing: add --step-test",
            )
            .exit();
    }

    // Flies into plane flight; the hold takes over on the last leg, and home ends it.
    let script = ScriptCommander::new(&args.script)
        .expect("failed to load flight script")
        .with_route_end(RouteEnd::Hold)
        .anchored();
    let home = PlaneHome {
        altitude_m: HOME_ALTITUDE_M,
        radius_m: HOME_RADIUS_M,
        overfly_m: HOME_OVERFLY_M,
    };
    let (commander, switch) = PlaneTuneFlight::new(script, home);

    let mut airframe = airframes::alia250();
    airframe.stabilizer.plane = scaled(airframe.stabilizer.plane, args.rate_scale);
    let cruise_thrust = airframe.guidance.plane.cruise_thrust;

    let bridge = XPlaneUdpBridge::connect(&args.host).expect("failed to open the socket");
    let vehicle = match XPlaneVehicle::connect(XPlaneConfiguration::new(bridge, airframe.mapping)) {
        Ok(vehicle) => vehicle,
        Err(e) => {
            error!("can't start: {e}");
            return ExitCode::FAILURE;
        }
    };

    // The link: the publisher writes it each tick (the flight loop's telemetry), and the poller
    // reads it and serves params between ticks. Queue sizes: 32 param requests, 16 replies.
    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .expect("failed to open link socket")
        .into_halves();
    let mut fc_link = FcLink::<32, 16>::new();
    let (poller, link) = fc_link.wire(link_rx, link_tx);
    let recorder = CsvRecorder::create(args.record.as_deref(), QuadPlaneColumns)
        .expect("failed to create the record CSV");

    let mut flight_loop = FlightLoop::new(
        commander,
        vehicle,
        QuadPlaneGuidance::new(airframe.guidance),
        QuadPlaneStabilizer::new(airframe.stabilizer),
        airframe.mixer,
        (link, (recorder, ModeLog::default())),
    );
    for (name, value) in &args.params {
        if let Err(e) = flight_loop.set_param(name, *value) {
            eprintln!("--param {name}={value}: {e:?}");
            return ExitCode::from(2);
        }
        info!("param {name} = {value}");
    }

    // The hold, plan, and envelope read the plane side as the params left it.
    let plane = *SideConfig::<PlaneSide>::side(flight_loop.stabilizer().config());
    let hold = PlaneTuneHoldConfig {
        recover_to: FlightRegime::Copter,
        return_radius_m: HOLD_RETURN_RADIUS_M,
        ..PlaneTuneHoldConfig::new(cruise_thrust, plane.trim_airspeed_mps)
    };
    let plan = plan(&args, &plane);
    let autotune =
        PlaneAutotune::new(&plan, PlaneEnvelope::for_plane(&plane)).expect("valid autotune plan");
    info!(
        "flying into plane flight; plane rate gains scaled by {}",
        args.rate_scale
    );
    let flight = Flight {
        switch,
        autotune,
        hold,
        poller,
    };
    let exit = fly(&mut flight_loop, flight);
    info!("X-Plane: {:?}", flight_loop.vehicle().bridge().stats());
    // Dropping the flight loop on return hands control back.
    exit
}

/// What [`fly`] runs alongside the flight loop.
struct Flight<'a, R> {
    switch: PlaneTuneSwitch,
    autotune: PlaneAutotune,
    /// The hold's config; its thrust becomes the thrust the level leg flew on.
    hold: PlaneTuneHoldConfig,
    poller: FcPoller<'a, R>,
}

/// Where the run stands, as [`fly`] tracks it between ticks.
#[derive(Default)]
struct RunState {
    host_slow: bool,
    ticks: u32,
    last_tick: Duration,
    inverted_since: Option<Duration>,
    stale_since: Option<Duration>,
    /// Since when the state is stale with X-Plane unpaused.
    unseen_since: Option<Duration>,
    /// Seconds flown in plane mode before the hold.
    plane_s: f32,
    /// Seconds flown straight and level on the last leg.
    level_s: f32,
    /// The plane setpoint's thrust, lagged ([`LINE_THRUST_TIME_CONSTANT_S`]).
    line_thrust: Option<f32>,
    /// When the hold took over.
    held_at: Option<Duration>,
    /// Every step test's summary, for the table at the end.
    step_tests: Vec<(PlaneLoop, StepSummary)>,
    failed: bool,
    finished: bool,
}

/// Flies `flight_loop` until it lands after the run, or stops on a fault: the lead into plane
/// flight, the handover to the hold, the autotune, and home.
fn fly<G, S, M, T, R>(
    flight_loop: &mut FlightLoop<PlaneTuneFlight, XPlaneVehicle<XPlaneUdpBridge>, G, S, M, T>,
    mut flight: Flight<'_, R>,
) -> ExitCode
where
    G: Guidance + Params,
    S: Stabilizer + Params + SideTunable<PlaneSide> + Tunable<TunedLoop>,
    M: Mixer,
    T: TelemetryPublisher,
    R: LinkRx,
    R::Error: std::fmt::Display,
{
    // Ctrl-C stops the loop cleanly (XPlaneVehicle hands control back on drop); so do
    // touchdown after the run and staying inverted.
    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");
    let runner = RealTimeRunner::new(LOOP_HZ).expect("valid loop frequency");
    let mut run = RunState::default();
    let mut faulted = false;
    runner.run_with(flight_loop, &shutdown, |flight_loop, stats| {
        let mode = flight_loop
            .guidance()
            .flight_mode()
            .map_or(FlightMode::Copter, |report| report.mode);
        // Inverted, in any phase, the vehicle never touches down to end the run.
        if !is_inverted(
            flight_loop.state(),
            &inverted_frame(flight_loop.stabilizer(), mode),
        ) {
            run.inverted_since = None;
        } else if run.inverted_since.is_none() {
            run.inverted_since = Some(stats.elapsed);
        }
        if let Some(since) = run.inverted_since
            && stats.elapsed.saturating_sub(since) >= INVERTED_EXIT
        {
            error!("inverted for {} s: stopping", INVERTED_EXIT.as_secs());
            faulted = true;
            shutdown.trigger();
            return;
        }
        // A freeze spoils a running probe's timing (it runs again).
        if stats.elapsed.saturating_sub(run.last_tick) > FREEZE_GAP {
            flight.autotune.note_stall();
        }
        run.last_tick = stats.elapsed;
        if stats.host_slow != run.host_slow {
            run.host_slow = stats.host_slow;
            info!(
                "autotune {}",
                if run.host_slow { "waits" } else { "goes on" }
            );
        }
        // A stale state is the aircraft flying on unseen; the vehicle logs why. Paused, the
        // aircraft is frozen with its state: the run waits on it for good, since ending it
        // would hand the throttles to X-Plane in the air.
        let stale = flight_loop.vehicle().state_is_stale();
        let unseen = stale && !flight_loop.vehicle().is_paused();
        run.unseen_since = if unseen {
            run.unseen_since.or(Some(stats.elapsed))
        } else {
            None
        };
        if let Some(since) = run.unseen_since
            && stats.elapsed.saturating_sub(since) >= STALE_EXIT
        {
            error!(
                "X-Plane state stale for {} s: stopping",
                STALE_EXIT.as_secs()
            );
            faulted = true;
            shutdown.trigger();
            return;
        }
        match (stale, run.stale_since) {
            (true, None) => {
                info!("X-Plane state stale: autotune waits");
                run.stale_since = Some(stats.elapsed);
            }
            (false, Some(since)) => {
                info!(
                    "X-Plane state back after {:.2} s",
                    stats.elapsed.saturating_sub(since).as_secs_f32()
                );
                run.stale_since = None;
            }
            _ => {}
        }
        flight.autotune.set_state_stale(stale);
        flight.autotune.set_host_slow(run.host_slow);
        match flight.poller.poll(flight_loop) {
            Ok(stats) if stats.decode_errors > 0 || stats.dropped > 0 => {
                debug!("link: {stats:?}");
            }
            Ok(_) => {}
            Err(e) => warn!("link receive failed: {e}"),
        }

        // Home waits for a fresh state: on a stale one, the touchdown detector would read the
        // held vertical speed as a touchdown and disarm in the air.
        if run.finished && !stale && !flight.switch.is_finished() {
            info!("autotune over: flying home");
            flight.switch.finish();
        }
        if flight.switch.is_finished() {
            if !flight_loop.armed() {
                info!("touched down: disarmed");
                shutdown.trigger();
            }
            return;
        }
        if !flight.switch.is_holding() {
            lead(flight_loop, &mut flight, &mut run, mode, stats.elapsed);
            return;
        }
        tune(flight_loop, &mut flight, &mut run, stats.elapsed);
    });
    if faulted || run.failed {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

/// The frame a tilt past 90° reads inverted in: the hover side's while it flies, else the
/// body's (plane flight, and the transitions between).
fn inverted_frame<S: Tunable<TunedLoop> + Tunable<PlaneLoop>>(
    stabilizer: &S,
    mode: FlightMode,
) -> UnitQuaternion<f32> {
    match mode {
        FlightMode::Copter => Tunable::<TunedLoop>::probe_frame(stabilizer),
        _ => Tunable::<PlaneLoop>::probe_frame(stabilizer),
    }
}

/// The lead: tracks plane time and the level-flight thrust, and hands over to the hold once
/// the plane has flown long enough and flies the last leg straight and level: a hold started
/// in the loiter at the leg's end keeps its bank's nose-up pitch, climbs, and slows.
fn lead<V, G, S, M, T, R>(
    flight_loop: &FlightLoop<PlaneTuneFlight, V, G, S, M, T>,
    flight: &mut Flight<'_, R>,
    run: &mut RunState,
    mode: FlightMode,
    now: Duration,
) where
    G: Guidance,
{
    if mode != FlightMode::Plane {
        run.plane_s = 0.0;
        return;
    }
    run.plane_s += LOOP_DT_S;
    if let Setpoint::Plane(setpoint) = flight_loop.setpoint() {
        let lagged = run.line_thrust.get_or_insert(setpoint.thrust);
        *lagged += (setpoint.thrust - *lagged) * LOOP_DT_S / LINE_THRUST_TIME_CONSTANT_S;
    }
    let state = flight_loop.state();
    let level = flight_loop.commander().on_last_leg()
        && state.roll_rad.abs() < LEVEL_BANK_RAD
        && state.velocity_mps.z.abs() < LEVEL_CLIMB_MPS
        && state.airspeed_mps.is_some_and(|airspeed_mps| {
            (airspeed_mps - flight.hold.trim_airspeed_mps).abs() < LEVEL_AIRSPEED_BAND_MPS
        });
    run.level_s = if level { run.level_s + LOOP_DT_S } else { 0.0 };
    if run.plane_s < PLANE_LEAD_S || run.level_s < LEVEL_HOLD_S {
        return;
    }
    let thrust = run.line_thrust.unwrap_or(flight.hold.thrust);
    info!(
        "hold takes over at thrust {thrust:.3}, airspeed {:.1} m/s",
        flight_loop.state().airspeed_mps.unwrap_or(f32::NAN)
    );
    flight.switch.hold(PlaneTuneHoldConfig {
        thrust,
        ..flight.hold
    });
    run.held_at = Some(now);
}

/// The hold: starts the autotune once the hold has settled, drives the hold's turns between
/// probes, recovers on a breach, and logs every event.
fn tune<V, G, S, M, T, R>(
    flight_loop: &mut FlightLoop<PlaneTuneFlight, V, G, S, M, T>,
    flight: &mut Flight<'_, R>,
    run: &mut RunState,
    now: Duration,
) where
    S: SideTunable<PlaneSide>,
{
    let held_s = run
        .held_at
        .map_or(0.0, |at| now.saturating_sub(at).as_secs_f32());
    if flight.autotune.status() == AutotuneStatus::Idle {
        if held_s < HOLD_LEAD_S {
            return;
        }
        match flight.autotune.start(flight_loop) {
            Ok(()) => info!("autotune: started"),
            Err(e) => warn!("autotune: can't start: {e}"),
        }
    }
    flight.switch.allow_turn(matches!(
        flight.autotune.status(),
        AutotuneStatus::Settling { .. }
    ));
    flight.autotune.set_maneuvering(flight.switch.turning());
    run.ticks = run.ticks.wrapping_add(1);
    if run.ticks.is_multiple_of(STATUS_EVERY_TICKS)
        && let Some((target, status)) = Tunable::<PlaneLoop>::probe(flight_loop.stabilizer())
    {
        debug!("autotune: {target:?} {status:?}");
    }
    let Some(event) = flight.autotune.update(flight_loop, LOOP_DT_S) else {
        return;
    };
    log_event(
        &event,
        SideConfig::<PlaneSide>::side(flight_loop.stabilizer().config()),
        "plane-autotune-xplane",
    );
    match event {
        AutotuneEvent::StageFailed { reason, .. } => {
            run.failed = true;
            if reason.is_breach() {
                info!("recovering: hovering where it is");
                flight.switch.recover();
            }
        }
        AutotuneEvent::StageDone {
            target,
            outcome: Outcome::Measured,
            steps: Some(steps),
            ..
        } => run.step_tests.push((target, steps)),
        _ => {}
    }
    if flight.autotune.is_running() {
        return;
    }
    // Every step test's summary, however the run ends.
    for (target, steps) in &run.step_tests {
        info!("autotune: {target:?} step test: {}", format_brief(steps));
    }
    run.finished = true;
}

/// Logs each change of flight mode, and why a requested regime is refused.
#[derive(Default)]
struct ModeLog {
    last: Option<FlightModeReport>,
}

impl TelemetryPublisher for ModeLog {
    fn publish(&mut self, record: &TelemetryRecord) {
        if self.last != Some(record.flight_mode) {
            let FlightModeReport { mode, refusal } = record.flight_mode;
            info!(
                "flight mode {mode:?}, refused: {refusal:?}, requested {:?}",
                record.command.regime
            );
            self.last = Some(record.flight_mode);
        }
    }
}

/// `--rate-scale`: a positive, finite factor (zero, negative, or NaN would invert or drop the
/// rate loops).
fn positive(value: &str) -> Result<f32, String> {
    match value.parse::<f32>() {
        Ok(scale) if scale > 0.0 && scale.is_finite() => Ok(scale),
        Ok(_) => Err("must be positive and finite".into()),
        Err(e) => Err(e.to_string()),
    }
}
