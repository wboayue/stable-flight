//! Autotunes a VTOL's plane side in RealFlight: flies a script into plane flight (by default a
//! hover climb, a transition onto a climbing leg, and a level leg back past the start at
//! 100 m), hands over to the attitude hold a plane autotune flies in (`PlaneTuneHold`, at the
//! thrust the level leg flew on) once the last leg flies straight and level for 3 s, tunes roll, then pitch, by the plane design
//! (`StagePlan::plane_design`, `architecture.md#autotune`), and flies home in plane, transitions
//! back, hovers over the start, and lands there (`stable_flight_sitl::autotune::PlaneTuneFlight`). Each axis is
//! fitted from a square wave on its rate loop (the fit logged: stiffness, damping, control,
//! lag, delay), its gains designed on the fit, and the design checked on attitude step pairs
//! (each pair logged). Between probes, past 600 m from where the hold started, the hold turns
//! back toward it, and no probe starts meanwhile. On completion it prints the tuned config as
//! an `airframes.rs` snippet. An envelope breach (`PlaneEnvelope::for_plane`: bank, pitch,
//! rate, 40 m of sink, airspeed) ends the run on the last kept gains, and the VTOL hovers
//! where it is, on its rotors, then lands there.
//!
//! `--rate-scale` starts from scaled plane rate gains (P, I, and feedforward, roll and pitch;
//! e.g. `0.3` or `3`), to check the tune recovers from them.
//!
//! `--step-test` appends step tests of roll and pitch rate and roll and pitch angle from the
//! hold: each step's response logged at `debug`, each loop's summary at `info`, and every step
//! test's summary again at the end. `--stages none --step-test` measures the hand tune.
//!
//! As `copter-autotune-realflight` does, a freeze (a tick over 50 ms after the last) makes the
//! running probe run again; while RealFlight runs slow (`RunStats::host_slow`) the running
//! probe stops and the autotune waits; a stale state (`RealFlightVehicle::state_is_stale`)
//! makes the probe run again, ends the run past 2 s (`PlaneEnvelope::max_stale_s`), and, down
//! for 30 s, stops the example with exit status 1. The run also ends with exit status 1 when
//! the vehicle stays inverted for 3 s (tilt past 90°: in the hover view while it hovers, in the
//! body frame otherwise), when RealFlight doesn't reset the aircraft before arming, or when a
//! stage fails. Params and telemetry go over the native link for the bridge, so QGC shows the
//! gains change. The RealFlight host defaults to `10.211.55.3:8080`; override it with
//! `--host`.
//!
//! `--airframe` picks the model: `explora` (the default, and the only one), the eXplora
//! tailsitter on `TailsitterGuidance` and `TailsitterStabilizer`. `--param NAME=VALUE`
//! (repeatable) sets a param before takeoff, as the link would.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example plane-autotune-realflight -- --record tune.csv
//! cargo run -p stable-flight-sitl-realflight --example plane-autotune-realflight -- --step-test
//! cargo run -p stable-flight-sitl-realflight --example plane-autotune-realflight -- --stages none --step-test
//! cargo run -p stable-flight-sitl-realflight --example plane-autotune-realflight -- --rate-scale 3
//! ```

use std::process::ExitCode;
use std::time::Duration;

use clap::error::ErrorKind;
use clap::{CommandFactory, Parser, ValueEnum};
use env_logger::Builder;
use log::{debug, error, info, warn};

use stable_flight::commanders::{PlaneTuneHoldConfig, RouteEnd};
use stable_flight::guidance::vtol::TailsitterGuidance;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::params::Params;
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::stabilizers::plane::{PlaneLoop, PlaneStabilizerConfig};
use stable_flight::stabilizers::vtol::TailsitterStabilizer;
use stable_flight::tuning::autotune::{
    AutotuneEvent, AutotuneStatus, Outcome, PlaneAutotune, PlaneEnvelope, PlaneSide, SideConfig,
    SideTunable, StagePlan,
};
use stable_flight::tuning::{StepSummary, Tunable};
use stable_flight::view::tailsitter_rotation;
use stable_flight::{
    FlightLoop, FlightMode, FlightModeReport, FlightRegime, Guidance, Mixer, Setpoint, Stabilizer,
    TelemetryPublisher, TelemetryRecord,
};
use stable_flight_link::LinkRx;
use stable_flight_link::fc::{FcLink, FcPoller};
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::autotune::{
    PlaneHome, PlaneTuneFlight, PlaneTuneSwitch, format_brief, is_inverted, log_event,
};
use stable_flight_sitl::cli::parse_param;
use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::recorder::{CsvRecorder, TailsitterColumns};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::airframes;
use stable_flight_sitl_realflight::realflight_bridge::{
    BridgeError, RealFlightBridge, RealFlightRemoteBridge,
};
use stable_flight_sitl_realflight::{
    REMOTE_IO_TIMEOUT, RealFlightConfiguration, RealFlightVehicle,
};

/// Connect timeout for the RealFlight proxy; a reconnect attempt blocks its tick.
const CONNECT_TIMEOUT: Duration = Duration::from_millis(250);
/// The flight loop's rate.
const LOOP_HZ: f64 = 250.0;
/// The flight loop's period, in s: the supervisor's `dt`.
const LOOP_DT_S: f32 = (1.0 / LOOP_HZ) as f32;
/// The longest wall-clock gap between ticks that isn't a freeze (`copter-autotune-realflight`'s,
/// #524).
const FREEZE_GAP: Duration = Duration::from_millis(50);
/// How long the vehicle's state may stay stale before the example stops, exit status 1 (#309).
const STALE_EXIT: Duration = Duration::from_secs(30);
/// How long the vehicle may stay inverted before the run ends (#523).
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
/// The plane loiter home flies at, and the back transition starts from, in m above the
/// start: the transition scripts' plane legs' 30 m.
const HOME_ALTITUDE_M: f32 = 30.0;
/// How far outside the plane guidance's loiter radius the flight home may hover back, in m: room
/// for the turn onto the loiter.
const HOME_TURN_MARGIN_M: f32 = 60.0;

/// Autotune of a VTOL's plane side in RealFlight.
#[derive(Parser)]
struct Args {
    /// Waypoint CSV that flies into plane flight (columns: x,y,altitude,cruise_speed,regime);
    /// the hold takes over at its last waypoint. Defaults to the airframe's.
    script: Option<String>,

    /// The RealFlight model.
    #[arg(long, value_enum, default_value_t = Model::Explora)]
    airframe: Model,

    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
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

    /// Writes every tick (`TailsitterColumns`) to this CSV, to diagnose a run.
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

/// `--airframe`.
#[derive(Clone, Copy, ValueEnum)]
enum Model {
    /// eXplora VTOL tailsitter: its plane side.
    Explora,
}

impl Model {
    /// The default script into plane flight.
    fn script(self) -> &'static str {
        match self {
            Model::Explora => "flights/explora_plane_tune.csv",
        }
    }
}

/// `--stages`.
#[derive(Clone, Copy, ValueEnum)]
enum Stages {
    /// Roll, then pitch: each axis fitted, its gains designed on the fit, then verified.
    Design,
    /// None: the hand tune (with `--step-test`, to measure it).
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
/// tests if asked for.
fn plan(args: &Args, config: &PlaneStabilizerConfig) -> Vec<StagePlan<PlaneSide>> {
    let mut plan = match args.stages {
        Stages::None => Vec::new(),
        Stages::Design => [PlaneLoop::Roll, PlaneLoop::Pitch]
            .map(|target| StagePlan::plane_design(target, config))
            .into(),
    };
    if args.step_test {
        plan.extend(STEP_TESTED.map(|target| StagePlan::plane_step_test(target, config)));
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

    // Flies into plane flight; the hold takes over at the last waypoint, and home ends it.
    let script = ScriptCommander::new(args.script.as_deref().unwrap_or(args.airframe.script()))
        .expect("failed to load flight script")
        .with_route_end(RouteEnd::Hold)
        .anchored();
    let Model::Explora = args.airframe;
    let mut airframe = airframes::explora_vtol();
    let home = PlaneHome {
        altitude_m: HOME_ALTITUDE_M,
        radius_m: airframe.guidance.plane.loiter_radius_m + HOME_TURN_MARGIN_M,
    };
    let (commander, switch) = PlaneTuneFlight::new(script, home);

    airframe.stabilizer.plane = scaled(airframe.stabilizer.plane, args.rate_scale);
    let plane = airframe.stabilizer.plane;
    let hold = PlaneTuneHoldConfig {
        recover_to: FlightRegime::Copter,
        ..PlaneTuneHoldConfig::new(
            airframe.guidance.plane.cruise_thrust,
            plane.trim_airspeed_mps,
        )
    };

    // A lost or stalled RealFlight connection is dropped and rebuilt with backoff; short
    // timeouts bound each attempt's stall. A failed reset would start the run from wherever
    // the last one left the aircraft.
    let host = args.host.clone();
    let connect =
        move || RealFlightRemoteBridge::with_timeouts(&host, CONNECT_TIMEOUT, REMOTE_IO_TIMEOUT);
    let vehicle = match RealFlightVehicle::try_new(RealFlightConfiguration::from_airframe(
        &airframe,
        connect().expect("Failed to create bridge"),
    )) {
        Ok(vehicle) => vehicle.with_reconnect(move || connect().map_err(BridgeError::from)),
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
    let columns = TailsitterColumns {
        view: tailsitter_rotation(),
    };
    let recorder = CsvRecorder::create(args.record.as_deref(), columns)
        .expect("failed to create the record CSV");

    let mut flight_loop = FlightLoop::new(
        commander,
        vehicle,
        TailsitterGuidance::new(airframe.guidance),
        TailsitterStabilizer::new(airframe.stabilizer),
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
    fly(&mut flight_loop, flight)
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
/// flight, the handover to the hold, the autotune, and home. Generic over the pipeline, so a
/// second airframe adds only its wiring.
fn fly<B, G, S, M, T, R>(
    flight_loop: &mut FlightLoop<PlaneTuneFlight, RealFlightVehicle<B>, G, S, M, T>,
    mut flight: Flight<'_, R>,
) -> ExitCode
where
    B: RealFlightBridge,
    G: Guidance + Params,
    S: Stabilizer + Params + SideTunable<PlaneSide> + Tunable<TunedLoop>,
    M: Mixer,
    T: TelemetryPublisher,
    R: LinkRx,
    R::Error: std::fmt::Display,
{
    // Ctrl-C stops the loop cleanly (RealFlightVehicle re-enables RC control on drop); so do
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
        // A stale state is the aircraft flying on unseen (#309).
        let stale = flight_loop.vehicle().state_is_stale();
        match (stale, run.stale_since) {
            (true, None) => {
                info!("RealFlight state stale: autotune waits");
                run.stale_since = Some(stats.elapsed);
            }
            (true, Some(since)) if stats.elapsed.saturating_sub(since) >= STALE_EXIT => {
                error!(
                    "RealFlight link down for {} s: stopping",
                    STALE_EXIT.as_secs()
                );
                faulted = true;
                shutdown.trigger();
                return;
            }
            (false, Some(since)) => {
                info!(
                    "RealFlight state back after {:.2} s",
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

/// The frame a tilt past 90° reads inverted in: the hover view while the hover side flies,
/// else the body (plane flight, and the transitions between, where the nose swings through
/// 90° of the view).
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
/// in the loiter at the leg's end keeps its bank's nose-up pitch, climbs, and slows (a dry
/// run started 2° high and settled 2 m/s slow, #535).
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
        "plane-autotune-realflight",
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
