//! Autotunes the Alia 250's hover in X-Plane: climbs to the hover script's waypoint, tunes
//! roll and pitch rate, then roll and pitch angle, by the large-copter step search, lands, and
//! disarms (`StagePlan::large_copter_search`, `architecture.md#autotune`). Each loop's P is
//! searched by pairs of slow steps from its start gains, verified, and kept only if no worse
//! than the start (each judgment logged with its steps). On completion it logs baseline vs
//! verify per loop and prints the tuned config as an `airframes.rs` snippet. A failed stage
//! (envelope breach, too many invalid steps) keeps the searched loops and puts back the rest.
//!
//! It flies the Alia's lift rotors alone on the copter controllers, which autotune needs:
//! [`airframes::alia250`]'s hover gains on `TableMixer::quad_x`, yaw reversed, at its mixer
//! config, which mixes the lift rotors as its quadplane mixer does; the pusher stays stopped and
//! the surfaces centered.
//!
//! `--rate-p-scale` starts from deliberately bad rate gains (e.g. `0.5` or `2`), to check the
//! tune recovers from them. `--param NAME=VALUE` (repeatable) sets a param before takeoff, as
//! the link would (e.g. `STAB_RATE_RLL_I=0.5`), after the scale; an unknown name or a rejected
//! value stops the example before it flies.
//!
//! `--step-test` appends step tests of the loops the search tunes, roll and pitch rate and angle,
//! on its steps (`StagePlan::large_copter_step_test`): alternating steps, each one's response
//! logged at `debug`, and a summary per loop at `info` (overshoot, rise, settling,
//! bounce-back: mean and range). Yaw rate isn't tested: a full yaw moment takes the Alia to
//! about 0.2 rad/s in 1 s, so the copter's 0.5 rad/s steps clip throughout and fail the run
//! (#540). `--stages none --step-test` measures the committed gains.
//! A freeze (a tick over 50 ms after the last) makes the running probe run again and excuses
//! breaches for a second. While the host runs slow (`RunStats::host_slow`), the running step
//! stops and the autotune waits. While the vehicle's state is stale
//! (`XPlaneVehicle::state_is_stale`: no frame, a lagging dataref, or X-Plane paused), the
//! autotune is told (`Autotune::set_state_stale`): the running probe runs again, and past 2 s
//! (`Envelope::max_stale_s`) the run ends `HostFault`, keeping the finished stages; the landing
//! waits for fresh state, and a state stale for 30 s, unpaused, ends the run with exit status 1.
//! Params and
//! telemetry go over the native link for the bridge, so QGC shows the gains change. X-Plane
//! listens on UDP 49000; override it with `--host`.
//!
//! The run ends with exit status 1 and control handed back when the vehicle stays inverted for
//! 3 s, so an unattended run doesn't wait on a crash. Reset X-Plane's flight before each run:
//! the script hovers over wherever the aircraft starts.
//!
//! ```text
//! cargo run --release -p stable-flight-sitl-xplane --example copter-autotune-xplane
//! cargo run --release -p stable-flight-sitl-xplane --example copter-autotune-xplane -- --step-test
//! cargo run --release -p stable-flight-sitl-xplane --example copter-autotune-xplane -- --stages none --step-test
//! cargo run --release -p stable-flight-sitl-xplane --example copter-autotune-xplane -- --stages none --step-test --param STAB_ATT_RLL_I=0.5
//! ```

use std::process::ExitCode;
use std::time::Duration;

use clap::error::ErrorKind;
use clap::{CommandFactory, Parser, ValueEnum};
use env_logger::Builder;
use log::{debug, error, info, warn};

use stable_flight::commanders::RouteEnd;
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::mixers::TableMixer;
use stable_flight::params::Params;
use stable_flight::stabilizers::copter::{CopterStabilizer, TunedLoop};
use stable_flight::tuning::autotune::{
    Autotune, AutotuneEvent, AutotuneStatus, Envelope, StagePlan,
};
use stable_flight::{Configurable, FlightLoop};
use stable_flight_link::fc::FcLink;
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::autotune::{ThenLand, format_brief, is_inverted, log_event};
use stable_flight_sitl::cli::parse_param;
use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::recorder::{CopterColumns, CsvRecorder};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_xplane::xplane_bridge::{DatarefSource, XPlaneUdpBridge};
use stable_flight_sitl_xplane::{XPlaneConfiguration, XPlaneVehicle, airframes};

/// The flight loop's rate.
const LOOP_HZ: f64 = 250.0;
/// The flight loop's period, in s: the supervisor's `dt`.
const LOOP_DT_S: f32 = (1.0 / LOOP_HZ) as f32;
/// The longest wall-clock gap between ticks that isn't a freeze: `copter-autotune-realflight`'s,
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
/// The rate loops (the others are angle and heading loops); `--rate-p-scale` scales all
/// three.
const RATE_LOOPS: [TunedLoop; 3] = [
    TunedLoop::RollRate,
    TunedLoop::PitchRate,
    TunedLoop::YawRate,
];

/// Autotune of the Alia 250's hover rate and angle loops in X-Plane.
#[derive(Parser)]
struct Args {
    /// Waypoint CSV to climb to and hover at (columns: x,y,altitude,cruise_speed).
    #[arg(default_value = "flights/alia_hover.csv")]
    script: String,

    /// X-Plane's UDP address (`host:port`).
    #[arg(long, default_value = "127.0.0.1:49000")]
    host: String,

    /// Scales the rate loops' P before tuning (roll, pitch, yaw), to start from bad gains.
    /// Positive.
    #[arg(long, default_value_t = 1.0, value_parser = positive)]
    rate_p_scale: f32,

    /// Set a param before takeoff, as `NAME=VALUE` (repeatable).
    #[arg(long = "param", value_parser = parse_param)]
    params: Vec<(String, f32)>,

    /// Which loops to tune.
    #[arg(long, value_enum, default_value_t = Stages::LargeSearch)]
    stages: Stages,

    /// After tuning, step-tests roll and pitch rate and angle and logs their responses.
    #[arg(long)]
    step_test: bool,

    /// Writes every tick (body rates, angles, moments, setpoint; `CopterColumns`) to this CSV,
    /// to diagnose a run.
    #[arg(long)]
    record: Option<String>,

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
    /// Roll and pitch rate, then angle, each P searched on slow steps from its start gains.
    LargeSearch,
    /// None: the committed gains (with `--step-test`, to measure them).
    None,
}

/// The loops `--step-test` measures, in order.
const STEP_TESTED: [TunedLoop; 4] = [
    TunedLoop::RollRate,
    TunedLoop::PitchRate,
    TunedLoop::Roll,
    TunedLoop::Pitch,
];

impl Stages {
    fn plan(self) -> Vec<StagePlan> {
        match self {
            Stages::None => Vec::new(),
            Stages::LargeSearch => StagePlan::large_copter_search().into(),
        }
    }
}

/// The run's plan: the tuning stages, then the step tests if asked for.
fn plan(args: &Args) -> Vec<StagePlan> {
    let mut plan = args.stages.plan();
    if args.step_test {
        plan.extend(STEP_TESTED.map(StagePlan::large_copter_step_test));
    }
    plan
}

fn main() -> ExitCode {
    Builder::from_default_env().format_timestamp_millis().init();

    let args = Args::parse();
    let plan = plan(&args);
    if plan.is_empty() {
        Args::command()
            .error(
                ErrorKind::MissingRequiredArgument,
                "--stages none tunes nothing: add --step-test",
            )
            .exit();
    }

    // Climbs to the script's waypoint and holds; `land` ends the flight after the autotune.
    let script = ScriptCommander::new(&args.script)
        .expect("failed to load flight script")
        .with_route_end(RouteEnd::Hold)
        .anchored();
    let (commander, land) = ThenLand::new(script);

    // The Alia's hover side: its hover gains (the baseline), its lift rotors on `quad_x` mixed
    // as its quadplane mixer mixes them, and its engine and surface mapping.
    let alia = airframes::alia250();
    let mut stabilizer_config = alia.stabilizer.hover;
    for target in RATE_LOOPS {
        target.gains_mut(&mut stabilizer_config).p *= args.rate_p_scale;
    }
    let mixer = TableMixer::quad_x()
        .with_reversed_yaw()
        .with_config(alia.mixer.config())
        .expect("the Alia's mixer config is valid");

    let bridge = XPlaneUdpBridge::connect(&args.host).expect("failed to open the socket");
    let vehicle = match XPlaneVehicle::connect(XPlaneConfiguration::new(bridge, alia.mapping)) {
        Ok(vehicle) => vehicle,
        Err(e) => {
            error!("can't start: {e}");
            return ExitCode::FAILURE;
        }
    };

    let guidance = CopterGuidance::new(alia.guidance.hover);
    let stabilizer = CopterStabilizer::new(stabilizer_config);

    // The link: the publisher writes it each tick (the flight loop's telemetry), and the poller
    // reads it and serves params between ticks. Queue sizes: 32 param requests, 16 replies.
    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .expect("failed to open link socket")
        .into_halves();
    let mut fc_link = FcLink::<32, 16>::new();
    let (mut poller, link) = fc_link.wire(link_rx, link_tx);
    let recorder = CsvRecorder::create(args.record.as_deref(), CopterColumns)
        .expect("failed to create the record CSV");
    let telemetry = (link, recorder);

    let mut flight_loop =
        FlightLoop::new(commander, vehicle, guidance, stabilizer, mixer, telemetry);

    for (name, value) in &args.params {
        if let Err(e) = flight_loop.set_param(name, *value) {
            eprintln!("--param {name}={value}: {e:?}");
            return ExitCode::from(2);
        }
        info!("param {name} = {value}");
    }

    let mut autotune = Autotune::new(&plan, Envelope::large_copter()).expect("valid autotune plan");

    // Ctrl-C stops the loop cleanly (XPlaneVehicle hands control back on drop); so do
    // touchdown after the autotune and staying inverted.
    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");

    info!(
        "climbing to the hover point; rate P scaled by {}",
        args.rate_p_scale
    );
    let runner = RealTimeRunner::new(LOOP_HZ).expect("valid loop frequency");
    let mut host_slow = false;
    let mut ticks = 0u32;
    let mut last_tick = Duration::ZERO;
    let mut inverted_since = None;
    let mut stayed_inverted = false;
    let mut stale_since = None;
    let mut unseen_since = None;
    let mut state_lost = false;
    let mut finished = false;
    let mut searched = Vec::new();
    runner.run_with(&mut flight_loop, &shutdown, |flight_loop, stats| {
        // Inverted, in any phase (climb, tune, landing), the vehicle never touches down to end
        // the run.
        if !is_inverted(flight_loop.state()) {
            inverted_since = None;
        } else if inverted_since.is_none() {
            inverted_since = Some(stats.elapsed);
        }
        if let Some(since) = inverted_since
            && stats.elapsed.saturating_sub(since) >= INVERTED_EXIT
        {
            error!("inverted for {} s: stopping", INVERTED_EXIT.as_secs());
            stayed_inverted = true;
            shutdown.trigger();
            return;
        }
        // A freeze spoils a running probe's timing (it runs again), and the vehicle moved
        // meanwhile.
        if stats.elapsed.saturating_sub(last_tick) > FREEZE_GAP {
            autotune.note_stall();
        }
        last_tick = stats.elapsed;
        // The runner logs the stretch itself, with its rate and time.
        if stats.host_slow != host_slow {
            host_slow = stats.host_slow;
            info!("autotune {}", if host_slow { "waits" } else { "goes on" });
        }
        // A stale state is the aircraft flying on unseen; the vehicle logs why.
        let stale = flight_loop.vehicle().state_is_stale();
        // Paused, the aircraft is frozen with its state: the run waits on it for good, since
        // ending it would hand the throttles to X-Plane in the air.
        let unseen = stale && !flight_loop.vehicle().is_paused();
        unseen_since = if unseen {
            unseen_since.or(Some(stats.elapsed))
        } else {
            None
        };
        if let Some(since) = unseen_since
            && stats.elapsed.saturating_sub(since) >= STALE_EXIT
        {
            error!(
                "X-Plane state stale for {} s: stopping",
                STALE_EXIT.as_secs()
            );
            state_lost = true;
            shutdown.trigger();
            return;
        }
        match (stale, stale_since) {
            (true, None) => {
                info!("X-Plane state stale: autotune waits");
                stale_since = Some(stats.elapsed);
            }
            (false, Some(since)) => {
                info!(
                    "X-Plane state back after {:.2} s",
                    stats.elapsed.saturating_sub(since).as_secs_f32()
                );
                stale_since = None;
            }
            _ => {}
        }
        autotune.set_state_stale(stale);
        autotune.set_host_slow(host_slow);
        // The landing waits for a fresh state: on a stale one, the touchdown detector would
        // read the held vertical speed as a touchdown and disarm in the air.
        if finished && !stale && !land.is_landing() {
            info!("landing");
            land.land();
        }
        match poller.poll(flight_loop) {
            Ok(stats) if stats.decode_errors > 0 || stats.dropped > 0 => {
                debug!("link: {stats:?}");
            }
            Ok(_) => {}
            Err(e) => warn!("link receive failed: {e}"),
        }

        if land.is_landing() {
            if !flight_loop.armed() {
                info!("touched down: disarmed");
                shutdown.trigger();
            }
            return;
        }
        if autotune.status() == AutotuneStatus::Idle
            && flight_loop.commander().at_hover_point(flight_loop.state())
        {
            match autotune.start(flight_loop) {
                Ok(()) => info!("autotune: started"),
                Err(e) => warn!("autotune: can't start: {e}"),
            }
        }
        ticks = ticks.wrapping_add(1);
        if ticks.is_multiple_of(STATUS_EVERY_TICKS)
            && let Some((target, status)) = flight_loop.probe()
        {
            debug!("autotune: {target:?} {status:?}");
        }
        let Some(event) = autotune.update(flight_loop, LOOP_DT_S) else {
            return;
        };
        log_event(
            &event,
            flight_loop.stabilizer().config(),
            "copter-autotune-xplane",
        );
        if let AutotuneEvent::StageDone {
            target,
            outcome,
            steps: Some(verify),
            baseline: Some(baseline),
            ..
        } = event
        {
            searched.push((target, outcome, baseline, verify));
        }
        // Every finished search's baseline vs verify, however the run ends.
        if !autotune.is_running() {
            for (target, outcome, baseline, verify) in &searched {
                info!(
                    "autotune: {target:?} {outcome:?}: {} -> {}",
                    format_brief(baseline),
                    format_brief(verify)
                );
            }
        }
        finished = !autotune.is_running();
    });
    info!("X-Plane: {:?}", flight_loop.vehicle().bridge().stats());
    // Dropping the flight loop on return hands control back.
    if stayed_inverted || state_lost {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

/// `--rate-p-scale`: a positive, finite factor (zero, negative, or NaN would invert or drop the
/// rate loops' P).
fn positive(value: &str) -> Result<f32, String> {
    match value.parse::<f32>() {
        Ok(scale) if scale > 0.0 && scale.is_finite() => Ok(scale),
        Ok(_) => Err("must be positive and finite".into()),
        Err(e) => Err(e.to_string()),
    }
}
