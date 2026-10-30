//! Autotunes a multirotor in RealFlight: climbs to the hover script's waypoint,
//! tunes roll and pitch rate, then roll and pitch angle, by design search, lands, and disarms
//! (`StagePlan::copter_design_search`, `architecture.md#autotune`). Each loop's axis is fitted
//! from a 3 s square wave on its angle loop (the fit logged: delay, motor lag, acceleration)
//! and its gains designed from the fit; once every loop is designed, each loop's P is searched
//! by pairs of steps from its design, verified, and kept only if no worse than the design
//! (each judgment logged with its steps); a design whose own steps miss the criterion gives
//! way to the loop's start gains. On completion it logs baseline vs verify per loop and
//! prints the tuned config as an `airframes.rs` snippet. A failed stage (envelope breach, too
//! many invalid steps, a fit the design can't use) keeps the searched loops and puts back the
//! rest.
//!
//! `--rate-p-scale` starts from deliberately bad rate gains (e.g. `0.1` or `10`), to check the
//! tune recovers from them.
//!
//! `--step-test` appends step tests of roll and pitch rate and angle: alternating steps,
//! each one's response logged at `debug`, and a summary per loop at `info` (overshoot, rise,
//! settling, bounce-back: mean and range). `--stages none --step-test` measures the hand tune.
//! A freeze (a tick over 50 ms after
//! the last) makes the running probe run again and excuses breaches for a second. While
//! RealFlight runs slow (`RunStats::host_slow`: under 225 Hz over 0.5 s until it regains
//! 240 Hz; it drops to about 120 Hz for stretches, #438), the running step stops and the
//! autotune waits: a step held for its ticks at half rate would tilt twice as far, and settles
//! would time out. A design stage's fit and hold run on: a slow host only lowers the design.
//! While the vehicle's state is stale (`RealFlightVehicle::state_is_stale`: an exchange failed,
//! or the link is down awaiting a reconnect), the autotune is told (`Autotune::set_state_stale`):
//! the running probe runs again, the jump when the link comes back fails nothing, and past
//! 2 s (`Envelope::max_stale_s`) the run ends `HostFault`, keeping the finished stages; the
//! landing waits for the link, and a link down for 30 s ends the run with exit status 1
//! (#309). `--drop-link SECS@AT` cuts the link for SECS seconds, AT seconds into the
//! autotune, to check that. Params and telemetry go over
//! the native link for the bridge, so QGC shows the gains change. The RealFlight host defaults to
//! `10.211.55.3:8080`; override it with `--host`.
//!
//! The run ends with exit status 1 and RC handed back when the vehicle stays inverted for 3 s
//! (in any phase: it never touches down to end the run) or RealFlight doesn't reset the aircraft
//! before arming, so unattended back-to-back runs don't wait on a crash.
//!
//! `--airframe` picks the model: `quadcopter-x` (the default) or `titan-cobra`, whose lift
//! rotors fly on `TableMixer::quad_x` with its hover gains, the pusher stopped and the surfaces
//! centered; the default script hovers over the chosen model's spawn point.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example copter-autotune-realflight
//! cargo run -p stable-flight-sitl-realflight --example copter-autotune-realflight -- --step-test
//! cargo run -p stable-flight-sitl-realflight --example copter-autotune-realflight -- --rate-p-scale 10
//! cargo run -p stable-flight-sitl-realflight --example copter-autotune-realflight -- --stages none --step-test
//! cargo run -p stable-flight-sitl-realflight --example copter-autotune-realflight -- --airframe titan-cobra --step-test
//! ```

use std::cell::Cell;
use std::process::ExitCode;
use std::rc::Rc;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use clap::error::ErrorKind;
use clap::{CommandFactory, Parser, ValueEnum};
use env_logger::Builder;
use log::{debug, error, info, warn};

use stable_flight::commanders::{RouteEnd, TouchdownDetector};
use stable_flight::controllers::Gains;
use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use stable_flight::mixers::TableMixer;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig, TunedLoop};
use stable_flight::tuning::autotune::{
    Autotune, AutotuneEvent, AutotuneStatus, Envelope, StagePlan,
};
use stable_flight::tuning::{Measurement, Stat, StepMetrics, StepSummary};
use stable_flight::{
    Command, Commander, Configurable, ControlInput, FlightLoop, LandCommand, VehicleState,
};
use stable_flight_link::fc::FcLink;
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::recorder::{CopterColumns, CsvRecorder};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::airframes::{self, Airframe};
use stable_flight_sitl_realflight::realflight_bridge::{
    BridgeError, ControlInputs, RealFlightBridge, RealFlightRemoteBridge, SimulatorState,
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
/// How close to the hover altitude the vehicle must be before the autotune starts, in m.
const START_ALTITUDE_TOLERANCE_M: f32 = 0.5;
/// The longest wall-clock gap between ticks that isn't a freeze: past the fit's 40 ms of delay
/// and a rate step's 30 ms rise. The runner's resyncs aren't freezes: at about 223 Hz it falls
/// 100 ms behind every few seconds with no tick late by much (QuadCopterX in RealFlight: 2,216
/// resyncs in a run, 3 gaps over 100 ms, #524).
const FREEZE_GAP: Duration = Duration::from_millis(50);
/// How long the vehicle's state may stay stale before the run ends, exit status 1: the link
/// isn't coming back, and an unattended batch shouldn't wait on it (#309).
const STALE_EXIT: Duration = Duration::from_secs(30);
/// How long the vehicle may stay inverted (tilted past 90°) before the run ends: past a tumble
/// through inverted, well inside the 10 s an unattended batch waits on a crash (#523).
const INVERTED_EXIT: Duration = Duration::from_secs(3);
/// How often a running stage's status is logged at `debug`, in ticks (1 s).
const STATUS_EVERY_TICKS: u32 = 250;
/// The rate loops (the others are angle and heading loops); `--rate-p-scale` scales all
/// three, though the default sequence leaves yaw rate out.
const RATE_LOOPS: [TunedLoop; 3] = [
    TunedLoop::RollRate,
    TunedLoop::PitchRate,
    TunedLoop::YawRate,
];

/// Autotune of a multirotor's roll and pitch loops in RealFlight.
#[derive(Parser)]
struct Args {
    /// Waypoint CSV to climb to and hover at (columns: x,y,altitude,cruise_speed); defaults to
    /// a hover over the airframe's spawn point.
    script: Option<String>,

    /// The RealFlight model.
    #[arg(long, value_enum, default_value_t = Model::QuadcopterX)]
    airframe: Model,

    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,

    /// Scales the rate loops' P before tuning (roll, pitch, yaw), to start from bad gains.
    /// Positive.
    #[arg(long, default_value_t = 1.0, value_parser = positive)]
    rate_p_scale: f32,

    /// Which loops to tune.
    #[arg(long, value_enum, default_value_t = Stages::DesignSearch)]
    stages: Stages,

    /// After tuning, step-tests roll and pitch rate and angle and logs their responses.
    #[arg(long)]
    step_test: bool,

    /// Writes every tick (body rates, angles, moments, setpoint; `CopterColumns`) to this CSV,
    /// to diagnose a run.
    #[arg(long)]
    record: Option<String>,

    /// Cuts the RealFlight link for SECS seconds, AT seconds after the autotune starts
    /// (`SECS@AT`, e.g. `1@30`): every exchange and reconnect attempt fails as on a lost
    /// connection, to check how the run handles it (#309).
    #[arg(long, value_parser = drop_link, value_name = "SECS@AT")]
    drop_link: Option<DropLink>,

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
    /// ArduPilot SITL_Models QuadCopterX.
    QuadcopterX,
    /// Titan Cobra VTOL: its lift rotors only.
    TitanCobra,
}

impl Model {
    /// The model's copter airframe: for the Cobra, its hover side on `quad_x` at its mixer
    /// config, which mixes its lift rotors as its quadplane mixer does, with its mapping.
    fn airframe(self) -> Airframe<CopterGuidanceConfig, CopterStabilizerConfig, TableMixer<4>> {
        match self {
            Model::QuadcopterX => airframes::quadcopter_x(),
            Model::TitanCobra => {
                let cobra = airframes::titan_cobra();
                Airframe {
                    guidance: cobra.guidance.hover,
                    stabilizer: cobra.stabilizer.hover,
                    mixer: TableMixer::quad_x()
                        .with_config(cobra.mixer.config())
                        .expect("the Cobra's mixer config is valid"),
                    mapping: cobra.mapping,
                    body_rotation: cobra.body_rotation,
                }
            }
        }
    }

    /// The default script: a hover over the model's spawn point.
    fn hover_script(self) -> &'static str {
        match self {
            Model::QuadcopterX => "flights/hover.csv",
            Model::TitanCobra => "flights/cobra_hover.csv",
        }
    }
}

/// `--drop-link`: when the link goes down, after the autotune starts, and for how long.
#[derive(Clone, Copy)]
struct DropLink {
    at: Duration,
    lasting: Duration,
}

impl DropLink {
    /// Whether the link is cut `since_start` after the autotune started.
    fn is_cut(&self, since_start: Duration) -> bool {
        (self.at..self.at + self.lasting).contains(&since_start)
    }
}

/// Parses `--drop-link`'s `SECS@AT`, each a non-negative number of seconds.
fn drop_link(value: &str) -> Result<DropLink, String> {
    let (lasting, at) = value.split_once('@').ok_or("expected SECS@AT")?;
    let seconds = |text: &str| {
        text.parse::<f32>()
            .ok()
            .and_then(|secs| Duration::try_from_secs_f32(secs).ok())
            .ok_or(format!("{text:?} isn't a number of seconds"))
    };
    Ok(DropLink {
        at: seconds(at)?,
        lasting: seconds(lasting)?,
    })
}

/// A RealFlight bridge whose link `cut` takes down: every call fails as a lost connection
/// does (`--drop-link`).
struct Cuttable<B> {
    bridge: B,
    cut: Arc<AtomicBool>,
}

impl<B: RealFlightBridge> Cuttable<B> {
    /// `Err` while the link is cut, else `call` on the bridge.
    fn unless_cut<R>(
        &self,
        call: impl FnOnce(&B) -> Result<R, BridgeError>,
    ) -> Result<R, BridgeError> {
        if self.cut.load(Ordering::Relaxed) {
            return Err(link_cut());
        }
        call(&self.bridge)
    }
}

impl<B: RealFlightBridge> RealFlightBridge for Cuttable<B> {
    fn exchange_data(&self, control: &ControlInputs) -> Result<SimulatorState, BridgeError> {
        self.unless_cut(|bridge| bridge.exchange_data(control))
    }

    fn enable_rc(&self) -> Result<(), BridgeError> {
        self.unless_cut(B::enable_rc)
    }

    fn disable_rc(&self) -> Result<(), BridgeError> {
        self.unless_cut(B::disable_rc)
    }

    fn reset_aircraft(&self) -> Result<(), BridgeError> {
        self.unless_cut(B::reset_aircraft)
    }
}

/// The error a cut link fails with: a lost connection, which the vehicle reconnects.
fn link_cut() -> BridgeError {
    BridgeError::Connection(std::io::Error::other("link cut by --drop-link"))
}

/// `--stages`.
#[derive(Clone, Copy, ValueEnum)]
enum Stages {
    /// Roll and pitch rate, then angle, designed from a fit of each axis, then searched from
    /// the design.
    DesignSearch,
    /// None: the hand tune (with `--step-test`, to measure it).
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
            Stages::DesignSearch => StagePlan::copter_design_search().into(),
        }
    }
}

/// The run's plan: the tuning stages, then the step tests if asked for.
fn plan(args: &Args) -> Vec<StagePlan> {
    let mut plan = args.stages.plan();
    if args.step_test {
        plan.extend(STEP_TESTED.map(StagePlan::copter_step_test));
    }
    plan
}

/// Flies the script until told to land (`land`), then lands where it is and disarms on
/// touchdown.
struct ThenLand {
    script: ScriptCommander,
    land: Rc<Cell<bool>>,
    landing: Option<LandCommand>,
    touchdown: TouchdownDetector,
}

impl Commander for ThenLand {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        if !self.land.get() {
            return self.script.next_command(dt, state);
        }
        let land = *self.landing.get_or_insert(LandCommand {
            position_m: state.position_m.xy(),
        });
        let down = self.touchdown.update(dt, state.velocity_mps.z);
        Command::new(!down, ControlInput::Land(land))
    }
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
    let land = Rc::new(Cell::new(false));
    let commander = ThenLand {
        script: ScriptCommander::new(
            args.script
                .as_deref()
                .unwrap_or(args.airframe.hover_script()),
        )
        .expect("failed to load flight script")
        .with_route_end(RouteEnd::Hold)
        .anchored(),
        land: land.clone(),
        landing: None,
        touchdown: TouchdownDetector::default(),
    };

    // The model: hand-tuned gains (the baseline), mixer, and motor-to-channel mapping.
    let airframe = args.airframe.airframe();
    let mut stabilizer_config = airframe.stabilizer;
    for target in RATE_LOOPS {
        target.gains_mut(&mut stabilizer_config).p *= args.rate_p_scale;
    }

    // A lost or stalled RealFlight connection is dropped and rebuilt with backoff; short
    // timeouts bound each attempt's stall. `cut` takes the link down for `--drop-link`.
    let host = args.host.clone();
    let cut = Arc::new(AtomicBool::new(false));
    let connect = {
        let cut = Arc::clone(&cut);
        move || -> Result<_, BridgeError> {
            if cut.load(Ordering::Relaxed) {
                return Err(link_cut());
            }
            let bridge =
                RealFlightRemoteBridge::with_timeouts(&host, CONNECT_TIMEOUT, REMOTE_IO_TIMEOUT)?;
            Ok(Cuttable {
                bridge,
                cut: Arc::clone(&cut),
            })
        }
    };
    // A failed reset would start the run from wherever the last one left the aircraft.
    let vehicle = match RealFlightVehicle::try_new(RealFlightConfiguration::from_airframe(
        &airframe,
        connect().expect("Failed to create bridge"),
    )) {
        Ok(vehicle) => vehicle.with_reconnect(connect),
        Err(e) => {
            error!("can't start: {e}");
            return ExitCode::FAILURE;
        }
    };

    let guidance = CopterGuidance::new(airframe.guidance);
    let stabilizer = CopterStabilizer::new(stabilizer_config);
    let mixer = airframe.mixer;

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

    let mut autotune = Autotune::new(&plan, Envelope::default()).expect("valid autotune plan");

    // Ctrl-C stops the loop cleanly (RealFlightVehicle re-enables RC control on drop); so do
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
    let mut started_at = None;
    let mut stale_since = None;
    let mut link_lost = false;
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
        if let (Some(drop), Some(started_at)) = (args.drop_link, started_at) {
            cut.store(
                drop.is_cut(stats.elapsed.saturating_sub(started_at)),
                Ordering::Relaxed,
            );
        }
        // A stale state is the aircraft flying on unseen (#309).
        let stale = flight_loop.vehicle().state_is_stale();
        match (stale, stale_since) {
            (true, None) => {
                info!("RealFlight state stale: autotune waits");
                stale_since = Some(stats.elapsed);
            }
            (true, Some(since)) if stats.elapsed.saturating_sub(since) >= STALE_EXIT => {
                error!(
                    "RealFlight link down for {} s: stopping",
                    STALE_EXIT.as_secs()
                );
                link_lost = true;
                shutdown.trigger();
                return;
            }
            (false, Some(since)) => {
                info!(
                    "RealFlight state back after {:.2} s",
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
        if finished && !stale && !land.get() {
            info!("landing");
            land.set(true);
        }
        match poller.poll(flight_loop) {
            Ok(stats) if stats.decode_errors > 0 || stats.dropped > 0 => {
                debug!("link: {stats:?}");
            }
            Ok(_) => {}
            Err(e) => warn!("link receive failed: {e}"),
        }

        if land.get() {
            if !flight_loop.armed() {
                info!("touched down: disarmed");
                shutdown.trigger();
            }
            return;
        }
        if autotune.status() == AutotuneStatus::Idle
            && at_hover_point(flight_loop.commander(), flight_loop.state())
        {
            match autotune.start(flight_loop) {
                Ok(()) => {
                    info!("autotune: started");
                    started_at = Some(stats.elapsed);
                }
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
        log_event(&event, flight_loop.stabilizer().config());
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
    // Dropping the flight loop on return hands RC back.
    if stayed_inverted || link_lost {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

/// Whether the vehicle is tilted past 90°: the cosine of the tilt (the attitude matrix's NED-down
/// component of body z) is negative. False for a NaN attitude.
fn is_inverted(state: &VehicleState) -> bool {
    state.attitude.to_rotation_matrix()[(2, 2)] < 0.0
}

/// Logs an autotune event (lifecycle, so `info!`/`warn!`; each step, including ones that run
/// again, at `debug!`); on completion, prints the tuned gains for pasting.
fn log_event(event: &AutotuneEvent, config: &CopterStabilizerConfig) {
    match event {
        AutotuneEvent::StageStarted(target) => info!("autotune: stage {target:?}"),
        AutotuneEvent::Measured {
            target,
            measurement: Measurement::Step(metrics),
        } => debug!("autotune: {target:?} step {}", format_step(metrics)),
        AutotuneEvent::Stalled {
            target,
            measurement: Some(Measurement::Step(metrics)),
        } => debug!(
            "autotune: {target:?} step {} (stalled, runs again)",
            format_step(metrics)
        ),
        AutotuneEvent::Stalled { target, .. } => {
            debug!("autotune: {target:?} probe stopped (host slow), runs again");
        }
        AutotuneEvent::Measured {
            target,
            measurement: Measurement::Identify(model),
        } => info!(
            "autotune: {target:?} fit: delay {} ticks, lag {}, accel {} ({model:?})",
            model.delay_ticks,
            format_ms(model.lag_s()),
            model
                .accel_radps2()
                .map_or("-".to_string(), |accel| format!("{accel:.0} rad/s²")),
        ),
        AutotuneEvent::Tried {
            target,
            gains,
            passed,
            steps,
        } => info!(
            "autotune: {target:?} {} {}: {}",
            format_gains(gains),
            if *passed { "passed" } else { "failed" },
            format_summary(steps),
        ),
        AutotuneEvent::StageDone {
            target,
            gains,
            outcome,
            steps,
            baseline,
        } => {
            if let Some(baseline) = baseline {
                info!("autotune: {target:?} baseline {}", format_summary(baseline));
            }
            if let Some(steps) = steps {
                info!("autotune: {target:?} steps {}", format_summary(steps));
            }
            info!(
                "autotune: {target:?} {outcome:?}: flies {}",
                format_gains(gains)
            );
        }
        AutotuneEvent::Lowered { target, gains } => warn!(
            "autotune: {target:?} start above the search's max, lowered to {}",
            format_gains(gains)
        ),
        AutotuneEvent::Unsettled { target, gains } => warn!(
            "autotune: {target:?} design {} didn't settle; fitting again",
            format_gains(gains)
        ),
        AutotuneEvent::StageFailed { target, reason } => {
            warn!("autotune: {target:?} failed: {reason:?}; the last kept gains fly");
        }
        AutotuneEvent::Complete => {
            info!("autotune: complete");
            println!("{}", airframe_snippet(config));
        }
        AutotuneEvent::Reverted => warn!("autotune: disarmed, gains reverted"),
        event => debug!("autotune: {event:?}"),
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

/// Whether the script has reached its hover waypoint and the vehicle is near its altitude.
fn at_hover_point(commander: &ThenLand, state: &VehicleState) -> bool {
    let follower = commander.script.follower();
    let Some(hover) = follower.waypoints().last() else {
        return false;
    };
    follower.is_complete()
        && (state.position_m.z - commander.script.origin_m().z - hover.altitude_m).abs()
            < START_ALTITUDE_TOLERANCE_M
}

/// One step's response: the step, then its metrics (normalized to the step).
fn format_step(metrics: &StepMetrics) -> String {
    format!(
        "{:+.3}: overshoot {:.2}, rise {}, settle {}, bounce {:.2}, final {:.2}, clipped {:.0} ms",
        metrics.amplitude,
        metrics.overshoot,
        format_ms(metrics.rise_time_s),
        format_ms(metrics.settling_time_s),
        metrics.bounce_back,
        metrics.final_value,
        metrics.saturated_s * 1e3,
    )
}

fn format_ms(time_s: Option<f32>) -> String {
    time_s.map_or("-".to_string(), |s| format!("{:.0} ms", s * 1e3))
}

/// A stage's steps: each metric's mean [min-max]; overshoot per direction; steps that never
/// rose or settled.
fn format_summary(summary: &StepSummary) -> String {
    let stat = |stat: &Stat, scale: f32| match stat.mean() {
        Some(mean) => format!(
            "{:.2} [{:.2}-{:.2}]",
            mean * scale,
            stat.min * scale,
            stat.max * scale
        ),
        None => "-".to_string(),
    };
    format!(
        "n {}: overshoot {} (+ {}, - {}), rise {} ms, settle {} ms, bounce {}, final {}; \
         not risen {}, not settled {}",
        summary.count(),
        stat(&summary.overshoot, 1.0),
        format_mean(&summary.overshoot_positive, 1.0),
        format_mean(&summary.overshoot_negative, 1.0),
        stat(&summary.rise_time_s, 1e3),
        stat(&summary.settling_time_s, 1e3),
        stat(&summary.bounce_back, 1.0),
        stat(&summary.final_value, 1.0),
        summary.not_risen,
        summary.not_settled,
    )
}

/// A summary's means, for the baseline-vs-verify table.
fn format_brief(summary: &StepSummary) -> String {
    format!(
        "rise {} ms, settle {} ms, overshoot {}, final {}",
        format_mean(&summary.rise_time_s, 1e3),
        format_mean(&summary.settling_time_s, 1e3),
        format_mean(&summary.overshoot, 1.0),
        format_mean(&summary.final_value, 1.0),
    )
}

/// A metric's mean times `scale`, or `-` with no samples.
fn format_mean(stat: &Stat, scale: f32) -> String {
    stat.mean()
        .map_or("-".to_string(), |mean| format!("{:.2}", mean * scale))
}

fn format_gains(gains: &Gains) -> String {
    format!(
        "Gains {{ p: {:.4}, i: {:.4}, d: {:.4} }}",
        gains.p, gains.i, gains.d
    )
}

/// The tuned gains, laid out like `airframes::quadcopter_x()` for pasting.
fn airframe_snippet(config: &CopterStabilizerConfig) -> String {
    let gains = |target: TunedLoop| format_gains(&target.gains(config));
    format!(
        "// copter-autotune-realflight\n\
         body_rate: BodyRateControllerConfig {{\n    \
             roll_gains: {},\n    \
             pitch_gains: {},\n    \
             yaw_gains: {},\n    \
             ..\n\
         }},\n\
         attitude: AttitudeControllerConfig {{\n    \
             roll_gains: {},\n    \
             pitch_gains: {},\n    \
             ..\n\
         }},\n\
         yaw: YawControllerConfig {{\n    \
             gains: {},\n    \
             ..\n\
         }},",
        gains(TunedLoop::RollRate),
        gains(TunedLoop::PitchRate),
        gains(TunedLoop::YawRate),
        gains(TunedLoop::Roll),
        gains(TunedLoop::Pitch),
        gains(TunedLoop::Heading),
    )
}
