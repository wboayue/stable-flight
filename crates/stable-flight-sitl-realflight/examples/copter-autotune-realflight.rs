//! Autotunes the quadcopter's PID loops in RealFlight (#149): climbs to the hover script's
//! waypoint, runs the relay autotune on each loop (roll, pitch, yaw rate, then roll, pitch, and
//! heading), lands, and disarms.
//!
//! Each stage hands one loop to a relay that measures its ultimate gain and period; the
//! Tyreus–Luyben rule turns them into gains, committed before the next stage
//! (`architecture.md#autotune`). The log has each stage's measurement and gains; on completion
//! the tuned config is printed as an `airframes.rs` snippet. A failed stage (envelope breach,
//! relay timeout) keeps the stages before it. `--rate-p-scale` starts from deliberately bad rate
//! gains (e.g. `0.1` or `10`), to check the tune recovers from them. Params and telemetry go over
//! the native link for the bridge, so QGC shows the gains change. The RealFlight host defaults to
//! `10.211.55.3:8080`; override it with `--host`.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example copter-autotune-realflight
//! cargo run -p stable-flight-sitl-realflight --example copter-autotune-realflight -- --rate-p-scale 10
//! ```

use std::cell::Cell;
use std::rc::Rc;
use std::time::Duration;

use clap::{Parser, ValueEnum};
use env_logger::Builder;
use log::{debug, info, warn};

use stable_flight::commanders::{RouteEnd, TouchdownDetector};
use stable_flight::controllers::Gains;
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig, TunedLoop};
use stable_flight::tuning::autotune::{
    Autotune, AutotuneEvent, AutotuneStatus, Envelope, StagePlan,
};
use stable_flight::{
    Command, Commander, Configurable, ControlInput, FlightLoop, LandCommand, VehicleState,
};
use stable_flight_link::fc::FcLink;
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::{BridgeError, RealFlightRemoteBridge};
use stable_flight_sitl_realflight::{
    REMOTE_IO_TIMEOUT, RealFlightConfiguration, RealFlightVehicle, airframes,
};

/// Connect timeout for the RealFlight proxy; a reconnect attempt blocks its tick.
const CONNECT_TIMEOUT: Duration = Duration::from_millis(250);
/// The flight loop's rate.
const LOOP_HZ: f64 = 250.0;
/// The flight loop's period, in s: the supervisor's `dt`.
const LOOP_DT_S: f32 = (1.0 / LOOP_HZ) as f32;
/// How close to the hover altitude the vehicle must be before the autotune starts, in m.
const START_ALTITUDE_TOLERANCE_M: f32 = 0.5;
/// The rate loops (the others are angle and heading loops).
const RATE_LOOPS: [TunedLoop; 3] = [
    TunedLoop::RollRate,
    TunedLoop::PitchRate,
    TunedLoop::YawRate,
];

/// Relay autotune of the quadcopter's PID loops in RealFlight.
#[derive(Parser)]
struct Args {
    /// Waypoint CSV to climb to and hover at (columns: x,y,altitude,cruise_speed).
    #[arg(default_value = "flights/hover.csv")]
    script: String,

    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,

    /// Scales the rate loops' P before tuning (roll, pitch, yaw), to start from bad gains.
    /// Positive.
    #[arg(long, default_value_t = 1.0, value_parser = positive)]
    rate_p_scale: f32,

    /// Which loops to tune.
    #[arg(long, value_enum, default_value_t = Stages::All)]
    stages: Stages,

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
    /// Rate loops, then angle and heading loops.
    All,
    /// Roll, pitch, and yaw rate.
    Rate,
    /// Roll, pitch, and heading.
    Angle,
}

impl Stages {
    fn plan(self) -> Vec<StagePlan> {
        let sequence = StagePlan::copter_sequence();
        let (rate, angle) = sequence.split_at(RATE_LOOPS.len());
        match self {
            Stages::All => sequence.to_vec(),
            Stages::Rate => rate.to_vec(),
            Stages::Angle => angle.to_vec(),
        }
    }
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

fn main() {
    Builder::from_default_env().format_timestamp_millis().init();

    let args = Args::parse();

    // Climbs to the script's waypoint and holds; `land` ends the flight after the autotune.
    let land = Rc::new(Cell::new(false));
    let commander = ThenLand {
        script: ScriptCommander::new(&args.script)
            .expect("failed to load flight script")
            .with_route_end(RouteEnd::Hold),
        land: land.clone(),
        landing: None,
        touchdown: TouchdownDetector::default(),
    };

    // QuadCopterX model: hand-tuned gains (the baseline), mixer, and motor-to-channel mapping.
    let airframe = airframes::quadcopter_x();
    let mut stabilizer_config = airframe.stabilizer;
    for target in RATE_LOOPS {
        target.gains_mut(&mut stabilizer_config).p *= args.rate_p_scale;
    }

    // A lost or stalled RealFlight connection is dropped and rebuilt with backoff; short
    // timeouts bound each attempt's stall.
    let host = args.host.clone();
    let connect =
        move || RealFlightRemoteBridge::with_timeouts(&host, CONNECT_TIMEOUT, REMOTE_IO_TIMEOUT);
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge: connect().expect("Failed to create bridge"),
        mapping: airframe.mapping,
    })
    .with_reconnect(move || connect().map_err(BridgeError::from));

    let guidance = CopterGuidance::new(airframe.guidance);
    let stabilizer = CopterStabilizer::new(stabilizer_config);
    let mixer = airframe.mixer;

    // The link: the publisher writes it each tick (the flight loop's telemetry), and the poller
    // reads it and serves params between ticks. Queue sizes: 32 param requests, 16 replies.
    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .expect("failed to open link socket")
        .into_halves();
    let mut fc_link = FcLink::<32, 16>::new();
    let (mut poller, telemetry) = fc_link.wire(link_rx, link_tx);

    let mut flight_loop =
        FlightLoop::new(commander, vehicle, guidance, stabilizer, mixer, telemetry);

    let mut autotune =
        Autotune::new(&args.stages.plan(), Envelope::default()).expect("valid autotune plan");

    // Ctrl-C stops the loop cleanly (RealFlightVehicle re-enables RC control on drop); so does
    // touchdown after the autotune.
    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");

    info!(
        "climbing to the hover point; rate P scaled by {}",
        args.rate_p_scale
    );
    let runner = RealTimeRunner::new(LOOP_HZ).expect("valid loop frequency");
    runner.run_with(&mut flight_loop, &shutdown, |flight_loop| {
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
                Ok(()) => info!("autotune: started"),
                Err(e) => warn!("autotune: can't start: {e}"),
            }
        }
        let Some(event) = autotune.update(flight_loop, LOOP_DT_S) else {
            return;
        };
        log_event(&event, flight_loop.stabilizer().config());
        if !autotune.is_running() {
            info!("landing");
            land.set(true);
        }
    });
}

/// Logs an autotune event (lifecycle, so `info!`/`warn!`); on completion, prints the tuned
/// gains for pasting.
fn log_event(event: &AutotuneEvent, config: &CopterStabilizerConfig) {
    match event {
        AutotuneEvent::StageStarted(target) => info!("autotune: tuning {target:?}"),
        AutotuneEvent::StageDone {
            target,
            result,
            gains,
        } => info!(
            "autotune: {target:?} Ku {:.3} Tu {:.3} s (relay {:.3}, cycle {:.4}, bias {:.3}) -> {}",
            result.point.ku,
            result.point.tu_s,
            result.relay,
            result.cycle_amplitude,
            result.bias,
            format_gains(gains)
        ),
        AutotuneEvent::StageFailed { target, reason } => {
            warn!("autotune: {target:?} failed: {reason:?}; earlier stages kept");
        }
        AutotuneEvent::Complete => {
            info!("autotune: complete");
            println!("{}", airframe_snippet(config));
        }
        AutotuneEvent::Reverted => warn!("autotune: disarmed, gains reverted"),
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
        && (state.position_m.z - hover.altitude_m).abs() < START_ALTITUDE_TOLERANCE_M
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
