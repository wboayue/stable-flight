//! Flies the Alia 250 through a waypoint script in X-Plane, using a [`ScriptCommander`], on
//! [`airframes::alia250`]: by default `flights/alia_transition_lap.csv`, from Oceano County
//! Airport (L52) so the plane legs fly over the ocean: a hover climb, a forward transition,
//! a box of plane legs offshore, a back transition, and a hover home.
//!
//! It flies on [`QuadPlaneGuidance`] and [`QuadPlaneStabilizer`]: the copter controllers on the
//! lift rotors, the plane controllers on the surfaces and pusher, blended through each
//! transition. A script's optional `regime` column asks for plane or copter per leg; each mode
//! change is logged, and `--record` writes one row per tick (`QuadPlaneColumns`: both sides'
//! outputs, every lift rotor, the pusher, and the surfaces; `plot_flight --record` plots it).
//!
//! [`QuadPlaneGuidance`]: stable_flight::guidance::vtol::QuadPlaneGuidance
//! [`QuadPlaneStabilizer`]: stable_flight::stabilizers::vtol::QuadPlaneStabilizer
//!
//! Script coordinates are meters north and east of where the aircraft was at connect (the
//! anchor the vehicle logs), and altitudes are above sea level, as `XPlaneVehicle` reports
//! position. The commander replays the waypoints once, then follows `--end`: `hold` (the
//! default) hovers on the last waypoint until interrupted, `loop` flies the path again and
//! again, and `land` lands on the last waypoint and disarms on touchdown.
//!
//! Load the Alia 250 at X-Plane's default weight (6,299 lb) on a runway, in calm weather.
//! X-Plane listens on UDP 49000; `--host` takes a hostname for X-Plane on another machine.
//! Params and telemetry go over the native link for the bridge (`just bridge`, with the
//! `--origin` the vehicle logs). `--param NAME=VALUE` (repeatable) sets a param before
//! takeoff, as the link would (e.g. `TRAN_ASPD_MIN=40`); an unknown name or a rejected value
//! stops the example before it flies.
//!
//! ```text
//! cargo run --release -p stable-flight-sitl-xplane --example quadplane-auto-xplane -- --end land --record lap.csv
//! cargo run --release -p stable-flight-sitl-xplane --example quadplane-auto-xplane -- --end land --param TRAN_ASPD_MIN=40
//! ```

use clap::{Parser, ValueEnum};
use env_logger::Builder;
use log::{debug, info, warn};

use stable_flight::commanders::RouteEnd;
use stable_flight::guidance::vtol::QuadPlaneGuidance;
use stable_flight::params::Params;
use stable_flight::stabilizers::vtol::QuadPlaneStabilizer;
use stable_flight::{FlightLoop, FlightModeReport, TelemetryPublisher, TelemetryRecord};
use stable_flight_link::fc::FcLink;
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::recorder::{CsvRecorder, QuadPlaneColumns};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_xplane::xplane_bridge::{DatarefSource, XPlaneUdpBridge};
use stable_flight_sitl_xplane::{XPlaneConfiguration, XPlaneVehicle, airframes};

/// Scripted Alia 250 flight in X-Plane: hover, and with a regime column, transitions and plane
/// legs.
#[derive(Parser)]
struct Args {
    /// Waypoint CSV to fly (columns: x,y,altitude,cruise_speed, optional regime).
    #[arg(default_value = "flights/alia_transition_lap.csv")]
    script: String,

    /// X-Plane's UDP address (`host:port`).
    #[arg(long, default_value = "127.0.0.1:49000")]
    host: String,

    /// What to do after the last waypoint.
    #[arg(long, value_enum, default_value_t = End::Hold)]
    end: End,

    /// Local address the link listens on.
    #[arg(long, default_value = DEFAULT_FC_ADDR)]
    link_bind: String,
    /// The bridge's link address.
    #[arg(long, default_value = DEFAULT_BRIDGE_ADDR)]
    link_peer: String,
    /// Write one CSV row per tick to this file.
    #[arg(long)]
    record: Option<String>,
    /// Set a param before takeoff, as `NAME=VALUE` (repeatable).
    #[arg(long = "param", value_parser = parse_param)]
    params: Vec<(String, f32)>,
}

/// `--param`'s `NAME=VALUE`.
fn parse_param(arg: &str) -> Result<(String, f32), String> {
    let (name, value) = arg.split_once('=').ok_or("expected NAME=VALUE")?;
    let value = value.trim().parse::<f32>().map_err(|e| e.to_string())?;
    Ok((name.trim().to_owned(), value))
}

/// `--end`: the core [`RouteEnd`], as a CLI value.
#[derive(Clone, Copy, ValueEnum)]
enum End {
    /// Hover on the last waypoint.
    Hold,
    /// Fly the path again.
    Loop,
    /// Land on the last waypoint and disarm.
    Land,
}

impl From<End> for RouteEnd {
    fn from(end: End) -> Self {
        match end {
            End::Hold => RouteEnd::Hold,
            End::Loop => RouteEnd::Loop,
            End::Land => RouteEnd::Land,
        }
    }
}

fn main() {
    // Initialize logger
    Builder::from_default_env().format_timestamp_millis().init();

    let args = Args::parse();

    // Replays the waypoints from the CSV script, then holds the last, loops, or lands.
    let commander = ScriptCommander::new(&args.script)
        .expect("failed to load flight script")
        .with_route_end(args.end.into());

    // Alia 250: tuned gains, mixer, and engine and surface mapping.
    let airframe = airframes::alia250();

    let bridge = XPlaneUdpBridge::connect(&args.host).expect("failed to open the socket");
    let vehicle = XPlaneVehicle::connect(XPlaneConfiguration::new(bridge, airframe.mapping))
        .expect("X-Plane not ready");

    // Hover and plane controllers under one transition; in the copter regime the position
    // loops turn each waypoint into an attitude setpoint for the lift rotors.
    let guidance = QuadPlaneGuidance::new(airframe.guidance);
    let stabilizer = QuadPlaneStabilizer::new(airframe.stabilizer);

    let mixer = airframe.mixer;

    // The link: the publisher writes it each tick (the flight loop's telemetry), and the poller
    // reads it and serves params between ticks. Queue sizes: 32 param requests, 16 replies.
    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .expect("failed to open link socket")
        .into_halves();
    let mut fc_link = FcLink::<32, 16>::new();
    let (mut poller, telemetry) = fc_link.wire(link_rx, link_tx);

    let recorder = CsvRecorder::create(args.record.as_deref(), QuadPlaneColumns)
        .expect("failed to create the record CSV");

    let mut flight_loop = FlightLoop::new(
        commander,
        vehicle,
        guidance,
        stabilizer,
        mixer,
        (telemetry, (recorder, ModeLog::default())),
    );

    for (name, value) in &args.params {
        if let Err(e) = flight_loop.set_param(name, *value) {
            eprintln!("--param {name}={value}: {e:?}");
            std::process::exit(2);
        }
        info!("param {name} = {value}");
    }

    // Ctrl-C stops the loop cleanly (XPlaneVehicle hands control back on drop).
    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");

    let runner = RealTimeRunner::new(250.0).expect("valid loop frequency"); // Hz
    let mut complete = false;
    runner.run_with(&mut flight_loop, &shutdown, |flight_loop, _| {
        if !complete && flight_loop.commander().follower().is_complete() {
            complete = true;
            match args.end {
                End::Land => {
                    let position_m = flight_loop.state().position_m;
                    info!(
                        "touched down at north {:.2} m, east {:.2} m: disarmed",
                        position_m.x, position_m.y
                    );
                }
                End::Hold => info!("reached the last waypoint: holding"),
                End::Loop => {}
            }
        }
        match poller.poll(flight_loop) {
            Ok(stats) if stats.decode_errors > 0 || stats.dropped > 0 => {
                debug!("link: {stats:?}");
            }
            Ok(_) => {}
            Err(e) => warn!("link receive failed: {e}"),
        }
    });
    info!("X-Plane: {:?}", flight_loop.vehicle().bridge().stats());
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
