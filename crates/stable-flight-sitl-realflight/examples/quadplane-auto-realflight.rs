//! Flies the Titan Cobra VTOL quadplane through a waypoint script in RealFlight, using a
//! [`ScriptCommander`], on [`airframes::titan_cobra`]: a hover over its spawn point by default.
//!
//! Reference: https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/QuadPlanes/Titan_Cobra_VTOL
//!
//! It flies on [`QuadPlaneGuidance`] and [`QuadPlaneStabilizer`]: the copter controllers on the
//! lift rotors, the plane controllers on the surfaces and pusher, blended through each
//! transition. A script's optional `regime` column asks for plane or copter per leg; each mode
//! change is logged, and `--record` writes one row per tick (`QuadPlaneColumns`: both sides'
//! outputs, every lift rotor, the pusher, and the surfaces).
//!
//! [`QuadPlaneGuidance`]: stable_flight::guidance::vtol::QuadPlaneGuidance
//! [`QuadPlaneStabilizer`]: stable_flight::stabilizers::vtol::QuadPlaneStabilizer
//!
//! The commander replays the waypoints in a CSV script (default `flights/cobra_hover.csv`,
//! holding 5 m over the spawn point) once, then follows `--end`: `hold` (the default) hovers on
//! the last waypoint until interrupted, `loop` flies the path again and again, and `land` lands
//! on the last waypoint and disarms on touchdown.
//!
//! Params and telemetry go over the native link for the bridge (`just bridge`), as
//! `copter-manual-realflight` does, so QGC shows the path and params can be tuned while it
//! flies.
//!
//! The script path is the first argument; the RealFlight host defaults to `10.211.55.3:8080`
//! via `--host`; the link addresses are `--link-bind` and `--link-peer`.
//!
//! ```text
//! cargo run -p stable-flight-bridge   # once, in a second terminal
//! cargo run -p stable-flight-sitl-realflight --example quadplane-auto-realflight -- --end land
//! cargo run -p stable-flight-sitl-realflight --example quadplane-auto-realflight -- --record hover.csv
//! cargo run -p stable-flight-sitl-realflight --example quadplane-auto-realflight -- --host 192.168.1.20:8080
//! ```

use std::time::Duration;

use clap::{Parser, ValueEnum};
use env_logger::Builder;
use log::{debug, info, warn};

use stable_flight::commanders::RouteEnd;
use stable_flight::guidance::vtol::QuadPlaneGuidance;
use stable_flight::stabilizers::vtol::QuadPlaneStabilizer;
use stable_flight::{FlightLoop, FlightModeReport, TelemetryPublisher, TelemetryRecord};
use stable_flight_link::fc::FcLink;
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::recorder::{CsvRecorder, QuadPlaneColumns};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::{BridgeError, RealFlightRemoteBridge};
use stable_flight_sitl_realflight::{
    REMOTE_IO_TIMEOUT, RealFlightConfiguration, RealFlightVehicle, airframes,
};

/// Connect timeout for the RealFlight proxy; a reconnect attempt blocks its tick.
const CONNECT_TIMEOUT: Duration = Duration::from_millis(250);

/// Scripted quadplane flight in RealFlight: hover, and with a regime column, transitions and
/// plane legs.
#[derive(Parser)]
struct Args {
    /// Waypoint CSV to fly (columns: x,y,altitude,cruise_speed, optional regime).
    #[arg(default_value = "flights/cobra_hover.csv")]
    script: String,

    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
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

    // Titan Cobra VTOL model: gains, mixer, and channel mapping.
    let airframe = airframes::titan_cobra();

    // A lost or stalled RealFlight connection is dropped and rebuilt with backoff; short
    // timeouts bound each attempt's stall.
    let host = args.host.clone();
    let connect =
        move || RealFlightRemoteBridge::with_timeouts(&host, CONNECT_TIMEOUT, REMOTE_IO_TIMEOUT);
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration::from_airframe(
        &airframe,
        connect().expect("Failed to create bridge"),
    ))
    .with_reconnect(move || connect().map_err(BridgeError::from));

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

    // Ctrl-C stops the loop cleanly (RealFlightVehicle re-enables RC control on drop).
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
