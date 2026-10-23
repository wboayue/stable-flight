//! Flies a scripted figure-eight in RealFlight using a [`ScriptCommander`].
//!
//! The commander replays the waypoints in a CSV script (default `flights/figure_eight.csv`,
//! which starts and ends over the QuadCopterX spawn point; regenerate it with
//! `cargo run --bin generate_figure_eight`) once, then follows `--end`:
//! `hold` (the default) hovers on the last waypoint until interrupted, `loop` flies the path
//! again and again, and `land` lands on the last waypoint and disarms on touchdown.
//!
//! Params and telemetry go over the native link for the bridge (`just bridge`), as
//! `copter-manual-realflight` does, so QGC shows the path and params can be tuned while it
//! flies.
//!
//! The script path is the first argument (default `flights/figure_eight.csv`); the RealFlight
//! host defaults to `10.211.55.3:8080` via `--host`; the link addresses are `--link-bind` and
//! `--link-peer`. `--record` writes one row per tick (`CopterColumns`).
//!
//! ```text
//! cargo run -p stable-flight-bridge   # once, in a second terminal
//! cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight
//! cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight -- --end loop
//! cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight -- --end land --record fig8.csv
//! cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight -- flights/wide.csv --host 192.168.1.20:8080
//! ```

use std::time::Duration;

use clap::{Parser, ValueEnum};
use env_logger::Builder;
use log::{debug, info, warn};

use stable_flight::FlightLoop;
use stable_flight::commanders::RouteEnd;
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight_link::fc::FcLink;
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::recorder::{CopterColumns, CsvRecorder};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::{BridgeError, RealFlightRemoteBridge};
use stable_flight_sitl_realflight::{
    REMOTE_IO_TIMEOUT, RealFlightConfiguration, RealFlightVehicle, airframes,
};

/// Connect timeout for the RealFlight proxy; a reconnect attempt blocks its tick.
const CONNECT_TIMEOUT: Duration = Duration::from_millis(250);

/// Scripted figure-eight flight in RealFlight.
#[derive(Parser)]
struct Args {
    /// Waypoint CSV to fly (columns: x,y,altitude,cruise_speed).
    #[arg(default_value = "flights/figure_eight.csv")]
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
        .with_route_end(args.end.into())
        .anchored();

    // QuadCopterX model: tuned gains, mixer, and motor-to-channel mapping.
    // Reference: https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX
    let airframe = airframes::quadcopter_x();

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

    // Position loops that turn each waypoint into an attitude setpoint.
    let guidance = CopterGuidance::new(airframe.guidance);

    let stabilizer = CopterStabilizer::new(airframe.stabilizer);

    let mixer = airframe.mixer;

    // The link: the publisher writes it each tick (the flight loop's telemetry), and the poller
    // reads it and serves params between ticks. Queue sizes: 32 param requests, 16 replies.
    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .expect("failed to open link socket")
        .into_halves();
    let mut fc_link = FcLink::<32, 16>::new();
    let (mut poller, telemetry) = fc_link.wire(link_rx, link_tx);

    let recorder = CsvRecorder::create(args.record.as_deref(), CopterColumns)
        .expect("failed to create the record CSV");

    let mut flight_loop = FlightLoop::new(
        commander,
        vehicle,
        guidance,
        stabilizer,
        mixer,
        (telemetry, recorder),
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
                    let position_m =
                        flight_loop.state().position_m - flight_loop.commander().origin_m();
                    info!(
                        "touched down {:.2} m north, {:.2} m east of the start: disarmed",
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
