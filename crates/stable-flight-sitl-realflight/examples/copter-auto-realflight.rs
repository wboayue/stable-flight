//! Flies a scripted figure-eight in RealFlight using a [`ScriptCommander`].
//!
//! The commander replays the waypoints in a CSV script (default `flights/figure_eight.csv`,
//! regenerate it with `cargo run --bin generate_figure_eight`) once and hovers on the last
//! waypoint until interrupted; `--loop` flies the path again and again instead.
//!
//! Params and telemetry go over the native link for the bridge (`just bridge`), as
//! `copter-manual-realflight` does, so QGC shows the path and params can be tuned while it
//! flies.
//!
//! The script path is the first argument (default `flights/figure_eight.csv`); the RealFlight
//! host defaults to `10.211.55.3:8080` via `--host`; the link addresses are `--link-bind` and
//! `--link-peer`.
//!
//! ```text
//! cargo run -p stable-flight-bridge   # once, in a second terminal
//! cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight
//! cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight -- --loop
//! cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight -- flights/wide.csv --host 192.168.1.20:8080
//! ```

use std::time::Duration;

use clap::Parser;
use env_logger::Builder;
use log::{debug, warn};

use stable_flight::FlightLoop;
use stable_flight::commanders::RouteEnd;
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight_link::fc::{FcLink, FcLinkConfig};
use stable_flight_link::publisher::LinkPublisherConfig;
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::RealFlightRemoteBridge;
use stable_flight_sitl_realflight::{RealFlightConfiguration, RealFlightVehicle, airframes};

/// How often a full telemetry record goes out: the bridge needs only its thrust, for throttle.
const RECORD_INTERVAL: Duration = Duration::from_millis(200);

/// Scripted figure-eight flight in RealFlight.
#[derive(Parser)]
struct Args {
    /// Waypoint CSV to fly (columns: x,y,altitude,cruise_speed).
    #[arg(default_value = "flights/figure_eight.csv")]
    script: String,

    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,

    /// Loop the path instead of holding on the last waypoint.
    #[arg(long = "loop")]
    looping: bool,

    /// Local address the link listens on.
    #[arg(long, default_value = DEFAULT_FC_ADDR)]
    link_bind: String,
    /// The bridge's link address.
    #[arg(long, default_value = DEFAULT_BRIDGE_ADDR)]
    link_peer: String,
}

fn main() {
    // Initialize logger
    Builder::from_default_env().format_timestamp_millis().init();

    let args = Args::parse();

    // Replays the waypoints from the CSV script, then holds the last or loops.
    let route_end = if args.looping {
        RouteEnd::Loop
    } else {
        RouteEnd::Hold
    };
    let commander = ScriptCommander::new(&args.script)
        .expect("failed to load flight script")
        .with_route_end(route_end);

    // QuadCopterX model: tuned gains, mixer, and motor-to-channel mapping.
    // Reference: https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX
    let airframe = airframes::quadcopter_x();

    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge: RealFlightRemoteBridge::new(&args.host).expect("Failed to create bridge"),
        mapping: airframe.mapping,
    });

    // Position loops that turn each waypoint into an attitude setpoint.
    let guidance = CopterGuidance::new(airframe.guidance);

    let stabilizer = CopterStabilizer::new(airframe.stabilizer);

    let mixer = airframe.mixer;

    // The link: the publisher writes it each tick (the flight loop's telemetry), and the poller
    // reads it and serves params between ticks. Up to 32 param requests and 16 replies queue.
    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .expect("failed to open link socket")
        .into_halves();
    let link_config = FcLinkConfig {
        publisher: LinkPublisherConfig {
            // Records carry the commanded thrust the bridge reports as VFR_HUD throttle.
            record_interval: Some(RECORD_INTERVAL),
            ..LinkPublisherConfig::default()
        },
        ..FcLinkConfig::default()
    };
    let mut fc_link = FcLink::<32, 16>::new();
    let (mut poller, telemetry) = fc_link.wire_with(link_rx, link_tx, link_config);

    let mut flight_loop =
        FlightLoop::new(commander, vehicle, guidance, stabilizer, mixer, telemetry);

    // Ctrl-C stops the loop cleanly (RealFlightVehicle re-enables RC control on drop).
    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");

    let runner = RealTimeRunner::new(250.0).expect("valid loop frequency"); // Hz
    runner.run_with(&mut flight_loop, &shutdown, |flight_loop| {
        match poller.poll(flight_loop) {
            Ok(stats) if stats.decode_errors > 0 || stats.dropped > 0 => {
                debug!("link: {stats:?}");
            }
            Ok(_) => {}
            Err(e) => warn!("link receive failed: {e}"),
        }
    });
}
