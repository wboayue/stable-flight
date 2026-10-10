//! Flies a quadcopter in RealFlight from a Logitech Extreme 3D joystick (or, with
//! `--commander spektrum`, a Spektrum transmitter through the WS3000T dongle), using the
//! hand-tuned gains from [`airframes::quadcopter_x`], with params and telemetry over the native
//! link (`stable-flight-link`, UDP) so they can be watched and tuned in flight.
//!
//! Reference: https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX
//!
//! The link binds `--link-bind` and talks to the bridge (`stable-flight-bridge`, or any host
//! speaking the link protocol) at `--link-peer`; both default to the localhost ports the bridge
//! expects. QGC connects through the bridge, which speaks MAVLink for the flight
//! controller: params, attitude, and position (records at 5 Hz carry the thrust QGC shows as
//! throttle). The bridge can keep running across flights. The RealFlight host defaults to
//! `10.211.55.3:8080`; override it with `--host`.
//!
//! ```text
//! cargo run -p stable-flight-bridge   # once, in a second terminal
//! cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight
//! cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- --host 192.168.1.20:8080
//! cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- --commander spektrum
//! cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- --link-peer 192.168.1.30:14561
//! ```

use std::time::Duration;

use clap::{Parser, ValueEnum};
use env_logger::Builder;
use log::{debug, warn};

use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::{Commander, FlightLoop};
use stable_flight_link::fc::{FcLink, FcLinkConfig};
use stable_flight_link::publisher::LinkPublisherConfig;
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::commanders::{Extreme3DCommander, SpektrumCommander};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::{BridgeError, RealFlightRemoteBridge};
use stable_flight_sitl_realflight::{RealFlightConfiguration, RealFlightVehicle, airframes};

/// How often a full telemetry record goes out: the bridge needs only its thrust, for throttle.
const RECORD_INTERVAL: Duration = Duration::from_millis(200);

/// Connect timeout for rebuilding a lost RealFlight connection; the attempt blocks its tick.
const RECONNECT_TIMEOUT: Duration = Duration::from_millis(250);

/// Stick-driven quadcopter flight in RealFlight, with params and telemetry over the native link.
#[derive(Parser)]
struct Args {
    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,
    /// Local address the link listens on.
    #[arg(long, default_value = DEFAULT_FC_ADDR)]
    link_bind: String,
    /// The bridge's link address.
    #[arg(long, default_value = DEFAULT_BRIDGE_ADDR)]
    link_peer: String,
    /// Stick input device.
    #[arg(long, value_enum, default_value_t = Device::Extreme3d)]
    commander: Device,
}

/// The stick input devices `--commander` selects.
#[derive(Clone, Copy, ValueEnum)]
enum Device {
    /// Logitech Extreme 3D joystick.
    Extreme3d,
    /// Spektrum transmitter through the WS3000T dongle.
    Spektrum,
}

fn main() {
    Builder::from_default_env().format_timestamp_millis().init();

    let args = Args::parse();

    // Stick commander drives the vehicle attitude/rate directly.
    match args.commander {
        Device::Extreme3d => fly(
            Extreme3DCommander::new().expect("Failed to initialize Extreme3D joystick"),
            &args,
        ),
        Device::Spektrum => fly(
            SpektrumCommander::new().expect("Failed to initialize Spektrum dongle"),
            &args,
        ),
    }
}

/// Flies the QuadCopterX pipeline from `commander` until Ctrl-C, with params and telemetry over
/// the native link.
fn fly(commander: impl Commander, args: &Args) {
    // QuadCopterX model: tuned gains, mixer, and motor-to-channel mapping.
    let airframe = airframes::quadcopter_x();

    // A lost RealFlight connection is rebuilt with backoff; a short timeout bounds the stall.
    // Each call times out after the bridge's default 100 ms, so a stalled proxy can't hang the
    // tick: it fails the exchange, and the reconnect takes over.
    let host = args.host.clone();
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge: RealFlightRemoteBridge::new(&args.host).expect("Failed to create bridge"),
        mapping: airframe.mapping,
    })
    .with_reconnect(move || {
        RealFlightRemoteBridge::with_timeout(&host, RECONNECT_TIMEOUT).map_err(BridgeError::from)
    });

    // Heading hold for the attitude-mode sticks; the stabilizer tracks the resulting setpoint.
    let guidance = CopterGuidance::new(airframe.guidance);
    let stabilizer = CopterStabilizer::new(airframe.stabilizer);
    let mixer = airframe.mixer;

    // The link: the publisher writes it each tick (the flight loop's telemetry), and the poller
    // reads it and serves params between ticks. Queue sizes: 32 param requests, 16 replies.
    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .expect("failed to open link socket")
        .into_halves();
    let link_config = FcLinkConfig {
        publisher: LinkPublisherConfig {
            // Records carry the commanded thrust the bridge reports as VFR_HUD throttle.
            record_interval: Some(RECORD_INTERVAL),
            ..LinkPublisherConfig::default()
        },
        // 8 frames and 4 param writes per tick: a "load from file" burst of hundreds spreads
        // over a few hundred milliseconds at 250 Hz.
        ..FcLinkConfig::default()
    };
    let mut fc_link = FcLink::<32, 16>::new();
    let (mut poller, link) = fc_link.wire_with(link_rx, link_tx, link_config);

    let mut flight_loop = FlightLoop::new(commander, vehicle, guidance, stabilizer, mixer, link);

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
