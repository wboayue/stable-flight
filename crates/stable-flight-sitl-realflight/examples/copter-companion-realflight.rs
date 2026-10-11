//! Flies a quadcopter in RealFlight from the native link (`stable-flight-link`, UDP), as a
//! companion computer would: the link's commands fly it, and a failsafe holds position when the
//! link goes quiet (or finishes a land it was flying), handing control back once it has been
//! back for a second.
//!
//! The commander is `FailsafeCommander<LinkCommander, HoldCommander>`: the link arms and
//! disarms (`Control::Arm`) and says what to fly (`Control::Command`); a hold never arms. Params
//! and telemetry go over the same link. The link binds `--link-bind` and talks to the companion
//! at `--link-peer` (the bridge's localhost ports by default). Drive it with
//! `stable-flight-tools`' `link_pilot`, which plays the companion: stop the pilot mid-flight
//! and the quad holds. The RealFlight host defaults to `10.211.55.3:8080`; override it with
//! `--host`.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example copter-companion-realflight
//! cargo run --bin link_pilot   # in a second terminal
//! ```

use std::time::Duration;

use clap::Parser;
use env_logger::Builder;
use log::{debug, info, warn};

use stable_flight::commanders::{FailsafeCommander, FailsafeConfig, HoldCommander};
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::{Commander, FailsafeState, FlightLoop};
use stable_flight_link::fc::{FcLink, FcLinkConfig};
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::{BridgeError, RealFlightRemoteBridge};
use stable_flight_sitl_realflight::{
    REMOTE_IO_TIMEOUT, RealFlightConfiguration, RealFlightVehicle, airframes,
};

/// Connect timeout for the RealFlight proxy; a reconnect attempt blocks its tick.
const CONNECT_TIMEOUT: Duration = Duration::from_millis(250);

/// Quadcopter flight in RealFlight from the native link, holding when the link goes quiet.
#[derive(Parser)]
struct Args {
    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,
    /// Local address the link listens on.
    #[arg(long, default_value = DEFAULT_FC_ADDR)]
    link_bind: String,
    /// The companion's link address.
    #[arg(long, default_value = DEFAULT_BRIDGE_ADDR)]
    link_peer: String,
}

fn main() {
    Builder::from_default_env().format_timestamp_millis().init();

    let args = Args::parse();

    // QuadCopterX model: tuned gains, mixer, and motor-to-channel mapping.
    let airframe = airframes::quadcopter_x();

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

    // Waypoint commands from the link become attitude setpoints here.
    let guidance = CopterGuidance::new(airframe.guidance);
    let stabilizer = CopterStabilizer::new(airframe.stabilizer);
    let mixer = airframe.mixer;

    // The link: the publisher writes it each tick (the flight loop's telemetry), the poller
    // reads it and serves params between ticks, and the link commander flies from its control
    // messages. Queue sizes: 32 param requests, 16 replies, 8 control messages (the default).
    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .expect("failed to open link socket")
        .into_halves();
    let mut fc_link = FcLink::<32, 16>::new();
    let (mut poller, link, link_commander) =
        fc_link.wire_linked(link_rx, link_tx, FcLinkConfig::default());

    // Hold position after 1 s without a message; hand back after 1 s of messages.
    let commander = FailsafeCommander::new(
        link_commander,
        HoldCommander::default(),
        FailsafeConfig::default(),
    )
    .expect("valid failsafe config");

    let mut flight_loop = FlightLoop::new(commander, vehicle, guidance, stabilizer, mixer, link);

    // Ctrl-C stops the loop cleanly (RealFlightVehicle re-enables RC control on drop).
    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");

    let mut failsafe = None;
    let runner = RealTimeRunner::new(250.0).expect("valid loop frequency"); // Hz
    runner.run_with(&mut flight_loop, &shutdown, |flight_loop, _| {
        match poller.poll(flight_loop) {
            Ok(stats) if stats.decode_errors > 0 || stats.dropped > 0 => {
                debug!("link: {stats:?}");
            }
            Ok(_) => {}
            Err(e) => warn!("link receive failed: {e}"),
        }
        // Log each handover (lifecycle events), not every tick.
        let state = flight_loop.commander().failsafe();
        if state != failsafe {
            match state {
                Some(FailsafeState::Fallback)
                    if flight_loop.commander().fallback().is_landing() =>
                {
                    info!("link quiet: landing");
                }
                Some(FailsafeState::Fallback) => info!("link quiet: holding position"),
                Some(FailsafeState::AwaitingSticks) => {
                    info!("link back: holding until the sticks move");
                }
                Some(FailsafeState::Primary) => info!("link back: flying from the link"),
                _ => {}
            }
            failsafe = state;
        }
    });
}
