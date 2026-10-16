//! Flies the Alia 250 in X-Plane from the native link (`stable-flight-link`, UDP), as a
//! companion computer would: the link's commands fly it, and a failsafe holds position when
//! the link goes quiet (or finishes a land it was flying), handing control back once it has
//! been back for a second. It flies the quadplane controllers of [`airframes::alia250`] in
//! the copter regime: the bridge asks for no other yet (#333).
//!
//! The commander is `FailsafeCommander<LinkCommander, HoldCommander>`: the link arms and
//! disarms (`Control::Arm`) and says what to fly (`Control::Command`); a hold never arms. Params
//! and telemetry go over the same link. The link binds `--link-bind` and talks to the companion
//! at `--link-peer` (the bridge's localhost ports by default). Drive it with QGC through
//! `just bridge --origin <lat,lon,0>`, taking the origin the vehicle logs at connect, so QGC's
//! map matches X-Plane; or with `stable-flight-tools`' `link_pilot`. X-Plane listens on UDP
//! 49000; override it with `--host`.
//!
//! ```text
//! cargo run -p stable-flight-sitl-xplane --example copter-companion-xplane
//! cargo run -p stable-flight-bridge -- --origin <logged origin>   # in a second terminal
//! ```

use clap::Parser;
use env_logger::Builder;
use log::{debug, info, warn};

use stable_flight::commanders::{FailsafeCommander, FailsafeConfig, HoldCommander};
use stable_flight::guidance::vtol::QuadPlaneGuidance;
use stable_flight::stabilizers::vtol::QuadPlaneStabilizer;
use stable_flight::{Commander, FailsafeState, FlightLoop};
use stable_flight_link::fc::{FcLink, FcLinkConfig};
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_xplane::xplane_bridge::{DatarefSource, XPlaneUdpBridge};
use stable_flight_sitl_xplane::{XPlaneConfiguration, XPlaneVehicle, airframes};

/// Alia 250 flight in X-Plane from the native link, holding when the link goes quiet.
#[derive(Parser)]
struct Args {
    /// X-Plane's UDP address (`host:port`).
    #[arg(long, default_value = "127.0.0.1:49000")]
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

    // Alia 250: tuned gains, mixer, and engine and surface mapping.
    let airframe = airframes::alia250();

    let bridge = XPlaneUdpBridge::connect(&args.host).expect("failed to open the socket");
    let vehicle = XPlaneVehicle::connect(XPlaneConfiguration::new(bridge, airframe.mapping))
        .expect("X-Plane not ready");

    // Waypoint commands from the link become attitude setpoints here.
    let guidance = QuadPlaneGuidance::new(airframe.guidance);
    let stabilizer = QuadPlaneStabilizer::new(airframe.stabilizer);
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

    // Ctrl-C stops the loop cleanly (XPlaneVehicle hands control back on drop).
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
    info!("X-Plane: {:?}", flight_loop.vehicle().bridge().stats());
}
