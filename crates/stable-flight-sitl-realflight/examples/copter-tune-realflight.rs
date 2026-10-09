//! Flies a quadcopter in RealFlight from a Logitech Extreme 3D joystick, as
//! `copter-manual-realflight` does, while serving params and telemetry over the native link
//! (`stable-flight-link`, UDP) so they can be read and tuned in flight.
//!
//! The link binds `--link-bind` (default `127.0.0.1:14560`) and talks to the bridge
//! (`stable-flight-bridge`, or any host speaking the link protocol) at `--link-peer` (default
//! `127.0.0.1:14561`). QGC connects through the bridge, which speaks MAVLink for the flight
//! controller; there is no direct MAVLink telemetry, so QGC shows params but not attitude or
//! position until the bridge translates state (#154). The RealFlight host defaults to
//! `10.211.55.3:8080`; override it with `--host`.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example copter-tune-realflight
//! cargo run -p stable-flight-bridge   # in a second terminal
//! cargo run -p stable-flight-sitl-realflight --example copter-tune-realflight -- --link-peer 192.168.1.30:14561
//! ```

use clap::Parser;
use env_logger::Builder;
use log::{debug, warn};

use stable_flight::FlightLoop;
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight_link::heapless::spsc::Queue;
use stable_flight_link::params::ParamServer;
use stable_flight_link::publisher::{LinkPublisher, LinkPublisherConfig};
use stable_flight_link::router::UplinkRouter;
use stable_flight_link::udp::UdpLink;
use stable_flight_link::{ParamRequest, ParamValue};
use stable_flight_sitl::commanders::extreme_3d::Extreme3DCommander;
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::RealFlightRemoteBridge;
use stable_flight_sitl_realflight::{RealFlightConfiguration, RealFlightVehicle, airframes};

/// Uplink frames decoded per tick; more wait for the next tick.
const MAX_FRAMES_PER_TICK: usize = 8;
/// Param writes applied per tick; a ground station's "load from file" bursts hundreds.
const MAX_PARAM_WRITES_PER_TICK: usize = 4;

/// Joystick-driven quadcopter flight in RealFlight, with params over the native link.
#[derive(Parser)]
struct Args {
    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,
    /// Local address the link listens on.
    #[arg(long, default_value = "127.0.0.1:14560")]
    link_bind: String,
    /// The bridge's link address.
    #[arg(long, default_value = "127.0.0.1:14561")]
    link_peer: String,
}

fn main() {
    Builder::from_default_env().format_timestamp_millis().init();

    let args = Args::parse();

    let commander = Extreme3DCommander::new().expect("Failed to initialize Extreme3D joystick");

    // QuadCopterX model: tuned gains, mixer, and motor-to-channel mapping.
    let airframe = airframes::quadcopter_x();

    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge: RealFlightRemoteBridge::new(&args.host).expect("Failed to create bridge"),
        mapping: airframe.mapping,
    });

    let guidance = CopterGuidance::new(airframe.guidance);
    let stabilizer = CopterStabilizer::new(airframe.stabilizer);
    let mixer = airframe.mixer;

    // The link: the router reads it between ticks, the publisher writes it each tick, and two
    // queues carry param requests to the server and its replies to the publisher.
    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .expect("failed to open link socket")
        .into_halves();
    let mut requests = Queue::<ParamRequest, 32>::new();
    let mut replies = Queue::<ParamValue, 16>::new();
    let (to_server, from_router) = requests.split();
    let (to_publisher, from_server) = replies.split();
    let mut router = UplinkRouter::new(link_rx, MAX_FRAMES_PER_TICK).with_params(to_server);
    let mut param_server = ParamServer::new(from_router, to_publisher, MAX_PARAM_WRITES_PER_TICK);

    let link =
        LinkPublisher::new(link_tx, LinkPublisherConfig::default()).with_param_replies(from_server);

    let mut flight_loop = FlightLoop::new(commander, vehicle, guidance, stabilizer, mixer, link);

    // Ctrl-C stops the loop cleanly (RealFlightVehicle re-enables RC control on drop).
    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");

    let runner = RealTimeRunner::new(250.0).expect("valid loop frequency"); // Hz
    runner.run_with(&mut flight_loop, &shutdown, |flight_loop| {
        match router.poll() {
            Ok(stats) if stats.decode_errors > 0 || stats.dropped > 0 => {
                debug!("link: {stats:?}");
            }
            Ok(_) => {}
            Err(e) => warn!("link receive failed: {e}"),
        }
        param_server.poll(flight_loop);
    });
}
