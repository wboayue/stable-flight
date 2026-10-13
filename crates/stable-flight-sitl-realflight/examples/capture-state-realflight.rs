//! Flies the quadcopter from a Logitech Extreme 3D joystick, exactly as
//! `copter-manual-realflight` does (or, with `--script`, a waypoint script as
//! `copter-auto-realflight` does), and records every raw `SimulatorState` RealFlight returns
//! to CSV, unconverted, with the 12 channels sent in the same exchange.
//!
//! RealFlight only reports state while its Link controller drives the model, so a passive
//! capture alongside a transmitter isn't possible; this taps the bridge under the normal
//! pipeline instead. The CSV is for checking the adapter's frame conversions against what
//! RealFlight actually reports. Fly a mix of roll, pitch, yaw, translation, and climb; stop
//! with Ctrl-C. Params and telemetry go over the native link for the bridge (`just bridge`), as
//! in `copter-manual-realflight` (`--link-bind`, `--link-peer`).
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example capture-state-realflight -- --output capture.csv
//! cargo run -p stable-flight-sitl-realflight --example capture-state-realflight -- --script flights/hover.csv
//! ```

use std::fs::File;
use std::io::BufWriter;
use std::path::PathBuf;
use std::time::Duration;

use clap::Parser;
use env_logger::Builder;
use log::{debug, info, warn};

use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::{Commander, FlightLoop};
use stable_flight_link::fc::FcLink;
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::commanders::extreme_3d::Extreme3DCommander;
use stable_flight_sitl::commanders::script::ScriptCommander;
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::capture::RecordingBridge;
use stable_flight_sitl_realflight::realflight_bridge::RealFlightRemoteBridge;
use stable_flight_sitl_realflight::{
    REMOTE_IO_TIMEOUT, RealFlightConfiguration, RealFlightVehicle, airframes,
};

/// Connect timeout for the RealFlight proxy.
const CONNECT_TIMEOUT: Duration = Duration::from_millis(250);

/// Joystick or scripted flight in RealFlight with a raw-state CSV capture.
#[derive(Parser)]
struct Args {
    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,
    /// CSV file to write.
    #[arg(long, default_value = "realflight-capture.csv")]
    output: PathBuf,
    /// Fly this waypoint script instead of the joystick.
    #[arg(long)]
    script: Option<String>,
    /// Local address the link listens on.
    #[arg(long, default_value = DEFAULT_FC_ADDR)]
    link_bind: String,
    /// The bridge's link address.
    #[arg(long, default_value = DEFAULT_BRIDGE_ADDR)]
    link_peer: String,
}

fn main() {
    Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp_millis()
        .init();

    let args = Args::parse();

    let out = BufWriter::new(File::create(&args.output).expect("Failed to create output"));
    let bridge = RecordingBridge::new(
        RealFlightRemoteBridge::with_timeouts(&args.host, CONNECT_TIMEOUT, REMOTE_IO_TIMEOUT)
            .expect("Failed to create bridge"),
        out,
    )
    .expect("Failed to write header");

    // The native link, for the bridge.
    let link = UdpLink::bind(&args.link_bind, &args.link_peer).expect("failed to open link socket");

    info!("Recording to {}; Ctrl-C to stop", args.output.display());
    match &args.script {
        Some(script) => fly(
            ScriptCommander::new(script).expect("failed to load flight script"),
            bridge,
            link,
        ),
        None => fly(
            Extreme3DCommander::new().expect("Failed to initialize Extreme3D joystick"),
            bridge,
            link,
        ),
    }
}

/// Flies the QuadCopterX pipeline from `commander` until Ctrl-C, with params and telemetry over
/// `link`.
fn fly(
    commander: impl Commander,
    bridge: RecordingBridge<RealFlightRemoteBridge, BufWriter<File>>,
    link: UdpLink,
) {
    let airframe = airframes::quadcopter_x();
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration::from_airframe(&airframe, bridge));
    let guidance = CopterGuidance::new(airframe.guidance);
    let stabilizer = CopterStabilizer::new(airframe.stabilizer);
    let mixer = airframe.mixer;
    // The link: the publisher writes it each tick (the flight loop's telemetry), and the poller
    // reads it and serves params between ticks. Queue sizes: 32 param requests, 16 replies.
    let (link_rx, link_tx) = link.into_halves();
    let mut fc_link = FcLink::<32, 16>::new();
    let (mut poller, telemetry) = fc_link.wire(link_rx, link_tx);

    let mut flight_loop =
        FlightLoop::new(commander, vehicle, guidance, stabilizer, mixer, telemetry);

    // Ctrl-C stops the loop; dropping the vehicle re-enables RC and flushes the capture.
    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");
    let runner = RealTimeRunner::new(250.0).expect("valid loop frequency"); // Hz
    runner.run_with(&mut flight_loop, &shutdown, |flight_loop, _| {
        match poller.poll(flight_loop) {
            Ok(stats) if stats.decode_errors > 0 || stats.dropped > 0 => {
                debug!("link: {stats:?}");
            }
            Ok(_) => {}
            Err(e) => warn!("link receive failed: {e}"),
        }
    });
}
