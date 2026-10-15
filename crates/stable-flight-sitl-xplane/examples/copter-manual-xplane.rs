//! Flies the Alia 250 in X-Plane in copter mode from a Logitech Extreme 3D joystick (or, with
//! `--commander spektrum`, a Spektrum transmitter through the WS3000T dongle), using
//! [`airframes::alia250`], with params and telemetry over the native link
//! (`stable-flight-link`, UDP) so they can be watched and tuned in flight.
//!
//! Load the Alia 250 at X-Plane's default weight (6,299 lb) on a runway. X-Plane listens on UDP
//! 49000; `--host` takes a hostname (e.g. `oya.local:49000`) for X-Plane on another machine.
//! The vehicle takes the throttles and surfaces once every state dataref has arrived, and logs
//! the bridge's `--origin` for where it started. Ctrl-C hands the aircraft back to X-Plane.
//!
//! The link binds `--link-bind` and talks to the bridge (`stable-flight-bridge`) at
//! `--link-peer`; both default to the localhost ports the bridge expects. `--record <csv>`
//! writes one row per tick: body rates, attitude, moments, the attitude setpoint, and the stick
//! yaw (`stable_flight_sitl::recorder::CopterColumns`).
//!
//! ```text
//! cargo run -p stable-flight-sitl-xplane --example copter-manual-xplane
//! cargo run -p stable-flight-sitl-xplane --example copter-manual-xplane -- --record flight.csv
//! ```

use clap::{Parser, ValueEnum};
use env_logger::Builder;
use log::{debug, info, warn};

use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::{Commander, FlightLoop};
use stable_flight_link::fc::FcLink;
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::commanders::{Extreme3DCommander, SpektrumCommander};
use stable_flight_sitl::recorder::{CopterColumns, CsvRecorder};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_xplane::xplane_bridge::{DatarefSource, XPlaneUdpBridge};
use stable_flight_sitl_xplane::{XPlaneConfiguration, XPlaneVehicle, airframes};

/// Stick-driven Alia 250 flight in X-Plane, with params and telemetry over the native link.
#[derive(Parser)]
struct Args {
    /// X-Plane's UDP address (`host:port`).
    #[arg(long, default_value = "127.0.0.1:49000")]
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
    /// Write one CSV row per tick to this file.
    #[arg(long)]
    record: Option<String>,
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

/// Flies the Alia pipeline from `commander` until Ctrl-C, with params and telemetry over the
/// native link.
fn fly(commander: impl Commander, args: &Args) {
    // Alia 250 in copter mode: tuned gains, mixer, and engine mapping.
    let airframe = airframes::alia250();

    let bridge = XPlaneUdpBridge::connect(&args.host).expect("failed to open the socket");
    let vehicle = XPlaneVehicle::connect(XPlaneConfiguration::new(bridge, airframe.mapping))
        .expect("X-Plane not ready");

    let guidance = CopterGuidance::new(airframe.guidance);
    let stabilizer = CopterStabilizer::new(airframe.stabilizer);
    let mixer = airframe.mixer;

    // The link: the publisher writes it each tick (the flight loop's telemetry), and the poller
    // reads it and serves params between ticks. Queue sizes: 32 param requests, 16 replies.
    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .expect("failed to open link socket")
        .into_halves();
    let mut fc_link = FcLink::<32, 16>::new();
    let (mut poller, link) = fc_link.wire(link_rx, link_tx);
    let recorder = CsvRecorder::create(args.record.as_deref(), CopterColumns)
        .expect("failed to create the record CSV");

    let mut flight_loop = FlightLoop::new(
        commander,
        vehicle,
        guidance,
        stabilizer,
        mixer,
        (link, recorder),
    );

    // Ctrl-C stops the loop cleanly (XPlaneVehicle hands control back on drop).
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
    info!("X-Plane: {:?}", flight_loop.vehicle().bridge().stats());
}
