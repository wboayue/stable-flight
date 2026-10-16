//! Flies the Alia 250 in X-Plane from a Logitech Extreme 3D joystick (or, with `--commander
//! spektrum`, a Spektrum transmitter through the WS3000T dongle): hover, the transition to
//! plane and back, and plane flight, on [`airframes::alia250`], with params and telemetry over
//! the native link (`stable-flight-link`, UDP) so they can be watched and tuned in flight.
//!
//! [`QuadPlaneGuidance`] and [`QuadPlaneStabilizer`] fly it:
//!
//! - **Copter:** the copter controllers on the lift rotors, the surfaces neutral, the pusher
//!   stopped. The sticks fly it as `copter-manual-realflight` flies a quad.
//! - **Plane:** the plane controllers on the surfaces and pusher: attitude sticks bank and
//!   pitch (turns coordinated), the throttle is the pusher's.
//! - **Regime:** Extreme 3D D-pad Right asks for plane, Left for copter (both attitude sticks),
//!   Up switches to rate sticks in the current regime; the Spektrum mode switch asks for plane
//!   at one end. Asking for plane starts the forward transition 20 m above where it armed:
//!   push the throttle up, since going forward the pusher flies the throttle stick. Each mode
//!   change is logged.
//!
//! [`QuadPlaneGuidance`]: stable_flight::guidance::vtol::QuadPlaneGuidance
//! [`QuadPlaneStabilizer`]: stable_flight::stabilizers::vtol::QuadPlaneStabilizer
//!
//! Load the Alia 250 at X-Plane's default weight (6,299 lb) on a runway, in calm weather.
//! X-Plane listens on UDP 49000; `--host` takes a hostname (e.g. `oya.local:49000`) for
//! X-Plane on another machine. The vehicle takes the throttles and surfaces once every state
//! dataref has arrived, and logs the bridge's `--origin` for where it started. Ctrl-C hands
//! the aircraft back to X-Plane.
//!
//! The link binds `--link-bind` and talks to the bridge (`stable-flight-bridge`) at
//! `--link-peer`; both default to the localhost ports the bridge expects. `--record <csv>`
//! writes one row per tick (`stable_flight_sitl::recorder::QuadPlaneColumns`: both sides'
//! outputs, every lift rotor, the pusher, the surfaces, and the lift share); `plot_flight
//! --record` plots it. `--param NAME=VALUE` (repeatable) sets a param before takeoff, as the
//! link would (e.g. `PSTAB_RLL_P=0.6`); an unknown name or a rejected value stops the example
//! before it flies.
//!
//! ```text
//! cargo run -p stable-flight-sitl-xplane --example quadplane-manual-xplane
//! cargo run -p stable-flight-sitl-xplane --example quadplane-manual-xplane -- --record flight.csv
//! ```

use clap::{Parser, ValueEnum};
use env_logger::Builder;
use log::{debug, info, warn};

use stable_flight::guidance::vtol::QuadPlaneGuidance;
use stable_flight::params::Params;
use stable_flight::stabilizers::vtol::QuadPlaneStabilizer;
use stable_flight::{Commander, FlightLoop, FlightModeReport, TelemetryPublisher, TelemetryRecord};
use stable_flight_link::fc::FcLink;
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::commanders::{Extreme3DCommander, SpektrumCommander};
use stable_flight_sitl::recorder::{CsvRecorder, QuadPlaneColumns};
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
    // Alia 250: tuned gains, mixer, and engine and surface mapping.
    let airframe = airframes::alia250();

    let bridge = XPlaneUdpBridge::connect(&args.host).expect("failed to open the socket");
    let vehicle = XPlaneVehicle::connect(XPlaneConfiguration::new(bridge, airframe.mapping))
        .expect("X-Plane not ready");

    // Hover and plane controllers under one transition.
    let guidance = QuadPlaneGuidance::new(airframe.guidance);
    let stabilizer = QuadPlaneStabilizer::new(airframe.stabilizer);
    let mixer = airframe.mixer;

    // The link: the publisher writes it each tick (the flight loop's telemetry), and the poller
    // reads it and serves params between ticks. Queue sizes: 32 param requests, 16 replies.
    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .expect("failed to open link socket")
        .into_halves();
    let mut fc_link = FcLink::<32, 16>::new();
    let (mut poller, link) = fc_link.wire(link_rx, link_tx);
    let recorder = CsvRecorder::create(args.record.as_deref(), QuadPlaneColumns)
        .expect("failed to create the record CSV");

    let mut flight_loop = FlightLoop::new(
        commander,
        vehicle,
        guidance,
        stabilizer,
        mixer,
        (link, (recorder, ModeLog::default())),
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
