//! Flies the eXplora VTOL tailsitter in RealFlight from a Logitech Extreme 3D joystick (or,
//! with `--commander spektrum`, a Spektrum transmitter through the WS3000T dongle): hover, the
//! transition to plane and back, and plane flight, on [`airframes::explora_vtol`], with params
//! and telemetry over the native link as in `copter-manual-realflight`.
//!
//! Reference: https://github.com/robustini/eXploraVTOL/tree/main/simulation
//!
//! The vehicle frame is the airplane's (the airframe's body rotation), so hover reads pitch 90°.
//! [`TailsitterGuidance`] and [`TailsitterStabilizer`] fly it:
//!
//! - **Copter:** the copter controllers in a tailsitter view, where nose-up hover reads level
//!   (ArduPilot's tailsitter view). The sticks fly it like a copter whose front is the belly:
//!   roll tilts it sideways (differential thrust), pitch tilts it toward its belly or back
//!   (elevons together), and yaw turns it about the vertical (elevons opposite).
//! - **Plane:** the plane controllers in the airplane frame: attitude sticks bank and pitch
//!   (turns coordinated), rate sticks fly acro (rolls, loops).
//! - **Regime:** Extreme 3D D-pad Right asks for plane, Left for copter (both attitude sticks),
//!   Up switches to rate sticks in the current regime; the Spektrum mode switch asks for plane
//!   at one end. Asking for plane starts the transition once 10 m above where it armed; each
//!   mode change is logged.
//!
//! `--record <csv>` writes one row per tick (mode, setpoint, moments, actuators, rates,
//! attitude, airspeed, climb rate) for reading the response to stick pulses and transitions.
//!
//! [`TailsitterGuidance`]: stable_flight::guidance::vtol::TailsitterGuidance
//! [`TailsitterStabilizer`]: stable_flight::stabilizers::vtol::TailsitterStabilizer
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example tailsitter-manual-realflight
//! cargo run -p stable-flight-sitl-realflight --example tailsitter-manual-realflight -- --record hover.csv
//! cargo run -p stable-flight-sitl-realflight --example tailsitter-manual-realflight -- --host 192.168.1.20:8080
//! ```

use std::time::Duration;

use clap::{Parser, ValueEnum};
use env_logger::Builder;
use log::{debug, info, warn};

use stable_flight::guidance::vtol::TailsitterGuidance;
use stable_flight::stabilizers::vtol::TailsitterStabilizer;
use stable_flight::{
    Commander, FlightLoop, FlightModeReport, TelemetryPublisher, TelemetryRecord, View,
};
use stable_flight_link::fc::FcLink;
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::commanders::{Extreme3DCommander, SpektrumCommander};
use stable_flight_sitl::recorder::{CsvRecorder, TailsitterColumns};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::{BridgeError, RealFlightRemoteBridge};
use stable_flight_sitl_realflight::{
    REMOTE_IO_TIMEOUT, RealFlightConfiguration, RealFlightVehicle, airframes,
};

/// Connect timeout for the RealFlight proxy; a reconnect attempt blocks its tick.
const CONNECT_TIMEOUT: Duration = Duration::from_millis(250);

/// Stick-driven tailsitter hover in RealFlight, with params and telemetry over the native link.
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

/// Flies the eXplora from `commander` until Ctrl-C, with params and telemetry over the native
/// link.
fn fly(commander: impl Commander, args: &Args) {
    // eXplora VTOL model: hover (view axes) and plane gains, transition, mixer, channel
    // mapping, and body rotation.
    let airframe = airframes::explora_vtol();

    let host = args.host.clone();
    let connect =
        move || RealFlightRemoteBridge::with_timeouts(&host, CONNECT_TIMEOUT, REMOTE_IO_TIMEOUT);
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration::from_airframe(
        &airframe,
        connect().expect("Failed to create bridge"),
    ))
    .with_reconnect(move || connect().map_err(BridgeError::from));

    // Hover (copter controllers in the tailsitter view) and plane controllers, under one
    // transition.
    let guidance = TailsitterGuidance::new(airframe.guidance);
    let stabilizer = TailsitterStabilizer::new(airframe.stabilizer);
    let mixer = airframe.mixer;

    // The link: the publisher writes it each tick (the flight loop's telemetry), and the poller
    // reads it and serves params between ticks. Queue sizes: 32 param requests, 16 replies.
    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .expect("failed to open link socket")
        .into_halves();
    let mut fc_link = FcLink::<32, 16>::new();
    let (mut poller, link) = fc_link.wire(link_rx, link_tx);
    let columns = TailsitterColumns {
        view: *View::tailsitter(()).rotation(),
    };
    let recorder = CsvRecorder::create(args.record.as_deref(), columns)
        .expect("failed to create the record CSV");

    let mut flight_loop = FlightLoop::new(
        commander,
        vehicle,
        guidance,
        stabilizer,
        mixer,
        (link, (recorder, ModeLog::default())),
    );

    // Ctrl-C stops the loop cleanly (RealFlightVehicle re-enables RC control on drop).
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
