//! Flies the eXplora VTOL tailsitter in RealFlight from the native link (`stable-flight-link`,
//! UDP), as `copter-companion-realflight` flies the quad: the link's commands fly it, its regime
//! requests (`Control::Regime`, e.g. QGC's transition button through the bridge) transition it
//! between hover and plane, and a failsafe holds position in hover when the link goes quiet,
//! handing control back once it has been back for a second.
//!
//! The commander is `FailsafeCommander<LinkCommander, HoldCommander>`; the pipeline is
//! `tailsitter-manual-realflight`'s ([`airframes::explora_vtol`]), so asking for plane starts
//! the transition once 10 m above where it armed. Each mode change, and each refusal, is logged.
//! Fly it with the bridge as a tailsitter, so QGC shows its VTOL state, the transition button,
//! and hover attitude in the hover view. The RealFlight host defaults to `10.211.55.3:8080`;
//! override it with `--host`. `--record <csv>` writes one row per tick (the tailsitter columns).
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example tailsitter-companion-realflight
//! cargo run -p stable-flight-bridge -- --vehicle-type tailsitter   # in a second terminal
//! ```

use std::time::Duration;

use clap::Parser;
use env_logger::Builder;
use log::{debug, info, warn};

use stable_flight::commanders::{FailsafeCommander, FailsafeConfig, HoldCommander};
use stable_flight::guidance::vtol::TailsitterGuidance;
use stable_flight::stabilizers::vtol::TailsitterStabilizer;
use stable_flight::view::tailsitter_rotation;
use stable_flight::{
    Commander, FailsafeState, FlightLoop, FlightModeReport, TelemetryPublisher, TelemetryRecord,
};
use stable_flight_link::fc::{FcLink, FcLinkConfig};
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::recorder::{CsvRecorder, TailsitterColumns};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::{BridgeError, RealFlightRemoteBridge};
use stable_flight_sitl_realflight::{
    REMOTE_IO_TIMEOUT, RealFlightConfiguration, RealFlightVehicle, airframes,
};

/// Connect timeout for the RealFlight proxy; a reconnect attempt blocks its tick.
const CONNECT_TIMEOUT: Duration = Duration::from_millis(250);

/// eXplora tailsitter flight in RealFlight from the native link, holding when the link goes
/// quiet.
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
    /// Write one CSV row per tick to this file.
    #[arg(long)]
    record: Option<String>,
}

fn main() {
    Builder::from_default_env().format_timestamp_millis().init();

    let args = Args::parse();

    // eXplora VTOL model: hover (view axes) and plane gains, transition, mixer, channel
    // mapping, and body rotation.
    let airframe = airframes::explora_vtol();

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

    // Hover (copter controllers in the tailsitter view) and plane controllers, under one
    // transition.
    let guidance = TailsitterGuidance::new(airframe.guidance);
    let stabilizer = TailsitterStabilizer::new(airframe.stabilizer);
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

    // Hold position (in hover) after 1 s without a message; hand back after 1 s of messages.
    let commander = FailsafeCommander::new(
        link_commander,
        HoldCommander::default(),
        FailsafeConfig::default(),
    )
    .expect("valid failsafe config");

    let columns = TailsitterColumns {
        view: tailsitter_rotation(),
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
