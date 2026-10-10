//! Flies the quadcopter from a Logitech Extreme 3D joystick, exactly as
//! `copter-manual-realflight` does (or, with `--script`, a waypoint script as
//! `copter-auto-realflight` does), and records every raw `SimulatorState` RealFlight returns
//! to CSV, unconverted, with the 12 channels sent in the same exchange.
//!
//! RealFlight only reports state while its Link controller drives the model, so a passive
//! capture alongside a transmitter isn't possible; this taps the bridge under the normal
//! pipeline instead. The CSV is for checking the adapter's frame conversions (#62) against what
//! RealFlight actually reports. Fly a mix of roll, pitch, yaw, translation, and climb; stop
//! with Ctrl-C. Params and telemetry go over the native link for the bridge (`just bridge`), as
//! in `copter-manual-realflight` (`--link-bind`, `--link-peer`).
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example capture-state-realflight -- --output capture.csv
//! cargo run -p stable-flight-sitl-realflight --example capture-state-realflight -- --script flights/hover.csv
//! ```

use std::cell::RefCell;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::time::Duration;

use clap::Parser;
use env_logger::Builder;
use log::{debug, info, warn};

use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::{Commander, FlightLoop};
use stable_flight_link::fc::{FcLink, FcLinkConfig};
use stable_flight_link::publisher::LinkPublisherConfig;
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::commanders::extreme_3d::Extreme3DCommander;
use stable_flight_sitl::commanders::script::ScriptCommander;
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::{
    BridgeError, ControlInputs, RealFlightBridge, RealFlightRemoteBridge, SimulatorState,
};
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

/// How often a full telemetry record goes out: the bridge needs only its thrust, for throttle.
const RECORD_INTERVAL: Duration = Duration::from_millis(200);

/// Number of RealFlight channels recorded after the state columns.
const CHANNELS: usize = 12;

/// One CSV column: its header name and how to read it from a report. Booleans record as 0/1.
type Column = (&'static str, fn(&SimulatorState) -> f32);

/// The state columns, in CSV order. The channels sent with the exchange follow them.
const COLUMNS: &[Column] = &[
    ("physics_time_s", |s| s.current_physics_time_s),
    ("roll_deg", |s| s.roll_deg),
    ("inclination_deg", |s| s.inclination_deg),
    ("azimuth_deg", |s| s.azimuth_deg),
    ("roll_rate_dps", |s| s.roll_rate_dps),
    ("pitch_rate_dps", |s| s.pitch_rate_dps),
    ("yaw_rate_dps", |s| s.yaw_rate_dps),
    ("q_w", |s| s.orientation.w),
    ("q_x", |s| s.orientation.x),
    ("q_y", |s| s.orientation.y),
    ("q_z", |s| s.orientation.z),
    ("position_x_m", |s| s.aircraft_position_x_m),
    ("position_y_m", |s| s.aircraft_position_y_m),
    ("altitude_asl_m", |s| s.altitude_asl_m),
    ("altitude_agl_m", |s| s.altitude_agl_m),
    ("velocity_world_x", |s| s.velocity_world_mps.x),
    ("velocity_world_y", |s| s.velocity_world_mps.y),
    ("velocity_world_z", |s| s.velocity_world_mps.z),
    ("velocity_body_x", |s| s.velocity_body_mps.x),
    ("velocity_body_y", |s| s.velocity_body_mps.y),
    ("velocity_body_z", |s| s.velocity_body_mps.z),
    ("acceleration_world_x", |s| s.acceleration_world_mps2.x),
    ("acceleration_world_y", |s| s.acceleration_world_mps2.y),
    ("acceleration_world_z", |s| s.acceleration_world_mps2.z),
    ("acceleration_body_x", |s| s.acceleration_body_mps2.x),
    ("acceleration_body_y", |s| s.acceleration_body_mps2.y),
    ("acceleration_body_z", |s| s.acceleration_body_mps2.z),
    ("is_touching_ground", |s| {
        f32::from(u8::from(s.is_touching_ground))
    }),
    ("is_locked", |s| f32::from(u8::from(s.is_locked))),
];

/// Writes the CSV header: the state columns, then `ch0`…`ch11`.
fn write_header(out: &mut impl Write) -> std::io::Result<()> {
    let names = COLUMNS.iter().map(|(name, _)| name.to_string());
    let channels = (0..CHANNELS).map(|i| format!("ch{i}"));
    writeln!(
        out,
        "{}",
        names.chain(channels).collect::<Vec<_>>().join(",")
    )
}

/// Passes every call through to `inner` and writes each returned state to `out`.
struct RecordingBridge<B> {
    inner: B,
    out: RefCell<BufWriter<File>>,
}

impl<B: RealFlightBridge> RealFlightBridge for RecordingBridge<B> {
    fn exchange_data(&self, control: &ControlInputs) -> Result<SimulatorState, BridgeError> {
        let state = self.inner.exchange_data(control)?;
        if let Err(e) = write_row(&mut *self.out.borrow_mut(), &state, control) {
            warn!("Failed to write capture row: {e}");
        }
        Ok(state)
    }

    fn enable_rc(&self) -> Result<(), BridgeError> {
        self.inner.enable_rc()
    }

    fn disable_rc(&self) -> Result<(), BridgeError> {
        self.inner.disable_rc()
    }

    fn reset_aircraft(&self) -> Result<(), BridgeError> {
        self.inner.reset_aircraft()
    }
}

impl<B> Drop for RecordingBridge<B> {
    fn drop(&mut self) {
        if let Err(e) = self.out.get_mut().flush() {
            warn!("Failed to flush capture: {e}");
        }
    }
}

/// Writes one CSV row: the state RealFlight returned, then the channels sent with that exchange.
fn write_row(
    out: &mut impl Write,
    s: &SimulatorState,
    control: &ControlInputs,
) -> std::io::Result<()> {
    let values = COLUMNS
        .iter()
        .map(|(_, read)| read(s))
        .chain(control.channels);
    for (i, value) in values.enumerate() {
        let separator = if i == 0 { "" } else { "," };
        write!(out, "{separator}{value}")?;
    }
    writeln!(out)
}

fn main() {
    Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp_millis()
        .init();

    let args = Args::parse();

    let mut out = BufWriter::new(File::create(&args.output).expect("Failed to create output"));
    write_header(&mut out).expect("Failed to write header");
    let bridge = RecordingBridge {
        inner: RealFlightRemoteBridge::with_timeouts(
            &args.host,
            CONNECT_TIMEOUT,
            REMOTE_IO_TIMEOUT,
        )
        .expect("Failed to create bridge"),
        out: RefCell::new(out),
    };

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
fn fly(commander: impl Commander, bridge: RecordingBridge<RealFlightRemoteBridge>, link: UdpLink) {
    let airframe = airframes::quadcopter_x();
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge,
        mapping: airframe.mapping,
    });
    let guidance = CopterGuidance::new(airframe.guidance);
    let stabilizer = CopterStabilizer::new(airframe.stabilizer);
    let mixer = airframe.mixer;
    // The link: the publisher writes it each tick (the flight loop's telemetry), and the poller
    // reads it and serves params between ticks. Queue sizes: 32 param requests, 16 replies.
    let (link_rx, link_tx) = link.into_halves();
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

    // Ctrl-C stops the loop; dropping the vehicle re-enables RC and flushes the capture.
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
