//! Records every raw X-Plane state sample to CSV, unconverted, while you fly with X-Plane's own
//! controls: it reads only, setting no override, so X-Plane keeps the aircraft.
//!
//! The CSV is for checking the backend's frame conversion offline
//! (`cargo run --bin check_frames -- --sim xplane capture.csv`). Fly a mix of roll, pitch, yaw,
//! translation, and climb, and sit still on the ground for a few seconds (the gravity check);
//! stop with Ctrl-C. X-Plane listens on UDP 49000 by default; `--host` takes a hostname (e.g.
//! `oya.local:49000`) for X-Plane on another machine.
//!
//! ```text
//! cargo run -p stable-flight-sitl-xplane --example capture-state-xplane -- --output capture.csv
//! ```

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::thread;
use std::time::Duration;

use clap::Parser;
use env_logger::Builder;
use log::{info, warn};

use stable_flight_sitl::ShutdownSignal;
use stable_flight_sitl_xplane::capture::{write_header, write_row};
use stable_flight_sitl_xplane::xplane_bridge::{DatarefSource, Latest, XPlaneUdpBridge};
use stable_flight_sitl_xplane::{DEFAULT_READY_TIMEOUT, StateDatarefs, wait_ready};

/// Gap between polls: well under X-Plane's frame time, so every frame is its own row.
const POLL_INTERVAL: Duration = Duration::from_millis(1);

/// Raw X-Plane state capture, flown with X-Plane's own controls.
#[derive(Parser)]
struct Args {
    /// X-Plane's UDP address (`host:port`).
    #[arg(long, default_value = "127.0.0.1:49000")]
    host: String,
    /// CSV file to write.
    #[arg(long, default_value = "xplane-capture.csv")]
    output: PathBuf,
}

fn main() {
    Builder::from_default_env().format_timestamp_millis().init();
    let args = Args::parse();

    let mut bridge = XPlaneUdpBridge::connect(&args.host).expect("failed to open the socket");
    let datarefs = StateDatarefs::subscribe(&mut bridge).expect("failed to subscribe");
    let mut latest = Latest::default();
    wait_ready(&mut bridge, &datarefs, &mut latest, DEFAULT_READY_TIMEOUT)
        .expect("X-Plane not ready");

    let mut out = BufWriter::new(File::create(&args.output).expect("failed to create the CSV"));
    write_header(&mut out, &[]).expect("failed to write the header");
    info!("Capturing to {}; Ctrl-C to stop", args.output.display());

    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");
    let mut rows = 0usize;
    while !shutdown.is_set() {
        match bridge.poll() {
            Ok(Some(frame)) => {
                latest.apply(&frame);
                if let Some(sample) = datarefs.sample(&latest) {
                    write_row(&mut out, &sample, &[]).expect("failed to write a row");
                    rows += 1;
                }
            }
            Ok(None) => thread::sleep(POLL_INTERVAL),
            Err(e) => warn!("X-Plane receive failed: {e}"),
        }
    }
    out.flush().expect("failed to flush the CSV");
    info!("{rows} samples written; {:?}", bridge.stats());
}
