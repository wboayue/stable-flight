//! Records RealFlight's raw `SimulatorState` to CSV while you fly with your own transmitter.
//!
//! Hands control to RealFlight's RC device (`enable_rc`), then polls `exchange_data` at a
//! fixed rate and writes every frame-relevant field unconverted, so the adapter's frame
//! conversions (#62) can be checked against what RealFlight actually reports. Fly a mix of
//! roll, pitch, yaw, translation, and climb; stop with Ctrl-C.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example capture-state-realflight -- --output capture.csv
//! ```

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use env_logger::Builder;
use log::{info, warn};

use stable_flight_sitl::ShutdownSignal;
use stable_flight_sitl_realflight::realflight_bridge::{
    ControlInputs, RealFlightBridge, RealFlightRemoteBridge, SimulatorState,
};

/// Raw RealFlight state capture.
#[derive(Parser)]
struct Args {
    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,
    /// CSV file to write.
    #[arg(long, default_value = "realflight-capture.csv")]
    output: PathBuf,
    /// Sample rate in Hz.
    #[arg(long, default_value_t = 50.0)]
    rate: f32,
}

const HEADER: &str = "physics_time_s,roll_deg,inclination_deg,azimuth_deg,\
roll_rate_dps,pitch_rate_dps,yaw_rate_dps,q_w,q_x,q_y,q_z,\
position_x_m,position_y_m,altitude_asl_m,altitude_agl_m,\
velocity_world_x,velocity_world_y,velocity_world_z,\
velocity_body_x,velocity_body_y,velocity_body_z,\
acceleration_world_x,acceleration_world_y,acceleration_world_z,\
acceleration_body_x,acceleration_body_y,acceleration_body_z,\
is_touching_ground,is_locked";

fn write_row(out: &mut impl Write, s: &SimulatorState) -> std::io::Result<()> {
    let q = s.orientation;
    let (vw, vb) = (s.velocity_world_mps, s.velocity_body_mps);
    let (aw, ab) = (s.acceleration_world_mps2, s.acceleration_body_mps2);
    writeln!(
        out,
        "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        s.current_physics_time_s,
        s.roll_deg,
        s.inclination_deg,
        s.azimuth_deg,
        s.roll_rate_dps,
        s.pitch_rate_dps,
        s.yaw_rate_dps,
        q.w,
        q.x,
        q.y,
        q.z,
        s.aircraft_position_x_m,
        s.aircraft_position_y_m,
        s.altitude_asl_m,
        s.altitude_agl_m,
        vw.x,
        vw.y,
        vw.z,
        vb.x,
        vb.y,
        vb.z,
        aw.x,
        aw.y,
        aw.z,
        ab.x,
        ab.y,
        ab.z,
        s.is_touching_ground,
        s.is_locked,
    )
}

fn main() {
    Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp_millis()
        .init();

    let args = Args::parse();
    let period = Duration::from_secs_f32(1.0 / args.rate);

    let bridge = RealFlightRemoteBridge::new(&args.host).expect("Failed to create bridge");
    // Fly with RealFlight's own RC device; Link only observes.
    if let Err(e) = bridge.enable_rc() {
        warn!("Failed to enable RC: {e}");
    }

    let mut out = BufWriter::new(File::create(&args.output).expect("Failed to create output"));
    writeln!(out, "{HEADER}").expect("Failed to write header");

    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");
    let input = ControlInputs::default();
    let mut rows = 0usize;
    let mut next = Instant::now();

    info!(
        "Recording to {} at {} Hz; Ctrl-C to stop",
        args.output.display(),
        args.rate
    );
    while !shutdown.is_set() {
        match bridge.exchange_data(&input) {
            Ok(state) => {
                write_row(&mut out, &state).expect("Failed to write row");
                rows += 1;
            }
            Err(e) => warn!("Exchange failed: {e}"),
        }

        next += period;
        if let Some(wait) = next.checked_duration_since(Instant::now()) {
            thread::sleep(wait);
        }
    }

    out.flush().expect("Failed to flush output");
    info!("Wrote {rows} rows to {}", args.output.display());
}
