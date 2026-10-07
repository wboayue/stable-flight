//! Flies the quadcopter from a Logitech Extreme 3D joystick, exactly as
//! `copter-manual-realflight` does (or, with `--script`, a waypoint script as
//! `copter-auto-realflight` does), and records every raw `SimulatorState` RealFlight returns
//! to CSV, unconverted, with the 12 channels sent in the same exchange.
//!
//! RealFlight only reports state while its Link controller drives the model, so a passive
//! capture alongside a transmitter isn't possible; this taps the bridge under the normal
//! pipeline instead. The CSV is for checking the adapter's frame conversions (#62) against what
//! RealFlight actually reports. Fly a mix of roll, pitch, yaw, translation, and climb; stop
//! with Ctrl-C.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example capture-state-realflight -- --output capture.csv
//! cargo run -p stable-flight-sitl-realflight --example capture-state-realflight -- --script flights/hover.csv
//! ```

use std::cell::RefCell;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;

use clap::Parser;
use env_logger::Builder;
use log::{info, warn};

use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::{Commander, FlightLoop};
use stable_flight_sitl::commanders::extreme_3d::Extreme3DCommander;
use stable_flight_sitl::commanders::script::ScriptCommander;
use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::{
    BridgeError, ControlInputs, RealFlightBridge, RealFlightRemoteBridge, SimulatorState,
};
use stable_flight_sitl_realflight::{RealFlightConfiguration, RealFlightVehicle, airframes};

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
}

const HEADER: &str = "physics_time_s,roll_deg,inclination_deg,azimuth_deg,\
roll_rate_dps,pitch_rate_dps,yaw_rate_dps,q_w,q_x,q_y,q_z,\
position_x_m,position_y_m,altitude_asl_m,altitude_agl_m,\
velocity_world_x,velocity_world_y,velocity_world_z,\
velocity_body_x,velocity_body_y,velocity_body_z,\
acceleration_world_x,acceleration_world_y,acceleration_world_z,\
acceleration_body_x,acceleration_body_y,acceleration_body_z,\
is_touching_ground,is_locked,\
ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,ch9,ch10,ch11";

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
    let q = s.orientation;
    let (vw, vb) = (s.velocity_world_mps, s.velocity_body_mps);
    let (aw, ab) = (s.acceleration_world_mps2, s.acceleration_body_mps2);
    write!(
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
    )?;
    for channel in control.channels {
        write!(out, ",{channel}")?;
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
    writeln!(out, "{HEADER}").expect("Failed to write header");
    let bridge = RecordingBridge {
        inner: RealFlightRemoteBridge::new(&args.host).expect("Failed to create bridge"),
        out: RefCell::new(out),
    };

    info!("Recording to {}; Ctrl-C to stop", args.output.display());
    match &args.script {
        Some(script) => fly(
            ScriptCommander::new(script).expect("failed to load flight script"),
            bridge,
        ),
        None => fly(Extreme3DCommander::default(), bridge),
    }
}

/// Flies the QuadCopterX pipeline from `commander` until Ctrl-C.
fn fly(commander: impl Commander, bridge: RecordingBridge<RealFlightRemoteBridge>) {
    let airframe = airframes::quadcopter_x();
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge,
        mapping: airframe.mapping,
    });
    let guidance = CopterGuidance::new(airframe.guidance);
    let stabilizer = CopterStabilizer::new(airframe.stabilizer);
    let mixer = airframe.mixer;
    let telemetry = MavlinkPublisher::new(MavlinkConfiguration {});

    let mut flight_loop =
        FlightLoop::new(commander, vehicle, guidance, stabilizer, mixer, telemetry);

    // Ctrl-C stops the loop; dropping the vehicle re-enables RC and flushes the capture.
    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");
    let runner = RealTimeRunner::new(250.0).expect("valid loop frequency"); // Hz
    runner.run(&mut flight_loop, &shutdown);
}
