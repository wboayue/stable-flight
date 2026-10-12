//! Hovers the eXplora VTOL tailsitter in RealFlight from a Logitech Extreme 3D joystick (or,
//! with `--commander spektrum`, a Spektrum transmitter through the WS3000T dongle), on the
//! [`airframes::explora_vtol`] mixer and mapping, with params and telemetry over the native link
//! as in `copter-manual-realflight`.
//!
//! Reference: https://github.com/robustini/eXploraVTOL/tree/main/simulation
//!
//! **Interim wiring until the nose-up stabilizer (#122):** the copter stabilizer flies the model
//! in RealFlight's hover-level frame (no body rotation, so hover reads level), and
//! [`HoverLevel`] turns its moments into the airplane-frame moments the tailsitter mixer takes.
//! In that frame the sticks fly it like a copter: roll tilts it sideways (differential thrust),
//! pitch tilts it toward its belly or back (elevons together), and yaw turns it about the
//! vertical (elevons opposite).
//!
//! `--record <csv>` writes one row per tick (sticks' moments, actuators, rates, attitude) for
//! reading the response to stick pulses.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example tailsitter-manual-realflight
//! cargo run -p stable-flight-sitl-realflight --example tailsitter-manual-realflight -- --record hover.csv
//! cargo run -p stable-flight-sitl-realflight --example tailsitter-manual-realflight -- --host 192.168.1.20:8080
//! ```

use std::fs::File;
use std::io::{BufWriter, Write};
use std::time::{Duration, Instant};

use clap::{Parser, ValueEnum};
use env_logger::Builder;
use log::{debug, warn};

use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::{
    ActuatorCommand, Commander, ControlOutput, FlightLoop, Mixer, Saturation, TelemetryPublisher,
    TelemetryRecord, euler_angles,
};
use stable_flight_link::fc::FcLink;
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::commanders::{Extreme3DCommander, SpektrumCommander};
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
    // eXplora VTOL model: mixer, channel mapping, and interim gains.
    let airframe = airframes::explora_vtol();

    // Interim (#122): fly in RealFlight's hover-level frame, not the airframe's +90° pitch.
    let host = args.host.clone();
    let connect =
        move || RealFlightRemoteBridge::with_timeouts(&host, CONNECT_TIMEOUT, REMOTE_IO_TIMEOUT);
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge: connect().expect("Failed to create bridge"),
        mapping: airframe.mapping,
        body_rotation: UnitQuaternion::identity(),
    })
    .with_reconnect(move || connect().map_err(BridgeError::from));

    let guidance = CopterGuidance::new(airframe.guidance);
    let stabilizer = CopterStabilizer::new(airframe.stabilizer);
    let mixer = HoverLevel::new(airframe.mixer, &airframe.body_rotation);

    // The link: the publisher writes it each tick (the flight loop's telemetry), and the poller
    // reads it and serves params between ticks. Queue sizes: 32 param requests, 16 replies.
    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .expect("failed to open link socket")
        .into_halves();
    let mut fc_link = FcLink::<32, 16>::new();
    let (mut poller, link) = fc_link.wire(link_rx, link_tx);
    let recorder =
        Recorder::create(args.record.as_deref()).expect("failed to create the record CSV");

    let mut flight_loop = FlightLoop::new(
        commander,
        vehicle,
        guidance,
        stabilizer,
        mixer,
        (link, recorder),
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

/// Interim (#122): turns moments in RealFlight's hover-level frame into the vehicle (airplane)
/// frame the tailsitter mixer takes.
///
/// The airframe's `body_rotation` is the vehicle frame expressed in RealFlight's, so its
/// inverse carries a RealFlight-frame vector into the vehicle frame, as `vehicle_state` does
/// for body rates. For the eXplora (+90° pitch): airplane roll is RealFlight's −yaw, pitch is
/// pitch, and airplane yaw is RealFlight's roll.
struct HoverLevel<M> {
    mixer: M,
    to_body: UnitQuaternion<f32>,
}

impl<M> HoverLevel<M> {
    fn new(mixer: M, body_rotation: &UnitQuaternion<f32>) -> Self {
        Self {
            mixer,
            to_body: body_rotation.inverse(),
        }
    }
}

impl<M: Mixer> Mixer for HoverLevel<M> {
    fn mix(&mut self, controls: &ControlOutput) -> ActuatorCommand {
        let moments = self.to_body * controls.moments;
        self.mixer
            .mix(&ControlOutput::new(controls.thrust, moments))
    }

    fn saturation(&self) -> Saturation {
        self.mixer.saturation()
    }
}

/// Writes one CSV row per tick to `--record`, for reading stick pulses; does nothing without it.
struct Recorder {
    out: Option<BufWriter<File>>,
    start: Instant,
}

impl Recorder {
    fn create(path: Option<&str>) -> std::io::Result<Self> {
        let out = match path {
            Some(path) => {
                let mut out = BufWriter::new(File::create(path)?);
                writeln!(
                    out,
                    "wall_s,armed,roll_rate,pitch_rate,yaw_rate,roll,pitch,heading,altitude,moment_x,moment_y,moment_z,thrust,motor_left,motor_right,elevon_left,elevon_right"
                )?;
                Some(out)
            }
            None => None,
        };
        Ok(Self {
            out,
            start: Instant::now(),
        })
    }
}

impl TelemetryPublisher for Recorder {
    fn publish(&mut self, record: &TelemetryRecord) {
        let Some(out) = &mut self.out else {
            return;
        };
        let state = record.state;
        let rates = state.angular_velocity_radps;
        let (roll, pitch, heading) = euler_angles(&state.attitude);
        let moments = record.control.moments;
        let actuators = record.actuators;
        // A failed write loses diagnostics, not flight: ignore it.
        let _ = writeln!(
            out,
            "{:.4},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
            self.start.elapsed().as_secs_f32(),
            u8::from(record.command.arm),
            rates.x,
            rates.y,
            rates.z,
            roll,
            pitch,
            heading,
            state.position_m.z,
            moments.x,
            moments.y,
            moments.z,
            record.control.thrust,
            actuators.motors[0],
            actuators.motors[1],
            actuators.servos[0],
            actuators.servos[1],
        );
    }
}
