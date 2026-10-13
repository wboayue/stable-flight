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

use std::fs::File;
use std::io::{BufWriter, Write};
use std::time::{Duration, Instant};

use clap::{Parser, ValueEnum};
use env_logger::Builder;
use log::{debug, info, warn};

use stable_flight::guidance::vtol::TailsitterGuidance;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::stabilizers::vtol::TailsitterStabilizer;
use stable_flight::{
    Commander, ControlInput, FlightLoop, FlightModeReport, Setpoint, TelemetryPublisher,
    TelemetryRecord, View, euler_angles,
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
    let recorder = Recorder::create(args.record.as_deref(), *View::tailsitter(()).rotation())
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

/// Writes one CSV row per tick to `--record`, for reading stick pulses and transitions; does
/// nothing without it. Rates, moments, and actuators are in the airplane body frame; `view_*`
/// are the attitude in the tailsitter view, where hover is level (the body Euler angles are
/// degenerate there). `mode` is the flight mode (0 copter, 1 plane, 2 to plane, 3 to copter)
/// and `refused` whether a requested regime is refused. `sp_kind` names guidance's setpoint;
/// `sp_roll`/`sp_pitch`/`sp_heading` are its angles (in the view for a copter attitude, the
/// airplane frame for a plane one; NaN otherwise) and `sp_p`/`sp_q`/`sp_r` its body rates in
/// rad/s (NaN for attitudes and stick-fraction rates). `sp_thrust` is linear thrust, before the
/// stabilizer's thrust curve; `thrust` is after it. `airspeed_mps` is NaN without air data,
/// `climb_mps` the vertical speed, up positive, and `stick_throttle` the commander's throttle
/// (or absolute thrust), before boost, floor, and sink guard.
struct Recorder {
    out: Option<BufWriter<File>>,
    start: Instant,
    view: UnitQuaternion<f32>,
}

impl Recorder {
    fn create(path: Option<&str>, view: UnitQuaternion<f32>) -> std::io::Result<Self> {
        let out = match path {
            Some(path) => {
                let mut out = BufWriter::new(File::create(path)?);
                writeln!(
                    out,
                    "wall_s,armed,mode,refused,roll_rate,pitch_rate,yaw_rate,roll,pitch,heading,view_roll,view_pitch,view_heading,altitude,airspeed_mps,moment_x,moment_y,moment_z,thrust,motor_left,motor_right,elevon_left,elevon_right,sp_kind,sp_roll,sp_pitch,sp_heading,sp_p,sp_q,sp_r,sp_thrust,climb_mps,stick_throttle"
                )?;
                Some(out)
            }
            None => None,
        };
        Ok(Self {
            out,
            start: Instant::now(),
            view,
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
        let view = state.rotate_body(&self.view);
        let moments = record.control.moments;
        let actuators = record.actuators;
        let nan = f32::NAN;
        let (sp_kind, sp_roll, sp_pitch, sp_heading, sp_p, sp_q, sp_r) = match record.setpoint {
            Setpoint::Attitude(s) => (
                "attitude",
                s.roll_rad,
                s.pitch_rad,
                s.heading_rad,
                nan,
                nan,
                nan,
            ),
            Setpoint::Plane(s) => ("plane", s.roll_rad, s.pitch_rad, nan, nan, nan, nan),
            Setpoint::BodyRate(s) => (
                "body_rate",
                nan,
                nan,
                nan,
                s.roll_radps,
                s.pitch_radps,
                s.yaw_radps,
            ),
            Setpoint::PlaneRate(s) => (
                "plane_rate",
                nan,
                nan,
                nan,
                s.roll_radps,
                s.pitch_radps,
                s.yaw_radps,
            ),
            _ => ("rate", nan, nan, nan, nan, nan, nan),
        };
        let mode = match record.flight_mode.mode {
            stable_flight::FlightMode::Copter => 0,
            stable_flight::FlightMode::Plane => 1,
            stable_flight::FlightMode::ToPlane => 2,
            stable_flight::FlightMode::ToCopter => 3,
            _ => 9,
        };
        let sp_thrust = record.setpoint.thrust();
        let stick_throttle = match &record.command.input {
            ControlInput::Attitude(c) => c.throttle,
            ControlInput::Rate(c) => c.throttle,
            ControlInput::AbsoluteAttitude(c) => c.thrust,
            _ => f32::NAN,
        };
        // A failed write loses diagnostics, not flight: ignore it.
        let _ = writeln!(
            out,
            "{:.4},{},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.3},{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.3},{:.4}",
            self.start.elapsed().as_secs_f32(),
            u8::from(record.command.arm),
            mode,
            u8::from(record.flight_mode.refusal.is_some()),
            rates.x,
            rates.y,
            rates.z,
            roll,
            pitch,
            heading,
            view.roll_rad,
            view.pitch_rad,
            view.heading_rad,
            state.position_m.z,
            state.valid_airspeed_mps().unwrap_or(f32::NAN),
            moments.x,
            moments.y,
            moments.z,
            record.control.thrust,
            actuators.motors[0],
            actuators.motors[1],
            actuators.servos[0],
            actuators.servos[1],
            sp_kind,
            sp_roll,
            sp_pitch,
            sp_heading,
            sp_p,
            sp_q,
            sp_r,
            sp_thrust,
            state.velocity_mps.z,
            stick_throttle,
        );
    }
}
