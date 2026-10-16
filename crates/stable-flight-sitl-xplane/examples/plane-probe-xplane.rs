//! Measures the Alia 250's wing in X-Plane for fitting a sim model to it (#392), on
//! [`airframes::alia250`]: steady level flight at several airspeeds, a climb at full power, and
//! surface pulses and roll holds in level flight.
//!
//! It flies a waypoint script (default `flights/alia_plane_probe.csv`, from Oceano, L52: a climb
//! in hover, a transition, then level plane legs offshore at 48, 46, 44, and 40 m/s and a second
//! lap at 44, and a back transition home) and records every tick (`--record`,
//! `QuadPlaneColumns`). For the climb at full power, fly `flights/alia_full_power.csv` on
//! `quadplane-auto-xplane` with `--param PGUID_THR_MIN=1`. On each
//! plane leg, once the airspeed has held within 1.5 m/s of the leg's speed and the wings within
//! 3° of level for 5 s, it adds a cruise moment of `--pulse` for 1 s on one axis at a time, each
//! sign in turn, with 4 s of quiet before each: roll (ailerons), pitch (elevators), and yaw
//! (rudders). The plane loops keep flying under the pulse, so the response is the airframe's and
//! the loops' together; the log names each pulse's start for the fit. After the pulses it holds
//! the ailerons at `--roll-hold` open loop (the roll loop's output replaced) for 1.5 s right,
//! 3 s left, and 1.5 s right, ending near wings level: the steady roll rate under a held
//! aileron measures the roll damping. Each leg runs the schedule once.
//!
//! The first recorded tick's UTC time is logged, to line the recording up with a side log of
//! X-Plane's own values (engine thrust and power). It lands on the script's last waypoint and
//! exits once disarmed; Ctrl-C stops it sooner (`XPlaneVehicle` hands control back).
//!
//! ```text
//! cargo run --release -p stable-flight-sitl-xplane --example plane-probe-xplane -- --record probe.csv
//! ```

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::{SystemTime, UNIX_EPOCH};

use clap::Parser;
use env_logger::Builder;
use log::{info, warn};

use stable_flight::commanders::RouteEnd;
use stable_flight::guidance::vtol::QuadPlaneGuidance;
use stable_flight::stabilizers::vtol::QuadPlaneStabilizer;
use stable_flight::{
    ActuatorCommand, ControlInput, ControlOutput, FlightLoop, FlightMode, Mixer, Saturation,
    TelemetryPublisher, TelemetryRecord, Vec3,
};
use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::recorder::{CsvRecorder, QuadPlaneColumns};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_xplane::xplane_bridge::XPlaneUdpBridge;
use stable_flight_sitl_xplane::{XPlaneConfiguration, XPlaneVehicle, airframes};

/// How long the airspeed and the wings hold before a leg's first pulse, in s.
const SETTLE_S: f32 = 5.0;
/// How close to the leg's speed the airspeed must hold, in m/s.
const SPEED_BAND_MPS: f32 = 1.5;
/// How close to level the wings must hold, in rad.
const BANK_BAND_RAD: f32 = 3.0 * core::f32::consts::PI / 180.0;
/// The roll holds: sign and seconds, right then left then right, so the bank comes back.
const ROLL_HOLDS: [(f32, f32); 3] = [(1.0, 1.5), (-1.0, 3.0), (1.0, 1.5)];
/// Quiet before each pulse, in s.
const QUIET_S: f32 = 4.0;
/// Each pulse, in s.
const PULSE_S: f32 = 1.0;
/// The pulses: axis (0 roll, 1 pitch, 2 yaw) and sign.
const PULSES: [(usize, f32); 6] = [
    (0, 1.0),
    (0, -1.0),
    (1, 1.0),
    (1, -1.0),
    (2, 1.0),
    (2, -1.0),
];
const AXES: [&str; 3] = ["roll", "pitch", "yaw"];

/// Plane-side measurements of the Alia in X-Plane: trim legs, a climb, and surface pulses.
#[derive(Parser)]
struct Args {
    /// Waypoint CSV to fly (columns: x,y,altitude,cruise_speed,regime).
    #[arg(default_value = "flights/alia_plane_probe.csv")]
    script: String,
    /// X-Plane's UDP address (`host:port`).
    #[arg(long, default_value = "127.0.0.1:49000")]
    host: String,
    /// Each pulse's cruise moment (normalized, as the plane stabilizer's output).
    #[arg(long, default_value_t = 0.15)]
    pulse: f32,
    /// The roll holds' aileron command, in place of the roll loop's (normalized).
    #[arg(long, default_value_t = 0.5)]
    roll_hold: f32,
    /// Write one CSV row per tick to this file.
    #[arg(long)]
    record: Option<String>,
}

fn main() {
    Builder::from_default_env().format_timestamp_millis().init();
    let args = Args::parse();

    let commander = ScriptCommander::new(&args.script)
        .expect("failed to load flight script")
        .with_route_end(RouteEnd::Land);

    let airframe = airframes::alia250();
    let bridge = XPlaneUdpBridge::connect(&args.host).expect("failed to open the socket");
    let vehicle = XPlaneVehicle::connect(XPlaneConfiguration::new(bridge, airframe.mapping))
        .expect("X-Plane not ready");

    let injection = Rc::new(Cell::new(Vec3::zeros()));
    let roll_hold = Rc::new(Cell::new(None));
    let probe = Rc::new(RefCell::new(Probe::new(
        (args.pulse, args.roll_hold),
        injection.clone(),
        roll_hold.clone(),
    )));
    let mixer = ProbeMixer {
        inner: airframe.mixer,
        injection,
        roll_hold,
    };
    let recorder = CsvRecorder::create(args.record.as_deref(), QuadPlaneColumns)
        .expect("failed to create the record CSV");

    let mut flight_loop = FlightLoop::new(
        commander,
        vehicle,
        QuadPlaneGuidance::new(airframe.guidance),
        QuadPlaneStabilizer::new(airframe.stabilizer),
        mixer,
        (recorder, ProbeLog(probe.clone())),
    );

    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");
    let runner = RealTimeRunner::new(250.0).expect("valid loop frequency"); // Hz
    runner.run_with(&mut flight_loop, &shutdown, |flight_loop, _| {
        if flight_loop.commander().follower().is_complete() && !flight_loop.armed() {
            info!("touched down: disarmed");
            shutdown.trigger();
        }
    });
    let legs = probe.borrow().legs_probed;
    if legs == 0 {
        warn!("no leg was probed");
    }
    info!("legs probed: {legs}");
}

/// The airframe's mixer with a probe's cruise moment added, and its roll moment replaced
/// during a roll hold.
struct ProbeMixer<M> {
    inner: M,
    injection: Rc<Cell<Vec3>>,
    roll_hold: Rc<Cell<Option<f32>>>,
}

impl<M: Mixer> Mixer for ProbeMixer<M> {
    fn mix(&mut self, controls: &ControlOutput) -> ActuatorCommand {
        let mut controls = *controls;
        if let Some(cruise) = controls.cruise.as_mut() {
            cruise.moments += self.injection.get();
            if let Some(roll) = self.roll_hold.get() {
                cruise.moments.x = roll;
            }
        }
        self.inner.mix(&controls)
    }

    fn saturation(&self) -> Saturation {
        self.inner.saturation()
    }
}

/// Where a leg's pulse schedule is.
#[derive(Clone, Copy, PartialEq)]
enum Phase {
    /// Waiting for a steady leg; time it has held.
    Settling(f32),
    /// Quiet before pulse `n`; time in it.
    Quiet(usize, f32),
    /// Pulse `n`; time in it.
    Pulse(usize, f32),
    /// Quiet before the roll holds; time in it.
    HoldQuiet(f32),
    /// Roll hold `n`; time in it.
    Hold(usize, f32),
    /// This leg is done.
    Done,
}

/// Runs each plane leg's pulse schedule.
struct Probe {
    pulse: f32,
    roll_hold: f32,
    injection: Rc<Cell<Vec3>>,
    roll_override: Rc<Cell<Option<f32>>>,
    phase: Phase,
    /// The current leg's waypoint, to tell legs apart.
    leg: Option<[f32; 3]>,
    elapsed_s: f32,
    started: bool,
    legs_probed: usize,
}

impl Probe {
    fn new(
        (pulse, roll_hold): (f32, f32),
        injection: Rc<Cell<Vec3>>,
        roll_override: Rc<Cell<Option<f32>>>,
    ) -> Self {
        Probe {
            pulse,
            roll_hold,
            injection,
            roll_override,
            phase: Phase::Settling(0.0),
            leg: None,
            elapsed_s: 0.0,
            started: false,
            legs_probed: 0,
        }
    }

    fn update(&mut self, record: &TelemetryRecord) {
        if !self.started {
            self.started = true;
            let unix_s = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map_or(f64::NAN, |d| d.as_secs_f64());
            info!("recording starts at unix {unix_s:.3}");
        }
        let dt = record.dt.as_secs_f32();
        self.elapsed_s += dt;
        let ControlInput::Waypoint(waypoint) = record.command.input else {
            return;
        };
        let leg = [
            waypoint.position_m.x,
            waypoint.position_m.y,
            waypoint.altitude_m,
        ];
        if self.leg != Some(leg) {
            self.leg = Some(leg);
            self.quiet();
            self.phase = Phase::Settling(0.0);
        }
        if record.flight_mode.mode != FlightMode::Plane {
            self.quiet();
            self.phase = Phase::Settling(0.0);
            return;
        }
        let state = &record.state;
        let airspeed = state.valid_airspeed_mps().unwrap_or(0.0);
        let steady = (airspeed - waypoint.cruise_speed_mps).abs() <= SPEED_BAND_MPS
            && state.roll_rad.abs() <= BANK_BAND_RAD;
        self.phase = match self.phase {
            Phase::Settling(held) if steady => {
                if held + dt >= SETTLE_S {
                    info!(
                        "t {:.1} s: leg at {:.0} m/s steady ({airspeed:.1} m/s, {:.0} m): pulsing",
                        self.elapsed_s, waypoint.cruise_speed_mps, state.position_m.z
                    );
                    Phase::Quiet(0, 0.0)
                } else {
                    Phase::Settling(held + dt)
                }
            }
            Phase::Settling(_) => Phase::Settling(0.0),
            Phase::Quiet(n, t) if t + dt >= QUIET_S => {
                let (axis, sign) = PULSES[n];
                let mut injection = Vec3::zeros();
                injection[axis] = sign * self.pulse;
                self.injection.set(injection);
                info!(
                    "t {:.3} s: pulse {} {:+.2} at {airspeed:.1} m/s",
                    self.elapsed_s,
                    AXES[axis],
                    sign * self.pulse
                );
                Phase::Pulse(n, 0.0)
            }
            Phase::Quiet(n, t) => Phase::Quiet(n, t + dt),
            Phase::Pulse(n, t) if t + dt >= PULSE_S => {
                self.injection.set(Vec3::zeros());
                if n + 1 < PULSES.len() {
                    Phase::Quiet(n + 1, 0.0)
                } else {
                    Phase::HoldQuiet(0.0)
                }
            }
            Phase::Pulse(n, t) => Phase::Pulse(n, t + dt),
            Phase::HoldQuiet(t) if t + dt >= QUIET_S => self.start_hold(0, airspeed),
            Phase::HoldQuiet(t) => Phase::HoldQuiet(t + dt),
            Phase::Hold(n, t) if t + dt >= ROLL_HOLDS[n].1 => {
                if n + 1 < ROLL_HOLDS.len() {
                    self.start_hold(n + 1, airspeed)
                } else {
                    self.quiet();
                    self.legs_probed += 1;
                    info!("t {:.1} s: leg probed", self.elapsed_s);
                    Phase::Done
                }
            }
            Phase::Hold(n, t) => Phase::Hold(n, t + dt),
            Phase::Done => Phase::Done,
        };
    }
}

impl Probe {
    /// No injection, no hold.
    fn quiet(&self) {
        self.injection.set(Vec3::zeros());
        self.roll_override.set(None);
    }

    /// Starts roll hold `n`.
    fn start_hold(&mut self, n: usize, airspeed: f32) -> Phase {
        let aileron = ROLL_HOLDS[n].0 * self.roll_hold;
        self.roll_override.set(Some(aileron));
        info!(
            "t {:.3} s: roll hold {aileron:+.2} for {:.1} s at {airspeed:.1} m/s",
            self.elapsed_s, ROLL_HOLDS[n].1
        );
        Phase::Hold(n, 0.0)
    }
}

/// Feeds each tick to the probe.
struct ProbeLog(Rc<RefCell<Probe>>);

impl TelemetryPublisher for ProbeLog {
    fn publish(&mut self, record: &TelemetryRecord) {
        self.0.borrow_mut().update(record);
    }
}
