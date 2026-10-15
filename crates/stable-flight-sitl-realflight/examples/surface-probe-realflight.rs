//! Checks which way a quadplane's surfaces push, in flight, on [`airframes::titan_cobra`]:
//! for finding reversed servos without risking a plane flight on them.
//!
//! It flies a waypoint script in copter flight with forward thrust on (default
//! `flights/cobra_probe.csv`: climb over the spawn point, a long leg ahead with no speed cap,
//! land at its end), so the pusher builds airspeed while the lift rotors hold attitude. Once the
//! airspeed has held `--min-airspeed` for a second, it adds a cruise moment of `--pulse` for
//! 0.5 s on one axis at a time, each sign in turn, with a second of quiet before each: roll
//! (ailerons), pitch, and yaw (the tail). The hover stabilizer holds attitude against each
//! pulse, so its lift-rotor moment on that axis shifts the other way when the surfaces push
//! as the mixer expects, and the same way when the servo is reversed. Each pulse's shift
//! (the last 0.3 s of the pulse against the last 0.5 s of quiet) is logged with a verdict.
//!
//! It lands on the script's last waypoint and exits once disarmed; Ctrl-C stops it sooner
//! (`RealFlightVehicle` hands RC back). Don't end a script with a fast leg flown tail-first:
//! the Cobra pitches up past its lift rotors' authority (#437). The RealFlight host defaults
//! to `10.211.55.3:8080` via `--host`.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example surface-probe-realflight -- --record probe.csv
//! ```

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::Duration;

use clap::Parser;
use env_logger::Builder;
use log::{info, warn};

use stable_flight::commanders::RouteEnd;
use stable_flight::guidance::vtol::QuadPlaneGuidance;
use stable_flight::stabilizers::vtol::QuadPlaneStabilizer;
use stable_flight::tuning::Stat;
use stable_flight::{
    ActuatorCommand, ControlOutput, FlightLoop, Mixer, Saturation, TelemetryPublisher,
    TelemetryRecord, Vec3,
};
use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::recorder::{CsvRecorder, QuadPlaneColumns};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::{BridgeError, RealFlightRemoteBridge};
use stable_flight_sitl_realflight::{
    REMOTE_IO_TIMEOUT, RealFlightConfiguration, RealFlightVehicle, airframes,
};

/// Connect timeout for the RealFlight proxy; a reconnect attempt blocks its tick.
const CONNECT_TIMEOUT: Duration = Duration::from_millis(250);
/// Forward thrust gain for the leg: the sim quadplane's (#427).
const FORWARD_THRUST_GAIN: f32 = 1.2;
/// How long the airspeed holds before the first pulse, in s.
const SETTLE_S: f32 = 1.0;
/// Quiet before each pulse, in s; its last [`BASELINE_S`] is the baseline.
const QUIET_S: f32 = 1.0;
const BASELINE_S: f32 = 0.5;
/// Each pulse, in s; its last [`RESPONSE_S`] is the response.
const PULSE_S: f32 = 0.5;
const RESPONSE_S: f32 = 0.3;
/// A shift smaller than this (normalized moment) is no response.
const MIN_SHIFT: f32 = 0.01;

/// The pulses: axis (0 roll, 1 pitch, 2 yaw) and sign.
const PULSES: [(usize, f32); 6] = [
    (0, 1.0),
    (0, -1.0),
    (1, 1.0),
    (1, -1.0),
    (2, 1.0),
    (2, -1.0),
];
const AXES: [&str; 3] = ["roll (ailerons)", "pitch (tail)", "yaw (tail)"];

/// In-flight check of a quadplane's surface directions in RealFlight.
#[derive(Parser)]
struct Args {
    /// Waypoint CSV to fly (columns: x,y,altitude,cruise_speed).
    #[arg(default_value = "flights/cobra_probe.csv")]
    script: String,
    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,
    /// Cruise moment of each pulse, normalized.
    #[arg(long, default_value_t = 0.3)]
    pulse: f32,
    /// Airspeed the pulses wait for, in m/s.
    #[arg(long, default_value_t = 10.0)]
    min_airspeed: f32,
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

    let mut airframe = airframes::titan_cobra();
    airframe.guidance.forward.gain = FORWARD_THRUST_GAIN;

    let host = args.host.clone();
    let connect =
        move || RealFlightRemoteBridge::with_timeouts(&host, CONNECT_TIMEOUT, REMOTE_IO_TIMEOUT);
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration::from_airframe(
        &airframe,
        connect().expect("Failed to create bridge"),
    ))
    .with_reconnect(move || connect().map_err(BridgeError::from));

    let injection = Rc::new(Cell::new(Vec3::zeros()));
    let probe = Rc::new(RefCell::new(Probe::new(
        args.pulse,
        args.min_airspeed,
        injection.clone(),
    )));
    let mixer = ProbeMixer {
        inner: airframe.mixer,
        injection,
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
    let runner = RealTimeRunner::new(250.0).expect("valid loop frequency");
    info!(
        "climbing; pulses of {} once airspeed holds {} m/s",
        args.pulse, args.min_airspeed
    );
    runner.run_with(&mut flight_loop, &shutdown, |flight_loop, _| {
        if flight_loop.commander().follower().is_complete() && !flight_loop.armed() {
            info!("touched down: disarmed");
            shutdown.trigger();
        }
    });
    if !probe.borrow().is_done() {
        warn!("stopped before every pulse ran");
    }
}

/// The airframe's mixer with a probe's cruise moment added.
struct ProbeMixer<M> {
    inner: M,
    injection: Rc<Cell<Vec3>>,
}

impl<M: Mixer> Mixer for ProbeMixer<M> {
    fn mix(&mut self, controls: &ControlOutput) -> ActuatorCommand {
        let mut controls = *controls;
        if let Some(cruise) = controls.cruise.as_mut() {
            cruise.moments += self.injection.get();
        }
        self.inner.mix(&controls)
    }

    fn saturation(&self) -> Saturation {
        self.inner.saturation()
    }
}

/// Where the pulse schedule is.
#[derive(Clone, Copy, PartialEq)]
enum Phase {
    /// Waiting for the airspeed; time it has held.
    Waiting(f32),
    /// Quiet before pulse `n`; time in it.
    Quiet(usize, f32),
    /// Pulse `n`; time in it.
    Pulse(usize, f32),
    Done,
}

/// Runs the pulse schedule each tick and judges each pulse from the lift rotors' moments.
struct Probe {
    pulse: f32,
    min_airspeed_mps: f32,
    injection: Rc<Cell<Vec3>>,
    phase: Phase,
    baseline: Stat,
    response: Stat,
    airspeed: Stat,
}

impl Probe {
    fn new(pulse: f32, min_airspeed_mps: f32, injection: Rc<Cell<Vec3>>) -> Self {
        Probe {
            pulse,
            min_airspeed_mps,
            injection,
            phase: Phase::Waiting(0.0),
            baseline: Stat::default(),
            response: Stat::default(),
            airspeed: Stat::default(),
        }
    }

    fn is_done(&self) -> bool {
        self.phase == Phase::Done
    }

    fn update(&mut self, record: &TelemetryRecord) {
        let dt = record.dt.as_secs_f32();
        let airspeed = record.state.valid_airspeed_mps().unwrap_or(0.0);
        let moments = record.control.moments;
        self.phase = match self.phase {
            Phase::Waiting(held) if airspeed >= self.min_airspeed_mps => {
                if held + dt >= SETTLE_S {
                    info!("airspeed {airspeed:.1} m/s: pulsing");
                    self.start_quiet(0)
                } else {
                    Phase::Waiting(held + dt)
                }
            }
            Phase::Waiting(_) => Phase::Waiting(0.0),
            Phase::Quiet(n, t) => {
                let (axis, _) = PULSES[n];
                if t >= QUIET_S - BASELINE_S {
                    self.baseline.add(moments[axis]);
                }
                if t + dt >= QUIET_S {
                    let (axis, sign) = PULSES[n];
                    let mut injection = Vec3::zeros();
                    injection[axis] = sign * self.pulse;
                    self.injection.set(injection);
                    Phase::Pulse(n, 0.0)
                } else {
                    Phase::Quiet(n, t + dt)
                }
            }
            Phase::Pulse(n, t) => {
                let (axis, _) = PULSES[n];
                if t >= PULSE_S - RESPONSE_S {
                    self.response.add(moments[axis]);
                    self.airspeed.add(airspeed);
                }
                if t + dt >= PULSE_S {
                    self.injection.set(Vec3::zeros());
                    self.judge(n);
                    if n + 1 < PULSES.len() {
                        self.start_quiet(n + 1)
                    } else {
                        info!("probe done");
                        Phase::Done
                    }
                } else {
                    Phase::Pulse(n, t + dt)
                }
            }
            Phase::Done => Phase::Done,
        };
    }

    fn start_quiet(&mut self, n: usize) -> Phase {
        self.baseline = Stat::default();
        self.response = Stat::default();
        self.airspeed = Stat::default();
        Phase::Quiet(n, 0.0)
    }

    /// Logs pulse `n`'s lift-moment shift and what it says about the surfaces.
    fn judge(&self, n: usize) {
        let (axis, sign) = PULSES[n];
        let mean = |stat: &Stat| stat.mean().unwrap_or(f32::NAN);
        let shift = mean(&self.response) - mean(&self.baseline);
        let verdict = if shift.abs() < MIN_SHIFT {
            "no response"
        } else if shift * sign < 0.0 {
            "pushes as the mixer expects"
        } else {
            "REVERSED"
        };
        info!(
            "{} {:+.2}: lift moment shift {:+.3} at {:.1} m/s: {verdict}",
            AXES[axis],
            sign * self.pulse,
            shift,
            mean(&self.airspeed),
        );
    }
}

/// Feeds each tick to the probe.
struct ProbeLog(Rc<RefCell<Probe>>);

impl TelemetryPublisher for ProbeLog {
    fn publish(&mut self, record: &TelemetryRecord) {
        self.0.borrow_mut().update(record);
    }
}
