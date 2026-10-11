//! Sweeps a RealFlight model's channels one by one and records every exchange (#120): for
//! mapping a new model's channels and checking the frame contract on it (`check_frames` in
//! `stable-flight-tools`), with no flight loop around it.
//!
//! Set the model in RealFlight so each actuator has its own channel (no model-side mixes or
//! gyro), put it on the ground, and name its motor and servo channels. The sweep holds every
//! channel at neutral (motors 0, everything else centered at 0.5), then:
//!
//! 1. rests 2 s (a static frame check);
//! 2. moves each servo to full one way, the other way, and back, 1 s each;
//! 3. pulses each motor alone at `--pulse-throttle` for 1 s;
//! 4. hops: every motor at `--hop-throttle` for 1 s, the first servo at full for its last half,
//!    then everything neutral for 3 s while the model falls (the dynamic frame check).
//!
//! `--script` replaces the sweep with a CSV of `t_s,channel,value` steps (each channel holds its
//! last value). The capture (`--output`) has the raw `SimulatorState` and the 12 channels per
//! exchange, as `capture-state-realflight` writes. Ctrl-C stops early and hands RC back.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example channel-sweep-realflight -- --motors 0,1 --servos 2,3
//! cargo run -p stable-flight-sitl-realflight --example channel-sweep-realflight -- --script flights/my-sweep.csv
//! ```

use std::fs::{self, File};
use std::io::BufWriter;
use std::path::PathBuf;
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use env_logger::Builder;
use log::{info, warn};

use stable_flight_sitl::ShutdownSignal;
use stable_flight_sitl_realflight::REMOTE_IO_TIMEOUT;
use stable_flight_sitl_realflight::capture::RecordingBridge;
use stable_flight_sitl_realflight::realflight_bridge::{
    ControlInputs, RealFlightBridge, RealFlightRemoteBridge,
};

/// Connect timeout for the RealFlight proxy.
const CONNECT_TIMEOUT: Duration = Duration::from_millis(250);
/// RealFlight's channel count.
const CHANNELS: usize = 12;
/// A centered channel.
const CENTER: f32 = 0.5;

/// Sweep a RealFlight model's channels and record the raw state.
#[derive(Parser)]
struct Args {
    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,
    /// CSV file to write.
    #[arg(long, default_value = "realflight-sweep.csv")]
    output: PathBuf,
    /// Motor channels (0–11), e.g. `0,1`: held at 0, pulsed, and used for the hop.
    #[arg(long, value_delimiter = ',')]
    motors: Vec<usize>,
    /// Servo channels (0–11), e.g. `2,3`: swept full both ways.
    #[arg(long, value_delimiter = ',')]
    servos: Vec<usize>,
    /// Throttle for each motor's solo pulse, 0–1.
    #[arg(long, default_value_t = 0.3)]
    pulse_throttle: f32,
    /// Throttle for the hop, 0–1.
    #[arg(long, default_value_t = 0.8)]
    hop_throttle: f32,
    /// Steps to run instead of the sweep: a CSV of `t_s,channel,value` rows.
    #[arg(long)]
    script: Option<PathBuf>,
    /// Exchanges per second.
    #[arg(long, default_value_t = 250.0)]
    rate_hz: f64,
}

/// At `t_s` seconds, set `channel` to `value` (it holds until the next step for it).
#[derive(Clone, Copy, Debug)]
struct Step {
    t_s: f32,
    channel: usize,
    value: f32,
}

/// Every channel at neutral: motors off, the rest centered.
fn neutral(motors: &[usize]) -> [f32; CHANNELS] {
    let mut channels = [CENTER; CHANNELS];
    for &motor in motors {
        channels[motor] = 0.0;
    }
    channels
}

/// The default sweep (see the module docs), as steps from neutral.
fn sweep(args: &Args) -> Vec<Step> {
    let mut steps = Vec::new();
    let mut t_s = 2.0; // rest
    let mut at = |t_s: f32, channel: usize, value: f32| {
        steps.push(Step {
            t_s,
            channel,
            value,
        });
    };
    for &servo in &args.servos {
        for value in [1.0, 0.0, CENTER] {
            at(t_s, servo, value);
            t_s += 1.0;
        }
    }
    for &motor in &args.motors {
        at(t_s, motor, args.pulse_throttle);
        at(t_s + 1.0, motor, 0.0);
        t_s += 2.0;
    }
    for &motor in &args.motors {
        at(t_s, motor, args.hop_throttle);
        at(t_s + 1.0, motor, 0.0);
    }
    if let Some(&servo) = args.servos.first() {
        at(t_s + 0.5, servo, 1.0);
        at(t_s + 1.0, servo, CENTER);
    }
    // The fall.
    let end = t_s + 4.0;
    if let Some(&motor) = args.motors.first() {
        at(end, motor, 0.0);
    }
    steps
}

/// Reads `t_s,channel,value` rows (a header line is skipped).
fn read_script(path: &PathBuf) -> Vec<Step> {
    let text = fs::read_to_string(path).expect("failed to read the script");
    text.lines()
        .filter(|line| !line.trim().is_empty())
        .filter_map(|line| {
            let mut fields = line.split(',').map(str::trim);
            let t_s = fields.next()?.parse().ok()?;
            let channel = fields.next()?.parse().ok()?;
            let value = fields.next()?.parse().ok()?;
            Some(Step {
                t_s,
                channel,
                value,
            })
        })
        .collect()
}

fn main() {
    Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp_millis()
        .init();

    let args = Args::parse();
    let channels_ok = |list: &[usize]| list.iter().all(|&c| c < CHANNELS);
    assert!(
        channels_ok(&args.motors) && channels_ok(&args.servos),
        "channels run 0–{}",
        CHANNELS - 1
    );
    let mut steps = match &args.script {
        Some(path) => read_script(path),
        None => sweep(&args),
    };
    assert!(
        !steps.is_empty(),
        "nothing to sweep: name --motors/--servos or a --script"
    );
    assert!(
        steps.iter().all(|step| step.channel < CHANNELS),
        "a step names a channel past {}",
        CHANNELS - 1
    );
    steps.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
    let end_s = steps.last().map_or(0.0, |step| step.t_s);

    let out = BufWriter::new(File::create(&args.output).expect("failed to create the output"));
    let bridge = RecordingBridge::new(
        RealFlightRemoteBridge::with_timeouts(&args.host, CONNECT_TIMEOUT, REMOTE_IO_TIMEOUT)
            .expect("failed to connect to RealFlight"),
        out,
    )
    .expect("failed to write the header");
    bridge
        .disable_rc()
        .expect("failed to take control (disable RC)");
    if let Err(e) = bridge.reset_aircraft() {
        warn!("failed to reset the aircraft: {e}");
    }

    let shutdown = ShutdownSignal::install().expect("failed to install the Ctrl-C handler");
    let period = Duration::from_secs_f64(1.0 / args.rate_hz);
    let mut control = ControlInputs {
        channels: neutral(&args.motors),
    };
    info!(
        "sweeping {} steps over {end_s:.1} s to {}; Ctrl-C to stop",
        steps.len(),
        args.output.display()
    );
    let start = Instant::now();
    let mut next = 0;
    while !shutdown.is_set() {
        let t_s = start.elapsed().as_secs_f32();
        while let Some(step) = steps.get(next).filter(|step| step.t_s <= t_s) {
            info!(
                "{:.2} s: channel {} → {:.2}",
                step.t_s, step.channel, step.value
            );
            control.channels[step.channel] = step.value;
            next += 1;
        }
        if let Err(e) = bridge.exchange_data(&control) {
            warn!("exchange failed, stopping: {e}");
            break;
        }
        if next >= steps.len() && t_s >= end_s {
            break;
        }
        let deadline = start + period.mul_f64((t_s as f64 / period.as_secs_f64()).floor() + 1.0);
        thread::sleep(deadline.saturating_duration_since(Instant::now()));
    }

    control.channels = neutral(&args.motors);
    if let Err(e) = bridge.exchange_data(&control) {
        warn!("failed to send neutral: {e}");
    }
    if let Err(e) = bridge.enable_rc() {
        warn!("failed to hand RC back: {e}");
    }
    info!("done");
}
