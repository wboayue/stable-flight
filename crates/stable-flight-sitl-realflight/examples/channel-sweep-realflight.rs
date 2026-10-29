//! Sweeps a RealFlight model's channels one by one and records every exchange: for
//! mapping a new model's channels and checking the frame contract on it (`check_frames` in
//! `stable-flight-tools`), with no flight loop around it.
//!
//! Set the model in RealFlight so each actuator has its own channel (no model-side mixes or
//! gyro), put it on the ground, and name its motor and servo channels. The sweep holds every
//! channel at neutral (motors 0, everything else centered at 0.5; `--neutral` overrides any,
//! e.g. a throttle or switch channel left off `--motors`), then:
//!
//! 1. rests 2 s (a static frame check);
//! 2. moves each servo to full one way, the other way, and back, 1 s each;
//! 3. pulses each motor alone at `--pulse-throttle` for 1 s;
//! 4. hops: every motor at `--hop-throttle` for 1 s, the first servo at full for its last half,
//!    then everything neutral while the model falls (the dynamic frame check).
//!
//! `--script` replaces the sweep with a CSV of `t_s,channel,value` steps (each channel holds its
//! last value; an optional header line is skipped). Recording goes on for `--tail-s` after the
//! last step, so its response is captured. The capture (`--output`) has the raw `SimulatorState` and the 12 channels per
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
use log::{error, info, warn};

use stable_flight_sitl::ShutdownSignal;
use stable_flight_sitl_realflight::capture::RecordingBridge;
use stable_flight_sitl_realflight::realflight_bridge::{
    ControlInputs, RealFlightBridge, RealFlightRemoteBridge,
};
use stable_flight_sitl_realflight::{CHANNEL_COUNT, REMOTE_IO_TIMEOUT};

/// Connect timeout for the RealFlight proxy.
const CONNECT_TIMEOUT: Duration = Duration::from_millis(250);
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
    /// Neutral values that override the defaults, as `channel:value` pairs, e.g. `4:0,5:1`.
    #[arg(long, value_delimiter = ',', value_parser = parse_neutral)]
    neutral: Vec<(usize, f32)>,
    /// How long to keep recording after the last step, in s.
    #[arg(long, default_value_t = 3.0)]
    tail_s: f32,
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

/// `--neutral`'s `channel:value`.
fn parse_neutral(text: &str) -> Result<(usize, f32), String> {
    let (channel, value) = text
        .split_once(':')
        .ok_or_else(|| format!("`{text}`: expected channel:value"))?;
    let channel: usize = channel
        .trim()
        .parse()
        .map_err(|e| format!("`{text}`: {e}"))?;
    let value: f32 = value.trim().parse().map_err(|e| format!("`{text}`: {e}"))?;
    if channel >= CHANNEL_COUNT || !(0.0..=1.0).contains(&value) {
        return Err(format!(
            "`{text}`: channel 0–{} and value 0–1",
            CHANNEL_COUNT - 1
        ));
    }
    Ok((channel, value))
}

/// Every channel at neutral: motors off, the rest centered, then `overrides`.
fn neutral(motors: &[usize], overrides: &[(usize, f32)]) -> [f32; CHANNEL_COUNT] {
    let mut channels = [CENTER; CHANNEL_COUNT];
    for &motor in motors {
        channels[motor] = 0.0;
    }
    for &(channel, value) in overrides {
        channels[channel] = value;
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
    steps
}

/// Reads `t_s,channel,value` rows; a first line that doesn't start with a number is a header.
/// Panics on any other row it can't read, naming the line.
fn read_script(path: &PathBuf) -> Vec<Step> {
    let text = fs::read_to_string(path).expect("failed to read the script");
    let mut steps = Vec::new();
    for (index, line) in text.lines().enumerate() {
        let fields: Vec<&str> = line.split(',').map(str::trim).collect();
        if line.trim().is_empty() || (index == 0 && fields[0].parse::<f32>().is_err()) {
            continue;
        }
        let parsed = match fields.as_slice() {
            [t_s, channel, value] => t_s
                .parse()
                .ok()
                .zip(channel.parse().ok())
                .zip(value.parse().ok()),
            _ => None,
        };
        let Some(((t_s, channel), value)) = parsed else {
            panic!(
                "script line {}: expected t_s,channel,value: `{line}`",
                index + 1
            );
        };
        steps.push(Step {
            t_s,
            channel,
            value,
        });
    }
    steps
}

fn main() {
    Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp_millis()
        .init();

    let args = Args::parse();
    let channels_ok = |list: &[usize]| list.iter().all(|&c| c < CHANNEL_COUNT);
    assert!(
        channels_ok(&args.motors) && channels_ok(&args.servos),
        "channels run 0–{}",
        CHANNEL_COUNT - 1
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
        steps.iter().all(|step| step.channel < CHANNEL_COUNT),
        "a step names a channel past {}",
        CHANNEL_COUNT - 1
    );
    steps.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
    let end_s = steps.last().map_or(0.0, |step| step.t_s) + args.tail_s;

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
    // A sweep from wherever the last run left the aircraft (upside down after a crash) measures
    // nothing.
    if let Err(e) = bridge.reset_aircraft() {
        error!("failed to reset the aircraft: {e}");
        if let Err(e) = bridge.enable_rc() {
            warn!("failed to hand RC back: {e}");
        }
        std::process::exit(1);
    }

    let shutdown = ShutdownSignal::install().expect("failed to install the Ctrl-C handler");
    let period = Duration::from_secs_f64(1.0 / args.rate_hz);
    let neutral = neutral(&args.motors, &args.neutral);
    info!("neutral: {neutral:?}");
    let mut control = ControlInputs { channels: neutral };
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

    control.channels = neutral;
    if let Err(e) = bridge.exchange_data(&control) {
        warn!("failed to send neutral: {e}");
    }
    if let Err(e) = bridge.enable_rc() {
        warn!("failed to hand RC back: {e}");
    }
    info!("done");
}
