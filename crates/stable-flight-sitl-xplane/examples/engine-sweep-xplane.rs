//! Sweeps a multirotor's lift engines in X-Plane from the ground and records every sample: the
//! yaw sign of each diagonal pair, the throttle at liftoff, the motor lag, and the response to
//! a roll, pitch, and yaw moment, for a new airframe's mixer and model, with no flight loop
//! around it.
//!
//! Put the aircraft on a runway with the parking brake off (the yaw pulses turn it on its
//! wheels). The sweep takes the throttles (`override_throttles`), holds engine 4 and up at 0,
//! and runs [`EngineSweep`]'s schedule on engines 0–3: a rest, each diagonal pair (0+2, then
//! 1+3) alone at `--yaw-throttle`, then all four ramping up until the aircraft leaves the
//! ground, a step above the liftoff throttle, a short roll, pitch, and yaw pulse on it (about
//! 1.4 s airborne in all), and the fall. It logs each yaw pulse's mean yaw
//! rate and heading change and the liftoff throttle, and writes the raw samples (`--output`)
//! with each tick's throttles and phase (`throttle0`–`throttle3`, `phase`). Ctrl-C stops early;
//! either way it zeroes the engines and hands the throttles back.
//!
//! ```text
//! cargo run -p stable-flight-sitl-xplane --example engine-sweep-xplane -- --output sweep.csv
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
use stable_flight_sitl_xplane::capture::{write_header, write_row};
use stable_flight_sitl_xplane::sweep::{EngineSweep, LIFT_ENGINES, Phase, SweepConfig};
use stable_flight_sitl_xplane::xplane_bridge::datarefs::{engine, overrides};
use stable_flight_sitl_xplane::xplane_bridge::{
    DatarefSink, DatarefSource, Latest, XPlaneUdpBridge,
};
use stable_flight_sitl_xplane::{DEFAULT_READY_TIMEOUT, StateDatarefs, XPlaneSample, wait_ready};

/// Tick length: the flight loop's 250 Hz.
const TICK: Duration = Duration::from_millis(4);
/// Engines written each tick: the lift engines and the pusher (engine 4), held at 0.
const ENGINES: usize = 5;

/// Lift-engine sweep of a multirotor on the ground in X-Plane.
#[derive(Parser)]
struct Args {
    /// X-Plane's UDP address (`host:port`).
    #[arg(long, default_value = "127.0.0.1:49000")]
    host: String,
    /// CSV file to write.
    #[arg(long, default_value = "xplane-sweep.csv")]
    output: PathBuf,
    /// Throttle of each yaw pulse, below liftoff.
    #[arg(long, default_value_t = SweepConfig::default().yaw_throttle)]
    yaw_throttle: f32,
    /// Throttle added at liftoff for the motor-lag step.
    #[arg(long, default_value_t = SweepConfig::default().step_throttle)]
    step_throttle: f32,
    /// Length of the step, s.
    #[arg(long, default_value_t = SweepConfig::default().step_s)]
    step_s: f32,
}

/// Mean yaw rate and heading change over one yaw pulse.
#[derive(Default)]
struct Pulse {
    yaw_rate_sum_degps: f32,
    samples: usize,
    first_heading_deg: Option<f32>,
    last_heading_deg: f32,
}

impl Pulse {
    fn add(&mut self, sample: &XPlaneSample) {
        let [w, x, y, z] = sample.quaternion;
        let heading_deg = (2.0 * (w * z + x * y))
            .atan2(1.0 - 2.0 * (y * y + z * z))
            .to_degrees();
        self.first_heading_deg.get_or_insert(heading_deg);
        self.last_heading_deg = heading_deg;
        self.yaw_rate_sum_degps += sample.body_rates_degps[2];
        self.samples += 1;
    }

    fn report(&self, pair: [usize; 2]) {
        if self.samples == 0 {
            return;
        }
        let turned = self.last_heading_deg - self.first_heading_deg.unwrap_or(0.0);
        let turned = (turned + 540.0).rem_euclid(360.0) - 180.0;
        info!(
            "Engines {}+{}: mean yaw rate {:+.2} °/s, heading {:+.1}° (positive: nose right)",
            pair[0],
            pair[1],
            self.yaw_rate_sum_degps / self.samples as f32,
            turned
        );
    }
}

fn main() {
    Builder::from_default_env().format_timestamp_millis().init();
    let args = Args::parse();
    let config = SweepConfig {
        yaw_throttle: args.yaw_throttle,
        step_throttle: args.step_throttle,
        step_s: args.step_s,
        ..SweepConfig::default()
    };

    let mut bridge = XPlaneUdpBridge::connect(&args.host).expect("failed to open the socket");
    let datarefs = StateDatarefs::subscribe(&mut bridge).expect("failed to subscribe");
    let mut latest = Latest::default();
    let mut sample = wait_ready(&mut bridge, &datarefs, &mut latest, DEFAULT_READY_TIMEOUT)
        .expect("X-Plane not ready");
    let engines: Vec<_> = engine::THRO_USE.iter().take(ENGINES).collect();

    let mut out = BufWriter::new(File::create(&args.output).expect("failed to create the CSV"));
    write_header(
        &mut out,
        &["throttle0", "throttle1", "throttle2", "throttle3", "phase"],
    )
    .expect("failed to write the header");

    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");
    bridge
        .write(&overrides::THROTTLES, true)
        .expect("failed to take the throttles");
    info!("Sweeping; Ctrl-C to stop");

    let mut sweep = EngineSweep::new(config);
    let mut pulses: [Pulse; 2] = Default::default();
    let mut throttles = [0.0; LIFT_ENGINES];
    let mut last = Instant::now();
    while !shutdown.is_set() && sweep.phase() != Phase::Done {
        let now = Instant::now();
        let dt_s = now.duration_since(last).as_secs_f32();
        last = now;

        match bridge.poll() {
            Ok(Some(frame)) => {
                latest.apply(&frame);
                if let Some(new) = datarefs.sample(&latest) {
                    sample = new;
                    let mut extra = throttles.to_vec();
                    extra.push(sweep.phase().code());
                    write_row(&mut out, &sample, &extra).expect("failed to write a row");
                    if let Phase::YawPulse(n) = sweep.phase() {
                        pulses[n].add(&sample);
                    }
                }
            }
            Ok(None) => {}
            Err(e) => warn!("X-Plane receive failed: {e}"),
        }

        let before = sweep.phase();
        throttles = sweep.step(dt_s, sample.on_ground);
        if sweep.phase() != before {
            info!("{:?}", sweep.phase());
        }
        for (i, dataref) in engines.iter().enumerate() {
            let throttle = throttles.get(i).copied().unwrap_or(0.0);
            if let Err(e) = bridge.write(dataref, throttle) {
                warn!("X-Plane write failed: {e}");
            }
        }
        thread::sleep(TICK.saturating_sub(now.elapsed()));
    }

    for dataref in &engines {
        bridge.write(dataref, 0.0).ok();
    }
    bridge.write(&overrides::THROTTLES, false).ok();
    out.flush().expect("failed to flush the CSV");

    for (pulse, pair) in pulses.iter().zip(config.yaw_pairs) {
        pulse.report(pair);
    }
    match sweep.liftoff_throttle() {
        Some(throttle) => info!("Liftoff at throttle {throttle:.3}"),
        None => info!("No liftoff"),
    }
    info!("{:?}", bridge.stats());
}
