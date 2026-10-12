//! Times RealFlight exchanges with no flight loop around them, to locate periodic stalls.
//!
//! Calls `exchange_data` with neutral inputs (motors off: run it with the aircraft on the
//! ground) at a paced rate, and logs every exchange slower than `--threshold-ms` with the time
//! since the previous slow one. Ctrl-C prints the latency percentiles and hands RC back to
//! RealFlight. A connection error, such as an exchange over the bridge's I/O timeout, stops the
//! probe and reports: a remote bridge is out of sync after one, so rerun the probe.
//!
//! By default it goes through the `realflight_bridge_proxy` at `--host`, as the flight examples
//! do. `--local <simulator addr>` talks to RealFlight directly instead (run it on the RealFlight
//! machine), leaving out the proxy and the network between the two.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example exchange-latency-realflight
//! cargo run -p stable-flight-sitl-realflight --example exchange-latency-realflight -- --local 127.0.0.1:18083
//! ```

use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use env_logger::{Builder, Env};
use log::{info, warn};

use stable_flight_sitl::ShutdownSignal;
use stable_flight_sitl_realflight::realflight_bridge::{
    BridgeError, Configuration, ControlInputs, RealFlightBridge, RealFlightLocalBridge,
    RealFlightRemoteBridge,
};
use stable_flight_sitl_realflight::{REMOTE_IO_TIMEOUT, SLOW_EXCHANGE};

/// Connect timeout for the RealFlight proxy.
const CONNECT_TIMEOUT: Duration = Duration::from_millis(250);

/// RealFlight exchange latency probe.
#[derive(Parser)]
struct Args {
    /// RealFlight bridge proxy to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,

    /// Talk to RealFlight at this address directly (e.g. `127.0.0.1:18083`) instead of the proxy.
    #[arg(long)]
    local: Option<String>,

    /// Exchanges per second.
    #[arg(long, default_value_t = 250.0)]
    rate: f64,

    /// Log exchanges slower than this, in milliseconds (default: the vehicle's `SLOW_EXCHANGE`).
    #[arg(long, default_value_t = SLOW_EXCHANGE.as_secs_f64() * 1000.0)]
    threshold_ms: f64,
}

fn main() {
    // Info unless RUST_LOG says otherwise.
    Builder::from_env(Env::default().default_filter_or("info"))
        .format_timestamp_millis()
        .init();

    let args = Args::parse();
    assert!(
        args.rate > 0.0 && args.rate.is_finite(),
        "--rate must be positive"
    );
    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");

    match &args.local {
        Some(simulator_host) => {
            let configuration = Configuration {
                simulator_host: simulator_host.clone(),
                ..Default::default()
            };
            let bridge = RealFlightLocalBridge::with_configuration(&configuration)
                .expect("failed to connect to RealFlight");
            info!("Timing exchanges with RealFlight at {simulator_host} directly");
            probe(&bridge, &args, &shutdown);
        }
        None => {
            let bridge = RealFlightRemoteBridge::with_timeouts(
                &args.host,
                CONNECT_TIMEOUT,
                REMOTE_IO_TIMEOUT,
            )
            .expect("failed to connect to the proxy");
            info!("Timing exchanges through the proxy at {}", args.host);
            probe(&bridge, &args, &shutdown);
        }
    }
}

/// Exchanges at `args.rate` until shutdown, then reports.
fn probe(bridge: &impl RealFlightBridge, args: &Args, shutdown: &ShutdownSignal) {
    if let Err(e) = bridge.disable_rc() {
        warn!("Failed to disable RC: {e}");
    }

    let period = Duration::from_secs_f64(1.0 / args.rate);
    let threshold = Duration::from_secs_f64(args.threshold_ms / 1000.0);
    let inputs = ControlInputs::default();
    let mut latencies = Vec::new();
    let (mut errors, mut previous_slow) = (0_u64, None::<Instant>);
    let mut next = Instant::now();

    while !shutdown.is_set() {
        let start = Instant::now();
        let result = bridge.exchange_data(&inputs);
        let elapsed = start.elapsed();
        latencies.push(elapsed);

        match result {
            Err(e @ BridgeError::Connection(_)) => {
                errors += 1;
                warn!(
                    "Exchange {} failed after {:.1} ms, stopping: {e}",
                    latencies.len(),
                    elapsed.as_secs_f64() * 1000.0
                );
                break;
            }
            Err(e) => {
                errors += 1;
                warn!("Exchange {} failed: {e}", latencies.len());
            }
            Ok(_) => {}
        }
        if elapsed > threshold {
            let since = previous_slow.map_or(String::from("first"), |at: Instant| {
                format!("{:.1} s after the previous", (start - at).as_secs_f64())
            });
            info!(
                "Exchange {}: {:.1} ms ({since})",
                latencies.len(),
                elapsed.as_secs_f64() * 1000.0
            );
            previous_slow = Some(start);
        }

        // Paced on a fixed schedule; after falling behind, resync rather than burst.
        next += period;
        let now = Instant::now();
        if next > now {
            thread::sleep(next - now);
        } else {
            next = now;
        }
    }

    if let Err(e) = bridge.enable_rc() {
        warn!("Failed to hand RC back to RealFlight: {e}");
    }
    report(&mut latencies, threshold, errors);
}

/// Logs the count, percentiles, and slow and failed exchanges.
fn report(latencies: &mut [Duration], threshold: Duration, errors: u64) {
    if latencies.is_empty() {
        return;
    }
    latencies.sort_unstable();
    let ms = |q: f64| {
        let i = ((latencies.len() - 1) as f64 * q).round() as usize;
        latencies[i].as_secs_f64() * 1000.0
    };
    let slow = latencies.iter().filter(|&&l| l > threshold).count();
    info!(
        "{} exchanges: p50 {:.2} ms, p99 {:.2} ms, p99.9 {:.2} ms, max {:.2} ms; {slow} slow, {errors} failed",
        latencies.len(),
        ms(0.5),
        ms(0.99),
        ms(0.999),
        ms(1.0),
    );
}
