//! `stable-flight-bridge`: runs on the companion computer (or a laptop) and translates between a
//! Stable Flight flight controller's native link and MAVLink, so a ground station such as QGC
//! can connect to it. Usage and scope: the crate README.
//!
//! ```text
//! cargo run -p stable-flight-bridge
//! cargo run -p stable-flight-bridge -- --gcs udpout:192.168.1.20:14550
//! ```

use std::io::ErrorKind;
use std::time::{Duration, Instant};

use clap::Parser;
use env_logger::Builder;
use log::{debug, info, warn};
use mavlink::common::MavMessage;
use mavlink::error::MessageReadError;
use mavlink::{MavConnection, MavHeader, MavlinkVersion};
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_link::{Downlink, LinkError, MAX_FRAME_LEN, RecvMessage, SendMessage};

mod bridge;
mod connect;
mod encode;
mod outbox;
mod params;
#[cfg(test)]
mod test_support;

use bridge::{Bridge, BridgeConfig};
use outbox::Outbox;

/// Messages read from each side per loop iteration, so neither side starves the other.
const MAX_READS_PER_ITERATION: usize = 64;
/// Sleep between loop iterations.
const IDLE_SLEEP: Duration = Duration::from_millis(1);
/// Shortest interval between reports of a repeating failure.
const ERROR_REPORT_INTERVAL: Duration = Duration::from_secs(1);

/// Bridges a flight controller's native link to a MAVLink ground station.
#[derive(Parser)]
#[command(version)]
struct Args {
    /// Local address the link listens on.
    #[arg(long, default_value = DEFAULT_BRIDGE_ADDR)]
    link_bind: String,
    /// The flight controller's link address.
    #[arg(long, default_value = DEFAULT_FC_ADDR)]
    link_peer: String,
    /// Ground station connection, as a `mavlink::connect` address.
    #[arg(long, default_value = "udpout:127.0.0.1:14550")]
    gcs: String,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    Builder::new()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .format_timestamp_millis()
        .init();

    let args = Args::parse();

    let (mut link_rx, mut link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .map_err(|e| {
            format!(
                "failed to open link {} -> {}: {e}",
                args.link_bind, args.link_peer
            )
        })?
        .into_halves();
    let mut gcs = mavlink::connect::<MavMessage>(&args.gcs)
        .map_err(|e| format!("failed to open ground station connection {}: {e}", args.gcs))?;
    gcs.set_protocol_version(MavlinkVersion::V2);
    info!(
        "bridging link {} <-> {} with ground station {}",
        args.link_bind, args.link_peer, args.gcs
    );

    let config = BridgeConfig::default();
    let header = MavHeader {
        system_id: config.system_id,
        component_id: config.component_id,
        // The connection numbers each message itself.
        sequence: 0,
    };
    let mut bridge = Bridge::new(config);
    let mut out = Outbox::default();
    let mut buf = [0u8; MAX_FRAME_LEN];
    let mut link_errors = ErrorReport::default();
    let mut gcs_errors = ErrorReport::default();

    loop {
        let now = Instant::now();

        for _ in 0..MAX_READS_PER_ITERATION {
            match link_rx.recv_message::<Downlink>(&mut buf) {
                Ok(Some(msg)) => bridge.on_downlink(&msg, now, &mut out),
                Ok(None) => break,
                // Only that frame is lost.
                Err(LinkError::Decode(e)) => debug!("dropped link frame: {e}"),
                Err(e) => {
                    link_errors.report(now, &format!("link receive failed: {e}"));
                    break;
                }
            }
        }

        for _ in 0..MAX_READS_PER_ITERATION {
            match gcs.try_recv() {
                Ok((sender, msg)) => bridge.on_mavlink(&sender, &msg, &mut out),
                Err(MessageReadError::Io(e)) if e.kind() == ErrorKind::WouldBlock => break,
                // Not ours to understand (another dialect, a bad CRC): skip it.
                Err(MessageReadError::Parse(e)) => debug!("dropped MAVLink message: {e}"),
                Err(e) => {
                    gcs_errors.report(now, &format!("ground station receive failed: {e}"));
                    break;
                }
            }
        }

        bridge.poll(now, &mut out);

        for msg in out.to_fc.drain(..) {
            if let Err(e) = link_tx.send_message(&msg, &mut buf) {
                link_errors.report(now, &format!("link send failed: {e}"));
            }
        }
        for msg in out.to_gcs.drain(..) {
            if let Err(e) = gcs.send(&header, &msg) {
                gcs_errors.report(now, &format!("ground station send failed: {e}"));
            }
        }

        std::thread::sleep(IDLE_SLEEP);
    }
}

/// Logs a failure at most once per [`ERROR_REPORT_INTERVAL`], counting the ones it skips, so
/// a dead peer doesn't flood the log.
#[derive(Default)]
struct ErrorReport {
    last: Option<Instant>,
    suppressed: u32,
}

impl ErrorReport {
    fn report(&mut self, now: Instant, message: &str) {
        if self
            .last
            .is_some_and(|last| now.saturating_duration_since(last) < ERROR_REPORT_INTERVAL)
        {
            self.suppressed = self.suppressed.saturating_add(1);
            return;
        }
        warn!(
            "{message} ({} earlier failures suppressed)",
            self.suppressed
        );
        self.last = Some(now);
        self.suppressed = 0;
    }
}
