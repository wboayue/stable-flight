//! `stable-flight-bridge`: runs on the companion computer (or a laptop) and translates between a
//! Stable Flight flight controller's native link and MAVLink, so a ground station such as QGC
//! can connect to it. Usage and scope: the crate README.
//!
//! ```text
//! cargo run -p stable-flight-bridge
//! cargo run -p stable-flight-bridge -- --gcs udpout:192.168.1.20:14550
//! ```

use std::time::Instant;

use clap::Parser;
use env_logger::Builder;
use log::info;
use mavlink::common::MavMessage;
use mavlink::{MavConnection, MavHeader, MavlinkVersion};
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};

mod bridge;
mod connect;
mod encode;
mod idle;
mod outbox;
mod params;
mod pump;
#[cfg(test)]
mod test_support;

use bridge::{Bridge, BridgeConfig};
use idle::IdleBackoff;
use pump::Pump;

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

    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
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
    let mut pump = Pump::new(link_rx, link_tx, Box::new(gcs), header, Bridge::new(config));
    let mut backoff = IdleBackoff::default();
    loop {
        let activity = pump.step(Instant::now());
        std::thread::sleep(backoff.next(&activity));
    }
}
