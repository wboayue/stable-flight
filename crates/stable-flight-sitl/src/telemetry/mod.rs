//! Includes components for transmitting flight data to external systems for logging, monitoring, and visualization.
use log::info;

use stable_flight::{TelemetryPublisher, TelemetryRecord};

pub mod file_logger;
pub mod mavlink;
mod throttle;

pub struct ConsoleTelemetryPublisher {
    // sample_rate: f64,
}

impl TelemetryPublisher for ConsoleTelemetryPublisher {
    fn publish(&mut self, record: &TelemetryRecord) {
        info!("Sending telemetry: {:?}", record.state);
    }
}
