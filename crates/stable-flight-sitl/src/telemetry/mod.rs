//! Includes components for transmitting flight data to external systems for logging, monitoring, and visualization.
use std::time::Duration;

use log::info;

use crate::{Command, TelemetryPublisher, VehicleState};

pub mod file_logger;
pub mod mavlink;
mod throttle;

pub struct ConsoleTelemetryPublisher {
    // sample_rate: f64,
}

impl TelemetryPublisher for ConsoleTelemetryPublisher {
    fn send_telemetry(&mut self, _dt: Duration, _command: &Command, telemetry: &VehicleState) {
        info!("Sending telemetry: {:?}", telemetry);
    }
}
