mod controllers;
pub mod realflight;
mod stabilizer;
pub mod sticks;
pub mod mixers;

use std::time::Duration;

pub use controllers::{AngleController, RateController};
pub use stabilizer::{FlightStabilizer, StabilizerConfiguration};

pub trait Stick {
    fn get_controls(&self) -> StickInput;
}

#[derive(Debug)]
pub struct StickInput {
    pub channels: [f32; 6],
}

#[derive(Debug)]
pub struct ControlOutput {}

#[derive(Debug)]
pub struct VehicleState {
    // roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, altitude, airspeed, heading, position
    // flight mode
}

pub trait Vehicle {
    fn get_state(&self, dt: Duration) -> VehicleState;
    fn command_actuators(&self, dt: Duration, stick: &StickInput, controls: &ControlOutput);
}

pub trait Controller {
    fn compute_desired_state(&self, controls: &StickInput, state: &VehicleState) -> VehicleState;
    fn compute_control_outputs(
        &self,
        desired_state: &VehicleState,
        state: &VehicleState,
    ) -> ControlOutput;
}

pub trait TelemetryPublisher {
    fn send_telemetry(&self, telemetry: &VehicleState);
}

pub struct ConsoleTelemetryPublisher {
    // sample_rate: f64,
}

impl TelemetryPublisher for ConsoleTelemetryPublisher {
    fn send_telemetry(&self, telemetry: &VehicleState) {
        println!("Sending telemetry: {:?}", telemetry);
    }
}

pub trait Mixer {
    fn mix_control_outputs(&self, controls: &ControlOutput);
}