pub mod controllers;
pub mod realflight;
mod stabilizer;
pub mod sticks;
pub mod mixers;
pub mod math;
pub mod commanders;

use std::time::Duration;

use commanders::{Command, Commander};
pub use controllers::{AngleController, RateController};
use math::Vec3;
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
    pub position: Vec3,
    pub velocity: Vec3,
    pub orientation: Vec3,
    pub angular_velocity: Vec3,
    // flight mode
    // heading
    // gps
}

impl Default for VehicleState {
    fn default() -> Self {
        VehicleState {
            position: Vec3 { x: 0.0, y: 0.0, z: 0.0 },
            velocity: Vec3 { x: 0.0, y: 0.0, z: 0.0 },
            orientation: Vec3 { x: 0.0, y: 0.0, z: 0.0 },
            angular_velocity: Vec3 { x: 0.0, y: 0.0, z: 0.0 },
        }
    }
}

pub trait Vehicle {
    fn get_state(&self, dt: Duration) -> VehicleState;
    fn command_actuators(&self, dt: Duration, stick: &Command, controls: &ControlOutput);
}

pub trait Controller {
    fn compute_desired_state(&self, controls: &Command, state: &VehicleState) -> VehicleState;
    fn control(
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