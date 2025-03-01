pub mod commanders;
pub mod controllers;
pub mod math;
pub mod mixers;
pub mod realflight;
pub mod stabilizer;
pub mod sticks;

use std::time::Duration;

use commanders::Command;
use math::Vec3;
pub use stabilizer::FlightStabilizer;

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
            position: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            velocity: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angular_velocity: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        }
    }
}

pub trait Vehicle {
    fn estimate_state(&self, dt: Duration) -> VehicleState;
    fn actuate(&self, actuator_command: ActuatorCommand);
}

pub trait Controller {
    fn control(&self, command: &Command, state: &VehicleState) -> ControlOutput;
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

pub struct ActuatorCommand {
    pub channels: [f32; 6],
}

pub trait Mixer {
    fn mix_control_outputs(&self, controls: &ControlOutput) -> ActuatorCommand;
}
