pub mod commanders;
pub mod controllers;
pub mod math;
pub mod mixers;
pub mod stabilizer;
pub mod vehicles;

use std::time::Duration;

use commanders::Command;
use log::info;
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

#[derive(Debug, Clone)]
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
        info!("Sending telemetry: {:?}", telemetry);
    }
}

#[derive(Debug)]
pub struct ActuatorCommand {
    pub motors: [f32; 8],
    pub rudder: f32,
    pub elevator: f32,
    pub aileron: f32,
    pub throttle: f32,
    pub elevon_left: f32,
    pub elevon_right: f32,
}

impl Default for ActuatorCommand {
    fn default() -> Self {
        ActuatorCommand {
            motors: [0.5; 8],
            rudder: 0.0,
            elevator: 0.0,
            aileron: 0.0,
            throttle: 0.0,
            elevon_left: 0.0,
            elevon_right: 0.0,
        }
    }
}

pub trait Mixer {
    fn mix(&self, controls: &ControlOutput) -> ActuatorCommand;
}
