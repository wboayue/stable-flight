pub mod commanders;
pub mod controllers;
pub mod math;
pub mod mixers;
pub mod stabilizer;
pub mod telemetry;
pub mod vehicles;

use std::time::Duration;

use log::info;
use uom::si::f32::*;

use commanders::Command;
use math::{Quaternion, Vec3};
pub use stabilizer::FlightStabilizer;

#[derive(Debug, Default, PartialEq)]
pub struct ControlOutput {
    pub thrust: f32,
    pub moments: Vec3<f32>,
}

impl ControlOutput {
    pub fn new(thrust: f32, moments: Vec3<f32>) -> ControlOutput {
        ControlOutput { thrust, moments }
    }
}

#[derive(Debug, Clone, Default)]
pub struct VehicleState {
    pub position: Vec3<Length>,
    pub velocity: Vec3<Velocity>,
    pub acceleration: Vec3<Acceleration>,
    pub attitude: Quaternion,
    pub angular_velocity: Vec3<AngularVelocity>,
    // flight mode
    // heading
    // gps
}

pub trait Vehicle {
    fn estimate_state(&self, dt: Duration) -> VehicleState;
    fn actuate(&mut self, actuator_command: &ActuatorCommand);
}

pub trait Controller {
    fn control(&mut self, dt: Duration, command: &Command, state: &VehicleState) -> ControlOutput;
}

pub trait TelemetryPublisher {
    fn send_telemetry(&mut self, dt: Duration, command: &Command, telemetry: &VehicleState);
}

pub struct ConsoleTelemetryPublisher {
    // sample_rate: f64,
}

impl TelemetryPublisher for ConsoleTelemetryPublisher {
    fn send_telemetry(&mut self, _dt: Duration, _command: &Command, telemetry: &VehicleState) {
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
            motors: [0.0; 8],
            rudder: 0.0,
            elevator: 0.0,
            aileron: 0.0,
            throttle: 0.0,
            elevon_left: 0.0,
            elevon_right: 0.0,
        }
    }
}

pub use mixers::Mixer;
