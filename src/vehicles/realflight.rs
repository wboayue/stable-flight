use std::time::Duration;

use log::{error, info, warn};
use realflight_bridge::{ControlInputs, RealFlightRemoteBridge, SimulatorState};

use crate::{math::Vec3, ActuatorCommand, Vehicle, VehicleState};

/// Mapping of actuator commands to RealFlight channels
#[derive(Default)]
pub struct ActuatorMapping {
    pub motors: [Option<usize>; 8],
    pub rudder: Option<usize>,
    pub elevator: Option<usize>,
    pub aileron: Option<usize>,
    pub throttle: Option<usize>,
    pub elevon_left: Option<usize>,
    pub elevon_right: Option<usize>,
}

pub struct RealFlightConfiguration {
    pub host: String,
    pub mapping: ActuatorMapping,
}

pub struct RealFlightVehicle {
    bridge: RealFlightRemoteBridge,
    estimated_state: VehicleState,
    mapping: ActuatorMapping,
}

impl Vehicle for RealFlightVehicle {
    fn estimate_state(&self, _dt: Duration) -> VehicleState {
        self.estimated_state.clone()
    }

    fn actuate(&mut self, command: ActuatorCommand) {
        info!("Actuating RealFlight with command: {:?}", command);

        let input = self.map_channels(command);

        info!("Actuating RealFlight with channels: {:?}", input);

        match self.bridge.exchange_data(&input) {
            Ok(state) => self.update_estimate(state),
            Err(e) => error!("Failed to exchange data with RealFlight: {}", e),
        }
    }
}

impl RealFlightVehicle {
    pub fn new(configuration: RealFlightConfiguration) -> RealFlightVehicle {
        let mut bridge = RealFlightRemoteBridge::new(&configuration.host).unwrap_or_else(|_| {
            panic!("Failed to connect to RealFlight at {}", configuration.host)
        });

        if let Err(e) = bridge.disable_rc() {
            warn!("Failed to disable RC: {}", e);
        }

        if let Err(e) = bridge.reset_aircraft() {
            warn!("Failed to reset aircraft: {}", e);
        }

        RealFlightVehicle {
            bridge,
            estimated_state: VehicleState::default(),
            mapping: configuration.mapping,
        }
    }

    fn map_channels(&self, command: ActuatorCommand) -> ControlInputs {
        let mut input = ControlInputs::default();

        for (i, val) in self.mapping.motors.iter().enumerate() {
            if let Some(value) = val {
                input.channels[*value] = command.motors[i];
            }
        }

        if let Some(i) = self.mapping.rudder {
            input.channels[i] = command.rudder;
        }

        if let Some(i) = self.mapping.elevator {
            input.channels[i] = command.elevator;
        }

        if let Some(i) = self.mapping.aileron {
            input.channels[i] = command.aileron;
        }

        if let Some(i) = self.mapping.throttle {
            input.channels[i] = command.throttle;
        }

        if let Some(i) = self.mapping.elevon_left {
            input.channels[i] = command.elevon_left;
        }

        if let Some(i) = self.mapping.elevon_right {
            input.channels[i] = command.elevon_right;
        }

        for i in 0..12 {
            input.channels[i] = 0.8;
        }
        input
    }

    fn update_estimate(&mut self, state: SimulatorState) {
        info!("Received state: {:?}", state);

        self.estimated_state.position = Vec3 {
            x: state.aircraft_position_x,
            y: state.aircraft_position_y,
            z: state.altitude_asl,
        };

        self.estimated_state.velocity = Vec3 {
            x: state.velocity_world_u,
            y: state.velocity_world_v,
            z: state.velocity_world_w,
        };

        self.estimated_state.acceleration = Vec3 {
            x: state.acceleration_world_ax,
            y: state.acceleration_world_ay,
            z: state.acceleration_world_az,
        };
    }
}

impl Drop for RealFlightVehicle {
    fn drop(&mut self) {
        self.bridge.enable_rc().unwrap();
    }
}
