use std::time::Duration;

use log::{error, info, warn};
use realflight_bridge::{Configuration, ControlInputs, RealFlightBridge, SimulatorState};

use crate::{ActuatorCommand, Vehicle, VehicleState};

/// Mapping of actuator commands to RealFlight channels
pub struct ActuatorMapping {
    pub motors: [Option<usize>; 8],
    pub rudder: Option<usize>,
    pub elevator: Option<usize>,
    pub aileron: Option<usize>,
    pub throttle: Option<usize>,
    pub elevon_left: Option<usize>,
    pub elevon_right: Option<usize>,
}

impl Default for ActuatorMapping {
    fn default() -> Self {
        ActuatorMapping {
            motors: [None, None, None, None, None, None, None, None],
            rudder: None,
            elevator: None,
            aileron: None,
            throttle: None,
            elevon_left: None,
            elevon_right: None,
        }
    }
}

pub struct RealFlightConfiguration {
    pub host: String,
    pub mapping: ActuatorMapping,
}

pub struct RealFlightVehicle {
    bridge: RealFlightBridge,
    estimated_state: VehicleState,
    mapping: ActuatorMapping,
}

impl Vehicle for RealFlightVehicle {
    fn estimate_state(&self, _dt: Duration) -> VehicleState {
        self.estimated_state.clone()
    }

    fn actuate(&self, command: ActuatorCommand) {
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
        let bridge_configuration = Configuration {
            simulator_host: configuration.host,
            connect_timeout: Duration::from_secs(1),
            ..Default::default()
        };
        let bridge = RealFlightBridge::new(&bridge_configuration).expect(&format!(
            "Failed to connect to RealFlight at {}",
            bridge_configuration.simulator_host
        ));

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
            input.channels[i] = 0.5;
        }
        input
    }

    fn update_estimate(&self, state: SimulatorState) {
        info!("Received state: {:?}", state);
        // self.estimated_state = state;
    }
}

impl Drop for RealFlightVehicle {
    fn drop(&mut self) {
        self.bridge.enable_rc().unwrap();
    }
}
