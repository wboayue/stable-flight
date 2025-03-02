use std::time::Duration;

use realflight_bridge::{Configuration, ControlInputs, RealFlightBridge};

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
        let mut input = ControlInputs::default();

        for (i, val) in self.mapping.motors.iter().enumerate() {
            if let Some(value) = val {
                input.channels[i] = command.motors[*value];
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

        if let Err(e) = self.bridge.exchange_data(&input) {
            println!("Failed to exchange data: {}", e);
        }
    }
}

impl RealFlightVehicle {
    pub fn new(configuration: RealFlightConfiguration) -> RealFlightVehicle {
        let configuration = Configuration {
            simulator_host: "192.168.4.117:18083".to_string(),
            ..Default::default()
        };
        let bridge = RealFlightBridge::new(&configuration).expect(&format!(
            "Failed to connect to RealFlight at {}",
            configuration.simulator_host
        ));

        if let Err(e) = bridge.reset_aircraft() {
            eprintln!("Failed to reset aircraft: {}", e);
        }

        if let Err(e) = bridge.disable_rc() {
            eprintln!("Failed to disable RC: {}", e);
        }

        RealFlightVehicle {
            bridge,
            estimated_state: VehicleState::default(),
            mapping: ActuatorMapping::default(),
        }
    }
}

impl Drop for RealFlightVehicle {
    fn drop(&mut self) {
        self.bridge.enable_rc().unwrap();
    }
}
