use std::time::Duration;

use realflight_bridge::{Configuration, ControlInputs, RealFlightBridge};

use crate::{ControlOutput, StickInput, Vehicle, VehicleState};

pub struct RealFlightVehicle {
    bridge: RealFlightBridge,
}

impl Vehicle for RealFlightVehicle {
    fn get_state(&self, dt: Duration) -> VehicleState {
        VehicleState {}
    }

    fn command_actuators(&self, dt: Duration, stick: &StickInput, controls: &ControlOutput) {
        let mut input = ControlInputs::default();
        for (i, channel) in stick.channels.iter().enumerate() {
            input.channels[i] = *channel;
        }

        println!("Commanding actuators: {:?}", input);

        self.bridge.exchange_data(&input).unwrap();
        // self.bridge.exchange_data(controls);
    }
}

impl RealFlightVehicle {
    pub fn new() -> RealFlightVehicle {
        let configuration = Configuration {
            simulator_url: "192.168.4.117:18083".to_string(),
            ..Default::default()
        };
        let simulator_url = configuration.simulator_url.clone();
        let bridge = RealFlightBridge::new(configuration).expect(&format!(
            "Failed to connect to RealFlight at {}",
            simulator_url
        ));

        if let Err(e) = bridge.reset_aircraft() {
            eprintln!("Failed to reset aircraft: {}", e);
        }

        if let Err(e) = bridge.disable_rc() {
            eprintln!("Failed to disable RC: {}", e);
        }

        RealFlightVehicle { bridge }
    }
}

impl Drop for RealFlightVehicle {
    fn drop(&mut self) {
        self.bridge.enable_rc().unwrap();
    }
}
