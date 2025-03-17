use std::time::Duration;

use log::{error, info, warn};
use realflight_bridge::{ControlInputs, RealFlightRemoteBridge, SimulatorState};
use robomath::{Quaternion, Vec3, to_radians};

use crate::{
    ActuatorCommand, Vehicle, VehicleState,
};

/// Mapping of actuator commands to RealFlight channels
#[derive(Default)]
pub struct ActuatorMapping {
    /// Mapping for up to 8 motor channels.
    pub motors: [Option<usize>; 8],
    /// Optional channel for the rudder.
    pub rudder: Option<usize>,
    /// Optional channel for the elevator.
    pub elevator: Option<usize>,
    /// Optional channel for the aileron.
    pub aileron: Option<usize>,
    /// Optional channel for the throttle.
    pub throttle: Option<usize>,
    /// Optional channel for the left elevon.
    pub elevon_left: Option<usize>,
    /// Optional channel for the right elevon.
    pub elevon_right: Option<usize>,
}

/// Configuration for connecting to a RealFlight simulation environment.
pub struct RealFlightConfiguration {
    /// The host address (and port) of the RealFlight simulator.
    pub host: String,
    /// Actuator channel mapping configuration.
    pub mapping: ActuatorMapping,
}

/// A vehicle implementation for the RealFlight simulation environment.
///
/// `RealFlightVehicle` communicates with the RealFlight simulator through a `RealFlightBridge`
/// to send actuator commands and receive state updates.
pub struct RealFlightVehicle {
    /// The communication bridge to the RealFlight simulator.
    bridge: RealFlightRemoteBridge,
    /// The current estimated state of the vehicle.
    estimated_state: VehicleState,
    /// Configuration mapping for actuator channels.
    mapping: ActuatorMapping,
}

impl Vehicle for RealFlightVehicle {
    /// Returns the current estimated state of the vehicle.
    fn estimate_state(&self, _dt: Duration) -> VehicleState {
        self.estimated_state.clone()
    }

    /// Sends actuator commands to the RealFlight simulator and receives state update.
    fn actuate(&mut self, command: &ActuatorCommand) {
        let input = self.map_channels(command);

        match self.bridge.exchange_data(&input) {
            Ok(state) => self.update_estimate(state),
            Err(e) => error!("Failed to exchange data with RealFlight: {}", e),
        }
    }
}

impl RealFlightVehicle {
    /// Constructs a new `RealFlightVehicle` using the given configuration.
    ///
    /// This function initializes the vehicle by disabling RC mode and resetting the aircraft.
    ///
    /// # Arguments
    ///
    /// * `configuration` - The connection and mapping configuration parameters.
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

    /// Maps the provided actuator command to the corresponding RealFlight control channels.
    ///
    /// This function uses the `ActuatorMapping` to populate the `ControlInputs` structure.
    ///
    /// # Arguments
    ///
    /// * `command` - The actuator command containing desired values.
    ///
    /// # Returns
    ///
    /// A `ControlInputs` structure with channels set according to the mapping.
    fn map_channels(&self, command: &ActuatorCommand) -> ControlInputs {
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

        input
    }

    /// Updates the internal state estimate based on data received from the simulator.
    ///
    /// # Arguments
    ///
    /// * `state` - The simulator state data received from the RealFlight bridge.
    fn update_estimate(&mut self, state: SimulatorState) {
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

        self.estimated_state.attitude = Quaternion {
            w: state.orientation_quaternion_w,
            x: state.orientation_quaternion_x,
            y: state.orientation_quaternion_y,
            z: state.orientation_quaternion_z,
        };

        self.estimated_state.angular_velocity = Vec3 {
            x: to_radians(state.roll_rate.value),
            y: to_radians(state.pitch_rate.value),
            z: to_radians(state.yaw_rate.value),
        };
    }
}

impl Drop for RealFlightVehicle {
    /// Cleans up the RealFlight vehicle by re-enabling RC mode.
    fn drop(&mut self) {
        if let Err(e) = self.bridge.enable_rc() {
            warn!("Failed to enable RC: {}", e);
        }
    }
}
