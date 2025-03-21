use std::time::Duration;

use log::{error, warn};
use realflight_bridge::{ControlInputs, RealFlightBridge, SimulatorState};
use robomath::{to_radians, Quaternion, Vec3};

use crate::{ActuatorCommand, Vehicle, VehicleState};

#[cfg(test)]
mod tests;

/// Defines the mapping between vehicle actuator commands and RealFlight simulator channels.
///
/// This structure allows flexible configuration of how actuator commands (motors, control surfaces)
/// are assigned to RealFlight's channel system (typically 8-16 channels).
///
/// # Examples
///
/// ```no_run
/// use stable_flight_sitl::vehicles::realflight::ActuatorMapping;
///
/// let mut mapping = ActuatorMapping::default();
/// mapping.motors[0] = Some(0);  // Motor 1 to channel 0
/// mapping.rudder = Some(1);     // Rudder to channel 1
/// mapping.throttle = Some(2);   // Throttle to channel 2
/// ```
#[derive(Default)]
pub struct ActuatorMapping {
    /// Array of optional channel assignments for up to 8 motors.
    /// Index represents motor number, value represents channel number (0-15).
    pub motors: [Option<usize>; 8],
    /// Optional channel number for rudder control.
    pub rudder: Option<usize>,
    /// Optional channel number for elevator control.
    pub elevator: Option<usize>,
    /// Optional channel number for aileron control.
    pub aileron: Option<usize>,
    /// Optional channel number for throttle control.
    pub throttle: Option<usize>,
    /// Optional channel number for left elevon control.
    pub elevon_left: Option<usize>,
    /// Optional channel number for right elevon control.
    pub elevon_right: Option<usize>,
}

/// Configuration parameters for connecting to a RealFlight simulation environment.
///
/// This structure combines the communication bridge with channel mapping configuration.
///
/// # Type Parameters
///
/// * `T` - The type implementing `RealFlightBridge` for simulator communication.
///
/// # Examples
///
/// ```no_run
/// use std::time::Duration;
/// use realflight_bridge::{RealFlightBridge, RealFlightLocalBridge};
/// use stable_flight_sitl::vehicles::realflight::{RealFlightConfiguration, RealFlightVehicle, ActuatorMapping};
/// use stable_flight_sitl::{ActuatorCommand, Vehicle};
///
/// let bridge = RealFlightLocalBridge::new().expect("Failed to create bridge");  
/// let config = RealFlightConfiguration {
///     mapping: ActuatorMapping::default(),
///     bridge,
/// };
/// ```
pub struct RealFlightConfiguration<T>
where
    T: RealFlightBridge,
{
    /// Configuration mapping actuator commands to simulator channels.
    pub mapping: ActuatorMapping,
    /// The bridge implementation for RealFlight communication.
    pub bridge: T,
}

/// A vehicle implementation interfacing with the RealFlight flight simulator.
///
/// This struct provides a bridge between generic vehicle control commands and the
/// RealFlight simulation environment, handling both actuator commands and state updates.
///
/// # Type Parameters
///
/// * `T` - The type implementing `RealFlightBridge` for simulator communication.
///
/// # Examples
///
/// ```no_run
/// use realflight_bridge::{RealFlightBridge, RealFlightLocalBridge};
/// use stable_flight_sitl::vehicles::realflight::{RealFlightConfiguration, RealFlightVehicle, ActuatorMapping};
/// use stable_flight_sitl::{ActuatorCommand, Vehicle};
/// use std::time::Duration;
///
/// let mut vehicle = RealFlightVehicle::new(RealFlightConfiguration {
///     mapping: ActuatorMapping::default(),
///     bridge: RealFlightLocalBridge::new().expect("Failed to create bridge"),
/// });
///
/// let command = ActuatorCommand::default();
/// vehicle.actuate(&command);
/// let state = vehicle.estimate_state(Duration::from_secs(1));
/// ```
pub struct RealFlightVehicle<T>
where
    T: RealFlightBridge,
{
    /// The communication bridge to the RealFlight simulator.
    bridge: T,
    /// Current estimated state of the vehicle based on simulator feedback.
    estimated_state: VehicleState,
    /// Configuration mapping actuator commands to simulator channels.
    mapping: ActuatorMapping,
}

impl<T> Vehicle for RealFlightVehicle<T>
where
    T: RealFlightBridge,
{
    /// Retrieves the current estimated state of the vehicle.
    ///
    /// # Arguments
    ///
    /// * `_dt` - Controller tick duration (currently unused as state comes from simulator).
    ///
    /// # Returns
    ///
    /// A `VehicleState` containing current position, velocity, attitude, etc.
    fn estimate_state(&self, _dt: Duration) -> VehicleState {
        self.estimated_state.clone()
    }

    /// Applies actuator commands and updates vehicle state from simulator feedback.
    ///
    /// # Arguments
    ///
    /// * `command` - The actuator commands to send to the simulator.
    fn actuate(&mut self, command: &ActuatorCommand) {
        let input = self.map_channels(command);

        match self.bridge.exchange_data(&input) {
            Ok(state) => self.update_estimate(state),
            Err(e) => error!("Failed to exchange data with RealFlight: {}", e),
        }
    }
}

impl<T> RealFlightVehicle<T>
where
    T: RealFlightBridge,
{
    /// Creates a new RealFlight vehicle instance with the given configuration.
    ///
    /// Initializes the simulator connection by disabling RC mode and resetting the aircraft.
    ///
    /// # Arguments
    ///
    /// * `configuration` - Configuration containing bridge and channel mapping.
    ///
    /// # Returns
    ///
    /// A new `RealFlightVehicle` instance ready for simulation.
    ///
    /// # Notes
    ///
    /// Warnings are logged if RC disable or aircraft reset operations fail.
    pub fn new(configuration: RealFlightConfiguration<T>) -> RealFlightVehicle<T> {
        if let Err(e) = configuration.bridge.disable_rc() {
            warn!("Failed to disable RC: {}", e);
        }

        if let Err(e) = configuration.bridge.reset_aircraft() {
            warn!("Failed to reset aircraft: {}", e);
        }

        RealFlightVehicle {
            bridge: configuration.bridge,
            estimated_state: VehicleState::default(),
            mapping: configuration.mapping,
        }
    }

    /// Converts actuator commands to RealFlight control inputs based on channel mapping.
    ///
    /// # Arguments
    ///
    /// * `command` - The actuator command containing control surface and motor values.
    ///
    /// # Returns
    ///
    /// A [ControlInputs] structure with channel values set according to the mapping.
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

    /// Updates the vehicle's estimated state based on simulator feedback.
    ///
    /// # Arguments
    ///
    /// * `state` - The simulator state containing position, velocity, and orientation data.
    ///
    /// # Notes
    ///
    /// Converts simulator units (degrees) to SI units (radians) where appropriate.
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

        self.estimated_state.roll = to_radians(state.roll.value);
        self.estimated_state.pitch = to_radians(state.inclination.value);
        self.estimated_state.heading = to_radians(state.azimuth.value);

    }
}

impl<T> Drop for RealFlightVehicle<T>
where
    T: RealFlightBridge,
{
    /// Cleans up by re-enabling RC mode when the vehicle is dropped.
    ///
    /// # Notes
    ///
    /// Logs a warning if RC mode cannot be re-enabled.
    fn drop(&mut self) {
        if let Err(e) = self.bridge.enable_rc() {
            warn!("Failed to enable RC: {}", e);
        }
    }
}
