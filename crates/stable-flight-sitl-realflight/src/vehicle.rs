use std::time::Duration;

use log::{error, warn};
use realflight_bridge::{ControlInputs, RealFlightBridge, SimulatorState};
use stable_flight::Vec3;
use stable_flight::nalgebra::{Quaternion, UnitQuaternion};

use stable_flight::{Actuator, ActuatorCommand, GRAVITY, StateEstimator, VehicleState};

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
/// use stable_flight_sitl_realflight::ActuatorMapping;
///
/// let mut mapping = ActuatorMapping::default();
/// mapping.motors[0] = Some(0);  // Motor 1 to channel 0
/// mapping.rudder = Some(1);     // Rudder to channel 1
/// mapping.throttle = Some(2);   // Throttle to channel 2
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
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
/// use stable_flight_sitl_realflight::realflight_bridge::RealFlightLocalBridge;
/// use stable_flight_sitl_realflight::{ActuatorMapping, RealFlightConfiguration};
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
/// # Timing
///
/// RealFlight Link sends controls and returns state in one round-trip, so each tick makes
/// exactly one exchange, in [`Actuator::actuate`]. [`StateEstimator::estimate_state`] returns
/// the state cached from that exchange: guidance and the stabilizer on tick *n* see the state RealFlight
/// reported when it received tick *n − 1*'s controls, one tick old. The one exception is the
/// priming call (`dt == Duration::ZERO`), which makes an exchange with neutral inputs so the
/// first state is real; if that exchange fails, it logs an error and returns
/// `VehicleState::default()`. `exchange_data` is a blocking network round-trip, so a slow link
/// stalls the tick. See the core `Vehicle` docs for the cross-backend contract.
///
/// # Frames
///
/// RealFlight's axes are undocumented and inconsistent across fields. A live capture (the
/// `capture-state-realflight` example) shows: position `x` east and `y` north, with altitude above
/// sea level; world velocity NED; the quaternion in `(w, y, x, -z)` order relative to NED → FRD;
/// azimuth and yaw rate counterclockwise-positive (nose left); roll, inclination, and roll and
/// pitch rates in the contract's sense; angles in degrees. World acceleration is unreliable, so it
/// is derived from the FRD body specific force. This type converts all of these to the
/// `VehicleState` frame contract (north/east/up, radians) at the boundary.
///
/// # Examples
///
/// ```no_run
/// use stable_flight_sitl_realflight::realflight_bridge::RealFlightLocalBridge;
/// use stable_flight_sitl_realflight::{ActuatorMapping, RealFlightConfiguration, RealFlightVehicle};
/// use stable_flight::{Actuator, ActuatorCommand, StateEstimator};
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

impl<T> StateEstimator for RealFlightVehicle<T>
where
    T: RealFlightBridge,
{
    /// Returns the vehicle state cached from the last simulator exchange.
    ///
    /// Does not contact the simulator, except on the priming call (`dt == Duration::ZERO`),
    /// which makes one exchange with neutral inputs (see [Timing](RealFlightVehicle#timing)).
    /// Otherwise `dt` is ignored: the simulator reports its own truth state.
    fn estimate_state(&mut self, dt: Duration) -> VehicleState {
        if dt.is_zero() {
            // initialize state if dt is zero
            let input = ControlInputs::default();
            match self.bridge.exchange_data(&input) {
                Ok(state) => self.estimated_state = vehicle_state(&state),
                Err(e) => error!("Failed initial exchange with RealFlight: {}", e),
            }
        }

        self.estimated_state.clone()
    }
}

impl<T> Actuator for RealFlightVehicle<T>
where
    T: RealFlightBridge,
{
    /// Applies actuator commands and updates vehicle state from simulator feedback.
    ///
    /// # Arguments
    ///
    /// * `command` - The actuator commands to send to the simulator.
    fn actuate(&mut self, command: &ActuatorCommand) {
        let input = self.map_channels(command);

        match self.bridge.exchange_data(&input) {
            Ok(state) => self.estimated_state = vehicle_state(&state),
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
}

/// Converts one RealFlight report to the `VehicleState` frame contract.
///
/// This is the whole of [`RealFlightVehicle`]'s state estimate, as a pure function, so it can
/// also convert recorded or replayed reports. See [Frames](RealFlightVehicle#frames) for
/// RealFlight's conventions.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl_realflight::realflight_bridge::SimulatorState;
/// use stable_flight_sitl_realflight::vehicle_state;
///
/// // RealFlight reports position east/north; the contract wants north/east.
/// let report = SimulatorState {
///     aircraft_position_x_m: 3.0,
///     aircraft_position_y_m: 4.0,
///     ..Default::default()
/// };
/// let state = vehicle_state(&report);
/// assert_eq!(state.position_m.x, 4.0);
/// assert_eq!(state.position_m.y, 3.0);
/// ```
pub fn vehicle_state(state: &SimulatorState) -> VehicleState {
    // RealFlight's quaternion axes are (y, x, -z) of NED → FRD. Taken as reported, without
    // renormalizing: it is already unit.
    let q = state.orientation;
    let attitude = UnitQuaternion::new_unchecked(Quaternion::new(q.w, q.y, q.x, -q.z));

    // World velocity is NED.
    let velocity = state.velocity_world_mps;

    // RealFlight's world acceleration is unreliable (z reads ~7.5 m/s² at rest), so rotate the
    // FRD body specific force to NED and add gravity back.
    let f = state.acceleration_body_mps2;
    let acceleration = attitude * Vec3::new(f.x, f.y, f.z) + Vec3::new(0.0, 0.0, GRAVITY);

    VehicleState {
        // Position is east/north; altitude is already up-positive.
        position_m: Vec3::new(
            state.aircraft_position_y_m,
            state.aircraft_position_x_m,
            state.altitude_asl_m,
        ),
        velocity_mps: Vec3::new(velocity.x, velocity.y, -velocity.z),
        acceleration_mps2: Vec3::new(acceleration.x, acceleration.y, -acceleration.z),
        attitude,
        // Yaw rate and azimuth are counterclockwise-positive (nose left).
        angular_velocity: Vec3::new(
            state.roll_rate_dps.to_radians(),
            state.pitch_rate_dps.to_radians(),
            -state.yaw_rate_dps.to_radians(),
        ),
        roll: state.roll_deg.to_radians(),
        pitch: state.inclination_deg.to_radians(),
        heading: -state.azimuth_deg.to_radians(),
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
