use std::time::Duration;

use log::{error, warn};
use realflight_bridge::{ControlInputs, RealFlightBridge, SimulatorState};
use stable_flight::Vec3;
use stable_flight::nalgebra::{Quaternion, UnitQuaternion};
use uom::si::acceleration::meter_per_second_squared;
use uom::si::f32::{Acceleration, Length, Velocity};
use uom::si::length::meter;
use uom::si::velocity::meter_per_second;

use stable_flight::{Actuator, ActuatorCommand, StateEstimator, VehicleState};

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
/// RealFlight reports world velocity north/east/down, position east/north with altitude
/// above sea level, and angles and body rates in degrees (see the `realflight-bridge`
/// `SimulatorState` frame docs). `VehicleState` wants north/east/up and radians (its frame
/// contract), so this type converts at the boundary. Several conversions don't meet that yet:
/// angles and body rates are converted to radians twice (about 57× too small), vertical
/// velocity and acceleration keep RealFlight's down-positive sign, position `x`/`y` are passed
/// through unswapped, and the yaw-rate sign and quaternion axes are taken as reported. The
/// ignored `conforms_*` tests pin the first two; the fix (#62) needs a flight check because the
/// current gains were tuned against them.
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
                Ok(state) => self.update_estimate(state),
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
    /// Wraps RealFlight's raw meters and m/s in `uom` quantities and converts degrees to
    /// radians (twice, for now; see [Frames](RealFlightVehicle#frames)).
    fn update_estimate(&mut self, state: SimulatorState) {
        self.estimated_state.position = Vec3::new(
            Length::new::<meter>(state.aircraft_position_x_m),
            Length::new::<meter>(state.aircraft_position_y_m),
            Length::new::<meter>(state.altitude_asl_m),
        );

        let velocity = state.velocity_world_mps;
        self.estimated_state.velocity = Vec3::new(
            Velocity::new::<meter_per_second>(velocity.x),
            Velocity::new::<meter_per_second>(velocity.y),
            Velocity::new::<meter_per_second>(velocity.z),
        );

        let acceleration = state.acceleration_world_mps2;
        self.estimated_state.acceleration = Vec3::new(
            Acceleration::new::<meter_per_second_squared>(acceleration.x),
            Acceleration::new::<meter_per_second_squared>(acceleration.y),
            Acceleration::new::<meter_per_second_squared>(acceleration.z),
        );

        // Taken as reported, without renormalizing: RealFlight's quaternion is already unit.
        let q = state.orientation;
        self.estimated_state.attitude =
            UnitQuaternion::new_unchecked(Quaternion::new(q.w, q.x, q.y, q.z));

        // Deliberately converted to radians twice, preserving the pre-2.0 behavior the gains
        // were tuned against (#62).
        self.estimated_state.angular_velocity = Vec3::new(
            state.roll_rate_dps.to_radians().to_radians(),
            state.pitch_rate_dps.to_radians().to_radians(),
            state.yaw_rate_dps.to_radians().to_radians(),
        );

        self.estimated_state.roll = state.roll_deg.to_radians().to_radians();
        self.estimated_state.pitch = state.inclination_deg.to_radians().to_radians();
        self.estimated_state.heading = state.azimuth_deg.to_radians().to_radians();
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
