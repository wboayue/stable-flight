use std::time::{Duration, Instant};

use log::{debug, info, warn};
use realflight_bridge::{BridgeError, ControlInputs, RealFlightBridge, SimulatorState};
use stable_flight::Vec3;
use stable_flight::nalgebra::{Quaternion, UnitQuaternion};

use stable_flight::{Actuator, ActuatorCommand, GRAVITY, StateEstimator, VehicleState};

use health::LinkHealth;

mod health;
#[cfg(test)]
mod tests;

/// [`RealFlightVehicle`] logs an exchange slower than this at `debug` (five ticks at 250 Hz),
/// to tell a stall in the RealFlight round trip from one elsewhere in the tick (#255). The
/// `exchange-latency-realflight` example's default threshold.
pub const SLOW_EXCHANGE: Duration = Duration::from_millis(20);

/// Builds a fresh bridge after the connection is lost; see
/// [`RealFlightVehicle::with_reconnect`].
type Reconnect<T> = Box<dyn FnMut() -> Result<T, BridgeError> + Send>;

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
/// stalls the tick; an exchange over 20 ms is logged at `debug` with its duration. See the core
/// `Vehicle` docs for the cross-backend contract.
///
/// # Connection loss
///
/// A failed exchange is logged at most once a second, with a count of the failures skipped in
/// between. The state estimate keeps the last state RealFlight reported until an exchange
/// succeeds again: outputs computed meanwhile never reach the simulator, and the controllers
/// only wind up against a frozen error, bounded by their `max_integral`.
///
/// A `BridgeError::Connection` can leave the bridge unusable (a remote bridge is then out of
/// sync for good). With [`with_reconnect`](RealFlightVehicle::with_reconnect), the vehicle
/// stops exchanging, waits 0.5 s, and builds a new bridge, doubling the wait after each failed
/// attempt up to 4 s (`INITIAL_BACKOFF_S`, `MAX_BACKOFF_S` in `vehicle/health.rs`). A new
/// bridge gets `disable_rc` again (but no aircraft reset: it is mid-flight) and replaces the old
/// one. Only a `Connection` error from `disable_rc` fails the attempt: RealFlight keeps the Link
/// controller across connections, so it may answer with a SOAP fault, which is logged and
/// ignored as in [`new`](RealFlightVehicle::new). Waits count the ticks' nominal `dt`, and an
/// attempt blocks its tick for as long as building the bridge takes, so give the closure a short
/// connect timeout. Other errors, or any error without the hook, just retry the exchange on
/// the next tick.
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
    /// Builds a new bridge after a connection error; `None` keeps the one bridge.
    reconnect: Option<Reconnect<T>>,
    /// Reconnect timing and failure-log rate limiting.
    health: LinkHealth,
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
        self.health.advance(dt.as_secs_f32());
        if dt.is_zero() {
            // initialize state if dt is zero
            self.exchange(&ControlInputs::default());
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
        self.exchange(&input);
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
            reconnect: None,
            health: LinkHealth::default(),
        }
    }

    /// Rebuilds the bridge with `reconnect` after a connection error, with backoff (see
    /// [Connection loss](RealFlightVehicle#connection-loss)).
    ///
    /// `reconnect` is called on the flight loop's thread and blocks the tick, so it should
    /// connect with a short timeout.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use std::time::Duration;
    /// use stable_flight_sitl_realflight::realflight_bridge::{BridgeError, RealFlightRemoteBridge};
    /// use stable_flight_sitl_realflight::{ActuatorMapping, RealFlightConfiguration, RealFlightVehicle};
    ///
    /// let host = String::from("127.0.0.1:8080");
    /// let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
    ///     mapping: ActuatorMapping::default(),
    ///     bridge: RealFlightRemoteBridge::new(&host).expect("Failed to create bridge"),
    /// })
    /// .with_reconnect(move || {
    ///     RealFlightRemoteBridge::with_timeout(&host, Duration::from_millis(250))
    ///         .map_err(BridgeError::from)
    /// });
    /// ```
    pub fn with_reconnect(
        mut self,
        reconnect: impl FnMut() -> Result<T, BridgeError> + Send + 'static,
    ) -> Self {
        self.reconnect = Some(Box::new(reconnect));
        self
    }

    /// Makes the tick's one exchange, updating the state on success. While the connection is
    /// lost, skips it, except to reconnect when an attempt is due.
    fn exchange(&mut self, input: &ControlInputs) {
        if self.health.is_down() && !(self.health.retry_due() && self.try_reconnect()) {
            return;
        }

        let start = Instant::now();
        let result = self.bridge.exchange_data(input);
        let elapsed = start.elapsed();
        if elapsed > SLOW_EXCHANGE {
            debug!("Slow RealFlight exchange: {} ms", elapsed.as_millis());
        }
        match result {
            Ok(state) => self.estimated_state = vehicle_state(&state),
            Err(e) => {
                self.health
                    .report(format_args!("Failed to exchange data with RealFlight: {e}"));
                if matches!(e, BridgeError::Connection(_)) && self.reconnect.is_some() {
                    self.health.connection_lost();
                }
            }
        }
    }

    /// Builds a new bridge and switches RealFlight to Link control on it (`disable_rc`). Returns
    /// whether it replaced the old one.
    fn try_reconnect(&mut self) -> bool {
        let Some(reconnect) = &mut self.reconnect else {
            return false;
        };

        let bridge = reconnect().and_then(|bridge| match bridge.disable_rc() {
            Err(e @ BridgeError::Connection(_)) => Err(e),
            // RealFlight keeps the Link controller across connections, so a new bridge can get a
            // "Preexisting controller reference" fault although the Link is still in control.
            Err(e) => {
                warn!("Failed to disable RC on the new RealFlight bridge: {e}");
                Ok(bridge)
            }
            Ok(()) => Ok(bridge),
        });
        match bridge {
            Ok(bridge) => {
                self.bridge = bridge;
                let failed = self.health.reconnected();
                info!("Reconnected to RealFlight after {failed} failed attempts");
                true
            }
            Err(e) => {
                let retry_in_s = self.health.reconnect_failed();
                self.health.report(format_args!(
                    "Failed to reconnect to RealFlight: {e}; retrying in {retry_in_s} s"
                ));
                false
            }
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
        angular_velocity_radps: Vec3::new(
            state.roll_rate_dps.to_radians(),
            state.pitch_rate_dps.to_radians(),
            -state.yaw_rate_dps.to_radians(),
        ),
        roll_rad: state.roll_deg.to_radians(),
        pitch_rad: state.inclination_deg.to_radians(),
        heading_rad: -state.azimuth_deg.to_radians(),
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
