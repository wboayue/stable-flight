use std::time::{Duration, Instant};

use log::{debug, info, warn};
use realflight_bridge::{
    BridgeError, ControlInputs, Quaternion as RealFlightQuaternion, RealFlightBridge,
    SimulatorState,
};
use stable_flight::Vec3;
use stable_flight::nalgebra::{Quaternion, UnitQuaternion};

use stable_flight::{Actuator, ActuatorCommand, GRAVITY, StateSource, VehicleState};

use crate::{ActuatorMapping, MappingError};
use health::LinkHealth;

mod health;
#[cfg(test)]
mod tests;

/// [`RealFlightVehicle`] logs an exchange slower than this at `debug` (five ticks at 250 Hz),
/// to tell a stall in the RealFlight round trip from one elsewhere in the tick (#255). The
/// `exchange-latency-realflight` example's default threshold.
pub const SLOW_EXCHANGE: Duration = Duration::from_millis(20);

/// I/O timeout for a `RealFlightRemoteBridge` (`with_timeouts`), as the examples use it.
///
/// It must stay above the proxy's request timeout, so a slow RealFlight comes back as an error
/// reply rather than a client timeout. RealFlight needs that raised from realflight-bridge's
/// 50 ms default: run the proxy with `--request-timeout-ms 150` (an aircraft reset, and the
/// exchanges just after it, take over 50 ms). A timeout drops the connection, and
/// [`with_reconnect`](RealFlightVehicle::with_reconnect) rebuilds it.
pub const REMOTE_IO_TIMEOUT: Duration = Duration::from_millis(250);

/// Builds a fresh bridge after the connection is lost; see
/// [`RealFlightVehicle::with_reconnect`].
type Reconnect<T> = Box<dyn FnMut() -> Result<T, BridgeError> + Send>;

/// Configuration parameters for connecting to a RealFlight simulation environment.
///
/// This structure combines the communication bridge with the channel mapping and the airframe's
/// body rotation.
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
/// use stable_flight::nalgebra::UnitQuaternion;
///
/// let bridge = RealFlightLocalBridge::new().expect("Failed to create bridge");
/// let config = RealFlightConfiguration {
///     mapping: ActuatorMapping::default(),
///     body_rotation: UnitQuaternion::identity(),
///     bridge,
/// };
/// ```
pub struct RealFlightConfiguration<T>
where
    T: RealFlightBridge,
{
    /// Configuration mapping actuator commands to simulator channels.
    pub mapping: ActuatorMapping,
    /// The vehicle's body frame expressed in RealFlight's: the attitude the vehicle reports when
    /// RealFlight reads level. Identity for a model whose RealFlight frame is already the
    /// vehicle's; see [Body rotation](RealFlightVehicle#body-rotation).
    pub body_rotation: UnitQuaternion<f32>,
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
/// exactly one exchange, in [`Actuator::actuate`]. [`StateSource::state`] returns
/// the state cached from that exchange: guidance and the stabilizer on tick *n* see the state RealFlight
/// reported when it received tick *n − 1*'s controls, one tick old. The one exception is the
/// priming call (`dt == Duration::ZERO`), which makes an exchange with neutral inputs so the
/// first state is real; if that exchange fails, it logs an error and returns
/// `VehicleState::default()`. `exchange_data` is a blocking network round-trip, so a slow link
/// stalls the tick; an exchange over 20 ms is logged at `debug` with its duration. The bridge
/// bounds the stall: `RealFlightRemoteBridge` times each call out (100 ms by default; the
/// examples use [`REMOTE_IO_TIMEOUT`]), and `RealFlightLocalBridge` each simulator request (50 ms
/// by default). A timeout is a `BridgeError::Connection` (see Connection loss). See the core
/// `Vehicle` docs for the cross-backend contract.
///
/// # Connection loss
///
/// A failed exchange is logged at most once a second, with a count of the failures skipped in
/// between. `state` keeps returning the last state RealFlight reported until an exchange
/// succeeds again: outputs computed meanwhile never reach the simulator, and the controllers
/// only wind up against a frozen error, bounded by their `max_integral`.
///
/// A `BridgeError::Connection` can leave the bridge unusable (a remote bridge is then out of
/// sync for good). With [`with_reconnect`](RealFlightVehicle::with_reconnect), the vehicle
/// drops the bridge at once (closing its connection: the RealFlight proxy serves one client at a
/// time, and accepts a new one only once the old one disconnects), waits 0.5 s, and builds a new bridge, doubling the wait after each failed
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
/// # Body rotation
///
/// Some models' RealFlight body frame isn't the vehicle's. The eXplora VTOL tailsitter reads level
/// sitting on its tail (a hover-level, copter-style frame), while a tailsitter's body frame is the
/// airplane's, x along the nose, so that hover is pitch +90° (#314). Like a flight controller
/// board's mounting, this is a constant rotation applied at the boundary:
/// [`RealFlightConfiguration::body_rotation`] `m`, the vehicle's body frame expressed in
/// RealFlight's. With RealFlight's attitude `q_rf`, the vehicle's attitude is `q_rf · m`, and
/// body rates and specific force (RF frame) are rotated by `m⁻¹`. World-frame quantities don't
/// change. The eXplora's `m` is +90° about body y: on its tail it reads pitch +90°, and level
/// forward flight reads pitch 0.
///
/// # Examples
///
/// ```no_run
/// use stable_flight_sitl_realflight::realflight_bridge::RealFlightLocalBridge;
/// use stable_flight_sitl_realflight::{ActuatorMapping, RealFlightConfiguration, RealFlightVehicle};
/// use stable_flight::nalgebra::UnitQuaternion;
/// use stable_flight::{Actuator, ActuatorCommand, StateSource};
/// use std::time::Duration;
///
/// let mut vehicle = RealFlightVehicle::new(RealFlightConfiguration {
///     mapping: ActuatorMapping::default(),
///     body_rotation: UnitQuaternion::identity(),
///     bridge: RealFlightLocalBridge::new().expect("Failed to create bridge"),
/// });
///
/// let command = ActuatorCommand::default();
/// vehicle.actuate(&command);
/// let state = vehicle.state(Duration::from_secs(1));
/// ```
pub struct RealFlightVehicle<T>
where
    T: RealFlightBridge,
{
    /// The communication bridge to the RealFlight simulator; `None` once a connection error
    /// dropped it, until a reconnect replaces it.
    bridge: Option<T>,
    /// The last state RealFlight reported.
    state: VehicleState,
    /// Configuration mapping actuator commands to simulator channels.
    mapping: ActuatorMapping,
    /// The vehicle's body frame expressed in RealFlight's.
    body_rotation: UnitQuaternion<f32>,
    /// Builds a new bridge after a connection error; `None` keeps the one bridge.
    reconnect: Option<Reconnect<T>>,
    /// Reconnect timing and failure-log rate limiting.
    health: LinkHealth,
}

impl<T> StateSource for RealFlightVehicle<T>
where
    T: RealFlightBridge,
{
    /// Returns the vehicle state cached from the last simulator exchange.
    ///
    /// Does not contact the simulator, except on the priming call (`dt == Duration::ZERO`),
    /// which makes one exchange with neutral inputs (see [Timing](RealFlightVehicle#timing)).
    /// Otherwise `dt` is ignored: the simulator reports its own truth state.
    fn state(&mut self, dt: Duration) -> VehicleState {
        self.health.advance(dt.as_secs_f32());
        if dt.is_zero() {
            // initialize state if dt is zero
            self.exchange(&ControlInputs::default());
        }

        self.state.clone()
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
        let input = self.mapping.channels(command);
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
    /// Warnings are logged if either fails; the vehicle is still created.
    ///
    /// # Panics
    ///
    /// If `configuration.mapping` is invalid ([`ActuatorMapping::validate`]); use
    /// [`try_new`](Self::try_new) to handle that as an error.
    pub fn new(configuration: RealFlightConfiguration<T>) -> RealFlightVehicle<T> {
        Self::try_new(configuration).unwrap_or_else(|e| panic!("invalid ActuatorMapping: {e}"))
    }

    /// Like [`new`](Self::new), but returns an invalid mapping as an error instead of
    /// panicking. Validates before touching the simulator.
    ///
    /// # Errors
    ///
    /// The mapping's first [`MappingError`].
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use stable_flight_sitl_realflight::realflight_bridge::RealFlightLocalBridge;
    /// use stable_flight_sitl_realflight::{
    ///     ActuatorMapping, MappingError, RealFlightConfiguration, RealFlightVehicle,
    /// };
    /// use stable_flight::nalgebra::UnitQuaternion;
    ///
    /// let mut mapping = ActuatorMapping::default();
    /// mapping.motors[0] = Some(12); // RealFlight has channels 0–11
    /// let result = RealFlightVehicle::try_new(RealFlightConfiguration {
    ///     mapping,
    ///     body_rotation: UnitQuaternion::identity(),
    ///     bridge: RealFlightLocalBridge::new().expect("Failed to create bridge"),
    /// });
    /// assert!(matches!(result, Err(MappingError::ChannelOutOfRange { channel: 12 })));
    /// ```
    pub fn try_new(
        configuration: RealFlightConfiguration<T>,
    ) -> Result<RealFlightVehicle<T>, MappingError> {
        configuration.mapping.validate()?;

        if let Err(e) = configuration.bridge.disable_rc() {
            warn!("Failed to disable RC: {}", e);
        }

        if let Err(e) = configuration.bridge.reset_aircraft() {
            warn!("Failed to reset aircraft: {}", e);
        }

        Ok(RealFlightVehicle {
            bridge: Some(configuration.bridge),
            state: VehicleState::default(),
            mapping: configuration.mapping,
            body_rotation: configuration.body_rotation,
            reconnect: None,
            health: LinkHealth::default(),
        })
    }

    /// Rebuilds the bridge with `reconnect` after a connection error, with backoff (see
    /// [Connection loss](RealFlightVehicle#connection-loss)).
    ///
    /// `reconnect` is called on the flight loop's thread and blocks the tick, so it should
    /// connect with a short timeout and keep a bounded I/O timeout ([`REMOTE_IO_TIMEOUT`]): the
    /// new bridge's `disable_rc` runs in the same tick.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use std::time::Duration;
    /// use stable_flight_sitl_realflight::realflight_bridge::{BridgeError, RealFlightRemoteBridge};
    /// use stable_flight_sitl_realflight::{
    ///     ActuatorMapping, REMOTE_IO_TIMEOUT, RealFlightConfiguration, RealFlightVehicle,
    /// };
    /// use stable_flight::nalgebra::UnitQuaternion;
    ///
    /// let host = String::from("127.0.0.1:8080");
    /// let connect = move || {
    ///     RealFlightRemoteBridge::with_timeouts(&host, Duration::from_millis(250), REMOTE_IO_TIMEOUT)
    /// };
    /// let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
    ///     mapping: ActuatorMapping::default(),
    ///     body_rotation: UnitQuaternion::identity(),
    ///     bridge: connect().expect("Failed to create bridge"),
    /// })
    /// .with_reconnect(move || connect().map_err(BridgeError::from));
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
        let Some(bridge) = &self.bridge else {
            return;
        };

        let start = Instant::now();
        let result = bridge.exchange_data(input);
        let elapsed = start.elapsed();
        if elapsed > SLOW_EXCHANGE {
            debug!("Slow RealFlight exchange: {} ms", elapsed.as_millis());
        }
        match result {
            Ok(state) => self.state = vehicle_state(&state, &self.body_rotation),
            Err(e) => {
                self.health
                    .report(format_args!("Failed to exchange data with RealFlight: {e}"));
                if matches!(e, BridgeError::Connection(_)) && self.reconnect.is_some() {
                    // Close the connection now, so the proxy can accept the reconnect.
                    self.bridge = None;
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
                self.bridge = Some(bridge);
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
}

/// Converts one RealFlight report to the `VehicleState` frame contract.
///
/// This is the whole of [`RealFlightVehicle`]'s state estimate, as a pure function, so it can
/// also convert recorded or replayed reports. See [Frames](RealFlightVehicle#frames) for
/// RealFlight's conventions, and [Body rotation](RealFlightVehicle#body-rotation) for
/// `body_rotation`, the vehicle's body frame expressed in RealFlight's (identity for most models).
///
/// # Examples
///
/// ```
/// use stable_flight::nalgebra::UnitQuaternion;
/// use stable_flight_sitl_realflight::realflight_bridge::{Quaternion, SimulatorState};
/// use stable_flight_sitl_realflight::vehicle_state;
///
/// // RealFlight reports position east/north; the contract wants north/east.
/// let report = SimulatorState {
///     aircraft_position_x_m: 3.0,
///     aircraft_position_y_m: 4.0,
///     ..Default::default()
/// };
/// let state = vehicle_state(&report, &UnitQuaternion::identity());
/// assert_eq!(state.position_m.x, 4.0);
/// assert_eq!(state.position_m.y, 3.0);
///
/// // A tailsitter whose RealFlight frame is hover-level: level in RealFlight is nose up.
/// let level = SimulatorState {
///     orientation: Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 },
///     ..Default::default()
/// };
/// let nose_up = UnitQuaternion::from_euler_angles(0.0, std::f32::consts::FRAC_PI_2, 0.0);
/// let state = vehicle_state(&level, &nose_up);
/// assert!((state.pitch_rad.to_degrees() - 90.0).abs() < 0.05);
/// ```
pub fn vehicle_state(state: &SimulatorState, body_rotation: &UnitQuaternion<f32>) -> VehicleState {
    // RealFlight's own body frame first; `rotate_body` then turns it into the vehicle's and
    // fills the Euler fields from the quaternion.
    let attitude = realflight_attitude(&state.orientation);

    // World velocity is NED.
    let velocity = state.velocity_world_mps;

    // RealFlight's world acceleration is unreliable (z reads ~7.5 m/s² at rest), so rotate the
    // FRD body specific force to NED and add gravity back. The world result doesn't depend on
    // the body frame.
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
        angular_velocity_radps: realflight_body_rates_radps(state),
        // From the quaternion (by `rotate_body`) rather than RealFlight's own Euler angles, so
        // they always agree with `attitude` (they match RealFlight's away from ±90° pitch,
        // where both are degenerate: roll and heading turn about the same axis).
        ..Default::default()
    }
    .rotate_body(body_rotation)
}

/// RealFlight's attitude (NED → RealFlight's FRD body frame), before any body rotation. Its
/// quaternion axes are (y, x, -z) of NED → FRD. Taken as reported, without renormalizing: it is
/// already unit.
fn realflight_attitude(q: &RealFlightQuaternion) -> UnitQuaternion<f32> {
    UnitQuaternion::new_unchecked(Quaternion::new(q.w, q.y, q.x, -q.z))
}

/// RealFlight's body rates in its FRD body frame, rad/s, before any body rotation. Yaw rate is
/// counterclockwise-positive (nose left).
fn realflight_body_rates_radps(state: &SimulatorState) -> Vec3 {
    Vec3::new(
        state.roll_rate_dps.to_radians(),
        state.pitch_rate_dps.to_radians(),
        -state.yaw_rate_dps.to_radians(),
    )
}

impl<T> Drop for RealFlightVehicle<T>
where
    T: RealFlightBridge,
{
    /// Cleans up by re-enabling RC mode when the vehicle is dropped.
    ///
    /// # Notes
    ///
    /// Logs a warning if RC mode cannot be re-enabled, including when a lost connection left
    /// no bridge to do it on.
    fn drop(&mut self) {
        let Some(bridge) = &self.bridge else {
            warn!("Failed to enable RC: no RealFlight connection");
            return;
        };
        if let Err(e) = bridge.enable_rc() {
            warn!("Failed to enable RC: {}", e);
        }
    }
}
