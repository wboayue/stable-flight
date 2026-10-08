//! [![github]](https://github.com/wboayue/stable-flight)&ensp;[![crates-io]](https://crates.io/crates/stable-flight)&ensp;[![license]](https://opensource.org/licenses/MIT)
//!
//! [github]: https://img.shields.io/badge/github-8da0cb?style=for-the-badge&labelColor=555555&logo=github
//! [crates-io]: https://img.shields.io/badge/crates.io-fc8d62?style=for-the-badge&labelColor=555555&logo=rust
//! [license]: https://img.shields.io/badge/License-MIT-blue.svg?style=for-the-badge&labelColor=555555
//!
//! <br>
//!
//! Flight-control core for the Stable Flight controller: the pipeline traits ([`Commander`],
//! [`StateEstimator`] + [`Actuator`] = [`Vehicle`], [`Guidance`], [`Stabilizer`], [`Mixer`],
//! [`TelemetryPublisher`]), the data passed between them, copter guidance, the cascaded-PID
//! stabilizers, the mixers, and [`FlightLoop`], which sequences one control tick.
//!
//! This crate is `no_std` and has no simulator, I/O, or timing dependencies, so the same code
//! runs in software-in-the-loop simulation
//! ([`stable-flight-sitl`](https://crates.io/crates/stable-flight-sitl)) and, later, on flight
//! hardware. See the
//! [architecture overview](https://github.com/wboayue/stable-flight/blob/main/architecture.md).

#![cfg_attr(not(test), no_std)]

pub mod commanders;
pub mod composition;
pub mod controllers;
pub mod flight_loop;
pub mod guidance;
mod math;
pub mod mixers;
pub mod stabilizers;
#[cfg(any(test, feature = "testing"))]
pub mod testing;

use core::time::Duration;

#[doc(inline)]
pub use composition::ComposedVehicle;
#[doc(inline)]
pub use flight_loop::FlightLoop;

use nalgebra::UnitQuaternion;

pub use math::euler_angles;

/// Re-export of the [`nalgebra`] version this crate's public types use; depend on it through
/// here so the versions can't diverge.
///
/// # Examples
///
/// ```
/// use stable_flight::VehicleState;
/// use stable_flight::nalgebra::UnitQuaternion;
///
/// let state = VehicleState {
///     attitude: UnitQuaternion::from_euler_angles(0.0, 0.1, 0.0),
///     ..Default::default()
/// };
/// assert!(state.attitude.angle() > 0.0);
/// ```
pub use nalgebra;

/// 2D vector used throughout the pipeline (waypoint positions, tilt, controller limits).
///
/// # Examples
///
/// ```
/// use stable_flight::Vec2;
///
/// let tilt = Vec2::new(0.3, -0.1);
/// assert_eq!(tilt.component_mul(&Vec2::new(2.0, 2.0)), Vec2::new(0.6, -0.2));
/// ```
pub type Vec2 = nalgebra::Vector2<f32>;

/// 3D vector used throughout the pipeline (state vectors, moments, rates). Element-wise
/// products are `component_mul`; `*` between two vectors is a matrix product.
///
/// # Examples
///
/// ```
/// use stable_flight::Vec3;
///
/// let rates = Vec3::new(3.0, 4.0, 0.0);
/// assert_eq!(rates.norm(), 5.0);
/// ```
pub type Vec3 = nalgebra::Vector3<f32>;

/// Standard gravity, in m/s².
pub const GRAVITY: f32 = 9.806_65;

// Commanders

/// Represents an entity that produces flight control commands.
///
/// The `Commander` trait is responsible for producing the next flight control command given the elapsed time (dt) and the current vehicle state (current_state).
/// This allows for time-dependent and state-dependent command generation, such as ramping values, coordinating timed sequences, or adjusting commands based on the aircraft’s position or velocity.
///
/// Implementors of this trait may include:
/// - **User input handlers**: Translate joystick/keyboard inputs into manual commands (e.g., [Command::Attitude] or [Command::Rate]).
/// - **Navigation systems**: Generate waypoint commands (e.g., [Command::Waypoint]) for autonomous flight to specific locations.
/// - **Mission controllers**: Produce commands based on predefined flight plans or mission objectives.
/// - **Formation controllers**: Maintain relative positioning between multiple aircraft in a formation.
///
/// # Examples
///
/// Implementing a simple `Commander` that always returns a constant `Attitude` command:
/// ```
/// use stable_flight::VehicleState;
/// use stable_flight::{Commander, Command, AttitudeCommand};
///
/// struct ConstantAttitudeCommander;
///
/// impl Commander for ConstantAttitudeCommander {
///     fn next_command(&mut self, dt: f32, current_state: &VehicleState) -> Command {
///         Command::Attitude(AttitudeCommand {
///             throttle: 0.5,  // 50% throttle
///             roll: 0.0,      // No roll
///             pitch: 0.0,     // No pitch
///             yaw: 0.0,       // No yaw
///             arm: true,      // Armed
///             mode: 0,        // Default mode
///         })
///     }
/// }
/// ```
pub trait Commander {
    /// Produces the next command to be executed by the flight control system.
    ///
    /// # Parameters
    ///
    /// - `dt`: The time elapsed since the last command was generated in seconds.
    ///   This allows for time-dependent command generation, such as ramping up values
    ///   gradually or handling timed sequences.
    /// - `current_state`: The current state of the vehicle, as a `VehicleState`.
    ///   Typically includes information like position, velocity, attitude, and other
    ///   relevant data, allowing the commander to generate state-dependent commands.
    ///
    /// # Returns
    ///
    /// A `Command` instance that can be one of the following variants:
    /// - `Command::Attitude`: Direct manual control inputs for the aircraft's attitude (orientation).
    /// - `Command::Rate`: Direct manual control inputs for the aircraft's rotational rates.
    /// - `Command::Waypoint`: High-level navigational instructions for autonomous flight to a specific position.
    fn next_command(&mut self, dt: f32, current_state: &VehicleState) -> Command;
}

/// Represents a flight control command to be executed by the flight system.
///
/// The `Command` enum encapsulates different types of commands that can be issued
/// to control an aircraft. It supports three types of commands, categorized into two
/// primary control paradigms:
///
/// * `Manual` - Low-level direct control inputs typically generated from user input devices.
///   This includes:
///   - `Attitude`: Controls the aircraft's orientation (roll, pitch, yaw).
///   - `Rate`: Controls the aircraft's rotational rates (rate of change of roll, pitch, yaw).
/// * `Waypoint` - High-level navigational commands used for autonomous flight.
///
/// This separation allows the flight control system to handle both manual piloting
/// and automated navigation with a unified command interface.
///
/// # Examples
///
/// Creating a manual attitude command:
/// ```
/// use stable_flight::{Command, AttitudeCommand};
///
/// let manual_cmd = Command::Attitude(AttitudeCommand {
///     throttle: 0.75,  // 75% throttle
///     roll: 0.1,       // Slight right bank
///     pitch: -0.05,    // Slight nose down
///     yaw: 0.0,        // No yaw input
///     ..Default::default()
/// });
/// ```
///
/// Creating a manual rate command:
/// ```
/// use stable_flight::{Command, RateCommand};
///
/// let rate_cmd = Command::Rate(RateCommand {
///     throttle: 0.5,   // 50% throttle
///     roll_rate: 0.2,  // Moderate roll rate
///     pitch_rate: 0.1, // Moderate pitch rate
///     yaw_rate: 0.0,   // No yaw rate
///     ..Default::default()
/// });
/// ```
///
/// Creating a waypoint command:
/// ```
/// use stable_flight::{Command, Vec2, WaypointCommand};
///
/// let waypoint_cmd = Command::Waypoint(WaypointCommand {
///     position: Vec2::new(1000.0, 2000.0),  // Target position in world coordinates
///     velocity: Some(Vec2::new(15.0, 0.0)), // Optional desired velocity at target
///     altitude: 500.0,                 // Target altitude in meters
///     cruise_speed: 25.0,              // Desired speed during transit
///     roll: Some(0.05),                // Optional roll angle in radians
/// });
/// ```
///
/// # Notes
/// - For `WaypointCommand`, optional fields like `velocity` and `roll` can be `None`. When `None`,
///   the flight controller may use default values or ignore the parameter, depending on its implementation.
#[derive(Debug, Clone, Copy)]
pub enum Command {
    /// Direct manual control inputs for the aircraft's attitude (orientation).
    ///
    /// These values typically range from -1.0 to 1.0 (or 0.0 to 1.0 for throttle)
    /// and directly influence the aircraft's roll, pitch, and yaw orientation.
    /// The ranges are conventional and may not be enforced by the system.
    Attitude(AttitudeCommand),

    /// Direct manual control inputs for the aircraft's rotational rate.
    ///
    /// These values typically range from -1.0 to 1.0 (or 0.0 to 1.0 for throttle)
    /// and directly influence the rate of change of roll, pitch, and yaw.
    /// The ranges are conventional and may not be enforced by the system.
    Rate(RateCommand),

    /// Autonomous navigation command specifying a destination point.
    ///
    /// This command provides high-level navigational instructions that are translated
    /// into appropriate control surface and throttle settings by the flight controller.
    Waypoint(WaypointCommand),
}

/// Represents direct manual control of an aircraft's attitude (orientation).
///
/// This struct encapsulates pilot inputs from control devices like joysticks,
/// yokes, or keyboard/mouse combinations to control the aircraft's orientation
/// (roll, pitch, yaw) and throttle. It is typically used within a `Command::Attitude`
/// variant to issue manual flight commands to the flight system.
///
/// # Value Ranges
///
/// - `throttle`: `0.0` (idle) to `1.0` (full power)
/// - `roll`: `-1.0` (full left bank) to `1.0` (full right bank)
/// - `pitch`: `-1.0` (full nose down) to `1.0` (full nose up)
/// - `yaw`: `-1.0` (full left turn) to `1.0` (full right turn)
/// - `arm`: `false` (disarmed) or `true` (armed)
/// - `mode`: `0` to `2` (specific meaning depends on the flight system)
///
/// Note: The specified ranges are conventional and recommended for typical use.
/// Values outside these ranges may be clamped, ignored, or cause undefined behavior
/// depending on the flight system's implementation.
///
/// # Examples
///
/// Creating an `AttitudeCommand` for a slight climb with a right bank:
/// ```
/// use stable_flight::AttitudeCommand;
///
/// let cmd = AttitudeCommand {
///     throttle: 0.75,  // 75% throttle
///     roll: 0.1,       // Slight right bank
///     pitch: 0.05,     // Slight nose up
///     yaw: 0.0,        // No yaw input
///     arm: true,       // Arm the aircraft
///     mode: 1,         // Example mode (e.g., stabilized)
/// };
/// ```
///
/// # Default Implementation
///
/// The `Default` trait is implemented for `AttitudeCommand`, providing the following
/// default values:
/// - `throttle`: `0.0`
/// - `roll`: `0.0`
/// - `pitch`: `0.0`
/// - `yaw`: `0.0`
/// - `arm`: `false`
/// - `mode`: `0`
#[derive(Debug, Default, Clone, Copy)]
pub struct AttitudeCommand {
    /// Controls the engine power output, ranging from `0.0` (idle) to `1.0` (maximum).
    pub throttle: f32,

    /// Controls the aircraft's roll/bank angle, ranging from `-1.0` (full left bank)
    /// to `1.0` (full right bank).
    pub roll: f32,

    /// Controls the aircraft's pitch angle, ranging from `-1.0` (full nose down)
    /// to `1.0` (full nose up).
    pub pitch: f32,

    /// Controls the aircraft's yaw rotation, ranging from `-1.0` (full left turn)
    /// to `1.0` (full right turn).
    pub yaw: f32,

    /// Controls the arming state of the aircraft (`false` for disarmed, `true` for armed).
    /// Arming typically enables motors or other safety-critical systems.
    pub arm: bool,

    /// Controls the aircraft's flight mode, ranging from `0` to `2`.
    /// The specific meaning of each value depends on the flight system implementation
    /// (e.g., `0` might be manual mode, `1` stabilized, `2` autonomous).
    pub mode: u8,
}

/// Represents direct manual control of an aircraft's rotational rates.
///
/// This struct encapsulates pilot inputs from control devices like joysticks,
/// yokes, or keyboard/mouse combinations to control the aircraft's rate of rotation
/// (roll rate, pitch rate, yaw rate) and throttle. It is typically used within a
/// `Command::Rate` variant to issue manual flight commands to the flight system.
///
/// The "rate" refers to the speed of rotation around each axis, typically measured
/// in a normalized range where `-1.0` and `1.0` represent the maximum rates supported
/// by the flight system (e.g., radians per second, scaled to system limits).
///
/// # Value Ranges
///
/// - `throttle`: `0.0` (idle) to `1.0` (full power)
/// - `roll_rate`: `-1.0` (maximum left roll rate) to `1.0` (maximum right roll rate)
/// - `pitch_rate`: `-1.0` (maximum nose-down rate) to `1.0` (maximum nose-up rate)
/// - `yaw_rate`: `-1.0` (maximum left yaw rate) to `1.0` (maximum right yaw rate)
/// - `arm`: `false` (disarmed) or `true` (armed)
/// - `mode`: `0` to `2` (specific meaning depends on the flight system)
///
/// Note: The specified ranges are conventional and recommended for typical use.
/// Values outside these ranges may be clamped, ignored, or cause undefined behavior
/// depending on the flight system's implementation.
///
/// # Examples
///
/// Creating a `RateCommand` for a moderate right roll and slight nose-up rate:
/// ```
/// use stable_flight::RateCommand;
///
/// let cmd = RateCommand {
///     throttle: 0.6,    // 60% throttle
///     roll_rate: 0.3,   // Moderate right roll rate
///     pitch_rate: 0.1,  // Slight nose-up rate
///     yaw_rate: 0.0,    // No yaw rate
///     arm: true,        // Arm the aircraft
///     mode: 1,          // Example mode (e.g., stabilized)
/// };
/// ```
///
/// # Default Implementation
///
/// The `Default` trait is implemented for `RateCommand`, providing the following
/// default values:
/// - `throttle`: `0.0`
/// - `roll_rate`: `0.0`
/// - `pitch_rate`: `0.0`
/// - `yaw_rate`: `0.0`
/// - `arm`: `false`
/// - `mode`: `0`
#[derive(Debug, Default, Clone, Copy)]
pub struct RateCommand {
    /// Controls the engine power output, ranging from `0.0` (idle) to `1.0` (maximum).
    pub throttle: f32,

    /// Controls the aircraft's roll rate, ranging from `-1.0` (maximum left roll rate)
    /// to `1.0` (maximum right roll rate).
    pub roll_rate: f32,

    /// Controls the aircraft's pitch rate, ranging from `-1.0` (maximum nose-down rate)
    /// to `1.0` (maximum nose-up rate).
    pub pitch_rate: f32,

    /// Controls the aircraft's yaw rate, ranging from `-1.0` (maximum left yaw rate)
    /// to `1.0` (maximum right yaw rate).
    pub yaw_rate: f32,

    /// Controls the arming state of the aircraft (`false` for disarmed, `true` for armed).
    /// Arming typically enables motors or other safety-critical systems.
    pub arm: bool,

    /// Controls the aircraft's flight mode, ranging from `0` to `2`.
    /// The specific meaning of each value depends on the flight system implementation
    /// (e.g., `0` might be manual mode, `1` stabilized, `2` autonomous).
    pub mode: u8,
}

/// Represents a navigational waypoint for autonomous flight.
///
/// A `WaypointCommand` instructs the aircraft to navigate to a specific position
/// in 2D space at a given altitude, optionally with a specified velocity to be achieved
/// at the target position and a desired roll angle during the approach. This struct is
/// typically used within a `Command::Waypoint` variant to issue autonomous navigation
/// commands to the flight system.
///
/// # Value Ranges
///
/// - `position`: X, Y coordinates in 2D space (units and coordinate system depend on the flight system, typically meters in world coordinates)
/// - `velocity`: Optional velocity vector in meters per second (no specific range; depends on system limits)
/// - `altitude`: Target altitude in meters (typically non-negative; negative values may be interpreted as ground level or cause undefined behavior)
/// - `cruise_speed`: Desired speed in meters per second (typically positive; zero or negative values may cause the aircraft to stop or behave unpredictably)
/// - `roll`: Optional roll angle in radians (typically between `-π` and `π`; depends on system limits)
///
/// Note: The specified ranges are conventional and recommended for typical use.
/// Values outside these ranges may be clamped, ignored, or cause undefined behavior
/// depending on the flight system's implementation.
///
/// # Examples
///
/// Creating a `WaypointCommand` to navigate to a position with a specific altitude and speed:
/// ```
/// use stable_flight::{Vec2, WaypointCommand};
///
/// let cmd = WaypointCommand {
///     position: Vec2::new(1000.0, 2000.0),  // Target position (X=1000m, Y=2000m)
///     velocity: Some(Vec2::new(15.0, 0.0)), // Arrive with 15 m/s eastward velocity
///     altitude: 500.0,                 // Target altitude of 500 meters
///     cruise_speed: 25.0,              // Travel at 25 m/s
///     roll: Some(0.05),                // Slight right roll during approach
/// };
/// ```
///
/// # Behavior of Optional Fields
///
/// - If `velocity` is `None`, the flight system may choose a default velocity or ignore velocity constraints.
/// - If `roll` is `None`, the flight system typically maintains a default roll (e.g., zero or coordinated turn behavior).
///
/// # Default Implementation
///
/// The `Default` trait is implemented for `WaypointCommand`, providing the following
/// default values:
/// - `position`: zero
/// - `velocity`: `None`
/// - `altitude`: `0.0`
/// - `cruise_speed`: `0.0`
/// - `roll`: `None`
#[derive(Debug, Default, Clone, Copy)]
pub struct WaypointCommand {
    /// The target position in 2D space (X, Y coordinates).
    ///
    /// The coordinate system and units depend on the flight system implementation
    /// (typically meters in world coordinates).
    pub position: Vec2,

    /// Optional desired velocity vector at the waypoint, in meters per second.
    ///
    /// When provided, the aircraft will attempt to arrive at the position with this
    /// velocity. This is useful for creating smooth paths through multiple waypoints.
    /// If `None`, the flight system may choose a default velocity or ignore velocity constraints.
    pub velocity: Option<Vec2>,

    /// Target altitude in meters.
    ///
    /// Typically non-negative; negative values may be interpreted as ground level
    /// or cause undefined behavior depending on the flight system.
    pub altitude: f32,

    /// Desired cruise speed in meters per second while traveling to the waypoint.
    ///
    /// Typically positive; zero or negative values may cause the aircraft to stop
    /// or behave unpredictably depending on the flight system.
    pub cruise_speed: f32,

    /// Optional desired roll angle in radians.
    ///
    /// When provided, the aircraft will attempt to maintain this roll angle while
    /// approaching the waypoint. This is useful for creating coordinated turns or
    /// acrobatic maneuvers. If `None`, the flight system typically maintains a default
    /// roll (e.g., zero or coordinated turn behavior). Values typically range between
    /// `-π` and `π`, depending on system limits.
    pub roll: Option<f32>,
}

/// Represents the high-level control outputs from the stabilization system.
///
/// This struct encapsulates the desired thrust and moments to be applied to the
/// vehicle, which are later translated to specific actuator commands by a `Mixer`.
///
/// # Examples
///
/// Creating a control output for a simple hover command:
/// ```
/// use stable_flight::{ControlOutput, Vec3};
///
/// let output = ControlOutput::new(
///     0.5, // 50% thrust
///     Vec3::zeros() // No roll, pitch, or yaw moments
/// );
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct ControlOutput {
    /// Normalized thrust value, typically ranging from 0.0 (no thrust) to 1.0 (maximum thrust).
    pub thrust: f32,

    /// Control moments in roll (x), pitch (y), and yaw (z) axes.
    /// Values are typically normalized between -1.0 and 1.0, where the magnitude
    /// represents the proportion of maximum control authority in each axis.
    pub moments: Vec3,
}

impl ControlOutput {
    /// Creates a new `ControlOutput` with the specified thrust and control moments.
    ///
    /// # Parameters
    ///
    /// - `thrust`: Normalized thrust value, typically between 0.0 and 1.0.
    /// - `moments`: Control moments as a 3D vector (roll, pitch, yaw).
    ///
    /// # Returns
    ///
    /// A new `ControlOutput` instance with the specified parameters.
    pub fn new(thrust: f32, moments: Vec3) -> ControlOutput {
        ControlOutput { thrust, moments }
    }
}

/// Represents the physical state of a vehicle in 3D space.
///
/// This struct encapsulates all the key state information needed for flight control,
/// including position, velocity, acceleration, attitude, and angular velocity.
/// Quantities are plain SI `f32`s; field names carry the unit (`position_m`, `velocity_mps`, …).
///
/// # Frame contract
///
/// This table is the reference for how every [`StateEstimator`] reports state; a backend
/// converts from its native frames at its boundary, and `testing::conformance` (`testing`
/// feature) checks it.
///
/// Two frames are involved. Attitude follows aerospace convention: the NED world frame
/// (`x` north, `y` east, `z` down) and the FRD body frame (`x` forward, `y` right, `z` down).
/// Translation uses the same north and east axes but flips `z` to point **up**, so altitude
/// and climb rate are positive: `position_m`/`velocity_mps`/`acceleration_mps2` `z` is NED `z`
/// negated.
///
/// | Quantity | Frame | Units | Positive |
/// |---|---|---|---|
/// | `position_m` | `x` north, `y` east, `z` up | m | `z` is altitude above the backend's datum (e.g. sea level) |
/// | `velocity_mps`, `acceleration_mps2` | as `position_m` | m/s, m/s² | `z` climbing |
/// | `attitude` | NED world → FRD body, ZYX (yaw, pitch, roll); `UnitQuaternion::from_euler_angles(roll, pitch, heading)` | — | [`euler_angles`] of it match the fields below |
/// | `roll` | about body `x` | rad | right wing down |
/// | `pitch` | about body `y` | rad | nose up |
/// | `heading` | about NED `z` | rad | clockwise from north (east = π/2); any range, e.g. [0, 2π) or [-π, π) |
/// | `angular_velocity` | FRD body: `x` roll, `y` pitch, `z` yaw | rad/s | roll right, nose up, nose right |
///
/// # Examples
///
/// Creating a default vehicle state:
/// ```
/// use stable_flight::VehicleState;
///
/// let state = VehicleState::default();
/// ```
#[derive(Debug, Clone, Default, PartialEq)]
pub struct VehicleState {
    /// World position (north, east, up); see the [frame contract](VehicleState#frame-contract).
    pub position_m: Vec3,

    /// World velocity (north, east, up).
    pub velocity_mps: Vec3,

    /// World acceleration (north, east, up).
    pub acceleration_mps2: Vec3,

    /// Orientation quaternion representing the vehicle's attitude.
    /// The quaternion represents the rotation from world frame to body frame.
    pub attitude: UnitQuaternion<f32>,

    /// Body angular velocity in roll (x), pitch (y), and yaw (z), in radians per second.
    pub angular_velocity: Vec3,

    /// Current roll angle in radians, right wing down positive.
    pub roll: f32,

    /// Current pitch angle in radians, nose up positive.
    pub pitch: f32,

    /// Current heading (yaw) angle in radians.
    /// 0 is north, with positive values rotating clockwise.
    pub heading: f32,
    // Note: Additional fields for flight mode, GPS data, etc. may be added in the future.
}

/// Represents a physical vehicle that can be controlled by the flight system.
///
/// A `Vehicle` is anything that is both a [`StateEstimator`] and an [`Actuator`]; the blanket
/// impl below provides it automatically, so implement those two traits rather than this one.
/// The split mirrors hardware, where sensing (IMU, estimator) and actuation (ESC/servo output)
/// are usually separate drivers, while a simulator bridge can implement both on one type.
///
/// Examples: a simulator bridge such as `RealFlightVehicle`, a hardware IMU/estimator paired
/// with an ESC driver (see [`ComposedVehicle`]), or a test double such as
/// `testing::NullVehicle` (`testing` feature).
///
/// # Timing contract
///
/// [`FlightLoop::step`] calls [`StateEstimator::estimate_state`] once, then
/// [`Actuator::actuate`] once, every tick. A backend must:
///
/// - report state in the [`VehicleState` frame contract](VehicleState#frame-contract);
/// - talk to its simulator or hardware at most once per call, and should not block for longer
///   than a tick (document it if a blocking link can);
/// - document how old the state from `estimate_state` is relative to the last `actuate`. A
///   backend whose link sends controls and returns state in one round-trip exchanges in
///   `actuate` and returns the cached result, so tick *n* stabilizes on state reported when
///   the simulator received tick *n − 1*'s controls. A backend with a separate sensor read
///   should read fresh state in `estimate_state`;
/// - produce real state on the priming call (`dt == Duration::ZERO`, made once by
///   [`FlightLoop::prime`] before any `actuate`), contacting the simulator if it must, and
///   document what it returns if that fails.
///
/// # Examples
///
/// ```
/// use std::time::Duration;
/// use stable_flight::{Actuator, ActuatorCommand, StateEstimator, Vehicle, VehicleState};
///
/// struct HoverSim {
///     state: VehicleState,
/// }
///
/// impl StateEstimator for HoverSim {
///     fn estimate_state(&mut self, _dt: Duration) -> VehicleState {
///         self.state.clone()
///     }
/// }
///
/// impl Actuator for HoverSim {
///     fn actuate(&mut self, _command: &ActuatorCommand) {}
/// }
///
/// fn requires_vehicle(_: &impl Vehicle) {}
/// requires_vehicle(&HoverSim { state: VehicleState::default() });
/// ```
pub trait Vehicle: StateEstimator + Actuator {}

impl<T: StateEstimator + Actuator> Vehicle for T {}

/// Component responsible for estimating the vehicle's physical state.
///
/// The `StateEstimator` trait defines the interface for components that analyze
/// sensor data and other inputs to estimate the current state of the vehicle,
/// including position, velocity, attitude, and other relevant parameters.
///
/// Implementors of this trait may include:
/// - **Sensor fusion algorithms**: Combine data from multiple sensors (IMU, GPS, etc.)
/// - **Kalman filters**: Estimate state with statistical techniques
/// - **Simulation interfaces**: Get state directly from physics engines
pub trait StateEstimator {
    /// Estimates the current vehicle state based on sensor inputs and time elapsed.
    ///
    /// Takes `&mut self` because estimators typically advance internal state on each call
    /// (e.g. a Kalman predict/update).
    ///
    /// # Parameters
    /// - `dt`: Time elapsed since the last state estimation. The first call, made once before
    ///   the control loop starts, passes `Duration::ZERO`; implementations may use it to
    ///   initialize (see `RealFlightVehicle`).
    ///
    /// # Returns
    /// A `VehicleState` representing the current estimated state of the vehicle.
    fn estimate_state(&mut self, dt: Duration) -> VehicleState;
}

/// Component responsible for controlling the vehicle's actuators.
///
/// The `Actuator` trait defines the interface for components that translate
/// actuator commands into actual physical actions, such as moving control surfaces
/// or adjusting motor speeds.
///
/// Implementors of this trait may include:
/// - **Motor controllers**: Control electric motors for thrust
/// - **Servo drivers**: Position control surfaces like elevators and ailerons
/// - **Simulation interfaces**: Send control commands to physics engines
pub trait Actuator {
    /// Actuates the vehicle's control surfaces and/or motors based on the command.
    ///
    /// # Parameters
    ///
    /// - `actuator_command`: The command values to apply to the actuators.
    fn actuate(&mut self, actuator_command: &ActuatorCommand);
}

/// What the [`Stabilizer`] should hold this tick, as produced by a [`Guidance`].
///
/// A [`Command`] says what the operator or mission wants (stick positions, a waypoint);
/// a `Setpoint` is the inner-loop target derived from it. Values other than the absolute
/// heading are normalized: the stabilizer owns the physical limits (maximum tilt and rates)
/// and the thrust curve, and saturates out-of-range or NaN inputs per the input policy.
///
/// # Examples
///
/// ```
/// use stable_flight::{AttitudeSetpoint, Setpoint};
///
/// // Level, north-facing hover at half linear thrust.
/// let hover = Setpoint::Attitude(AttitudeSetpoint { thrust: 0.5, ..Default::default() });
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Setpoint {
    /// Hold a roll/pitch attitude and an absolute heading.
    Attitude(AttitudeSetpoint),
    /// Hold body rates.
    Rate(RateSetpoint),
}

/// Attitude target for the [`Stabilizer`]; see [`Setpoint`].
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct AttitudeSetpoint {
    /// Roll as a fraction of the stabilizer's maximum tilt, `-1.0` (left) to `1.0` (right).
    pub roll: f32,
    /// Pitch as a fraction of the stabilizer's maximum tilt, `-1.0` (nose down) to `1.0`
    /// (nose up).
    pub pitch: f32,
    /// Absolute heading target in radians, in the
    /// [frame contract](VehicleState#frame-contract)'s `heading` convention.
    pub heading: f32,
    /// Linear collective thrust, `0.0` to `1.0`, before the stabilizer's thrust curve.
    pub thrust: f32,
}

/// Body-rate target for the [`Stabilizer`]; see [`Setpoint`].
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct RateSetpoint {
    /// Roll rate as a fraction of the stabilizer's maximum, `-1.0` to `1.0` (right positive).
    pub roll_rate: f32,
    /// Pitch rate as a fraction of the stabilizer's maximum, `-1.0` to `1.0` (nose up
    /// positive).
    pub pitch_rate: f32,
    /// Yaw rate as a fraction of the stabilizer's maximum, `-1.0` to `1.0` (nose right
    /// positive).
    pub yaw_rate: f32,
    /// Linear collective thrust, `0.0` to `1.0`, before the stabilizer's thrust curve.
    pub thrust: f32,
}

/// Turns the commander's [`Command`] into an inner-loop [`Setpoint`].
///
/// Guidance is the outer loop: waypoint navigation (position and altitude hold), and the
/// interpretation of pilot sticks that needs memory, such as integrating yaw stick into a
/// heading-hold target. The [`Stabilizer`] then tracks the setpoint. [`FlightLoop`] calls both
/// every tick; a guidance that runs its loops slower paces them itself from `dt`. Why they are
/// split is in the
/// [architecture overview](https://github.com/wboayue/stable-flight/blob/main/architecture.md#core-components).
///
/// # Examples
///
/// A guidance that flies rate commands through unchanged, and otherwise levels out with
/// thrust cut:
///
/// ```
/// use stable_flight::{AttitudeSetpoint, Command, Guidance, RateSetpoint, Setpoint, VehicleState};
///
/// struct RateOnly;
///
/// impl Guidance for RateOnly {
///     fn guide(&mut self, _dt: f32, command: &Command, state: &VehicleState) -> Setpoint {
///         match command {
///             Command::Rate(rate) => Setpoint::Rate(RateSetpoint {
///                 roll_rate: rate.roll_rate,
///                 pitch_rate: rate.pitch_rate,
///                 yaw_rate: rate.yaw_rate,
///                 thrust: rate.throttle,
///             }),
///             _ => Setpoint::Attitude(AttitudeSetpoint {
///                 heading: state.heading,
///                 ..Default::default()
///             }),
///         }
///     }
/// }
/// ```
pub trait Guidance {
    /// Produces this tick's inner-loop setpoint.
    ///
    /// # Parameters
    ///
    /// - `dt`: Time elapsed since the last guidance step in seconds. Must be positive;
    ///   implementations may assert it (runners guarantee it).
    /// - `command`: The commander's current command.
    /// - `state`: The current estimated state of the vehicle.
    fn guide(&mut self, dt: f32, command: &Command, state: &VehicleState) -> Setpoint;

    /// Primes guidance from a known-good vehicle state before the first tick.
    ///
    /// Warms internal state (e.g. the heading-hold target) so guidance starts from where the
    /// vehicle is rather than from zero. Called once before the control loop starts. The
    /// default does nothing.
    fn prime(&mut self, _state: &VehicleState) {}
}

/// Component responsible for flight control and stabilization: the inner loop.
///
/// The `Stabilizer` trait defines the interface for components that implement
/// control algorithms to track a [`Setpoint`] from [`Guidance`], producing the thrust and
/// moments that the [`Mixer`] turns into actuator commands.
///
/// Implementors of this trait may include:
/// - **PID controllers**: Simple feedback control for attitude stability
/// - **LQR controllers**: Optimal control for complex flight dynamics
/// - **Model predictive controllers**: Advanced control with future prediction
/// - **Cascaded control systems**: Hierarchical controllers for different aspects of flight
pub trait Stabilizer {
    /// Generates control outputs based on the current setpoint and vehicle state.
    ///
    /// # Parameters
    ///
    /// - `dt`: Time elapsed since the last control step in seconds. Must be positive;
    ///   implementations may assert it (runners guarantee it).
    /// - `setpoint`: The attitude or rate target to track.
    /// - `state`: The current estimated state of the vehicle.
    ///
    /// # Returns
    ///
    /// A `ControlOutput` containing the desired thrust and control moments.
    fn stabilize(&mut self, dt: f32, setpoint: &Setpoint, state: &VehicleState) -> ControlOutput;

    /// Primes the stabilizer from a known-good vehicle state before the first tick.
    ///
    /// Warms internal state (e.g. integrators or filters) so control starts from where the
    /// vehicle is rather than from zero. Called once before the control loop starts, after
    /// [`Guidance::prime`]. The default does nothing.
    fn prime(&mut self, _state: &VehicleState) {}
}

/// Component responsible for publishing telemetry data from the flight system.
///
/// The `TelemetryPublisher` trait defines the interface for components that send
/// flight data to external systems for logging, monitoring, or visualization.
/// This can include sending data to ground control stations, logging systems,
/// or other monitoring tools.
///
/// Implementors of this trait may include:
/// - **Mavlink publishers**: Send telemetry via the MAVLink protocol
/// - **Data loggers**: Record flight data for later analysis
/// - **Visualization interfaces**: Send data to real-time flight displays
pub trait TelemetryPublisher {
    /// Publishes one control-loop tick to external systems.
    ///
    /// Fire-and-forget: the flight loop cannot act on a telemetry failure, so implementations
    /// handle (log) their own errors instead of returning them.
    ///
    /// # Parameters
    ///
    /// - `record`: Everything the loop produced this tick; see [`TelemetryRecord`].
    fn publish(&mut self, record: &TelemetryRecord);
}

/// One control-loop tick, as handed to a [`TelemetryPublisher`].
///
/// The loop assembles it after actuating, so every stage's output is available. Fields borrow
/// from the loop and live only for the tick; publishers copy what they need to keep. New
/// fields can be added without changing the `publish` signature.
///
/// # Examples
///
/// ```
/// use std::time::Duration;
/// use stable_flight::{
///     ActuatorCommand, AttitudeCommand, Command, ControlOutput, Setpoint, TelemetryPublisher,
///     TelemetryRecord, VehicleState,
/// };
///
/// struct ThrustLogger;
///
/// impl TelemetryPublisher for ThrustLogger {
///     fn publish(&mut self, record: &TelemetryRecord) {
///         println!("{:?}: thrust {}", record.dt, record.control.thrust);
///     }
/// }
///
/// let command = Command::Attitude(AttitudeCommand::default());
/// let setpoint = Setpoint::Attitude(Default::default());
/// let state = VehicleState::default();
/// let control = ControlOutput::default();
/// let actuators = ActuatorCommand::default();
///
/// ThrustLogger.publish(&TelemetryRecord {
///     dt: Duration::from_millis(4),
///     command: &command,
///     setpoint: &setpoint,
///     state: &state,
///     control: &control,
///     actuators: &actuators,
/// });
/// ```
#[derive(Debug, Clone, Copy)]
pub struct TelemetryRecord<'a> {
    /// Nominal loop period (`1 / target frequency`), not measured elapsed time.
    pub dt: Duration,
    /// Command the commander produced.
    pub command: &'a Command,
    /// Setpoint guidance derived from `command`.
    pub setpoint: &'a Setpoint,
    /// Estimated vehicle state the stabilizer acted on.
    pub state: &'a VehicleState,
    /// Stabilizer output (thrust and moments).
    pub control: &'a ControlOutput,
    /// Mixer output sent to the actuators.
    pub actuators: &'a ActuatorCommand,
}

/// Represents commands sent to the vehicle's actuators.
///
/// This struct encapsulates all possible actuation commands for various vehicle types,
/// including multirotor motors, fixed-wing control surfaces, and hybrid configurations.
/// Not all fields will be used for every vehicle type; the specific mapping is determined
/// by the `Mixer` implementation.
///
/// # Examples
///
/// Creating actuator commands for a quadcopter:
/// ```
/// use stable_flight::ActuatorCommand;
///
/// let mut cmd = ActuatorCommand::default();
/// cmd.motors = [0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0]; // Four motors at 50% power
/// ```
///
/// Creating actuator commands for a fixed-wing aircraft:
/// ```
/// use stable_flight::ActuatorCommand;
///
/// let mut cmd = ActuatorCommand::default();
/// cmd.throttle = 0.7;    // 70% throttle
/// cmd.elevator = 0.1;    // Slight up elevator
/// cmd.aileron = -0.05;   // Slight left roll
/// cmd.rudder = 0.0;      // No rudder
/// ```
#[derive(Debug, Clone, Copy)]
pub struct ActuatorCommand {
    /// Motor output values, ranging from 0.0 (off) to 1.0 (full power).
    /// For multirotors, the specific mapping of array indices to physical motors
    /// depends on the vehicle configuration and mixer.
    pub motors: [f32; 8],

    /// Rudder control value, ranging from -1.0 (full left) to 1.0 (full right).
    /// Used primarily for fixed-wing aircraft.
    pub rudder: f32,

    /// Elevator control value, ranging from -1.0 (full down) to 1.0 (full up).
    /// Used primarily for fixed-wing aircraft.
    pub elevator: f32,

    /// Aileron control value, ranging from -1.0 (roll left) to 1.0 (roll right).
    /// Used primarily for fixed-wing aircraft.
    pub aileron: f32,

    /// Throttle control value, ranging from 0.0 (idle) to 1.0 (full power).
    /// Used primarily for fixed-wing aircraft with a single motor/engine.
    pub throttle: f32,

    /// Left elevon control value, ranging from -1.0 to 1.0.
    /// Used for flying wing or delta wing configurations.
    pub elevon_left: f32,

    /// Right elevon control value, ranging from -1.0 to 1.0.
    /// Used for flying wing or delta wing configurations.
    pub elevon_right: f32,
}

impl Default for ActuatorCommand {
    fn default() -> Self {
        ActuatorCommand {
            motors: [0.0; 8],
            rudder: 0.0,
            elevator: 0.0,
            aileron: 0.0,
            throttle: 0.0,
            elevon_left: 0.0,
            elevon_right: 0.0,
        }
    }
}

// == Mixers ==

/// Component responsible for converting control outputs to actuator commands.
///
/// The `Mixer` trait defines the interface for components that translate
/// generalized control outputs (thrust and moments) into specific actuator
/// commands appropriate for the vehicle configuration. It maps abstract control
/// concepts to physical actuator movements.
///
/// Implementors of this trait may include:
/// - **Quadcopter mixers**: Map controls to four motors in different configurations (X, +)
/// - **Hexacopter mixers**: Map controls to six motors
/// - **Fixed-wing mixers**: Map controls to throttle, ailerons, elevator, and rudder
pub trait Mixer {
    /// Transforms abstract control outputs into specific actuator commands.
    ///
    /// This method is the core functionality of the `Mixer` trait, providing the critical
    /// translation layer between the stabilization system's desired control effects and
    /// the actual physical actuator settings needed to achieve those effects.
    ///
    /// # Parameters
    ///
    /// * `controls` - Reference to a [`ControlOutput`] struct containing normalized thrust
    ///   and moment values that represent the desired control effect.
    ///
    /// # Returns
    ///
    /// An [`ActuatorCommand`] containing specific actuator settings appropriate for the
    /// vehicle's physical configuration. Depending on the vehicle type, different fields
    /// in the returned struct will be populated:
    ///
    /// * For multicopters: Primarily the `motors` array
    /// * For fixed-wing: `throttle`, `aileron`, `elevator`, and `rudder`
    /// * For hybrid vehicles: Combination of the above
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::{ControlOutput, Mixer, Vec3};
    /// use stable_flight::mixers::QuadCopterMixer;
    ///
    /// let mixer = QuadCopterMixer::default();
    ///
    /// // Create control output with 70% thrust and slight roll right
    /// let control_output = ControlOutput::new(
    ///     0.7,                    // 70% thrust
    ///     Vec3::new(0.1, 0.0, 0.0)     // slight roll right, no pitch, no yaw
    /// );
    ///
    /// // Convert abstract controls into specific motor commands
    /// let actuator_command = mixer.mix(&control_output);
    ///
    /// // Now actuator_command contains specific motor values for a quadcopter
    /// ```
    fn mix(&self, controls: &ControlOutput) -> ActuatorCommand;
}
